// ISC License - see native_rw/LICENSE.
//
// Contributor interface + concrete contributors, each written ONCE templated
// on the scalar type S and instantiated for S = double (numeric cross-check)
// and S = heyoka::expression (symbolic build).
//
// DESIGN DECISIONS (see DESIGN.md Addendum 1):
//  - A4: NO EffectorMassProps aggregation. HubContributorT owns mass/inertia
//    directly. There is no aggregate mass-property path.
//  - A5: gravity runs FIRST; g_N is passed down as Vector3<S>.
//  - Constraint (1) from the validated prototype: Js*gs*gs^T must NOT go into
//    matrixD for a balanced rotor (it cancels exactly against the spin
//    equation; folding it in costs 8.1e-5 in omegaDot and breaks momentum
//    conservation). The rotor contributes only -w x h - u*gs to vecRot, and
//    D carries the hub inertia alone.
//  - A3: only + - * / and sqrt anywhere. No clamps in the graph; the caller
//    saturates u before writing the input slots.
//  - A6: autonomous build. No time variable is emitted. `integTime` is carried
//    for future use only.
#ifndef OMNI_CONTRIBUTORS_HPP
#define OMNI_CONTRIBUTORS_HPP

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include "omni/backsub_t.hpp"
#include "omni/mrp_t.hpp"

namespace omni {

// ---------------------------------------------------------------------------
// Evaluation context: full state + full parameter vector, one instantiation.
// Contributors slice what they need via offsets assigned at registration.
// Hub attitude/rate/position live at FIXED offsets (A1: 12 hub states first).
// ---------------------------------------------------------------------------
template <typename S>
struct EvalCtx {
    const std::vector<S>& states;
    const std::vector<S>& pars;

    Vector3<S> r() const {
        Vector3<S> v; v[0] = states[0]; v[1] = states[1]; v[2] = states[2]; return v;
    }
    Vector3<S> vel() const {
        Vector3<S> v; v[0] = states[3]; v[1] = states[4]; v[2] = states[5]; return v;
    }
    Vector3<S> sigma() const {
        Vector3<S> v; v[0] = states[6]; v[1] = states[7]; v[2] = states[8]; return v;
    }
    Vector3<S> omega() const {
        Vector3<S> v; v[0] = states[9]; v[1] = states[10]; v[2] = states[11]; return v;
    }
};

// ---------------------------------------------------------------------------
// Base interface.
// ---------------------------------------------------------------------------
template <typename S>
class ContributorT {
public:
    virtual ~ContributorT() = default;

    virtual std::string kind() const = 0;
    virtual size_t n_states() const = 0;
    virtual size_t n_inputs() const = 0;
    // Own parameters EXCLUDING inputs (inputs live in the leading block).
    virtual size_t n_pars() const = 0;
    virtual std::vector<std::string> state_names() const = 0;
    virtual std::vector<std::string> input_names() const = 0;
    virtual std::vector<std::string> par_names() const = 0;
    virtual std::vector<double> par_defaults() const = 0;

    // Offsets assigned by OmniSystem at registration (A2).
    size_t state_offset = 0;
    size_t input_offset = 0;
    size_t par_offset = 0;

    virtual void updateContributions(double integTime,
                                     BackSubMatricesT<S>& blocks,
                                     const Vector3<S>& g_N,
                                     const EvalCtx<S>& ctx) const = 0;

    // Derivatives of this contributor's OWN states (length n_states()).
    virtual std::vector<S> state_derivatives(const Vector3<S>& omega,
                                             const Vector3<S>& omegaDot,
                                             const EvalCtx<S>& ctx) const = 0;

    // DynamicEffector channel: default no-op so existing contributors are
    // untouched.  `dt` is supplied because on-time -> duty conversion needs it
    // (exactly as stock's computeForceTorque(t, timeStep)), and ctx is needed
    // because some contributions (e.g. F_N = R^T F_B) depend on attitude.
    virtual void computeForceTorque(double /*integTime*/, double /*dt*/,
                                   ForceTorqueT<S>& /*ft*/,
                                   const EvalCtx<S>& /*ctx*/) const {}

    // Contributor-owned input conversion (DESIGN Q2).  `raw` is the plain
    // payload read from the Basilisk message by the module (message types stay
    // in the module); the contributor converts it -- including anything
    // dt-dependent such as a thruster quantizer -- into its own parameter
    // slots.  n_raw is the length of the payload.
    virtual void fillInputs(double dt, const double* raw, size_t n_raw,
                            std::vector<S>& pars) const {
        (void)dt; (void)raw; (void)n_raw; (void)pars;
    }

    // True if this contributor needs the DynamicEffector force/torque channel
    // to be evaluated each step (false = contributes nothing there).
    virtual bool has_force_torque() const { return false; }
};

// ---------------------------------------------------------------------------
// Hub: translational + rotational rigid-body blocks.
// Config: mass, full inertia I and its inverse invI are PARAMETERS (A2: they
// live in ctx.pars at [par_offset, par_offset+13)); r_BcB_B must be zero.
//
// INPUT-INERTIA SEMANTICS (measured against stock to 8 digits): the input I
// follows stock IHubPntBc_B semantics, i.e. the TOTAL inertia including the
// wheels. The back-substitution D block carries the hub-only part:
//   D = I - Dcorr,  Dcorr = sum_wheels Js * gs * gs^T  (full outer product,
//   so tilted wheels are handled identically).
// Dcorr reads Js from the wheels' own par slots (fully parametric), so
// Monte-Carlo dispersion keeps working; only the spin axes are structural.
// Gravity-gradient and translational terms keep using the TOTAL I (that is
// the mass distribution gravity sees). Do NOT confuse this with ADDING
// Js*gs*gs^T to D (constraint 1): D never exceeds the input I.
// ---------------------------------------------------------------------------
template <typename S>
class HubContributorT : public ContributorT<S> {
public:
    HubContributorT(bool use_gravity_gradient, double mu)
        : use_gg_(use_gravity_gradient), mu_(mu) {}
    // WP4 aggregate c_B (STRUCTURAL double, NOT a par slot: n_pars()==7 and
    // par_names() unchanged, A2/JIT safe). Verbatim record of add_hub r_BcB_B.
    // C-point integrator KEPT (consumers use r_CN_N, P-WP4.2 0-vs-17 files).
    // Sign binding: r_BN = r_CN + R^T c_B, v_BN = v_CN + R^T (w x c_B)
    // (|rBN-rCN|=0.1 exact). Host FD anchor flips B/C + output map together.
    //
    // MEASURED stock semantics (fire-step 0.18461538 == shifted-arm/I to 8
    // digits, BSK 2.12.0): the input I is treated as ALREADY about the CoM
    // (IHubPntBc_B "Bc" = center of mass), so D carries NO parallel-axis term
    // and there is NO c x (m g) gravity moment and NO B/C transport coupling
    // (translation is formulated about C). The offset enters ONLY through
    // (a) shifted moment arms (r - c_B) x F at the wrench consumers and
    // (b) the B/C output kinematics in writeMsgs. A B-point formulation with
    // B/C blocks + PA + c x mg was implemented first and REJECTED by the
    // bench (idle drift + wrong fire scaling); reverted, do not re-add.
    void setComOffset(double cx, double cy, double cz) {
        c_B_[0] = cx; c_B_[1] = cy; c_B_[2] = cz;
    }
    std::array<double, 3> comOffset() const { return c_B_; }
    // WP4 call site (DOCUMENTED ONLY, do not edit ThrusterContributorT here):
    // ThrusterContributorT::computeForceTorque dynamics arm must read
    // hub.comOffset() and use (r_thr - c_B) x F for torqueExternalPntB_B,
    // while thrusterOutMsgs thrustTorquePntB_B telemetry stays r_thr x F
    // (about B, UNSHIFTED, P-WP4.4). Friction is shaft torque: NO arm.
    // rWB_B fenced: balanced-only negligible unless rotor mass counts
    // toward aggregate c_B (P-WP4.3 decides; until then ignore + guard).

    std::string kind() const override { return "hub"; }
    size_t n_states() const override { return 0; }
    size_t n_inputs() const override { return 0; }
    // mass + the 6 packed inertia entries.  NOTE: there is deliberately NO
    // invI6 slot -- the back-substitution solve computes the inverse
    // symbolically from matrixD, so an inverse-inertia parameter would be
    // declared-but-never-referenced.  (Detected because heyoka requires the
    // pars vector to match the graph exactly, and the two disagreed.)
    size_t n_pars() const override { return 7; }
    std::vector<std::string> state_names() const override { return {}; }
    std::vector<std::string> input_names() const override { return {}; }
    std::vector<std::string> par_names() const override {
        return {"mass", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"};
    }
    std::vector<double> par_defaults() const override {
        return {750.0, 900.0, 800.0, 600.0, 0.0, 0.0, 0.0};
    }

    void updateContributions(double /*integTime*/,
                             BackSubMatricesT<S>& b,
                             const Vector3<S>& g_N,
                             const EvalCtx<S>& ctx) const override {
        const std::vector<S>& P = ctx.pars;
        const size_t o = this->par_offset;
        const S& mass = P[o];
        Matrix3<S> I;
        I(0, 0) = P[o + 1]; I(1, 1) = P[o + 2]; I(2, 2) = P[o + 3];
        I(0, 1) = P[o + 4]; I(1, 0) = P[o + 4];
        I(0, 2) = P[o + 5]; I(2, 0) = P[o + 5];
        I(1, 2) = P[o + 6]; I(2, 1) = P[o + 6];

        // A += m*I3 ; D += I - Dcorr  (accumulate; blocks are zero-initialised).
        // I is ALREADY about the CoM (measured stock semantics -- see the
        // class doc): NO parallel-axis term, NO B/C coupling, NO c x mg
        // moment. Dcorr removes the wheels' spin inertia so D is the rotor-
        // free part. Dcorr is read from the wheels' OWN par slots
        // (parametric: Monte-Carlo runs may disperse Js per run); only the
        // spin axes are structural. GG below keeps the total I.
        Matrix3<S> D = I;
        for (size_t w = 0; w < wheel_axes_.size() && w < wheel_js_off_.size(); ++w) {
            const S& Js = P[wheel_js_off_[w]];
            const S j00 = Js * make_scalar<S>(wheel_axes_[w][0] * wheel_axes_[w][0]);
            const S j01 = Js * make_scalar<S>(wheel_axes_[w][0] * wheel_axes_[w][1]);
            const S j02 = Js * make_scalar<S>(wheel_axes_[w][0] * wheel_axes_[w][2]);
            const S j11 = Js * make_scalar<S>(wheel_axes_[w][1] * wheel_axes_[w][1]);
            const S j12 = Js * make_scalar<S>(wheel_axes_[w][1] * wheel_axes_[w][2]);
            const S j22 = Js * make_scalar<S>(wheel_axes_[w][2] * wheel_axes_[w][2]);
            D(0, 0) = D(0, 0) - j00;
            D(0, 1) = D(0, 1) - j01; D(1, 0) = D(1, 0) - j01;
            D(0, 2) = D(0, 2) - j02; D(2, 0) = D(2, 0) - j02;
            D(1, 1) = D(1, 1) - j11;
            D(1, 2) = D(1, 2) - j12; D(2, 1) = D(2, 1) - j12;
            D(2, 2) = D(2, 2) - j22;
        }
        for (int i = 0; i < 3; ++i) {
            b.A(i, i) = b.A(i, i) + mass;
            for (int j = 0; j < 3; ++j) b.D(i, j) = b.D(i, j) + D(i, j);
        }
        // vecTrans += m * g_N  (gravity force in N frame, at CoM -- unchanged).
        for (int i = 0; i < 3; ++i) b.vecTrans[i] = b.vecTrans[i] + mass * g_N[i];

        const Vector3<S> omega = ctx.omega();
        // vecRot += -(omega x (I*omega))
        const Vector3<S> Iw = matvec3(I, omega);
        const Vector3<S> wxIw = cross3(omega, Iw);
        for (int i = 0; i < 3; ++i) b.vecRot[i] = b.vecRot[i] - wxIw[i];

        if (use_gg_) {
            // Gravity-gradient torque with full inertia (matches
            // heyoka_attitude.py): Tgg = k * (rB x (I*rB)), k = 3*mu/|r|^5.
            const Vector3<S> rN = ctx.r();
            const MRPt<S> mrp(ctx.sigma());
            const Matrix3<S> R = mrp.toRotationMatrix();
            const Vector3<S> rB = matvec3(R, rN);
            const Vector3<S> IrB = matvec3(I, rB);
            const S r2 = dot3(rN, rN);
            const S rn = sqrt_scalar(r2);
            const S invR5 = make_scalar<S>(1.0) / (r2 * r2 * rn);
            const S k = make_scalar<S>(3.0 * mu_) * invR5;
            const Vector3<S> tgg = scale_vec3(cross3(rB, IrB), k);
            for (int i = 0; i < 3; ++i) b.vecRot[i] = b.vecRot[i] + tgg[i];
        }
    }

    std::vector<S> state_derivatives(const Vector3<S>&, const Vector3<S>&,
                                     const EvalCtx<S>&) const override {
        return {};
    }

    // Wheels' spin axes (structural) + Js par-slot offsets (parametric),
    // assigned by OmniSystem at refresh so Dcorr follows per-run dispersion.
    // Empty when no wheels are registered.
    std::vector<Vector3<double>> wheel_axes_;
    std::vector<size_t> wheel_js_off_;

private:
    bool use_gg_;
    double mu_;  // baked constant (matches native_rw; MC does not disperse mu)
    // WP4: aggregate c_B record (structural; read by the thruster arm shift
    // wiring in OmniSystem::refresh and the writeMsgs B/C split). The hub
    // blocks themselves intentionally ignore it (measured stock semantics).
    std::array<double, 3> c_B_{0.0, 0.0, 0.0};
};

// ---------------------------------------------------------------------------
// Reaction wheel: balanced rotor about a body-fixed spin axis gs.
// Structural: gs (unit axis, double), wheel index for naming.
// Parameters: Js (1). Inputs: u (1, motor torque, already saturated by caller)
//   + tauF (1, WP2 friction torque on the wheel, module-side signed, verbatim).
// States: W (1, wheel speed relative to hub).
//
// Constraint (1): D is untouched. The ONLY rotational contribution is
//   vecRot += -(omega x h) - u*gs - s*tauF*gs,  h = Js*(W + omega.gs)*gs,
// with s = frictionHubSign_ (default +1, stock react-on-hub per F4).
// Spin: Wdot = (u + tauF - Js*(gs.omegaDot))/Js  (coupled, stock ordering).
// Spin: Wdot = u/Js  (matches heyoka_rw.py / native_rw / stock parity).
// ---------------------------------------------------------------------------
template <typename S>
class ReactionWheelContributorT : public ContributorT<S> {
public:
    ReactionWheelContributorT(Vector3<double> gs, int index)
        : gs_(gs), index_(index) {}

    std::string kind() const override { return "rw"; }
    size_t n_states() const override { return 1; }
    size_t n_inputs() const override { return 2; }
    size_t n_pars() const override { return 1; }  // Js
    std::vector<std::string> state_names() const override {
        return {"W" + std::to_string(index_ + 1)};
    }
    std::vector<std::string> input_names() const override {
        return {"u" + std::to_string(index_ + 1), "tauF" + std::to_string(index_ + 1)};
    }
    std::vector<std::string> par_names() const override {
        return {"Js" + std::to_string(index_ + 1)};
    }
    std::vector<double> par_defaults() const override {
        return {0.07957747154594767};  // HR16 rotor inertia
    }

    void updateContributions(double /*integTime*/,
                             BackSubMatricesT<S>& b,
                             const Vector3<S>& /*g_N*/,
                             const EvalCtx<S>& ctx) const override {
        const S& Js = ctx.pars[this->par_offset];
        const S& u = ctx.pars[this->input_offset];
        const S& tauF = ctx.pars[this->input_offset + 1];
        const S& W = ctx.states[this->state_offset];
        const Vector3<S> omega = ctx.omega();
        const Vector3<S> gs = typed_axis();

        // h = Js * W * gs -- RELATIVE rotor momentum (W is the relative spin
        // rate). Stock parity (upstream reactionWheelStateEffector.cpp):
        //   vecRot -= gs*(u+f) + Js*W*(omega x gs)  ==  -omega x h - u*gs.
        // The ABSOLUTE form h = Js*(W + omega.gs)*gs DOUBLE-COUNTS: the
        // Js*(omega.gs)*gs momentum already lives in the total inertia the
        // hub block inverts as D = I - sum Js*gs*gs^T. The double count
        // cancels exactly for 3 orthonormal wheels (extra = -Js*omega x
        // omega = 0), which is why every 3-wheel gate passed while 1-2
        // wheels drifted ~1e-5 -- found by the swap 1-vs-3-wheel parity
        // matrix, fixed here. (DESIGN constraint 1 refined accordingly.)
        const Vector3<S> h = scale_vec3(gs, Js * W);
        // vecRot += -(omega x h) - u*gs - s*tauF*gs  (A3: + - * only; tauF
        // read verbatim like u; s = frictionHubSign_, default +1 = exact
        // stock react-on-hub per F4; NO tanh/min/max in the graph path).
        const Vector3<S> wxh = cross3(omega, h);
        const Vector3<S> ugu = scale_vec3(gs, u);
        const Vector3<S> fgu = scale_vec3(gs, tauF * make_scalar<S>(frictionHubSign_));
        for (int i = 0; i < 3; ++i) b.vecRot[i] = b.vecRot[i] - wxh[i] - ugu[i] - fgu[i];
        // NOTE: D is deliberately untouched (constraint 1).
    }

    std::vector<S> state_derivatives(const Vector3<S>&, const Vector3<S>& omegaDot,
                                     const EvalCtx<S>& ctx) const override {
        // COUPLED spin (stock parity): the motor torque balances the rate of
        // change of the rotor's ABSOLUTE momentum,
        //   Js*dW = u - Js*(gs.omegaDot),
        // evaluated sequentially after the hub solve provides omegaDot
        // (exactly stock's ordering: spacecraft solves back-substitution,
        // then effectors update states). The decoupled form (u/Js) drops the
        // second term; hub trajectories then still match (same Schur hub
        // equation) but wheel speeds drift by int(gs.omegaDot) -- invisible
        // on heavy buses, degrees on momentum-stiff ones. Verified: stock
        // wheel-speed gain == (u/Js)*I/(I-Js) to 4 digits across inertias.
        const S& Js = ctx.pars[this->par_offset];
        const S& u = ctx.pars[this->input_offset];
        const S& tauF = ctx.pars[this->input_offset + 1];
        const Vector3<S> gs = typed_axis();
        return {(u + tauF - Js * dot3(gs, omegaDot)) / Js};
    }

private:
    Vector3<S> typed_axis() const {
        Vector3<S> g;
        g[0] = make_scalar<S>(gs_[0]);
        g[1] = make_scalar<S>(gs_[1]);
        g[2] = make_scalar<S>(gs_[2]);
        return g;
    }

    Vector3<double> gs_;
    int index_;
 public:
    // WP2 hub-reaction sign hook (F4): +1 = friction reacts on hub exactly
    // like motor torque (stock parity default); module-side documented probe.
    double frictionHubSign_ = 1.0;
};

// ---------------------------------------------------------------------------
// Central gravity: point-mass field. Produces g_N; runs FIRST (A5).
// No states, no inputs, no parameters (mu is a baked structural constant).
// ---------------------------------------------------------------------------
template <typename S>
class CentralGravityContributorT : public ContributorT<S> {
public:
    explicit CentralGravityContributorT(double mu) : mu_(mu) {}

    std::string kind() const override { return "gravity"; }
    size_t n_states() const override { return 0; }
    size_t n_inputs() const override { return 0; }
    size_t n_pars() const override { return 0; }
    std::vector<std::string> state_names() const override { return {}; }
    std::vector<std::string> input_names() const override { return {}; }
    std::vector<std::string> par_names() const override { return {}; }
    std::vector<double> par_defaults() const override { return {}; }

    Vector3<S> field(const Vector3<S>& rN) const {
        const S r2 = dot3(rN, rN);
        const S rn = sqrt_scalar(r2);
        const S invR3 = make_scalar<S>(1.0) / (r2 * rn);
        return scale_vec3(rN, make_scalar<S>(-mu_) * invR3);
    }

    void updateContributions(double, BackSubMatricesT<S>&, const Vector3<S>&,
                             const EvalCtx<S>&) const override {
        // Nothing: the field is produced via field() BEFORE the block pass.
    }

    std::vector<S> state_derivatives(const Vector3<S>&, const Vector3<S>&,
                                     const EvalCtx<S>&) const override {
        return {};
    }

private:
    double mu_;
};

// ---------------------------------------------------------------------------
// Thruster: DynamicEffector-style contributor (stock's second channel).
//
// N fixed thrusters, each with a body-frame position r_i, thrust direction
// d_i and max thrust Tmax_i.  Contributes NO state (thrusters are not
// stateEffectors) and NO blocks -- only the ForceTorqueT channel:
//
//   F_B = sum_i q_i * Tmax_i * d_i
//   T_B = sum_i r_i x F_i              (about CoM)
//   F_N = R^T F_B                       (rotate to inertial)
//
// where q_i in {0, 1/6, 5/6, 1} is the MEASURED effector staircase.
//
// The quantizer is emphatically NOT in the symbolic graph (design A3): it
// needs min/max comparisons.  It runs at step time, in fillInputs(), on the
// plain double message payload.  The symbolic graph sees q_i as par[i]
// verbatim, so the graph stays clean and the JIT stays fast.
//
// Configuration is STRUCTURAL (baked as doubles, like native_rw's wheel
// axes): r, d, Tmax and the thruster count.  Only the duty levels q_i are
// parameters.  Dispersing thruster geometry would mean re-registering.
// ---------------------------------------------------------------------------
template <typename S>
class ThrusterContributorT : public ContributorT<S> {
public:
    ThrusterContributorT(std::vector<std::array<double, 3>> dirs,
                         std::vector<std::array<double, 3>> pos,
                         std::vector<double> tmax)
        : dirs_(std::move(dirs)), pos_(std::move(pos)), tmax_(std::move(tmax)) {
        n_ = dirs_.size();
    }

    std::string kind() const override { return "thruster"; }
    size_t n_states() const override { return 0; }
    // The duty levels live in the INPUT block; the thruster has NO own
    // parameters of its own.  (Declaring both would double-count the q slots
    // and produce a par vector longer than the symbolic graph.)
    // WP3b: one trailing "swirlScale" input slot [N-m] carries the global
    // MaxSwirlTorque (module-written per tick, A2 by name; default 0.0, so
    // off by default). A dedicated slot is required because the swirl torque
    // points along the thrust axis d, which r x F folding cannot produce.
    size_t n_inputs() const override { return n_ + 1; }
    size_t n_pars() const override { return 0; }
    std::vector<std::string> state_names() const override { return {}; }
    std::vector<std::string> input_names() const override {
        std::vector<std::string> o;
        for (size_t i = 0; i < n_; ++i) o.push_back("q" + std::to_string(i));
        o.push_back("swirlScale");
        return o;
    }
    std::vector<std::string> par_names() const override { return {}; }
    std::vector<double> par_defaults() const override { return {}; }
    bool has_force_torque() const override { return true; }

    void updateContributions(double, BackSubMatricesT<S>&, const Vector3<S>&,
                             const EvalCtx<S>&) const override {
        // Deliberately empty: thrusters never touch the back-substitution
        // blocks.  They act through the ForceTorqueT channel only.
    }

    std::vector<S> state_derivatives(const Vector3<S>&, const Vector3<S>&,
                                     const EvalCtx<S>&) const override {
        return {};
    }

    void computeForceTorque(double, double, ForceTorqueT<S>& ft,
                            const EvalCtx<S>& ctx) const override {
        // F_B, T_B from the duty slots; F_N = R^T F_B.
        const std::vector<S>& P = ctx.pars;
        Vector3<S> fB = zero_vec3<S>();
        Vector3<S> tB = zero_vec3<S>();
        for (size_t i = 0; i < n_; ++i) {
            const S q = P[this->input_offset + i];
            if constexpr (!is_expression_v<S>) {
                if (q == static_cast<S>(0)) continue;  // skip idle thrusters
            }
            Vector3<S> d;
            d[0] = make_scalar<S>(dirs_[i][0]);
            d[1] = make_scalar<S>(dirs_[i][1]);
            d[2] = make_scalar<S>(dirs_[i][2]);
            const S Fmag = q * make_scalar<S>(tmax_[i]);
            const Vector3<S> Fi = scale_vec3(d, Fmag);
            fB = vecadd3(fB, Fi);
            Vector3<S> r;
            // WP4 measured stock semantics (fire-step bench): dynamics moment
            // arms shift by -c_B (about CoM); TELEMETRY thrustTorquePntB_B
            // stays r_thr x F about B (computed module-side, untouched here).
            r[0] = make_scalar<S>(pos_[i][0] - comOff_[0]);
            r[1] = make_scalar<S>(pos_[i][1] - comOff_[1]);
            r[2] = make_scalar<S>(pos_[i][2] - comOff_[2]);
            tB = vecadd3(tB, cross3(r, Fi));
            // WP3b swirl (M6): axis torque about the thrust direction, scaled
            // by the achieved duty in the q slot (A3: pure arithmetic only).
            const S swirl = P[this->input_offset + n_] * q;
            tB = vecadd3(tB, scale_vec3(d, swirl));
        }
        // F_N = R^T F_B  (rotate body -> inertial)
        const MRPt<S> mrp(ctx.sigma());
        const Matrix3<S> R = mrp.toRotationMatrix();
        Matrix3<S> Rt;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) Rt(i, j) = R(j, i);
        ft.forceExternal_B = fB;
        ft.torqueExternalPntB_B = tB;
        ft.forceExternal_N = matvec3(Rt, fB);
    }

    // Contributor-owned message conversion (DESIGN Q2).  `raw` is the plain
    // duty/on-time payload the module read from the Basilisk message; convert
    // it into the quantized staircase levels and write them into the input
    // slots.  `dt` is needed because duty = onTime / dt.
    void fillInputs(double dt, const double* raw, size_t n_raw,
                    std::vector<S>& pars) const override {
        (void)dt; (void)n_raw;
        // WP3b min-pulse extension point: gating (suppress/merge) is owned by
        // the module staging layer (stageThrusters/openT block) BEFORE the
        // quantizer, so levels arriving here are already gated. This stays a
        // verbatim ZOH copy of the N duty slots; the trailing "swirlScale"
        // slot is module-written and never part of raw.
        if (raw == nullptr) return;
        for (size_t i = 0; i < n_; ++i) {
            pars[this->input_offset + i] = make_scalar<S>(raw[i]);
        }
    }

    // WP3b module-side hooks (doubles only; never in the graph, A3).
    // Descending-order polynomial (stock thrBlowDownCoeff convention); empty
    // means no model => 1.0. Fuel-mass depletion stays OUT (M6, labeled).
    static double blowdownPoly(const std::vector<double>& coeffs, double x) {
        if (coeffs.empty()) return 1.0;
        double y = 0.0;
        for (double c : coeffs) y = y * x + c;
        return y;
    }
    // Min-pulse suppress gate (M3 histogram): sub-MinOnTime requests drop to
    // zero; firing requests pass through. Merge/extend memory lives in the
    // staging layer's MinPulseState (documented, default suppress-only).
    static double minPulseGate(double onTimeCmd, double q, double minOnTime) {
        if (minOnTime > 0.0 && onTimeCmd < minOnTime) return 0.0;
        return q;
    }

    // The measured staircase, applied at step time (never in the graph).
    static double quantize(double duty) {
        if (duty <= 0.0) return 0.0;
        if (duty < 0.5) return 1.0 / 6.0;
        if (duty < 1.0) return 5.0 / 6.0;
        return 1.0;
    }

    // Effective valve-open duration [s] for a commanded on-time within a step
    // of dt: the staircase maps commanded duty to achieved duty, i.e. the
    // valve stays open for the QUANTIZED duration at full thrust (verified
    // against stock to all probed edges, including exact 0.5/1.0 and
    // over/negative commands). The module switches arcs on these times.
    static double onTime(double onTimeCmd, double dt) {
        if (!(dt > 0.0)) return 0.0;
        return quantize(onTimeCmd / dt) * dt;
    }

    size_t nThrusters() const { return n_; }

    // WP4: aggregate-CoM offset for the dynamics arm shift ((r - c) x F).
    // Wired by OmniSystem::refresh from hub_.rBcB (mirror of the hub Dcorr
    // wiring); default zeros = pre-WP4 behavior. Structural (doubles).
    void set_com_offset(double cx, double cy, double cz) {
        comOff_[0] = cx; comOff_[1] = cy; comOff_[2] = cz;
    }

private:
    std::vector<std::array<double, 3>> dirs_;
    std::vector<std::array<double, 3>> pos_;
    std::vector<double> tmax_;
    size_t n_ = 0;
    std::array<double, 3> comOff_{0.0, 0.0, 0.0};
};

}  // namespace omni

#endif  // OMNI_CONTRIBUTORS_HPP
