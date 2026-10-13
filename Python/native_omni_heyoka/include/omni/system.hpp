// ISC License - see native_rw/LICENSE.
//
// OmniSystem: runtime-composed spacecraft dynamics.
//
// Registration records are the single source of truth. From them the system
// derives (a) the state/parameter layout (A1/A2), (b) a numeric contributor
// set (S = double) for cross-checks, and (c) a symbolic contributor set
// (S = heyoka::expression) from which the EOM falls out as an expression
// graph -- the Option-A method.
//
// Parameter layout (A2): [inputs in registration order] then [own parameters
// in registration order]. Hub owns mass + I6 (7); the inverse is derived
// symbolically from D via inv3(), so no invI6 slots exist (heyoka rejects
// declared-but-unreferenced pars). Wheels add u+tauF inputs (2/wheel, WP2)
// + Js (1/wheel); thrusters add q+swirlScale inputs + masked cB pars (WP3b/8);
// env adds its input block (WP5). E.g. hub + 3 wheels = 6 + 7 + 3 = 16 pars;
// hub + 12 thrusters = 13 + 7 = 20 default, +3 cB = 23 full-mask; hub-only = 7.
#ifndef OMNI_SYSTEM_HPP
#define OMNI_SYSTEM_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "omni/contributors.hpp"
#include "omni/environment.hpp"
#include "omni/force_torque.hpp"

namespace omni {

class OmniSystem {
public:
    OmniSystem() : u_max_(0.2), use_gg_(true), mu_(3.986004418e14), dirty_(true) {}

    // --- registration ----------------------------------------------------
    void add_hub(double mass, const std::vector<double>& I9,
                 const std::vector<double>& r_BcB_B) {
        if (has_hub_) throw std::invalid_argument("OmniSystem: only one hub allowed");
        if (!(mass > 0.0)) throw std::invalid_argument("OmniSystem: mass must be positive");
        if (I9.size() != 9) throw std::invalid_argument("OmniSystem: I9 needs 9 entries");
        if (r_BcB_B.size() != 3) throw std::invalid_argument("OmniSystem: r_BcB_B needs 3 entries");
        // WP4: aggregate c_B recorded (C-point integrator kept). WP0 offsetOn_
        // guard lives module-side (spacecraft.hpp); system stores verbatim.
        // Symmetry + nonsingularity check (matches native_rw's Reset error).
        Eigen::Matrix3d I;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) I(i, j) = I9[size_t(i) * 3 + size_t(j)];
        if ((I - I.transpose()).cwiseAbs().maxCoeff() > 1e-9)
            throw std::invalid_argument("OmniSystem: inertia must be symmetric");
        if (std::abs(I.determinant()) < 1e-30)
            throw std::invalid_argument("OmniSystem: inertia matrix is singular");
        hub_.has = true;
        hub_.mass = mass;
        hub_.I9 = I9;
        hub_.rBcB = r_BcB_B;
        has_hub_ = true;
        dirty_ = true;
    }

    void add_reaction_wheel(const std::vector<double>& gs, double Js, double /*Jt_ignored*/) {
        // WP25-B2.2: Jt (transverse rotor inertia) is INTENTIONALLY unused --
        // validated, not an oversight. Constraint 1 (contributors.hpp): a
        // balanced rotor contributes NO Js*gs*gs^T to matrixD and the spin
        // equation needs only Js; the transverse part lives in the total
        // input inertia the hub block inverts. Telemetry echoes Jt = 0.5*Js
        // (writeMsgs, labeled). Callers pass 0.0; any value is accepted and
        // ignored by design (it never enters D, vecRot, or the spin).
        if (gs.size() != 3) throw std::invalid_argument("OmniSystem: gs needs 3 entries");
        const double n = std::sqrt(gs[0] * gs[0] + gs[1] * gs[1] + gs[2] * gs[2]);
        if (!(n > 0.0)) throw std::invalid_argument("OmniSystem: gs must be nonzero");
        if (!(Js > 0.0)) throw std::invalid_argument("OmniSystem: Js must be positive");
        // Normalise the axis (structural constant).
        wheels_.push_back({{gs[0] / n, gs[1] / n, gs[2] / n}, Js});
        order_.push_back('w');  // registration order drives layout (A2)
        dirty_ = true;
    }

    // Thruster cluster: DynamicEffector channel, no state, no blocks.
    void add_thrusters(std::vector<std::array<double, 3>> dirs,
                       std::vector<std::array<double, 3>> pos,
                       std::vector<double> tmax) {
        if (dirs.size() != pos.size() || dirs.size() != tmax.size())
            throw std::invalid_argument("OmniSystem: thruster dirs/pos/tmax size mismatch");
        for (size_t i = 0; i < dirs.size(); ++i) {
            const double n = std::sqrt(dirs[i][0] * dirs[i][0] + dirs[i][1] * dirs[i][1]
                                       + dirs[i][2] * dirs[i][2]);
            if (!(n > 0.0))
                throw std::invalid_argument("OmniSystem: thruster direction must be nonzero");
            if (!(tmax[i] > 0.0))
                throw std::invalid_argument("OmniSystem: thruster Tmax must be positive");
            dirs[i][0] /= n; dirs[i][1] /= n; dirs[i][2] /= n;
        }
        thrDirs_ = std::move(dirs);
        thrPos_ = std::move(pos);
        thrTmax_ = std::move(tmax);
        has_thrusters_ = true;
        // add_thrusters overwrites the cluster: re-register at the end.
        order_.erase(std::remove(order_.begin(), order_.end(), 't'), order_.end());
        order_.push_back('t');
        dirty_ = true;
    }

    void set_gravity(double mu) {
        if (!(mu > 0.0)) throw std::invalid_argument("OmniSystem: mu must be positive");
        mu_ = mu;
        has_gravity_ = true;
        dirty_ = true;
    }

    // --- Phase-2 swap: fuel mass (single tank, always LAST in layout) --------
    // No records needed: depletion flows via state/inputs, and hub mass/I
    // feedback is applied module-side on the hub par slots. Appended after
    // the order_ loop so fuel states are always last (documented layout).
    void add_fuel_tank() {
        if (has_fuel_) throw std::invalid_argument("OmniSystem: only one fuel tank allowed");
        has_fuel_ = true;
        dirty_ = true;
    }

    // --- WP5 environment (single 'e' cluster) --------------------------------
    // Every call below sets env_.has (lazy single registration in refresh()).
    void addEnvironmentBody(double mu, double ox, double oy, double oz) {
        if (!(mu > 0.0)) throw std::invalid_argument("OmniSystem: env body mu must be positive");
        env_.bodyMu.push_back(mu);
        env_.bodyOff0.push_back({ox, oy, oz});
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }
    void setJ2(double J2) {
        if (!std::isfinite(J2) || J2 < 0.0)
            throw std::invalid_argument("OmniSystem: J2 must be finite non-negative");
        env_.J2 = J2;
        env_.j2On = true;  // WP0: off unless explicitly set (mission USE_J2=False)
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }
    void setDragParams(double CdA, double cx, double cy, double cz) {
        if (!(CdA >= 0.0) || !std::isfinite(CdA))
            throw std::invalid_argument("OmniSystem: CdA must be finite non-negative");
        env_.dragOn = true;
        env_.CdA = CdA;
        env_.comOff = {cx, cy, cz};
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }
    void addSRPFacet(double area, double nx, double ny, double nz,
                     double copx, double copy, double copz,
                     double c_diffuse, double c_specular) {
        const double n = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (!(area > 0.0)) throw std::invalid_argument("OmniSystem: facet area must be positive");
        if (!(n > 0.0)) throw std::invalid_argument("OmniSystem: facet normal must be nonzero");
        EnvRec::Facet f;
        f.area = area;
        f.nHat = {nx / n, ny / n, nz / n};
        f.rCop = {copx, copy, copz};
        f.cd = c_diffuse; f.cs = c_specular;
        env_.facets.push_back(f);
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }
    void setSRPCannonball(double area, double cr) {
        if (!(area > 0.0)) throw std::invalid_argument("OmniSystem: cannonball area must be positive");
        if (!std::isfinite(cr) || cr < 0.0)
            throw std::invalid_argument("OmniSystem: cr must be finite non-negative");
        env_.cannonballOn = true;
        env_.cbArea = area; env_.cbCr = cr;
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }
    void setSunState(double px, double py, double pz) {
        env_.sunState = {px, py, pz};
        env_.hasSun = true;
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }
    void setMagneticMoment(double mx, double my, double mz) {
        env_.magOn = true;
        env_.magM = {mx, my, mz};
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }
    void setMagneticDipole(double g10, double g11, double h11) {
        env_.g10 = g10; env_.g11 = g11; env_.h11 = h11;  // centered-dipole preset, recorded
        env_.has = true;
        ensure_env_order();
        dirty_ = true;
    }

    void set_use_gravity_gradient(bool v) { use_gg_ = v; dirty_ = true; }
    // WP25-B2.2: u_max_ is a RECORD-ONLY default, not a dead accessor. It is
    // read by the standalone-OmniSystem Python harness (driver.u_max_of
    // default, presets system.set_u_max) as the caller-side saturation bound.
    // It is INTENTIONALLY unread by the module/vec graphs: saturation is a
    // caller-side pre-write clamp (A3 -- min/max/if degrade the Taylor
    // graph), so wiring it into the graph would be the bug, and deleting it
    // would break driver/presets/mock. The SysModel path clamps per wheel in
    // UpdateState from wheelUMax (setUMax, post-registration effective); the
    // vec path reads back stock-saturated u_current.
    void set_u_max(double v) {
        if (!(v >= 0.0)) throw std::invalid_argument("OmniSystem: u_max must be non-negative");
        u_max_ = v;
    }

    void set_default_state(const std::vector<double>& s) {
        refresh();
        if (s.size() != n_states_)
            throw std::invalid_argument("OmniSystem: default state has wrong size");
        default_state_ = s;
        has_default_state_ = true;
    }

    // --- layout (A1/A2) ----------------------------------------------------
    size_t n_states() const { refresh(); return n_states_; }
    size_t n_inputs() const { refresh(); return n_inputs_; }
    size_t n_pars() const { refresh(); return n_pars_; }
    std::vector<std::string> state_names() const { refresh(); return state_names_; }
    std::vector<std::string> input_names() const { refresh(); return input_names_; }
    std::vector<std::string> par_names() const { refresh(); return par_names_; }
    std::vector<double> default_pars() const { refresh(); return par_defaults_; }
    std::vector<double> default_state() const {
        refresh();
        if (!has_default_state_) throw std::runtime_error("OmniSystem: no default state set");
        return default_state_;
    }
    // Contributor-owned input conversion for the thruster channel (Q2).
    // `levels` are already-quantized duty levels; the contributor writes them
    // into its own parameter slots.
    void fill_thruster_inputs(double dt, const double* levels, size_t n,
                              std::vector<double>& pars) const {
        refresh();
        for (const auto& c : num_) {
            if (c->has_force_torque()) {
                c->fillInputs(dt, levels, n, pars);
                return;
            }
        }
    }

    // Effective valve-open durations for one step (one per thruster): the
    // contributor-owned duty -> quantized on-time mapping the module's arc
    // loop switches on. Empty when no thruster contributor is registered.
    std::vector<double> thruster_on_times(double dt, const double* onTimeCmd,
                                          size_t n) const {
        refresh();
        std::vector<double> out;
        for (const auto& c : num_) {
            if (c->has_force_torque()) {
                const auto* t =
                    static_cast<const ThrusterContributorT<double>*>(c.get());
                out.reserve(t->nThrusters());
                for (size_t i = 0; i < t->nThrusters() && i < n; ++i)
                    out.push_back(ThrusterContributorT<double>::onTime(onTimeCmd[i], dt));
                return out;
            }
        }
        return out;
    }

    double u_max() const { return u_max_; }

    // Input-slot offsets of each wheel's motor-torque slot, in wheel
    // registration order. The module must write motor torques through these
    // (the RW input block is NOT guaranteed to start at par 0 once a
    // thruster cluster is registered first).
    std::vector<size_t> wheel_input_offsets() const {
        refresh();
        std::vector<size_t> out;
        for (const auto& c : num_) {
            if (c->kind() == "rw") {
                if (c->n_inputs() != 2)
                    throw std::runtime_error("OmniSystem: wheel input block changed shape");
                out.push_back(c->input_offset);
            }
        }
        return out;
    }

    // WP2: second-slot (tauF) offsets, one per wheel in registration order =
    // wheel offset + 1. Mirrors wheel_input_offsets(); the module writes the
    // module-side friction (+ JitterSimple noise) here. No refresh-par-pass
    // change needed: the input-block pass and par_names_ derive the 2nd slot
    // from ReactionWheelContributorT::n_inputs/input_names (A2).
    std::vector<size_t> friction_input_offsets() const {
        refresh();
        std::vector<size_t> out;
        for (const auto& c : num_) {
            if (c->kind() == "rw") {
                if (c->n_inputs() != 2)
                    throw std::runtime_error("OmniSystem: wheel input block changed shape");
                out.push_back(c->input_offset + 1);
            }
        }
        return out;
    }

    // Env slot bases in EnvContributorT order (body 3*Nb, drag 6, srp 6,
    // mag 3). has=false when no env cluster is registered. The module writes
    // per-tick (/per-arc) wrenches through these (dynamic layout, A2).
    struct EnvSlots {
        bool has = false;
        size_t body = 0, drag = 0, srp = 0, mag = 0, nBodies = 0;
    };
    EnvSlots env_slot_offsets() const {
        refresh();
        EnvSlots out;
        for (const auto& c : num_) {
            if (c->kind() == "env") {
                const auto* e = static_cast<const EnvContributorT<double>*>(c.get());
                out.has = true;
                out.body = e->bodyOffset();
                out.drag = e->dragOffset();
                out.srp = e->srpOffset();
                out.mag = e->magOffset();
                out.nBodies = e->nBodies();
                return out;
            }
        }
        return out;
    }

    size_t par_index(const std::string& name) const {
        refresh();
        for (size_t i = 0; i < par_names_.size(); ++i)
            if (par_names_[i] == name) return i;
        throw std::invalid_argument("OmniSystem: unknown par '" + name + "'");
    }

    // Non-throwing par probe (masked-out thruster c-axes have no slot).
    bool has_par(const std::string& name) const {
        refresh();
        for (const auto& nm : par_names_)
            if (nm == name) return true;
        return false;
    }

    // --- numeric evaluation (same contributor code, S = double) ------------
    // Returns [rDDot(3), omegaDot(3), Wdot...].
    std::vector<double> accelerations(const std::vector<double>& state,
                                      const std::vector<double>& pars) const {
        refresh();
        check_sizes(state, pars);
        const EvalCtx<double> ctx{state, pars};
        Vector3<double> gN = gravity_field(ctx.r());
        for (const auto& c : num_) {
            if (c->kind() == "env") {
                const auto* e = static_cast<const EnvContributorT<double>*>(c.get());
                gN = vecadd3(gN, e->gravityExtra(ctx.r(), ctx));
            }
        }
        BackSubMatricesT<double> b;
        for (const auto& c : num_) c->updateContributions(0.0, b, gN, ctx);

        // DynamicEffector channel: collect, then fold into the solve (Q1).
        // Two-pass: non-env first, env LAST. The thruster contributor
        // overwrites ft (parallel ownership, untouchable); env accumulates.
        ForceTorqueT<double> ft;
        for (const auto& c : num_) {
            if (c->has_force_torque() && c->kind() != "env")
                c->computeForceTorque(0.0, 0.0, ft, ctx);
        }
        for (const auto& c : num_) {
            if (c->has_force_torque() && c->kind() == "env")
                c->computeForceTorque(0.0, 0.0, ft, ctx);
        }

        Vector3<double> rDDot, wDot;
        solve_backsub(b, rDDot, wDot, &ft);
        std::vector<double> out;
        out.insert(out.end(), {rDDot[0], rDDot[1], rDDot[2], wDot[0], wDot[1], wDot[2]});
        const Vector3<double> om = ctx.omega();
        for (const auto& c : num_) {
            auto d = c->state_derivatives(om, wDot, ctx);
            out.insert(out.end(), d.begin(), d.end());
        }
        return out;
    }

    // Full state derivative [v, rDDot, sdot, wDot, Wdots...].
    std::vector<double> state_derivative(const std::vector<double>& state,
                                         const std::vector<double>& pars) const {
        refresh();
        check_sizes(state, pars);
        const std::vector<double> acc = accelerations(state, pars);
        std::vector<double> out;
        out.reserve(n_states_);
        out.insert(out.end(), {state[3], state[4], state[5]});       // rdot = v
        out.insert(out.end(), {acc[0], acc[1], acc[2]});              // vdot
        Vector3<double> s, w;
        s[0] = state[6]; s[1] = state[7]; s[2] = state[8];
        w[0] = state[9]; w[1] = state[10]; w[2] = state[11];
        const Vector3<double> sd = mrp_kinematics(s, w);
        out.insert(out.end(), {sd[0], sd[1], sd[2]});                // sdot
        out.insert(out.end(), {acc[3], acc[4], acc[5]});              // wdot
        out.insert(out.end(), acc.begin() + 6, acc.end());            // Wdots
        return out;
    }

    // --- symbolic build (the whole point; S = heyoka::expression) ----------
    std::vector<std::pair<heyoka::expression, heyoka::expression>> build_symbolic() const {
        refresh();
        std::vector<heyoka::expression> vars;
        vars.reserve(n_states_);
        for (const auto& nm : state_names_) vars.emplace_back(heyoka::variable{nm});
        std::vector<heyoka::expression> par;
        par.reserve(n_pars_);
        for (size_t i = 0; i < n_pars_; ++i) par.push_back(heyoka::par[i]);
        return build_symbolic_from_ctx(vars, par);
    }

    // heyoka requires the passed par count to EXACTLY match the referenced
    // set. Thruster CoM axes are masked at the source (ThrusterContributorT
    // only registers axes its geometry couples), so c-slots never dangle.
    // Any OTHER geometry-degenerate trailing slot is suffix-dropped here;
    // anything else (interior gap, the Update-4 invI6 class) throws LOUDLY
    // instead of silently misbuilding.
    // WP25-C2 M5-rest ordering constraint: the c-subset is always trailing
    // own-pars (env/fuel contribute n_pars==0), so the mask is suffix-safe by
    // construction; heyoka prints par[i] as p<i> (pinned by the printer test
    // in tests/test_wp0_guards.py) — if the printer ever changes, the parse
    // below breaks loudly (seen stays empty), never silently.
    // Returns the kept count k; callers pass pars[0..k) and must bound all
    // per-tick get_pars_data() pushes by k (heap safety!).
    static size_t checked_par_count(
        const std::vector<std::pair<heyoka::expression, heyoka::expression>>& sysrows,
        size_t n_pars) {
        std::set<size_t> seen;
        for (const auto& pr : sysrows) {
            for (const auto& p : heyoka::get_params(pr.second)) {
                std::ostringstream os;
                os << p;
                const std::string s = os.str();
                if (s.size() < 2 || s[0] != 'p') continue;
                size_t v = 0;
                bool ok = true;
                for (size_t i = 1; i < s.size(); ++i) {
                    if (s[i] < '0' || s[i] > '9') { ok = false; break; }
                    v = v * 10 + static_cast<size_t>(s[i] - '0');
                }
                if (ok) seen.insert(v);
            }
        }
        size_t k = 0;
        while (k < n_pars && seen.count(k)) ++k;
        if (seen.size() != k)
            throw std::runtime_error(
                "OmniSystem: non-suffix par gap in symbolic graph "
                "(structurally dead par slot; remove it, Update-4 style)");
        return k;
    }

    // String marshalling for the Python shim: RHS strings, LHS = state_names().
    // Python reparses each with hy.expression(s) -- verified exact round-trip.
    std::vector<std::string> build_symbolic_rhs_strings() const {
        const auto sys = build_symbolic();
        std::vector<std::string> out;
        out.reserve(sys.size());
        for (const auto& pr : sys) {
            std::ostringstream os;
            os << pr.second;
            out.push_back(os.str());
        }
        return out;
    }

private:
    // -- registration records ------------------------------------------------
    struct HubRec { bool has = false; double mass = 0; std::vector<double> I9; std::vector<double> rBcB{0.0, 0.0, 0.0}; };
    struct WheelRec { std::array<double, 3> gs; double Js; };
    // WP5 env record (HubRec-like). has flips on the FIRST env call below;
    // the cluster then registers ONCE as order code 'e'. Wrench geometry
    // (facets/CdA/dipole) is recorded for replay symmetry but its MATH is
    // module-side (A3); only bodyMu/J2/Re/j2On enter the graph (g_N channel).
    struct EnvRec {
        bool has = false;
        std::vector<double> bodyMu;
        std::vector<std::array<double, 3>> bodyOff0;  // static/initial offsets
        double J2 = 0.0;
        double Re = 6378137.0;
        bool j2On = false;
        bool dragOn = false;
        double CdA = 0.0;
        std::array<double, 3> comOff{0.0, 0.0, 0.0};
        struct Facet { double area = 0; std::array<double, 3> nHat{1,0,0};
                       std::array<double, 3> rCop{0,0,0}; double cd = 0, cs = 0; };
        std::vector<Facet> facets;
        bool cannonballOn = false;
        double cbArea = 0.0, cbCr = 0.0;
        std::array<double, 3> sunState{0.0, 0.0, 0.0};
        bool hasSun = false;
        bool magOn = false;
        std::array<double, 3> magM{0.0, 0.0, 0.0};
        double g10 = 0.0, g11 = 0.0, h11 = 0.0;
    };
    EnvRec env_;
    HubRec hub_;
    bool has_hub_ = false;
    std::vector<WheelRec> wheels_;
    // Registration order across kinds ('w' per wheel, 't' for the thruster
    // cluster, 'e' for the single WP5 env cluster holding ALL env effects).
    // refresh() builds contributors in this order so the dynamic
    // input/parameter layout (A2) follows registration, not a hard-coded
    // kind order. Callers must therefore resolve input slots via
    // wheel_input_offsets(), never by assuming wheels come first.
    std::vector<char> order_;
    std::vector<std::array<double,3>> thrDirs_;
    std::vector<std::array<double,3>> thrPos_;
    std::vector<double> thrTmax_;
    bool has_thrusters_ = false;
    bool has_fuel_ = false;
    double mu_ = 3.986004418e14;
    bool has_gravity_ = false;
    bool use_gg_ = true;
    double u_max_ = 0.2;
    std::vector<double> default_state_;
    bool has_default_state_ = false;

    // -- derived layout (mutable cache) --------------------------------------
    mutable bool dirty_ = true;
    mutable size_t n_states_ = 0, n_inputs_ = 0, n_pars_ = 0;
    mutable std::vector<std::string> state_names_, input_names_, par_names_;
    mutable std::vector<double> par_defaults_;
    mutable std::vector<std::unique_ptr<ContributorT<double>>> num_;
    mutable std::vector<std::unique_ptr<ContributorT<heyoka::expression>>> sym_;

    static const std::vector<std::string>& hub_state_names() {
        static const std::vector<std::string> k = {
            "rx", "ry", "rz", "vx", "vy", "vz",
            "s1", "s2", "s3", "omx", "omy", "omz"};
        return k;
    }

    // Register the single 'e' order entry once (mirrors the 't' erase/re-push,
    // but without erase: bodies accumulate, the cluster never re-registers).
    void ensure_env_order() {
        if (std::find(order_.begin(), order_.end(), 'e') == order_.end())
            order_.push_back('e');
    }

    void check_sizes(const std::vector<double>& s, const std::vector<double>& p) const {
        if (s.size() != n_states_)
            throw std::invalid_argument("OmniSystem: state has wrong size");
        if (p.size() != n_pars_)
            throw std::invalid_argument("OmniSystem: pars has wrong size");
    }

    Vector3<double> gravity_field(const Vector3<double>& rN) const {
        if (!has_gravity_) {
            Vector3<double> z; z[0] = z[1] = z[2] = 0.0; return z;
        }
        for (const auto& c : num_) {
            if (c->kind() == "gravity") {
                const auto* g = static_cast<const CentralGravityContributorT<double>*>(c.get());
                return g->field(rN);
            }
        }
        Vector3<double> z; z[0] = z[1] = z[2] = 0.0; return z;
    }

    // Helper: EvalCtx borrows vectors; for the symbolic path we pass the live
    // var/par vectors directly (see build_symbolic_from_ctx).
    static const std::vector<heyoka::expression>& vars_as_state(
        const std::vector<heyoka::expression>& v) {
        return v;
    }

    std::vector<std::pair<heyoka::expression, heyoka::expression>> build_symbolic_from_ctx(
        const std::vector<heyoka::expression>& vars,
        const std::vector<heyoka::expression>& par) const {
        const EvalCtx<heyoka::expression> ctx{vars, par};
        // g_N first (A5).
        Vector3<heyoka::expression> gN;
        {
            bool found = false;
            Vector3<heyoka::expression> rN;
            rN[0] = vars[0]; rN[1] = vars[1]; rN[2] = vars[2];
            for (const auto& c : sym_) {
                if (c->kind() == "gravity") {
                    const auto* g =
                        static_cast<const CentralGravityContributorT<heyoka::expression>*>(c.get());
                    gN = g->field(rN);
                    found = true;
                    break;
                }
            }
            if (!found) gN = zero_vec3<heyoka::expression>();
        }
        for (const auto& c : sym_) {
            if (c->kind() == "env") {
                const auto* e =
                    static_cast<const EnvContributorT<heyoka::expression>*>(c.get());
                Vector3<heyoka::expression> rN;
                rN[0] = vars[0]; rN[1] = vars[1]; rN[2] = vars[2];
                gN = vecadd3(gN, e->gravityExtra(rN, ctx));
            }
        }
        BackSubMatricesT<heyoka::expression> b;
        for (const auto& c : sym_) c->updateContributions(0.0, b, gN, ctx);

        // DynamicEffector channel: same contributor code, expression-valued.
        // The force/torque it writes is a function of the parameter slots, so
        // the symbolic graph carries the wrench exactly as params (design A3:
        // no clamps -- the quantizer runs at step time, not here).
        ForceTorqueT<heyoka::expression> ft;
        for (const auto& c : sym_) {
            if (c->has_force_torque() && c->kind() != "env")
                c->computeForceTorque(0.0, 0.0, ft, ctx);
        }
        for (const auto& c : sym_) {
            if (c->has_force_torque() && c->kind() == "env")
                c->computeForceTorque(0.0, 0.0, ft, ctx);
        }

        Vector3<heyoka::expression> rDDot, wDot;
        solve_backsub(b, rDDot, wDot, &ft);

        Vector3<heyoka::expression> s, w;
        s[0] = vars[6]; s[1] = vars[7]; s[2] = vars[8];
        w[0] = vars[9]; w[1] = vars[10]; w[2] = vars[11];
        const Vector3<heyoka::expression> sd = mrp_kinematics(s, w);

        std::vector<std::pair<heyoka::expression, heyoka::expression>> sys;
        sys.reserve(n_states_);
        // rdot = v
        for (int i = 0; i < 3; ++i) sys.emplace_back(vars[i], vars[3 + i]);
        // vdot
        for (int i = 0; i < 3; ++i) sys.emplace_back(vars[3 + i], rDDot[i]);
        // sdot
        for (int i = 0; i < 3; ++i) sys.emplace_back(vars[6 + i], sd[i]);
        // wdot
        for (int i = 0; i < 3; ++i) sys.emplace_back(vars[9 + i], wDot[i]);
        // wheel states
        size_t k = 12;
        for (const auto& c : sym_) {
            auto d = c->state_derivatives(w, wDot, ctx);
            for (const auto& e : d) sys.emplace_back(vars[k++], e);
        }
        return sys;
    }

    void refresh() const {
        if (!dirty_) return;
        if (!has_hub_) throw std::runtime_error("OmniSystem: no hub registered");
        if (env_.has && env_.j2On && !has_gravity_)
            throw std::runtime_error("OmniSystem: J2 requires central gravity");

        num_.clear();
        sym_.clear();
        state_names_ = hub_state_names();
        input_names_.clear();
        par_names_.clear();
        par_defaults_.clear();

        // Gravity first in the contributor ORDER (A5: g_N runs first).
        if (has_gravity_) {
            num_.push_back(std::make_unique<CentralGravityContributorT<double>>(mu_));
            sym_.push_back(
                std::make_unique<CentralGravityContributorT<heyoka::expression>>(mu_));
        }
        // Hub (no states of its own; no inputs).
        {
            auto h = std::make_unique<HubContributorT<double>>(use_gg_, mu_);
            auto hs = std::make_unique<HubContributorT<heyoka::expression>>(use_gg_, mu_);
            h->par_offset = 0;  // assigned below in the par pass
            hs->par_offset = 0;
            num_.push_back(std::move(h));
            sym_.push_back(std::move(hs));
        }
        // Thruster cluster then wheels, BOTH in registration order (A2): a
        // single pass over the order log. (A fixed kind order here once
        // routed motor torques into thruster-duty slots whenever both were
        // present — the module wrote P[0..N] assuming wheels first.)
        size_t wpos = 0;
        for (char k : order_) {
            if (k == 't') {
                if (!has_thrusters_) throw std::runtime_error("OmniSystem: stale order log");
                num_.push_back(std::make_unique<ThrusterContributorT<double>>(
                    thrDirs_, thrPos_, thrTmax_));
                sym_.push_back(std::make_unique<ThrusterContributorT<heyoka::expression>>(
                    thrDirs_, thrPos_, thrTmax_));
            } else if (k == 'e') {
                if (!env_.has) throw std::runtime_error("OmniSystem: stale order log");
                num_.push_back(std::make_unique<EnvContributorT<double>>(
                    env_.bodyMu, mu_, env_.J2, env_.Re, env_.j2On));
                sym_.push_back(std::make_unique<EnvContributorT<heyoka::expression>>(
                    env_.bodyMu, mu_, env_.J2, env_.Re, env_.j2On));
            } else {
                if (wpos >= wheels_.size())
                    throw std::runtime_error("OmniSystem: stale order log");
                Vector3<double> gs;
                gs[0] = wheels_[wpos].gs[0]; gs[1] = wheels_[wpos].gs[1]; gs[2] = wheels_[wpos].gs[2];
                num_.push_back(std::make_unique<ReactionWheelContributorT<double>>(gs, int(wpos)));
                sym_.push_back(
                    std::make_unique<ReactionWheelContributorT<heyoka::expression>>(gs, int(wpos)));
                ++wpos;
            }
        }
        if (wpos != wheels_.size())
            throw std::runtime_error("OmniSystem: stale order log");
        // Fuel LAST (documented layout; gather maps the stock name here).
        if (has_fuel_) {
            num_.push_back(std::make_unique<FuelMassContributorT<double>>());
            sym_.push_back(std::make_unique<FuelMassContributorT<heyoka::expression>>());
        }

        // WP4: wire aggregate c_B into hub + thruster records BEFORE the par
        // pass, so thruster cBx/cBy/cBz par_defaults snapshot the record
        // (modules overwrite per tick when live). Telemetry stays about-B.
        {
            for (auto& h : num_)
                if (h->kind() == "hub")
                    static_cast<HubContributorT<double>*>(h.get())->setComOffset(
                        hub_.rBcB[0], hub_.rBcB[1], hub_.rBcB[2]);
            for (auto& h : sym_)
                if (h->kind() == "hub")
                    static_cast<HubContributorT<heyoka::expression>*>(h.get())->setComOffset(
                        hub_.rBcB[0], hub_.rBcB[1], hub_.rBcB[2]);
            for (auto& c : num_)
                if (c->kind() == "thruster")
                    static_cast<ThrusterContributorT<double>*>(c.get())->set_com_offset(
                        hub_.rBcB[0], hub_.rBcB[1], hub_.rBcB[2]);
            for (auto& c : sym_)
                if (c->kind() == "thruster")
                    static_cast<ThrusterContributorT<heyoka::expression>*>(c.get())->set_com_offset(
                        hub_.rBcB[0], hub_.rBcB[1], hub_.rBcB[2]);
        }

        // State layout: 12 hub states, then contributor states (A1).
        size_t soff = 12;
        for (auto& c : num_) {
            c->state_offset = soff;
            soff += c->n_states();
            for (const auto& nm : c->state_names()) state_names_.push_back(nm);
        }
        {
            size_t k = 12;
            for (auto& c : sym_) {
                c->state_offset = k;
                k += c->n_states();
            }
        }
        n_states_ = soff;

        // Input block first (A2).
        size_t ioff = 0;
        for (auto& c : num_) {
            c->input_offset = ioff;
            ioff += c->n_inputs();
            for (const auto& nm : c->input_names()) input_names_.push_back(nm);
        }
        {
            size_t k = 0;
            for (auto& c : sym_) {
                c->input_offset = k;
                k += c->n_inputs();
            }
        }
        n_inputs_ = ioff;

        // Then own parameters in registration order (A2). Input slots come
        // first in the par vector with the same names.
        par_names_ = input_names_;
        par_defaults_.assign(n_inputs_, 0.0);
        size_t poff = n_inputs_;
        // NOTE: hub par_offset was parked at 0 above; assign properly here.
        for (auto& c : num_) {
            c->par_offset = poff;
            poff += c->n_pars();
            for (const auto& nm : c->par_names()) par_names_.push_back(nm);
            for (double d : c->par_defaults()) par_defaults_.push_back(d);
        }
        {
            size_t k = n_inputs_;
            for (auto& c : sym_) {
                c->par_offset = k;
                k += c->n_pars();
            }
        }
        n_pars_ = poff;

        // Hub D-correction wiring (stock total-inertia semantics): the hub
        // reads each wheel's Js from its own par slot (parametric), with the
        // structural spin axes alongside. Wheels in contributor order.
        {
            HubContributorT<double>* hubd = nullptr;
            for (auto& h : num_)
                if (h->kind() == "hub")
                    hubd = static_cast<HubContributorT<double>*>(h.get());
            HubContributorT<heyoka::expression>* hubs = nullptr;
            for (auto& h : sym_)
                if (h->kind() == "hub")
                    hubs = static_cast<HubContributorT<heyoka::expression>*>(h.get());
            size_t wpos = 0;
            for (auto& c : num_) {
                if (c->kind() != "rw") continue;
                Vector3<double> ax;
                ax[0] = wheels_[wpos].gs[0];
                ax[1] = wheels_[wpos].gs[1];
                ax[2] = wheels_[wpos].gs[2];
                hubd->wheel_axes_.push_back(ax);
                hubd->wheel_js_off_.push_back(c->par_offset);
                ++wpos;
            }
            wpos = 0;
            for (auto& c : sym_) {
                if (c->kind() != "rw") continue;
                Vector3<double> ax;
                ax[0] = wheels_[wpos].gs[0];
                ax[1] = wheels_[wpos].gs[1];
                ax[2] = wheels_[wpos].gs[2];
                hubs->wheel_axes_.push_back(ax);
                hubs->wheel_js_off_.push_back(c->par_offset);
                ++wpos;
            }
        }

        // (WP4 wiring moved before the par pass; see above.)

        // Fill hub parameter defaults from the registration record.
        // Hub is num_[has_gravity ? 1 : 0]; its par block is mass + I6 (7).
        // NOTE: there is deliberately NO invI6 block: solve_backsub derives
        // the inverse symbolically from D via inv3(), so an inverse-inertia
        // parameter would be declared-but-never-referenced and heyoka
        // rejects the par-count mismatch at construction.
        {
            ContributorT<double>* hub = nullptr;
            for (auto& c : num_)
                if (c->kind() == "hub") hub = c.get();
            if (!hub) throw std::runtime_error("OmniSystem: hub missing after refresh");
            const size_t o = hub->par_offset;
            // I6 from the record (row-major 3x3 -> packed xx,yy,zz,xy,xz,yz).
            Eigen::Matrix3d I;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) I(i, j) = hub_.I9[size_t(i) * 3 + size_t(j)];
            par_defaults_[o] = hub_.mass;
            par_defaults_[o + 1] = I(0, 0); par_defaults_[o + 2] = I(1, 1);
            par_defaults_[o + 3] = I(2, 2); par_defaults_[o + 4] = I(0, 1);
            par_defaults_[o + 5] = I(0, 2); par_defaults_[o + 6] = I(1, 2);
        }
        // Wheel Js defaults from records.
        {
            size_t w = 0;
            for (auto& c : num_) {
                if (c->kind() != "rw") continue;
                par_defaults_[c->par_offset] = wheels_[w++].Js;
            }
        }

        dirty_ = false;
    }
};

}  // namespace omni

#endif  // OMNI_SYSTEM_HPP
