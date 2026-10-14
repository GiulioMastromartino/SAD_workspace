// ISC License - see native_rw/LICENSE.
//
// WP5 environment framework: EnvContributorT<S> + module-side TRUE laws.
//
// WHAT THIS IS
//   The wrench-producing environment effects (PLAN_ENVELOPE.md Rev 1 WP5),
//   all module-side with the ForceTorque channel, one par-input block per
//   effect. The symbolic graph stays pure (A3): it sees the env input slots
//   verbatim, plus + - * / sqrt gravity-point-mass math for N-body/J2.
//
// OWNERSHIP DECISION (binding, Rev 1 WP5)
//   One EnvContributorT cluster holding ALL env effects, order code 'e'
//   (single entry in OmniSystem::order_), mirroring the thruster-cluster
//   pattern ('t'): the cluster is registered once (lazily, on the first env
//   API call) and never re-registered; adding bodies only changes its size.
//   Per-effect order codes ('g','j','d','s','m') were REJECTED: they would
//   fragment the input layout and complicate the WP3 arc-hook ordering for
//   zero generality gain (env effects have no states and no own pars).
//
// INPUT LAYOUT (A2: name-derived; gates/benches use par_names(), never a
// hard-coded count). Offsets below are relative to the cluster's
// input_offset, assigned by OmniSystem::refresh() in registration order:
//
//   [bodyOffset + 3*i + {0,1,2}]  envG{i}x, envG{i}y, envG{i}z
//       N-body offsets refreshed PER TICK from SpicePlanetStateMsg /
//       planetBodyInMsg (ZOH staleness ~1e-10 m/s^2 @ dt=0.1; frozen-whole-
//       run struck -- Sun tide 1.4e-6 m/s^2 is fatal over 3 yr). Sign-fixed:
//       a = SUM mu_i (r_i - r) / |r_i - r|^3 (central-sign inward ledger).
//   [dragOffset + 0..5]  dragFx, dragFy, dragFz, dragTx, dragTy, dragTz
//       Body-frame drag wrench, module-computed per tick (per arc for the
//       fast variable) from AtmoPropsMsg.neutralDensity + WindMsg.v_air_N.
//   [srpOffset + 0..5]   srpFx, srpFy, srpFz, srpTx, srpTy, srpTz
//       Body-frame SRP wrench, module-computed per tick (per arc across the
//       penumbra) from full SpicePlanetStateMsg sun state x eclipse scalar.
//   [magOffset + 0..2]   magTx, magTy, magTz   (torque-only; force is zero)
//       Body-frame magnetic torque, module-computed per tick (per arc; the
//       attitude-rotated field is the fast variable) as tau = m x B_B with
//       B_B = dcm_BN(sigma) * magField_N.
//
//   n_inputs() = 3*Nbodies + 6 + 6 + 3. n_states() = 0, n_pars() = 0.
//   Gravity (N-body + J2) goes through the g_N channel (gravityExtra(),
//   summed by OmniSystem into g_N before the block pass), NOT through
//   ForceTorque. J2 is ECI-equatorial analytic (zonal symmetry, no rotating
//   frame): J2 = -sqrt(5)*Cbar20, maxDeg=2 pin. J2/muCentral/Re are
//   STRUCTURAL (baked doubles); only the body offsets are input slots, so
//   the graph keeps the r-dependence (no ZOH staleness within a step).
//
// A3: the graph reads every env slot verbatim. All clamps/guards/branches
//   (|v|<=1e-12 -> 0, (1+densityCorrection)>=0, eclipse default 1.0, shadow
//   clamping, Gt transpose, maxMtbDipoles clamp) run MODULE-SIDE in the
//   omni::env doubles helpers below / OmniSpacecraft::updateEnvInputs, never
//   in gravityExtra()/computeForceTorque().
//
// ABSENT-LINK TRUTH TABLE (P1, enforced module-side at tick time):
//   drag-density unlinked  -> stock bskError (omni off-by-default = LABELED
//                             deviation: dragOn_==false zeroes the slots).
//   wind unlinked          -> silent-ON inertial fallback (v_rel = v_BN_N).
//   SRP-sun unlinked       -> bskError.
//   eclipse unlinked       -> silent-ON 1.0 (full sun; single field
//                             shadowFactor -- illuminationFactor is a
//                             deprecated C #define alias, never read).
//   eclipse no sc/planet   -> stock bskError (N/A omni-side, documented).
//   MTB cmd/mag/params unlinked -> bskError (omni: magOn_ requires a linked
//                             magField msg AND a nonzero setMagneticMoment).
//
// ERP: FIELD-ONLY, NO FORCE. earthRadiationModel has no force consumer in
//   this build (facetERPDynamicEffector absent); the module holds the ERP
//   ReadFunctor for message-equality only. This header declares NO ERP slots
//   and computeForceTorque() adds NO ERP term -- by construction, not by flag.
//
// PER-ARC HOOK (WP3-compatible): fast-varying effects (drag/SRP/mag) are
//   refreshed per arc by the module rewriting the env slots before each
//   propagate_for (see the `// WP5-arc-hook` marker in UpdateState). The hook
//   writes env slots ONLY; it never touches openT/sw/mask/fill_thruster_inputs
//   (same UpdateState loop, sequenced C1->C2, independently revertible only
//   as a whole contributor).
//
// FORCE-TORQUE ACCUMULATION: computeForceTorque() ACCUMULATES (+=) onto the
//   incoming ft. OmniSystem MUST evaluate env contributors LAST in the
//   ForceTorque loop (two-pass: non-env, then env), because the thruster
//   contributor overwrites ft and is owned by a parallel agent (untouchable).
#ifndef OMNI_ENVIRONMENT_HPP
#define OMNI_ENVIRONMENT_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "omni/contributors.hpp"
#include "omni/force_torque.hpp"
#include "omni/linear3.hpp"
#include "omni/mrp_t.hpp"
#include "omni/scalar_traits.hpp"

namespace omni {

// ---------------------------------------------------------------------------
// Shared constants (ledger-pinned; single source for module + benches).
// ---------------------------------------------------------------------------
namespace env_const {
constexpr double AU_M = 1.495978707e11;        // m
constexpr double P_SRP_1AU = 4.5398178907e-6;  // N/m^2 @ 1 AU (ledger SRP gate)
constexpr double EARTH_RADIUS_M = 6378137.0;   // m (J2 reference radius)
constexpr double MU_SUN = 1.32712440018e20;    // m^3/s^2
constexpr double MU_MOON = 4.9048695e12;       // m^3/s^2
constexpr double J2_EARTH = 1.08262668e-3;     // -sqrt(5)*Cbar20
constexpr double OMEGA_EARTH = 7.2921159e-5;   // rad/s (zeroWind corotation)
}  // namespace env_const

// ---------------------------------------------------------------------------
// Module-side (A3) doubles helpers: TRUE laws, plain double arithmetic with
// whatever branches/guards the physics needs. The symbolic graph NEVER calls
// these; the module calls them per tick (/per arc) and writes the results
// into the env input slots. Mirrored 1:1 in numpy in tests/test_wp5_env.py.
// ---------------------------------------------------------------------------
namespace env {

using Vec3 = std::array<double, 3>;

inline Vec3 vec_add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
inline Vec3 vec_sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline Vec3 vec_scale(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
inline Vec3 vec_cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}
inline double vec_dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double vec_norm(const Vec3& v) { return std::sqrt(vec_dot(v, v)); }

using Mat3 = std::array<std::array<double, 3>, 3>;

// Body-from-inertial DCM from MRPs (avsEigenMRP semantics -- the SAME formula
// as omni::MRPt<S>::toRotationMatrix(), written here in doubles so the module
// needs no Eigen/BSK dependency for the per-tick wrench rotation):
//   R = I + (8*S^2 - 4*(1-|s|^2)*S) / (1+|s|^2)^2.
inline Mat3 dcmBN_from_sigma(const Vec3& sig) {
    const double s1 = sig[0], s2 = sig[1], s3 = sig[2];
    const double sn2 = s1 * s1 + s2 * s2 + s3 * s3;
    const double d = (1.0 + sn2) * (1.0 + sn2);
    const double k8 = 8.0 / d;
    const double c = 4.0 * (1.0 - sn2) / d;
    // S^2 entries of the skew matrix S(s).
    const double s11 = -s2 * s2 - s3 * s3, s12 = s1 * s2, s13 = s1 * s3;
    const double s21 = s1 * s2, s22 = -s1 * s1 - s3 * s3, s23 = s2 * s3;
    const double s31 = s1 * s3, s32 = s2 * s3, s33 = -s1 * s1 - s2 * s2;
    Mat3 R{};
    R[0] = {1.0 + k8 * s11, k8 * s12 + c * s3, k8 * s13 - c * s2};
    R[1] = {k8 * s21 - c * s3, 1.0 + k8 * s22, k8 * s23 + c * s1};
    R[2] = {k8 * s31 + c * s2, k8 * s32 - c * s1, 1.0 + k8 * s33};
    return R;
}

inline Vec3 matvec_BN(const Mat3& R, const Vec3& vN) {
    // v_B = R * v_N.
    return {R[0][0] * vN[0] + R[0][1] * vN[1] + R[0][2] * vN[2],
            R[1][0] * vN[0] + R[1][1] * vN[1] + R[1][2] * vN[2],
            R[2][0] * vN[0] + R[2][1] * vN[1] + R[2][2] * vN[2]};
}

// N-body acceleration, SIGN-FIXED: a = SUM mu_i (r_i - r)/|r_i - r|^3.
// Ledger: central-only points inward; Sun tide 5.37e-7 m/s^2 at LEO.
inline Vec3 nbodyAccel(const Vec3& rN,
                       const std::vector<double>& mus,
                       const std::vector<Vec3>& offsetsN) {
    Vec3 a{0.0, 0.0, 0.0};
    for (size_t i = 0; i < mus.size() && i < offsetsN.size(); ++i) {
        const Vec3 d = vec_sub(offsetsN[i], rN);
        const double d2 = vec_dot(d, d);
        const double dn = std::sqrt(d2);
        if (!(dn > 0.0)) continue;  // module-side singularity guard (A3: graph has none)
        a = vec_add(a, vec_scale(d, mus[i] / (d2 * dn)));
    }
    return a;
}

// ECI-equatorial J2 DELTA-acceleration (total minus central), zonal symmetry,
// no rotating frame. f = 1.5*J2*(Re/n)^2; z2 = (z/n)^2;
// corr = [1+f(1-5z2), 1+f(1-5z2), 1+f(3-5z2)]; dA = (-mu/n^3) r (corr-1).
// Ledger: equatorial rel 1.4379e-3, inclined45 rel 1.6077e-3.
inline Vec3 j2DeltaA_eci(const Vec3& rN, double mu, double J2, double Re) {
    const double n2 = vec_dot(rN, rN);
    const double n = std::sqrt(n2);
    if (!(n > 0.0) || J2 == 0.0) return {0.0, 0.0, 0.0};
    const double f = 1.5 * J2 * (Re * Re) / n2;
    const double z2 = (rN[2] * rN[2]) / n2;
    const double k = -mu / (n2 * n);
    const double dxy = k * f * (1.0 - 5.0 * z2);
    const double dz = k * f * (3.0 - 5.0 * z2);
    return {dxy * rN[0], dxy * rN[1], dz * rN[2]};
}

struct WrenchB {
    Vec3 F{};  // body-frame force [N]
    Vec3 T{};  // body-frame torque about CoM [Nm]
};

// Drag TRUE law (stock dragDynamicEffector assembly):
//   F_B = 1/2 rho CdA |v_rel_B|^2 (-vhat), T = F_B x dragArm_B,
//   |v| <= 1e-12 -> 0, effective density rho*(1+corr) with (1+corr) >= 0.
// vRel_B is ALREADY body-frame (module rotates v_BN_N - v_air_N by dcm_BN).
// dragArm_B is the moment arm about B (callers pass rBcB - comOffset, NOT
// verbatim comOffset -- L16 rename; the old comOffset_B name invited a
// verbatim revert). Helper semantics UNCHANGED: T = F x arm.
// Ledger LEO: |F| = 1.2937e-4 N (rho=2e-12, Cd=2.2, A=1, v=LEO circular).
// TORQUE SIGN (host-measured 2026-10-13, re-measured live by the in-tree
// host test tests/test_wp45_env_offset.py::test_stock_drag_arm_rule_rbcb_minus_com):
// omni [0,0,-1.07812560e-09] vs stock [0,0,+1.07812559e-09], |.| equal to
// 8 digits, sign flipped on the torque-active axis): stock takes
// T = F x dragArm (== -(dragArm x F)). The old dragArm x F order is kept
// here as the documented-rejected form; do NOT revert without a
// firing-proven host leg.
inline WrenchB dragWrench(double rho, double CdA, const Vec3& vRel_B,
                          const Vec3& dragArm_B, double densityCorr = 0.0) {
    WrenchB w{};
    double eff = 1.0 + densityCorr;
    if (!(eff >= 0.0)) eff = 0.0;
    const double rhoEff = rho * eff;
    if (!(rhoEff > 0.0) || !(CdA > 0.0)) return w;
    const double sp = vec_norm(vRel_B);
    if (!(sp > 1e-12)) return w;
    // F = 1/2 rho CdA |v|^2 (-vhat) = -1/2 rho CdA |v| v (no division).
    const Vec3 F = vec_scale(vRel_B, -0.5 * rhoEff * CdA * sp);
    w.F = F;
    // Stock-measured +z convention (see TRUE-law header): T = F x dragArm.
    w.T = vec_cross(F, dragArm_B);
    return w;
}

// Cannonball SRP (stock radiationPressure, staged first):
//   F_B = -shadow * P * A * (1+cr) * sHat_B, P = P_SRP*(AU/d)^2.
// Ledger: |F| = 7.2637e-6 N (A=1, cr=0.6, d=1 AU, shadow=1).
inline Vec3 srpCannonball(double area, double cr, double P, double shadow,
                          const Vec3& sHat_B) {
    if (!(area > 0.0) || !(shadow > 0.0)) return {0.0, 0.0, 0.0};
    return vec_scale(sHat_B, -shadow * P * area * (1.0 + cr));
}

// Occlusion-free projected area (PROJECTION PROOF HOOK): the bench asserts
// facetedSpacecraftProjectedArea == max(0, nHat.sHat)*A per facet on the
// occlusion-free 6-facet box before any re-derivation consumes the stock
// facetProjectedAreaInMsgs path.
inline double projectedArea(double facetArea, const Vec3& nHat_B, const Vec3& sHat_B) {
    const double c = vec_dot(nHat_B, sHat_B);
    return (c > 0.0) ? c * facetArea : 0.0;
}

// Faceted SRP TRUE law per facet (stock facetedSRPEffector two-term law):
//   F = -shadow*P*A_proj*[(1-cs)*sHat + 2*(cd/3 + cs*cosT)*nHat],
//   T = r_Cop x F, cosT = A_proj/A. Back-facing facets (A_proj=0) give 0.
inline WrenchB srpFacet(double area, const Vec3& nHat_B, const Vec3& rCop_B,
                        double c_diffuse, double c_specular, double P, double shadow,
                        const Vec3& sHat_B) {
    WrenchB w{};
    if (!(area > 0.0) || !(shadow > 0.0)) return w;
    const double Aproj = projectedArea(area, nHat_B, sHat_B);
    if (!(Aproj > 0.0)) return w;
    const double cosT = Aproj / area;
    const Vec3 t1 = vec_scale(sHat_B, 1.0 - c_specular);
    const Vec3 t2 = vec_scale(nHat_B, 2.0 * (c_diffuse / 3.0 + c_specular * cosT));
    w.F = vec_scale(vec_add(t1, t2), -shadow * P * Aproj);
    w.T = vec_cross(rCop_B, w.F);
    return w;
}

// Magnetic TRUE law: tau_B = m x B_B (stock MtbEffector assembly for
// identity-Gt). B_B = dcm_BN(sigma) * magField_N hijacked from WMM /
// centered-dipole. Row-major -> col-major Gt transpose + maxMtbDipoles clamp
// are module-side (caller) concerns; this is the bare cross product.
// Ledger: |B| = 22912.88 nT, tau = m x B_B for m = [0.5, 0, 0].
inline Vec3 magTorque(const Vec3& m_B, const Vec3& B_B) { return vec_cross(m_B, B_B); }

}  // namespace env

// ---------------------------------------------------------------------------
// EnvContributorT<S>: the single env cluster (order code 'e').
// ---------------------------------------------------------------------------
template <typename S>
class EnvContributorT : public ContributorT<S> {
public:
    // bodyMu: structural mu per N-body (offsets are input slots, refreshed
    // per tick). muCentral: central mu for the J2 scaling. J2/Re/j2On:
    // structural ECI zonal config (WP0: j2On_ defaults false; setJ2 enables).
    EnvContributorT(std::vector<double> bodyMu, double muCentral, double J2,
                    double Re, bool j2On)
        : bodyMu_(std::move(bodyMu)),
          muCentral_(muCentral),
          J2_(J2),
          Re_(Re),
          j2On_(j2On) {}

    std::string kind() const override { return "env"; }
    size_t n_states() const override { return 0; }
    // 3 per N-body + drag 6 + SRP 6 + mag-torque 3. ERP: NO slots (field-only).
    size_t n_inputs() const override { return 3 * bodyMu_.size() + 6 + 6 + 3; }
    size_t n_pars() const override { return 0; }
    std::vector<std::string> state_names() const override { return {}; }
    std::vector<std::string> input_names() const override {
        std::vector<std::string> o;
        for (size_t i = 0; i < bodyMu_.size(); ++i) {
            o.push_back("envG" + std::to_string(i) + "x");
            o.push_back("envG" + std::to_string(i) + "y");
            o.push_back("envG" + std::to_string(i) + "z");
        }
        for (const char* n : {"dragFx", "dragFy", "dragFz", "dragTx", "dragTy", "dragTz"})
            o.emplace_back(n);
        for (const char* n : {"srpFx", "srpFy", "srpFz", "srpTx", "srpTy", "srpTz"})
            o.emplace_back(n);
        for (const char* n : {"magTx", "magTy", "magTz"}) o.emplace_back(n);
        return o;
    }
    std::vector<std::string> par_names() const override { return {}; }
    std::vector<double> par_defaults() const override { return {}; }
    bool has_force_torque() const override { return true; }

    // Slot base offsets (relative to the cluster input_offset).
    size_t bodyOffset() const { return this->input_offset; }
    size_t dragOffset() const { return this->input_offset + 3 * bodyMu_.size(); }
    size_t srpOffset() const { return dragOffset() + 6; }
    size_t magOffset() const { return srpOffset() + 6; }
    size_t nBodies() const { return bodyMu_.size(); }

    void updateContributions(double, BackSubMatricesT<S>&, const Vector3<S>&,
                             const EvalCtx<S>&) const override {
        // Deliberately empty: env gravity flows via gravityExtra() (summed
        // into g_N by OmniSystem BEFORE the block pass, A5), and the wrench
        // flows via computeForceTorque(). Env never touches the blocks.
    }

    std::vector<S> state_derivatives(const Vector3<S>&, const Vector3<S>&,
                                     const EvalCtx<S>&) const override {
        return {};
    }

    // Extra gravity acceleration in N frame (N-body SIGN-FIXED sum + J2 ECI
    // delta). Only + - * / sqrt: graph-safe (A3). No |d| guard in the graph
    // (module-side nbodyAccel() has one; the graph matches stock's
    // singularity the same way central gravity does).
    Vector3<S> gravityExtra(const Vector3<S>& rN, const EvalCtx<S>& ctx) const {
        const std::vector<S>& P = ctx.pars;
        Vector3<S> a = zero_vec3<S>();
        for (size_t b = 0; b < bodyMu_.size(); ++b) {
            Vector3<S> off;
            off[0] = P[this->input_offset + 3 * b];
            off[1] = P[this->input_offset + 3 * b + 1];
            off[2] = P[this->input_offset + 3 * b + 2];
            const Vector3<S> d = vecsub3(off, rN);
            const S d2 = dot3(d, d);
            const S dn = sqrt_scalar(d2);
            // WP25-C2 M4 posture: no dn==0 guard in the graph — deferred div-by-zero matches stock's singularity; module-side omni::env::nbodyAccel skips dn<=0 (A3).
            const S inv = make_scalar<S>(1.0) / (d2 * dn);
            a = vecadd3(a, scale_vec3(d, make_scalar<S>(bodyMu_[b]) * inv));
        }
        if (j2On_) {
            const S n2 = dot3(rN, rN);
            const S nn = sqrt_scalar(n2);
            const S invN3 = make_scalar<S>(1.0) / (n2 * nn);
            const S f =
                make_scalar<S>(1.5 * J2_) * (make_scalar<S>(Re_ * Re_) / n2);
            const S z2 = (rN[2] * rN[2]) / n2;
            const S one = make_scalar<S>(1.0);
            const S dxy = make_scalar<S>(-muCentral_) * invN3 * f *
                          (one - make_scalar<S>(5.0) * z2);
            const S dz = make_scalar<S>(-muCentral_) * invN3 * f *
                         (make_scalar<S>(3.0) - make_scalar<S>(5.0) * z2);
            a[0] = a[0] + dxy * rN[0];
            a[1] = a[1] + dxy * rN[1];
            a[2] = a[2] + dz * rN[2];
        }
        return a;
    }

    void computeForceTorque(double, double, ForceTorqueT<S>& ft,
                            const EvalCtx<S>& ctx) const override {
        // Verbatim slot reads (A3) + body->inertial rotation (same MRP
        // formula as the thruster channel). ACCUMULATES onto ft: the system
        // evaluates env contributors LAST (two-pass ForceTorque loop), so the
        // thruster overwrite (parallel ownership, untouchable) is preserved.
        const std::vector<S>& P = ctx.pars;
        const size_t dO = dragOffset(), sO = srpOffset(), mO = magOffset();
        Vector3<S> fB = zero_vec3<S>();
        Vector3<S> tB = zero_vec3<S>();
        for (int i = 0; i < 3; ++i) {
            fB[i] = fB[i] + P[dO + size_t(i)] + P[sO + size_t(i)];
            tB[i] = tB[i] + P[dO + size_t(3 + i)] + P[sO + size_t(3 + i)] +
                    P[mO + size_t(i)];
        }
        // F_N = R^T F_B (rotate body -> inertial).
        const MRPt<S> mrp(ctx.sigma());
        const Matrix3<S> R = mrp.toRotationMatrix();
        Matrix3<S> Rt;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) Rt(i, j) = R(j, i);
        const Vector3<S> fN = matvec3(Rt, fB);
        ft.forceExternal_B = vecadd3(ft.forceExternal_B, fB);
        ft.forceExternal_N = vecadd3(ft.forceExternal_N, fN);
        ft.torqueExternalPntB_B = vecadd3(ft.torqueExternalPntB_B, tB);
    }

private:
    std::vector<double> bodyMu_;  // structural mu per N-body
    double muCentral_;            // central mu for J2 scaling
    double J2_;                   // structural zonal coefficient
    double Re_;                   // structural reference radius [m]
    bool j2On_;                   // WP0 flag (default false)
};

}  // namespace omni

#endif  // OMNI_ENVIRONMENT_HPP
