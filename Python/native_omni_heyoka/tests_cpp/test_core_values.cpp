// Absolute-value pins for the omni core, evaluated directly in C++.
//
// WHY THIS EXISTS NEXT TO t2-t8: the scratch harnesses compare against a
// transcribed Python reference (same-team transcription on both sides -- a
// correlated error cancels invisibly). Every expected value below is
// HAND-DERIVED in the comments (integer/exact-decimal arithmetic or
// 8-digit long division); there is no reference implementation to agree
// with, only arithmetic. A failure here means the CODE drifted from the
// math, not from another model of the math.
//
// SCOPE (deliberate): header-resident math + the numeric OmniSystem path on
// double. Module-side wiring (live-c tracking, Stribeck latch state, arc
// hooks, B/C scatter) is NOT reachable without Basilisk and stays covered
// by the swap ledgers; each section notes its boundary. Direction of the
// MRP DCM vs stock is pinned by swap_p1:thr_offset, not re-derived here.
//
// Build & run: ./build.sh (needs heyoka+EIGEN+Boost include paths AND
// libheyoka at link with an embedded rpath at runtime: even double-only
// paths reference heyoka symbols via the dual-instantiation members, so the
// bare link fails and the libheyoka fallback is the normal path, not an
// exception). Exit 0 iff every check passes.
#include "omni/environment.hpp"
#include "omni/system.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using omni::OmniSystem;

static int n_ok = 0, n_fail = 0;

#define CHECK_NEAR(got, want, tol, label)                                     \
    do {                                                                      \
        double _g = (got), _w = (want);                                       \
        double _d = std::abs(_g - _w);                                        \
        double _s = std::max(1.0, std::abs(_w));                              \
        if (_d <= (tol) * _s) { ++n_ok; }                                     \
        else {                                                                \
            ++n_fail;                                                         \
            std::printf("FAIL %s:%d %s: got %.17g want %.17g\n", __FILE__,     \
                        __LINE__, label, _g, _w);                             \
        }                                                                     \
    } while (0)

#define CHECK_EQ(got, want, label)                                            \
    do {                                                                      \
        if ((got) == (want)) { ++n_ok; }                                      \
        else {                                                                \
            ++n_fail;                                                         \
            std::printf("FAIL %s:%d %s: got %.17g want %.17g (exact)\n",      \
                        __FILE__, __LINE__, label, (double)(got),             \
                        (double)(want));                                      \
        }                                                                     \
    } while (0)

// --- S1: hub + central gravity on a circular-orbit point -------------------
static void s1_hub_gravity() {
    OmniSystem sys;
    sys.set_use_gravity_gradient(false);
    sys.set_gravity(3.986004418e14);
    sys.add_hub(20.0, {10, 0, 0, 0, 11, 0, 0, 0, 12}, {0, 0, 0});
    CHECK_EQ(sys.n_states(), 12u, "hub n_states");
    CHECK_EQ(sys.n_pars(), 7u, "hub n_pars (mass+I6)");
    std::vector<double> x(12, 0.0);
    x[0] = 1e7;
    std::vector<double> P = sys.default_pars();
    P[sys.par_index("mass")] = 20.0;
    // vdot = -mu/R^2 x-hat: 3.986004418e14/1e14 = 3.986004418 exactly.
    std::vector<double> d = sys.state_derivative(x, P);
    CHECK_EQ(d[0], 0.0, "rdot.x");
    CHECK_NEAR(d[3], -3.986004418, 1e-12, "vdot.x circular");
    CHECK_EQ(d[4], 0.0, "vdot.y");
    CHECK_EQ(d[9], 0.0, "wdot.x torque-free");
    CHECK_EQ(d[10], 0.0, "wdot.y torque-free");
    CHECK_EQ(d[11], 0.0, "wdot.z torque-free");
}

// --- S2: MRP kinematics at zero attitude is omega/4 (exact, x/4 scaling) ---
static void s2_mrp_kinematics() {
    OmniSystem sys;
    sys.set_use_gravity_gradient(false);
    sys.set_gravity(3.986004418e14);
    sys.add_hub(20.0, {10, 0, 0, 0, 11, 0, 0, 0, 12}, {0, 0, 0});
    std::vector<double> x(12, 0.0);
    x[0] = 1e7;
    x[9] = 0.1; x[10] = 0.2; x[11] = 0.3;
    std::vector<double> P = sys.default_pars();
    P[sys.par_index("mass")] = 20.0;
    std::vector<double> d = sys.state_derivative(x, P);
    // B(s=0) = I, sdot = 0.25*omega; x/4 is exact in binary.
    CHECK_EQ(d[6], 0.025, "sdot.x");
    CHECK_EQ(d[7], 0.05, "sdot.y");
    CHECK_EQ(d[8], 0.075, "sdot.z");
}

// --- S3: MRP DCM at s=(0.1,0,0): Rx(4*atan(0.1)) ---------------------------
// d = 1.01^2 = 1.0201, k8 = 8/1.0201 = 7.8423686, c = 3.96/1.0201 = 3.8819724.
// R00 = 1, R11 = R22 = 1-0.078423686 = 0.92157632, R12 = +0.1c = +0.38819724,
// R21 = -0.1c, rest 0. NOTE (load-bearing): signs are IMPLEMENTATION locks;
// direction-vs-stock is pinned by swap_p1:thr_offset (2.24e-08), which would
// fail at O(1) under a flipped convention -- not re-derived here.
static void s3_mrp_dcm() {
    omni::Vector3<double> s;
    s[0] = 0.1; s[1] = 0.0; s[2] = 0.0;
    omni::MRPt<double> mr(s);
    omni::Matrix3<double> R = mr.toRotationMatrix();
    CHECK_EQ(R(0, 0), 1.0, "R00");
    CHECK_EQ(R(0, 1), 0.0, "R01");
    CHECK_EQ(R(0, 2), 0.0, "R02");
    CHECK_EQ(R(1, 0), 0.0, "R10");
    CHECK_EQ(R(2, 0), 0.0, "R20");
    CHECK_NEAR(R(1, 1), 0.92157632, 1e-8, "R11 cos");
    CHECK_NEAR(R(2, 2), 0.92157632, 1e-8, "R22 cos");
    CHECK_NEAR(R(1, 2), 0.38819724, 1e-8, "R12 +sin");
    CHECK_NEAR(R(2, 1), -0.38819724, 1e-8, "R21 -sin");
    // Orthonormality (convention-free): rows unit + mutually orthogonal.
    double r1n = R(1, 0) * R(1, 0) + R(1, 1) * R(1, 1) + R(1, 2) * R(1, 2);
    CHECK_NEAR(r1n, 1.0, 1e-12, "row1 unit");
    double d12 = R(1, 0) * R(2, 0) + R(1, 1) * R(2, 1) + R(1, 2) * R(2, 2);
    CHECK_NEAR(d12, 0.0, 1e-12, "rows 1.2 orthogonal");
}

// --- S4: Schur back-substitution, coupled blocks ---------------------------
// D = diag(10,11,12), A = 2I, B = C = I, t=(1,0,0), r=0:
// Sch = 2I - D^-1 = diag(1.9, 1.9090909, 1.9166667);
// rDDot = (1/1.9, 0, 0) = (0.5263157894736842, 0, 0);
// wDot = D^-1(0 - rDDot) = (-0.05263157894736842, 0, 0).
static void s4_backsub() {
    omni::BackSubMatricesT<double> m;
    omni::set_diag(m.A, 2.0, 2.0, 2.0);
    omni::set_diag(m.D, 10.0, 11.0, 12.0);
    omni::set_identity(m.B);
    omni::set_identity(m.C);
    m.vecTrans[0] = 1.0;
    omni::Vector3<double> rDDot, wDot;
    omni::solve_backsub(m, rDDot, wDot);
    CHECK_NEAR(rDDot[0], 0.5263157894736842, 1e-12, "rDDot.x schur");
    CHECK_EQ(rDDot[1], 0.0, "rDDot.y");
    CHECK_NEAR(wDot[0], -0.05263157894736842, 1e-12, "wDot.x schur");
    CHECK_EQ(wDot[1], 0.0, "wDot.y");
}

// --- S5: linear3 exact-integer pins (det=1 matrix) --------------------------
// A = [1 2 3; 0 1 4; 5 6 0], det = -24+40-15 = 1;
// inv = adjugate^T = [-24 18 5; 20 -15 -4; -5 4 1] (all exact in binary).
static void s5_linear3() {
    omni::Matrix3<double> A;
    A(0, 0) = 1; A(0, 1) = 2; A(0, 2) = 3;
    A(1, 0) = 0; A(1, 1) = 1; A(1, 2) = 4;
    A(2, 0) = 5; A(2, 1) = 6; A(2, 2) = 0;
    CHECK_EQ(omni::det3(A), 1.0, "det");
    omni::Matrix3<double> I = omni::matmul3(A, omni::inv3(A));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_EQ(I(i, j), (i == j) ? 1.0 : 0.0, "A*inv==I");
    omni::Vector3<double> a, b;
    a[0] = 1; a[1] = 0; a[2] = 0;
    b[0] = 0; b[1] = 1; b[2] = 0;
    omni::Vector3<double> c = omni::cross3(a, b);
    CHECK_EQ(c[0], 0.0, "cross.x");
    CHECK_EQ(c[1], 0.0, "cross.y");
    CHECK_EQ(c[2], 1.0, "cross.z");
}

// --- S6: drag TRUE law, LEO point ------------------------------------------
// v = (0,-7660,0), rho = 2e-12, CdA = 2.2:
//   scale = -0.5*2e-12*2.2*7660 = -1.6852e-8
//   (0.5*2e-12 = 1e-12; *2.2 = 2.2e-12; *7660 = 1.6852e-8);
//   F = scale*v = (0, 1.6852e-8*7660, 0) = (0, 1.2908632e-4, 0) N
//   (1.6852*7660 = 12908.632);
// arm = (0.05,0,0): T = F x arm = (0, 0, -1.2908632e-4*0.05)
//   = (0, 0, -6.454316e-6) N.m. (Pure-function pin: the ARM RULE vs stock
// lives in the ledgers; here the cross-product law itself is locked.)
static void s6_drag() {
    omni::env::Vec3 v = {0.0, -7660.0, 0.0};
    omni::env::Vec3 arm = {0.05, 0.0, 0.0};
    omni::env::WrenchB w = omni::env::dragWrench(2e-12, 2.2, v, arm, 0.0);
    CHECK_NEAR(w.F[1], 1.2908632e-4, 1e-12, "drag F");
    CHECK_EQ(w.F[0], 0.0, "drag F.x");
    CHECK_NEAR(w.T[2], -6.454316e-6, 1e-12, "drag T");
    CHECK_EQ(w.T[0], 0.0, "drag T.x");
    omni::env::WrenchB z = omni::env::dragWrench(0.0, 2.2, v, arm, 0.0);
    CHECK_EQ(z.F[1], 0.0, "drag rho=0 F");
    CHECK_EQ(z.T[2], 0.0, "drag rho=0 T");
}

// --- S7: cannonball SRP ----------------------------------------------------
// A=1, cr=0.5, P=4.56e-6, shadow=1, sHat=+x:
//   F = -1*4.56e-6*1*1.5*x = (-6.84e-6, 0, 0) N. shadow=0 -> exact zero.
static void s7_srp_cannonball() {
    omni::env::Vec3 s = {1.0, 0.0, 0.0};
    omni::env::Vec3 F = omni::env::srpCannonball(1.0, 0.5, 4.56e-6, 1.0, s);
    CHECK_NEAR(F[0], -6.84e-6, 1e-15, "srp F");
    CHECK_EQ(F[1], 0.0, "srp F.y");
    omni::env::Vec3 Z = omni::env::srpCannonball(1.0, 0.5, 4.56e-6, 0.0, s);
    CHECK_EQ(Z[0], 0.0, "srp shadow=0");
}

// --- S8: faceted SRP, normal incidence -------------------------------------
// nHat=sHat=+x, A=2, cd=0.1, cs=0.7, P=4.56e-6, shadow=1, rCop=(0,0.5,0):
//   Aproj = 2, cosT = 1, t1+t2 = 0.3+2(0.1/3+0.7) = 1.7666666666666666;
//   F = -4.56e-6*2*1.7666666666666666 x = (-1.6112e-5, 0, 0) N;
//   T = rCop x F = (0, 0, -0.5*Fx) = (0, 0, 8.056e-6) N.m.
static void s8_srp_facet() {
    CHECK_EQ(omni::env::projectedArea(2.0, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}), 2.0,
             "proj full");
    CHECK_EQ(omni::env::projectedArea(2.0, {-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}), 0.0,
             "proj backface");
    omni::env::WrenchB w = omni::env::srpFacet(2.0, {1.0, 0.0, 0.0}, {0.0, 0.5, 0.0},
                                     0.1, 0.7, 4.56e-6, 1.0, {1.0, 0.0, 0.0});
    CHECK_NEAR(w.F[0], -1.6112e-5, 1e-12, "facet F");
    CHECK_NEAR(w.T[2], 8.056e-6, 1e-12, "facet T");
    omni::env::WrenchB z = omni::env::srpFacet(2.0, {-1.0, 0.0, 0.0}, {0.0, 0.5, 0.0},
                                     0.1, 0.7, 4.56e-6, 1.0, {1.0, 0.0, 0.0});
    CHECK_EQ(z.F[0], 0.0, "facet backface F");
    CHECK_EQ(z.T[2], 0.0, "facet backface T");
}

// --- S9: thruster force + torque, zero offset -------------------------------
// dir=(0,0,1) Tmax=1 q=1 at pos=(1,0,0): F_B=(0,0,1) N;
// T = r x F = (0,-1,0) N.m; w = (0,-1/11,0); hub 20 kg: F/m = (0,0,0.05).
// Mask pin (X1): dirs z-only -> cBx,cBy live, cBz absent.
static void s9_thruster() {
    OmniSystem sys;
    sys.set_use_gravity_gradient(false);
    sys.set_gravity(3.986004418e14);
    sys.add_hub(20.0, {10, 0, 0, 0, 11, 0, 0, 0, 12}, {0, 0, 0});
    sys.add_thrusters({{0, 0, 1}}, {{1, 0, 0}}, {1.0});
    CHECK_EQ(sys.n_states(), 12u, "thr n_states");
    CHECK_EQ(sys.n_pars(), 11u, "thr n_pars (2 in + 7 hub + 2 c)");
    CHECK_EQ(sys.has_par("cBx") ? 1.0 : 0.0, 1.0, "cBx live");
    CHECK_EQ(sys.has_par("cBy") ? 1.0 : 0.0, 1.0, "cBy live");
    CHECK_EQ(sys.has_par("cBz") ? 1.0 : 0.0, 0.0, "cBz masked");
    std::vector<double> x(12, 0.0);
    x[0] = 1e7;
    std::vector<double> P = sys.default_pars();
    P[sys.par_index("mass")] = 20.0;
    P[sys.par_index("q0")] = 1.0;
    std::vector<double> d = sys.state_derivative(x, P);
    CHECK_NEAR(d[3], -3.986004418, 1e-12, "thr rDDot.x (gravity)");
    CHECK_EQ(d[5], 0.05, "thr rDDot.z (F/m)");
    CHECK_EQ(d[9 + 1], -0.09090909090909091, "thr wDot.y (T/I)");
    CHECK_EQ(d[9], 0.0, "thr wDot.x");
}

// --- S10: offset arm (r-c)xF, NON-degenerate geometry -----------------------
// Same rig with rBcB=(0.05,0,0) (auto-fed to the contributor at refresh):
// arm = (1,0,0)-(0.05,0,0) = (0.95,0,0); T = (0,-0.95,0);
// w = (0,-0.95/11,0) = (0,-0.08636363636363636,0). h != 2c by construction
// (the degenerate h=2c fixture cannot prove this; see H9).
static void s10_offset_arm() {
    OmniSystem sys;
    sys.set_use_gravity_gradient(false);
    sys.set_gravity(3.986004418e14);
    sys.add_hub(20.0, {10, 0, 0, 0, 11, 0, 0, 0, 12}, {0.05, 0, 0});
    sys.add_thrusters({{0, 0, 1}}, {{1, 0, 0}}, {1.0});
    std::vector<double> P = sys.default_pars();
    CHECK_EQ(P[sys.par_index("cBx")], 0.05, "cBx snapshot = rBcB");
    P[sys.par_index("mass")] = 20.0;
    P[sys.par_index("q0")] = 1.0;
    std::vector<double> x(12, 0.0);
    x[0] = 1e7;
    std::vector<double> d = sys.state_derivative(x, P);
    CHECK_NEAR(d[10], -0.08636363636363636, 1e-15, "offset wDot.y");
}

// --- S11: fuel depletion law sign/rate (mdot slot only) ---------------------
// add_fuel_tank appends fuelMass LAST (n=13); mFuelDot = -mdot exactly.
// Module-side live mass/I/c feedback is NOT in OmniSystem (vec module owns
// it) -- this pins the law, not the feedback.
static void s11_fuel() {
    OmniSystem sys;
    sys.set_use_gravity_gradient(false);
    sys.set_gravity(3.986004418e14);
    sys.add_hub(20.0, {10, 0, 0, 0, 11, 0, 0, 0, 12}, {0, 0, 0});
    sys.add_fuel_tank();
    CHECK_EQ(sys.n_states(), 13u, "fuel n_states (hub12+fuel)");
    CHECK_EQ(sys.n_pars(), 8u, "fuel n_pars (7 hub + mdot)");
    std::vector<double> x(13, 0.0);
    x[0] = 1e7;
    x[12] = 10.0;
    std::vector<double> P = sys.default_pars();
    P[sys.par_index("mass")] = 20.0;
    P[sys.par_index("mdot")] = 0.002;
    std::vector<double> d = sys.state_derivative(x, P);
    CHECK_EQ(d[12], -0.002, "mFuelDot = -mdot");
}

int main() {
    s1_hub_gravity();
    s2_mrp_kinematics();
    s3_mrp_dcm();
    s4_backsub();
    s5_linear3();
    s6_drag();
    s7_srp_cannonball();
    s8_srp_facet();
    s9_thruster();
    s10_offset_arm();
    s11_fuel();
    std::printf("%s: %d checks passed, %d failed\n",
                n_fail == 0 ? "PASS" : "FAIL", n_ok, n_fail);
    return n_fail == 0 ? 0 : 1;
}
