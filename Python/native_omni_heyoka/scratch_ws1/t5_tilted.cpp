// Tilted-wheel validation: does the contributor framework generalise to
// arbitrary spin axes with NO change to the solve?
//
// Reference: hand-derived Newton-Euler for balanced rotors about ARBITRARY
// axes g_i (unit), extending the body-axis case (g_i = e_i) in heyoka_rw.py,
// in STOCK input semantics (input I = total incl. wheels; hub block inverts
// D = I - sum_i Js_i*g_i*g_i^T) with stock-ordered coupled spin
// (Js*dW = u - Js*(g.wdot), evaluated after the hub solve) and RELATIVE
// rotor momentum h_i = Js_i*W_i (upstream reactionWheelStateEffector.cpp:
// vecRot -= gs*u + Js*W*(omega x gs)). The ABSOLUTE form
// h_i = Js_i*(W_i + omega.g_i) DOUBLE-COUNTS against the subtracted D and
// is wrong for non-orthonormal wheel sets (found by the swap 1-vs-3-wheel
// parity matrix; the extra term cancels exactly for 3 orthonormal wheels,
// which is why every 3-wheel gate passed with either form):
//
//   h_i    = Js_i * W_i                                      (spin momentum)
//   L      = I*omega + sum_i Js_i*W_i * g_i
//   D      = I - sum_i Js_i * g_i * g_i^T
//   wdot   = D^-1 * [ -(omega x L) - sum_i u_i*g_i ]
//   Wdot_i = (u_i - Js_i * (g_i.wdot)) / Js_i
//
// Validates: (a) acceleration vs this reference, (b) |H| conservation with
// u = 0, where H = I*omega + sum Js*W*g is the exactly-conserved total.
#include "omni/system.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

int main() {
    std::cout << std::setprecision(17);

    // --- tilted spin axes (unit): a pyramidal RM20-style mounting ----------
    const double s3 = 0.5773502691896258;  // 1/sqrt(3)
    const std::vector<std::vector<double>> axes = {
        {s3, s3, s3}, {-s3, s3, s3}, {s3, -s3, s3}};

    omni::OmniSystem sys;
    sys.set_gravity(3.986004418e14);
    sys.set_use_gravity_gradient(false);  // isolate the wheel contribution
    // generic (non-diagonal) inertia to make the test non-trivial
    const double Ixx = 900.0, Iyy = 800.0, Izz = 600.0,
                 Ixy = 12.0, Ixz = -7.5, Iyz = 4.0;
    sys.add_hub(750.0, {Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz}, {0, 0, 0});
    for (const auto& a : axes) sys.add_reaction_wheel(a, 0.07957747154594767, 0.0);

    std::cout << "states=" << sys.n_states() << " pars=" << sys.n_pars()
              << " inputs=" << sys.n_inputs() << "  (tilted pyre, 3 wheels)\n";

    const auto P = sys.default_pars();
    // rebuild the wheel Js defaults (they come from the records)
    std::vector<double> Pw = P;
    // Name-based par slots (A2; layout shifted when the tauF input slots
    // landed: u_i are NOT 0..2 anymore).
    const size_t uidx[3] = {sys.par_index("u1"), sys.par_index("u2"),
                            sys.par_index("u3")};
    const size_t mO = sys.par_index("mass");
    const size_t iO[6] = {sys.par_index("Ixx"), sys.par_index("Iyy"),
                          sys.par_index("Izz"), sys.par_index("Ixy"),
                          sys.par_index("Ixz"), sys.par_index("Iyz")};
    size_t jO[3] = {sys.par_index("Js1"), sys.par_index("Js2"),
                    sys.par_index("Js3")};

    // --- reference accelerations -------------------------------------------
    auto ref = [&](const std::vector<double>& x, const std::vector<double>& P,
                   int nw) {
        const double& IxxR = P[iO[0]]; const double& IyyR = P[iO[1]];
        const double& IzzR = P[iO[2]]; const double& IxyR = P[iO[3]];
        const double& IxzR = P[iO[4]]; const double& IyzR = P[iO[5]];
        const double omx = x[9], omy = x[10], omz = x[11];
        const double rx = x[0], ry = x[1], rz = x[2];
        Eigen::Matrix3d I;
        I << IxxR, IxyR, IxzR, IxyR, IyyR, IyzR, IxzR, IyzR, IzzR;
        Eigen::Vector3d L = I * Eigen::Vector3d(omx, omy, omz);
        Eigen::Vector3d ugu = Eigen::Vector3d::Zero();
        Eigen::Matrix3d D = I;
        std::vector<double> js;
        for (int i = 0; i < nw; ++i) js.push_back(P[jO[i]]);
        double us[3] = {P[uidx[0]], P[uidx[1]], P[uidx[2]]};
        Eigen::Vector3d omega(omx, omy, omz);
        for (int i = 0; i < nw; ++i) {
            Eigen::Vector3d g(axes[size_t(i)][0], axes[size_t(i)][1], axes[size_t(i)][2]);
            const double W = x[12 + i];
            L += js[size_t(i)] * W * g;
            ugu += us[i] * g;
            D -= js[size_t(i)] * g * g.transpose();
        }
        Eigen::Vector3d g = -(omega.cross(L)) - ugu;
        const Eigen::Vector3d wd = D.inverse() * g;
        const double R = std::sqrt(rx * rx + ry * ry + rz * rz);
        const double k = -3.986004418e14 / (R * R * R);
        std::vector<double> out = {rx * k, ry * k, rz * k, wd[0], wd[1], wd[2]};
        for (int i = 0; i < nw; ++i) {
            Eigen::Vector3d g(axes[size_t(i)][0], axes[size_t(i)][1], axes[size_t(i)][2]);
            out.push_back((us[i] - js[size_t(i)] * g.dot(wd)) / js[size_t(i)]);
        }
        return out;
    };

    const int NW = 3;
    std::mt19937_64 rng(999);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    double worst = 0.0;
    for (int k = 0; k < 400; ++k) {
        std::vector<double> x(12 + NW), P = Pw;
        for (int i = 0; i < 12 + NW; ++i) x[i] = U(rng) * (i < 3 ? 1e7 : 1.0);
        x[0] = std::abs(x[0]) + 1e6; x[1] = std::abs(x[1]) + 1e6; x[2] = std::abs(x[2]) + 1e6;
        for (int i = 0; i < NW; ++i) P[uidx[i]] = U(rng) * 0.2;
        // random TOTAL inertia (stock IHubPntBc_B semantics; hub block
        // inverts D = I - sum Js*g*g^T, computed inside the reference)
        const double a = 900.0 + U(rng) * 50, b = 800.0 + U(rng) * 50,
                     c = 600.0 + U(rng) * 50;
        const double d = U(rng) * 30, e = U(rng) * 30, f = U(rng) * 30;
        P[mO] = 750.0;
        P[iO[0]] = a; P[iO[1]] = b; P[iO[2]] = c;
        P[iO[3]] = d; P[iO[4]] = e; P[iO[5]] = f;
        for (int i = 0; i < NW; ++i) P[jO[i]] = 0.05 + std::abs(U(rng)) * 0.1;

        const std::vector<double> A = sys.state_derivative(x, P);
        const std::vector<double> B = ref(x, P, NW);  // [vdot(3), wdot(3), Wdot(NW)]

        // MRP kinematics: sdot = 0.25 * B(s) * omega  (from heyoka_rw.py)
        const double q1 = x[6], q2 = x[7], q3 = x[8];
        const double qm1 = x[9], qm2 = x[10], qm3 = x[11];
        const double qn2 = q1 * q1 + q2 * q2 + q3 * q3;
        const double sd1 = 0.25 * ((1.0 - qn2 + 2.0 * q1 * q1) * qm1
                                   + 2.0 * (q1 * q2 - q3) * qm2
                                   + 2.0 * (q1 * q3 + q2) * qm3);
        const double sd2 = 0.25 * (2.0 * (q2 * q1 + q3) * qm1
                                   + (1.0 - qn2 + 2.0 * q2 * q2) * qm2
                                   + 2.0 * (q2 * q3 - q1) * qm3);
        const double sd3 = 0.25 * (2.0 * (q3 * q1 - q2) * qm1
                                   + 2.0 * (q3 * q2 + q1) * qm2
                                   + (1.0 - qn2 + 2.0 * q3 * q3) * qm3);

        // full order: [rdot(3), vdot(3), sdot(3), wdot(3), Wdot(NW)]
        const std::vector<double> full = {
            x[3], x[4], x[5], B[0], B[1], B[2], sd1, sd2, sd3,
            B[3], B[4], B[5], B[6], B[7], B[8]};
        if (full.size() != A.size()) {
            std::cout << "SIZE MISMATCH " << full.size() << " vs " << A.size() << "\n";
            return 2;
        }
        double e2 = 0.0;
        for (int i = 0; i < NW + 12; ++i)
            e2 = std::max(e2, std::abs(A[i] - full[i]) / std::max(1.0, std::abs(full[i])));
        worst = std::max(worst, e2);
    }
    std::cout << "[GATE T1] tilted-wheel accelerations vs hand-derived reference: "
              << "max rel err = " << worst << "\n";
    if (!(worst < 1e-12)) { std::cout << "T1 FAILED\n"; return 1; }

    // --- |H| conservation with tilted wheels, u = 0 ------------------------
    // Conserved total under stock semantics: H = I*omega + sum Js*W*g.
    auto Hmag = [&](const std::vector<double>& x, const std::vector<double>& P) {
        Eigen::Vector3d omega(x[9], x[10], x[11]);
        Eigen::Matrix3d I;
        I << P[iO[0]], P[iO[3]], P[iO[4]], P[iO[3]], P[iO[1]], P[iO[5]],
             P[iO[4]], P[iO[5]], P[iO[2]];
        Eigen::Vector3d L = I * omega;
        for (int i = 0; i < NW; ++i)
            L += P[jO[i]] * x[12 + i] *
                 Eigen::Vector3d(axes[size_t(i)][0], axes[size_t(i)][1], axes[size_t(i)][2]);
        return L.norm();
    };
    std::vector<double> x = {1.0e7, 2.0e6, -1.5e6, -700.0, 5500.0, 3200.0,
                             0.10, -0.20, 0.30, 0.01, -0.02, 0.03,
                             100.0, 200.0, 300.0};
    std::vector<double> P0 = sys.default_pars();
    for (int i = 0; i < NW; ++i) P0[uidx[i]] = 0.0;
    const double H0 = Hmag(x, P0);
    std::vector<double> y = x;
    const double dt = 0.01;
    double worstH = 0.0;
    for (int k = 0; k < 500; ++k) {
        const std::vector<double> d = sys.state_derivative(y, P0);
        for (int i = 0; i < 12 + NW; ++i) y[i] += dt * d[i];
        const double s2 = y[6] * y[6] + y[7] * y[7] + y[8] * y[8];
        if (s2 > 1.0) { y[6] /= -s2; y[7] /= -s2; y[8] /= -s2; }
        worstH = std::max(worstH, std::abs(Hmag(y, P0) - H0) / H0);
    }
    std::cout << "[GATE T2] torque-free |H| conservation (tilted, Euler): "
              << worstH << "\n";
    if (!(worstH < 1e-5)) { std::cout << "T2 FAILED\n"; return 1; }

    std::cout << "\nTILTED-WHEEL GENERALISATION: PASS\n";
    return 0;
}
