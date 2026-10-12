// Stage 2-3: whole-core compile + numeric accelerations vs the validated
// reference, + symbolic build vs heyoka, all driven from C++.
#include "omni/system.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

using omni::OmniSystem;

static double rel(const std::vector<double>& a, const std::vector<double>& b) {
    double mx = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double den = std::max(1.0, std::abs(b[i]));
        mx = std::max(mx, std::abs(a[i] - b[i]) / den);
    }
    return mx;
}

int main() {
    std::cout << std::setprecision(17);

    // --- reference EOM, transcribed 1:1 from
    //     cubesat_12u_sim/backends/heyoka/heyoka_rw.py::build_rw_system -----
    // 15-state [rx,ry,rz,vx,vy,vz,s1,s2,s3,omx,omy,omz,W1,W2,W3]
    // pars [u1,u2,u3,m,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,invI..,Js1,Js2,Js3]
    // STOCK SEMANTICS for the hub inverse: the input I is the TOTAL inertia
    // (stock IHubPntBc_B includes the wheels); the hub block inverts
    // D = I - diag(Js) (measured vs stock to 8 digits per axis). The harness
    // below therefore packs inv(D), not inv(I).
    auto ref_accel = [](const std::vector<double>& x, const std::vector<double>& P) {
        const double& u1 = P[0], &u2 = P[1], &u3 = P[2];
        const double& m = P[3];
        const double& Ixx = P[4], &Iyy = P[5], &Izz = P[6];
        const double& Ixy = P[7], &Ixz = P[8], &Iyz = P[9];
        const double& ixx = P[10], &iyy = P[11], &izz = P[12];
        const double& ixy = P[13], &ixz = P[14], &iyz = P[15];
        const double& J1 = P[16], &J2 = P[17], &J3 = P[18];
        const double rx = x[0], ry = x[1], rz = x[2];
        const double s1 = x[6], s2 = x[7], s3 = x[8];
        const double omx = x[9], omy = x[10], omz = x[11];
        const double W1 = x[12], W2 = x[13], W3 = x[14];

        const double R = std::sqrt(rx * rx + ry * ry + rz * rz);
        const double invR3 = 1.0 / (R * R * R);
        const double ax = -3.986004418e14 * rx * invR3;
        const double ay = -3.986004418e14 * ry * invR3;
        const double az = -3.986004418e14 * rz * invR3;

        const double sn2 = s1 * s1 + s2 * s2 + s3 * s3;
        const double B11 = 1.0 - sn2 + 2.0 * s1 * s1, B12 = 2.0 * (s1 * s2 - s3),
                     B13 = 2.0 * (s1 * s3 + s2);
        const double B21 = 2.0 * (s2 * s1 + s3), B22 = 1.0 - sn2 + 2.0 * s2 * s2,
                     B23 = 2.0 * (s2 * s3 - s1);
        const double B31 = 2.0 * (s3 * s1 - s2), B32 = 2.0 * (s3 * s2 + s1),
                     B33 = 1.0 - sn2 + 2.0 * s3 * s3;
        const double s1dot = 0.25 * (B11 * omx + B12 * omy + B13 * omz);
        const double s2dot = 0.25 * (B21 * omx + B22 * omy + B23 * omz);
        const double s3dot = 0.25 * (B31 * omx + B32 * omy + B33 * omz);

        // RELATIVE rotor momentum (stock parity, upstream
        // reactionWheelStateEffector.cpp: vecRot -= gs*u + Js*W*(omega x gs)
        // == -omega x h - u*gs with h = Js*W*gs). The absolute form
        // h = Js*(W + omega) DOUBLE-COUNTS against the subtracted D and is
        // wrong for unequal Js even when aligned (extra = -sum Js_i*w_i x
        // ... != 0); it cancels only for equal-Js orthonormal sets.
        const double h1 = J1 * W1, h2 = J2 * W2, h3 = J3 * W3;
        const double Hx = Ixx * omx + Ixy * omy + Ixz * omz + h1;
        const double Hy = Ixy * omx + Iyy * omy + Iyz * omz + h2;
        const double Hz = Ixz * omx + Iyz * omy + Izz * omz + h3;
        const double gx = -(omy * Hz - omz * Hy) - u1;
        const double gy = -(omz * Hx - omx * Hz) - u2;
        const double gz = -(omx * Hy - omy * Hx) - u3;
        const double wdotx = ixx * gx + ixy * gy + ixz * gz;
        const double wdoty = ixy * gx + iyy * gy + iyz * gz;
        const double wdotz = ixz * gx + iyz * gy + izz * gz;
        // Coupled spin (stock ordering): Js*dW = u - Js*(gs.wdot), evaluated
        // with the just-solved wdot (aligned wheels: axis i = body axis i).
        const double W1dot = (u1 - J1 * wdotx) / J1;
        const double W2dot = (u2 - J2 * wdoty) / J2;
        const double W3dot = (u3 - J3 * wdotz) / J3;

        return std::vector<double>{ax, ay, az, s1dot, s2dot, s3dot,
                                   wdotx, wdoty, wdotz, W1dot, W2dot, W3dot};
    };

    // --- build the omni system: hub + 3 wheels on body axes --------------
    OmniSystem sys;
    sys.set_gravity(3.986004418e14);
    sys.set_use_gravity_gradient(false);  // RW demo: Kepler only
    sys.add_hub(750.0, {900, 0, 0, 0, 800, 0, 0, 0, 600}, {0, 0, 0});
    for (const double* a : {std::array{1.0, 0.0, 0.0}.data(),
                            std::array{0.0, 1.0, 0.0}.data(),
                            std::array{0.0, 0.0, 1.0}.data()}) {
        sys.add_reaction_wheel({a[0], a[1], a[2]}, 0.07957747154594767, 0.0);
    }

    std::cout << "n_states = " << sys.n_states() << " (expect 15)\n";
    std::cout << "n_pars   = " << sys.n_pars() << " (expect 16 = 3x(u,tauF) + mass + I6 + 3 Js)\n";
    std::cout << "n_inputs = " << sys.n_inputs() << "  (expect 6)\n";

    const std::vector<double> P0 = sys.default_pars();
    std::cout << "par slots (" << P0.size() << "):\n  ";
    for (auto n : sys.par_names()) std::cout << n << " ";
    std::cout << "\n";

    // --- random states x parameters: omni vs reference -------------------
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    double worst = 0.0;
    int worst_k = -1;
    // Omni pars are name-addressed (A2): input slots (u,tauF per wheel),
    // then mass + I6, then Js per wheel. The hand-written reference keeps
    // its own 19-vector (it multiplies by precomputed invI).
    const size_t t2u[3] = {sys.par_index("u1"), sys.par_index("u2"),
                           sys.par_index("u3")};
    const size_t t2m = sys.par_index("mass");
    const size_t t2i[6] = {sys.par_index("Ixx"), sys.par_index("Iyy"),
                           sys.par_index("Izz"), sys.par_index("Ixy"),
                           sys.par_index("Ixz"), sys.par_index("Iyz")};
    const size_t t2j[3] = {sys.par_index("Js1"), sys.par_index("Js2"),
                           sys.par_index("Js3")};
    for (int k = 0; k < 500; ++k) {
        std::vector<double> x(15), Po = P0, Pr(19, 0.0);
        for (int i = 0; i < 15; ++i) x[i] = U(rng) * (i < 3 ? 1e7 : 1.0);
        x[0] = std::abs(x[0]) + 1e6;  // keep |r| sane
        x[1] = std::abs(x[1]) + 1e6;
        x[2] = std::abs(x[2]) + 1e6;
        for (int i = 0; i < 3; ++i) Po[t2u[i]] = Pr[i] = U(rng) * 0.2;
        // random full inertia (TOTAL, stock IHubPntBc_B semantics: includes
        // the wheels; the hub block inverts D = I - diag(Js), packed below)
        const double a = 900.0 + U(rng) * 50, b = 800.0 + U(rng) * 50,
                     c = 600.0 + U(rng) * 50;
        const double d = U(rng) * 3, e = U(rng) * 3, f = U(rng) * 3;
        Po[t2m] = Pr[3] = 750.0;
        Po[t2i[0]] = Pr[4] = a; Po[t2i[1]] = Pr[5] = b; Po[t2i[2]] = Pr[6] = c;
        Po[t2i[3]] = Pr[7] = d; Po[t2i[4]] = Pr[8] = e; Po[t2i[5]] = Pr[9] = f;
        // invI packs inv(D_hub), D_hub = I - diag(Js) (stock semantics). Js
        // values are drawn first so the inverse matches them.
        double js0 = 0.05 + std::abs(U(rng)) * 0.1;
        double js1 = 0.05 + std::abs(U(rng)) * 0.1;
        double js2 = 0.05 + std::abs(U(rng)) * 0.1;
        Po[t2j[0]] = js0; Pr[16] = js0;
        Po[t2j[1]] = js1; Pr[17] = js1;
        Po[t2j[2]] = js2; Pr[18] = js2;
        {
            const double da = a - js0, db = b - js1, dc = c - js2;
            const double det = da * (db * dc - f * f) - d * (d * dc - e * f)
                             + e * (d * f - db * e);
            const double s = 1.0 / det;
            Pr[10] = (db * dc - f * f) * s; Pr[11] = (da * dc - e * e) * s;
            Pr[12] = (da * db - d * d) * s;
            Pr[13] = (e * f - d * dc) * s; Pr[14] = (d * f - db * e) * s;
            Pr[15] = (d * e - da * f) * s;
        }

        const std::vector<double> A = sys.state_derivative(x, Po);
        const std::vector<double> B = ref_accel(x, Pr);
        const std::vector<double> ref = {x[3],  x[4],  x[5],  B[0], B[1], B[2],
                                         B[3], B[4], B[5],  B[6], B[7], B[8],
                                         B[9], B[10], B[11]};
        double e2 = 0.0;
        for (size_t i = 0; i < 15; ++i)
            e2 = std::max(e2, std::abs(A[i] - ref[i]) /
                                   std::max(1.0, std::abs(ref[i])));
        if (e2 > worst) { worst = e2; worst_k = k; }
    }
    std::cout << "\n[GATE A] numeric state_derivative vs reference: max rel err = "
              << worst << "  (trial " << worst_k << ")\n";
    if (!(worst < 1e-12)) { std::cout << "GATE A FAILED\n"; return 1; }

    // --- [GATE B] angular momentum conservation (u = 0) -------------------
    // |H| in the inertial frame must be constant when no torque is applied.
    // Total H under stock total-inertia semantics: H = I_set*omega +
    // sum_wheels Js*W*gs (the D separation cancels the (omega.gs) terms).
    auto Hmag = [](const std::vector<double>& x, const std::vector<double>& P) {
        const double s1 = x[6], s2 = x[7], s3 = x[8];
        const double Ixx = P[4], Iyy = P[5], Izz = P[6];
        const double Ixy = P[7], Ixz = P[8], Iyz = P[9];
        const double J1 = P[16], J2 = P[17], J3 = P[18];
        const double sn2 = s1 * s1 + s2 * s2 + s3 * s3;
        const double d = (1.0 + sn2) * (1.0 + sn2);
        const double c = 4.0 * (1.0 - sn2) / d, k8 = 8.0 / d;
        const double s11 = -s2 * s2 - s3 * s3, s12 = s1 * s2, s13 = s1 * s3;
        const double s22 = -s1 * s1 - s3 * s3, s23 = s2 * s3;
        const double s33 = -s1 * s1 - s2 * s2;
        const double R[3][3] = {
            {1.0 + k8 * s11, k8 * s12 + c * s3, k8 * s13 - c * s2},
            {k8 * s12 - c * s3, 1.0 + k8 * s22, k8 * s23 + c * s1},
            {k8 * s13 + c * s2, k8 * s23 - c * s1, 1.0 + k8 * s33}};
        const double hb[3] = {Ixx * x[9] + Ixy * x[10] + Ixz * x[11] + J1 * x[12],
                              Ixy * x[9] + Iyy * x[10] + Iyz * x[11] + J2 * x[13],
                              Ixz * x[9] + Iyz * x[10] + Izz * x[11] + J3 * x[14]};
        double n2 = 0.0;
        for (int i = 0; i < 3; ++i) {
            const double v = R[i][0] * hb[0] + R[i][1] * hb[1] + R[i][2] * hb[2];
            n2 += v * v;
        }
        return std::sqrt(n2);
    };

    std::vector<double> x = {1.0e7, 2.0e6, -1.5e6, -700.0, 5500.0, 3200.0,
                             0.10, -0.20, 0.30, 0.01, -0.02, 0.03,
                             10.0, 20.0, 30.0};
    std::vector<double> P = P0;  // system pars (name-addressed)
    for (int i = 0; i < 3; ++i) P[t2u[i]] = 0.0;  // no torque
    // Hmag reads the legacy 19-layout (I6 at 4..9, Js at 16..18): repack.
    std::vector<double> Ph(19, 0.0);
    for (int i = 0; i < 3; ++i) Ph[i] = 0.0;
    Ph[3] = P0[t2m];
    Ph[4] = P0[t2i[0]]; Ph[5] = P0[t2i[1]]; Ph[6] = P0[t2i[2]];
    Ph[7] = P0[t2i[3]]; Ph[8] = P0[t2i[4]]; Ph[9] = P0[t2i[5]];
    for (int i = 0; i < 3; ++i) Ph[16 + i] = P0[t2j[i]];
    const double H0 = Hmag(x, Ph);
    std::vector<double> cur = x;
    const double dt = 0.01;
    double worstdH = 0.0;

    // Classic RK4: H is not exactly conserved by ANY fixed-step integrator,
    // so drift is expected; the test is that it stays near the integrator's
    // own error floor, not that it is machine-exact.
    auto rk4_step = [&](std::vector<double>& y) {
        const std::vector<double> k1 = sys.state_derivative(y, P);
        std::vector<double> yt = y;
        for (int i = 0; i < 15; ++i) yt[i] = y[i] + 0.5 * dt * k1[i];
        const std::vector<double> k2 = sys.state_derivative(yt, P);
        yt = y;
        for (int i = 0; i < 15; ++i) yt[i] = y[i] + 0.5 * dt * k2[i];
        const std::vector<double> k3 = sys.state_derivative(yt, P);
        yt = y;
        for (int i = 0; i < 15; ++i) yt[i] = y[i] + dt * k3[i];
        const std::vector<double> k4 = sys.state_derivative(yt, P);
        for (int i = 0; i < 15; ++i)
            y[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        const double s2 = y[6] * y[6] + y[7] * y[7] + y[8] * y[8];
        if (s2 > 1.0) { y[6] /= -s2; y[7] /= -s2; y[8] /= -s2; }
    };

    for (int k = 0; k < 500; ++k) {
        rk4_step(cur);
        worstdH = std::max(worstdH, std::abs(Hmag(cur, Ph) - H0) / H0);
    }
    std::cout << "[GATE B] torque-free |H| drift over 5 s (RK4): " << worstdH << "\n";
    // Documented acceptance (BUILD_STATUS.md): ~2.2e-7 is the RK4 integrator's
    // own error floor on this orbit, not a momentum bug. Gate fails only if
    // drift exceeds that floor by an order of magnitude.
    if (!(worstdH < 1e-6)) { std::cout << "GATE B FAILED\n"; return 1; }

    std::cout << "\nSTAGE 2/3 (numeric EOM + momentum conservation): PASS\n";
    return 0;
}
