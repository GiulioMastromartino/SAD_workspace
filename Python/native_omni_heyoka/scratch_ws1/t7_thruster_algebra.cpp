// Gate T1 (algebra): OmniSystem thruster channel vs an independent reference
// transcribed from cubesat_12u_sim/core/actuators.py::jets_wrench_quantized
// (staircase {0,1/6,5/6,1}) + point-mass Kepler + full-inertia gravity
// gradient, over random states and random duty patterns.
//
// The reference is deliberately NOT shared code: any sign/frame error in the
// contributor (F_B/T_B assembly, F_N = R^T F_B, GG torque) shows up here.
#include "omni/system.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

namespace {
// --- independent reference ------------------------------------------------
// DCM body-from-inertial, validated entry-by-entry against
// Basilisk.utilities.RigidBodyKinematics.MRP2C.
void dcm_from_mrp(const double s[3], double R[3][3]) {
    const double s1 = s[0], s2 = s[1], s3 = s[2];
    const double sn2 = s1 * s1 + s2 * s2 + s3 * s3;
    const double d = (1.0 + sn2) * (1.0 + sn2);
    const double c = 4.0 * (1.0 - sn2) / d, k8 = 8.0 / d;
    const double S[3][3] = {{0, -s3, s2}, {s3, 0, -s1}, {-s2, s1, 0}};
    double S2[3][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) S2[i][j] += S[i][k] * S[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = (i == j ? 1.0 : 0.0) + k8 * S2[i][j] - c * S[i][j];
}

double stair(double u) {
    if (u <= 0.0) return 0.0;
    if (u < 0.5) return 1.0 / 6.0;
    if (u < 1.0) return 5.0 / 6.0;
    return 1.0;
}

// 12U bus geometry (cubesat_12u_sim.core.config).
const double DIRS[12][3] = {{-1,0,0},{-1,0,0},{1,0,0},{1,0,0},
                            {0,0,-1},{0,0,-1},{0,0,1},{0,0,1},
                            {0,-1,0},{0,-1,0},{0,1,0},{0,1,0}};
const double POS[12][3] = {{0.1,0.1,0},{0.1,-0.1,0},{-0.1,0.1,0},{-0.1,-0.1,0},
                           {0.1,0,0.15},{-0.1,0,0.15},{0.1,0,-0.15},{-0.1,0,-0.15},
                           {0,0.1,0.15},{0,0.1,-0.15},{0,-0.1,0.15},{0,-0.1,-0.15}};
constexpr double MASS = 20.0;
constexpr double IXX = 0.21666667, IYY = 0.21666667, IZZ = 0.13333333;
constexpr double MU = 398600441800000.0;

// Full state derivative [v, a, sdot, wdot] for hub-only + thrusters.
void ref_derivative(const double x[12], const double q[12], double out[12]) {
    double F[3] = {}, T[3] = {};
    for (int i = 0; i < 12; ++i) {
        const double f = stair(q[i]);  // Tmax = 1 N
        for (int k = 0; k < 3; ++k) F[k] += f * DIRS[i][k];
        const double Fx = f * DIRS[i][0], Fy = f * DIRS[i][1], Fz = f * DIRS[i][2];
        T[0] += POS[i][1] * Fz - POS[i][2] * Fy;
        T[1] += POS[i][2] * Fx - POS[i][0] * Fz;
        T[2] += POS[i][0] * Fy - POS[i][1] * Fx;
    }
    const double r[3] = {x[0], x[1], x[2]};
    const double s[3] = {x[6], x[7], x[8]};
    const double w[3] = {x[9], x[10], x[11]};
    double R[3][3];
    dcm_from_mrp(s, R);
    // F_N = R^T F_B
    double FN[3] = {};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) FN[i] += R[k][i] * F[k];
    const double rn = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    const double invR3 = 1.0 / (rn * rn * rn);
    double a[3];
    for (int i = 0; i < 3; ++i) a[i] = -MU * r[i] * invR3 + FN[i] / MASS;
    // GG torque: Tgg = k (rB x (I rB)), rB = R r
    double rB[3] = {};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) rB[i] += R[i][k] * r[k];
    const double IrB[3] = {IXX * rB[0], IYY * rB[1], IZZ * rB[2]};
    const double k = 3.0 * MU / (rn * rn * rn * rn * rn);
    const double Tgg[3] = {k * (rB[1] * IrB[2] - rB[2] * IrB[1]),
                           k * (rB[2] * IrB[0] - rB[0] * IrB[2]),
                           k * (rB[0] * IrB[1] - rB[1] * IrB[0])};
    const double Iw[3] = {IXX * w[0], IYY * w[1], IZZ * w[2]};
    const double wxIw[3] = {w[1]*Iw[2] - w[2]*Iw[1], w[2]*Iw[0] - w[0]*Iw[2],
                            w[0]*Iw[1] - w[1]*Iw[0]};
    const double gx = -wxIw[0] + T[0] + Tgg[0];
    const double gy = -wxIw[1] + T[1] + Tgg[1];
    const double gz = -wxIw[2] + T[2] + Tgg[2];
    const double wdot[3] = {gx / IXX, gy / IYY, gz / IZZ};
    // MRP kinematics
    const double sn2 = s[0]*s[0] + s[1]*s[1] + s[2]*s[2];
    const double B11 = 1-sn2+2*s[0]*s[0], B12 = 2*(s[0]*s[1]-s[2]), B13 = 2*(s[0]*s[2]+s[1]);
    const double B21 = 2*(s[1]*s[0]+s[2]), B22 = 1-sn2+2*s[1]*s[1], B23 = 2*(s[1]*s[2]-s[0]);
    const double B31 = 2*(s[2]*s[0]-s[1]), B32 = 2*(s[2]*s[1]+s[0]), B33 = 1-sn2+2*s[2]*s[2];
    const double sd[3] = {0.25*(B11*w[0]+B12*w[1]+B13*w[2]),
                          0.25*(B21*w[0]+B22*w[1]+B23*w[2]),
                          0.25*(B31*w[0]+B32*w[1]+B33*w[2])};
    for (int i = 0; i < 3; ++i) {
        out[i] = x[3 + i];
        out[3 + i] = a[i];
        out[6 + i] = sd[i];
        out[9 + i] = wdot[i];
    }
}
}  // namespace

int main() {
    std::cout << std::setprecision(17);
    omni::OmniSystem sys;
    sys.set_gravity(MU);
    sys.set_use_gravity_gradient(true);
    sys.add_hub(MASS, {IXX,0,0, 0,IYY,0, 0,0,IZZ}, {0,0,0});
    std::vector<std::array<double,3>> d, p;
    std::vector<double> tm(12, 1.0);
    for (int i = 0; i < 12; ++i) {
        d.push_back({DIRS[i][0], DIRS[i][1], DIRS[i][2]});
        p.push_back({POS[i][0], POS[i][1], POS[i][2]});
    }
    sys.add_thrusters(d, p, tm);
    std::cout << "states=" << sys.n_states() << " (expect 12), pars=" << sys.n_pars()
              << " (expect 23 = 12 q + swirl + mass + I6 + 3 cB)\n";

    std::mt19937_64 rng(777);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    double worst = 0.0;
    int worst_k = -1;
    for (int k = 0; k < 500; ++k) {
        std::vector<double> x(12), pars = sys.default_pars();
        x[0] = 0.9e7 + std::abs(U(rng)) * 0.3e7;
        x[1] = U(rng) * 0.4e7;
        x[2] = U(rng) * 0.4e7;
        for (int i = 3; i < 6; ++i) x[i] = U(rng) * 6000.0;
        for (int i = 6; i < 9; ++i) x[i] = U(rng) * 0.4;  // |sigma| < 1
        for (int i = 9; i < 12; ++i) x[i] = U(rng) * 0.05;
        double q[12];
        for (int i = 0; i < 12; ++i) {
            // duties spanning the staircase edges, incl. exact edges.
            // NOTE: the quantizer runs at STEP TIME (fillInputs), not in the
            // EOM: pars carry already-quantized levels verbatim (design A3).
            // Mirror that boundary here: quantize once, hand levels to both.
            const double r = U(rng);
            const double raw = (k % 4 == 0) ? (r * 0.5) : (r < -0.5 ? 0.0 : (r < 0.0 ? 0.49 : (r < 0.5 ? 0.5 : 1.0)));
            q[i] = stair(raw);
            pars[size_t(i)] = q[i];
        }
        const std::vector<double> A = sys.state_derivative(x, pars);
        double B[12];
        double xa[12];
        for (int i = 0; i < 12; ++i) xa[i] = x[size_t(i)];
        ref_derivative(xa, q, B);
        double e = 0.0;
        for (size_t i = 0; i < 12; ++i)
            e = std::max(e, std::abs(A[i] - B[i]) / std::max(1.0, std::abs(B[i])));
        if (e > worst) { worst = e; worst_k = k; }
    }
    std::cout << "[GATE T1] thruster state_derivative vs independent reference: max rel err = "
              << worst << " (trial " << worst_k << ")\n";
    if (!(worst < 1e-12)) { std::cout << "GATE T1 FAILED\n"; return 1; }
    std::cout << "GATE T1: PASS\n";
    return 0;
}
