// Fused RTN-PID closed-loop batch evaluator (tuning surrogate in C++).
//
// Mirrors optimization/objective.py exactly: TunedOrbitPID control law +
// _rk4_step_orbit propagation + alloc_eff scaling + box-score. One call
// evaluates B candidates (one CMA-ES generation); per-lane gains differ,
// the initial state is shared.
//
// Conventions match fastloop.cpp: row-major plain loops, no Eigen, -O3
// friendly. Floating point is NOT bit-identical to numpy (different op
// order); parity is validated with tolerance in test_optimization.py.
#include <cmath>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace {

// ---- tiny 3-vector helpers ----
inline double dot3(const double *a, const double *b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline void cross3(const double *a, const double *b, double *o) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
inline double norm3(const double *a) { return std::sqrt(dot3(a, a)); }
inline double clip1(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Kepler accel at r (mu captured by caller).
inline void kepler_accel(const double *r, double mu, double *o) {
    double R = norm3(r);
    double invR3 = 1.0 / (R * R * R);
    o[0] = -mu * r[0] * invR3;
    o[1] = -mu * r[1] * invR3;
    o[2] = -mu * r[2] * invR3;
}

// One RK4 step for 6-state [r,v] with piecewise-constant ECI accel.
// Same tableau as nmpc._rk4_step_orbit.
inline void rk4_step(double *x, const double *a_eci, double dt, double mu) {
    double r0[3] = {x[0], x[1], x[2]}, v0[3] = {x[3], x[4], x[5]};
    double k1[6], k2[6], k3[6], k4[6], tmp[6], ak[3];

    kepler_accel(r0, mu, ak);
    k1[0] = v0[0]; k1[1] = v0[1]; k1[2] = v0[2];
    k1[3] = ak[0] + a_eci[0]; k1[4] = ak[1] + a_eci[1]; k1[5] = ak[2] + a_eci[2];
    for (int i = 0; i < 6; ++i) tmp[i] = x[i] + 0.5 * dt * k1[i];
    kepler_accel(tmp, mu, ak);
    k2[0] = tmp[3]; k2[1] = tmp[4]; k2[2] = tmp[5];
    k2[3] = ak[0] + a_eci[0]; k2[4] = ak[1] + a_eci[1]; k2[5] = ak[2] + a_eci[2];
    for (int i = 0; i < 6; ++i) tmp[i] = x[i] + 0.5 * dt * k2[i];
    kepler_accel(tmp, mu, ak);
    k3[0] = tmp[3]; k3[1] = tmp[4]; k3[2] = tmp[5];
    k3[3] = ak[0] + a_eci[0]; k3[4] = ak[1] + a_eci[1]; k3[5] = ak[2] + a_eci[2];
    for (int i = 0; i < 6; ++i) tmp[i] = x[i] + dt * k3[i];
    kepler_accel(tmp, mu, ak);
    k4[0] = tmp[3]; k4[1] = tmp[4]; k4[2] = tmp[5];
    k4[3] = ak[0] + a_eci[0]; k4[4] = ak[1] + a_eci[1]; k4[5] = ak[2] + a_eci[2];
    for (int i = 0; i < 6; ++i)
        x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

// Semi-major axis (vis-viva), same guards as nmpc.semi_major_axis.
inline double semi_major_axis(const double *r, const double *v, double mu) {
    double rn = norm3(r);
    if (rn < 1.0) rn = 1.0;
    double v2 = dot3(v, v);
    double denom = 2.0 / rn - v2 / mu;
    if (std::fabs(denom) < 1e-15) return 1e30;
    return 1.0 / denom;
}

}  // namespace

// Evaluate B PID candidates; params (B,9) physical values:
//   [kp_r, ki_r, kd_r, kp_t, ki_t, kp_n, ki_n, kd_n, box_r].
// out (B,4): [dv, max_abs_r_err, final_e_a, cost].
void pid_batch_eval(
    py::array_t<double, py::array::c_style | py::array::forcecast> params,
    py::array_t<double, py::array::c_style | py::array::forcecast> x0,
    double dt, int n_steps, double spec_box, double w_viol, double w_final,
    double alloc_eff, double mu, double r_orbit, double a_max,
    double int_clamp, double deadband,
    py::array_t<double, py::array::c_style> out) {
    auto P = params.unchecked<2>();
    auto X0 = x0.unchecked<1>();
    auto O = out.mutable_unchecked<2>();
    const int B = (int)P.shape(0);

    double x0v[6];
    for (int i = 0; i < 6; ++i) x0v[i] = X0(i);

    for (int j = 0; j < B; ++j) {
        const double kp_r = P(j, 0), ki_r = P(j, 1), kd_r = P(j, 2),
                     kp_t = P(j, 3), ki_t = P(j, 4), kp_n = P(j, 5),
                     ki_n = P(j, 6), kd_n = P(j, 7), box_r = P(j, 8);
        double x[6];
        for (int i = 0; i < 6; ++i) x[i] = x0v[i];
        double int_r = 0.0, int_t = 0.0, int_n = 0.0;
        double dv = 0.0, max_err = 0.0;

        for (int s = 0; s < n_steps; ++s) {
            const double *r = x, *v = x + 3;
            double r_norm = norm3(r);
            double a_cmd[3] = {0.0, 0.0, 0.0};

            bool coast = (box_r > 0.0 && std::fabs(r_norm - r_orbit) < box_r);
            if (coast) {
                int_r *= 0.9;
                int_t *= 0.9;
                int_n *= 0.9;
            } else {
                // RTN frame (controller.rtn_frame)
                double R[3] = {r[0] / r_norm, r[1] / r_norm, r[2] / r_norm};
                double h[3];
                cross3(r, v, h);
                double hn = norm3(h);
                double N[3] = {h[0] / hn, h[1] / hn, h[2] / hn};
                double T[3];
                cross3(N, R, T);
                double v_r = dot3(v, R), v_t = dot3(v, T), v_n = dot3(v, N);
                double r_n = dot3(r, N);
                double v_circ = std::sqrt(mu / r_norm);

                double e_r = r_orbit - r_norm;
                double pr = int_r, pt = int_t, pn = int_n;
                int_r = clip1(int_r + e_r * dt, -int_clamp, int_clamp);
                double a_R = kp_r * e_r + ki_r * int_r - kd_r * v_r;
                double e_t = v_circ - v_t;
                int_t = clip1(int_t + e_t * dt, -int_clamp, int_clamp);
                double a_T = kp_t * e_t + ki_t * int_t;
                double e_n = -r_n;
                int_n = clip1(int_n + e_n * dt, -int_clamp, int_clamp);
                double a_N = kp_n * e_n + ki_n * int_n - kd_n * v_n;

                double a_rtn[3] = {clip1(a_R, -a_max, a_max),
                                   clip1(a_T, -a_max, a_max),
                                   clip1(a_N, -a_max, a_max)};
                bool all_db = (std::fabs(a_rtn[0]) < deadband &&
                               std::fabs(a_rtn[1]) < deadband &&
                               std::fabs(a_rtn[2]) < deadband);
                if (all_db) {
                    int_r = pr;
                    int_t = pt;
                    int_n = pn;
                } else {
                    for (int i = 0; i < 3; ++i)
                        if (std::fabs(a_rtn[i]) < deadband) a_rtn[i] = 0.0;
                    for (int i = 0; i < 3; ++i)
                        a_cmd[i] = R[i] * a_rtn[0] + T[i] * a_rtn[1] + N[i] * a_rtn[2];
                }
            }

            double a_app[3] = {a_cmd[0] * alloc_eff, a_cmd[1] * alloc_eff,
                               a_cmd[2] * alloc_eff};
            double an = norm3(a_app);
            // finite guard (mirrors simulate's non-finite rejection)
            if (!(an < 1e300)) {
                dv = 1e300;
                max_err = 1e300;
                break;
            }
            dv += an * dt;
            rk4_step(x, a_app, dt, mu);
            double rn2 = norm3(x);
            bool bad = false;
            for (int i = 0; i < 6; ++i)
                if (!std::isfinite(x[i])) bad = true;
            if (bad) {
                dv = 1e300;
                max_err = 1e300;
                break;
            }
            double err = std::fabs(rn2 - r_orbit);
            if (err > max_err) max_err = err;
        }

        double e_a = (dv > 1e299)
                         ? 1e300
                         : semi_major_axis(x, x + 3, mu) - r_orbit;
        double excess = max_err / spec_box - 1.0;
        if (!(excess > 0.0)) excess = 0.0;
        double cost = (dv > 1e299) ? 1e300
                                  : dv + w_viol * excess * excess +
                                        w_final * (e_a / 50e3) * (e_a / 50e3);
        O(j, 0) = dv;
        O(j, 1) = max_err;
        O(j, 2) = e_a;
        O(j, 3) = cost;
    }
}

PYBIND11_MODULE(_pidbatch, m) {
    m.doc() = "Fused RTN-PID closed-loop batch evaluator (tuning surrogate).";
    m.def("pid_batch_eval", &pid_batch_eval,
          "Evaluate B PID candidates in one call.",
          py::arg("params"), py::arg("x0"), py::arg("dt"), py::arg("n_steps"),
          py::arg("spec_box"), py::arg("w_viol"), py::arg("w_final"),
          py::arg("alloc_eff"), py::arg("mu"), py::arg("r_orbit"),
          py::arg("a_max"), py::arg("int_clamp"), py::arg("deadband"),
          py::arg("out"));
}
