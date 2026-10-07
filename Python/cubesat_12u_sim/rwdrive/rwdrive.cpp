// C++ Basilisk SysModel: Heyoka 15-state MRP + reaction-wheel dynamics.
//
// Drop-in replacement for Basilisk's spacecraft + reactionWheelStateEffector
// modules in the scenarioAttitudeFeedbackRW demo. Reads motor torque commands,
// propagates the coupled hub+wheel dynamics via Heyoka Taylor integration,
// and writes SCStatesMsg. All other Basilisk modules (navigation, reference,
// controller, allocator) work unchanged.
//
// 15-state: [r(3), v(3), sigma(3), omega(3), Omega_wheels(3)]
// 19-par:   [u1,u2,u3, m, Ixx,Iyy,Izz,Ixy,Ixz,Iyz,
//            invIxx,invIyy,invIzz,invIxy,invIxz,invIyz, Js1,Js2,Js3]
//
// Dynamics (body axes = wheel axes, BalancedWheels model):
//   wheel:  Js_i * dW_i = u_i
//   hub:    I*dw = -w x (I*w + h) - u,  h_i = Js_i*(W_i + w_i)
//   MRP kinematics, Kepler translation (no thrust force).

#include <cmath>
#include <unordered_map>
#include <vector>

#include <heyoka/heyoka.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using heyoka::expression;
using heyoka::prime;

// ---- MRP helpers (same formulas as heyoka_attitude) ----
inline void mrp_diff_c(const double *b, const double *r, double *e) {
    double b2 = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    double r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    double c0 = b[1] * r[2] - b[2] * r[1];
    double c1 = b[2] * r[0] - b[0] * r[2];
    double c2 = b[0] * r[1] - b[1] * r[0];
    double den = 1.0 + b2 * r2 + 2.0 * (b[0] * r[0] + b[1] * r[1] + b[2] * r[2]);
    e[0] = (b[0] * (1.0 - r2) - r[0] * (1.0 - b2) + 2.0 * c0) / den;
    e[1] = (b[1] * (1.0 - r2) - r[1] * (1.0 - b2) + 2.0 * c1) / den;
    e[2] = (b[2] * (1.0 - r2) - r[2] * (1.0 - b2) + 2.0 * c2) / den;
    double n2 = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    if (n2 > 1.0) {
        e[0] = -e[0] / n2;
        e[1] = -e[1] / n2;
        e[2] = -e[2] / n2;
    }
}

// Build the 15-state MRP + RW system (same as heyoka_rw.build_rw_system).
static std::vector<std::pair<expression, expression>> build_rw_system(double mu) {
    auto vars = heyoka::make_vars("rx", "ry", "rz", "vx", "vy", "vz",
                                  "s1", "s2", "s3", "omx", "omy", "omz",
                                  "W1", "W2", "W3");
    expression rx = vars[0], ry = vars[1], rz = vars[2];
    expression vx = vars[3], vy = vars[4], vz = vars[5];
    expression s1 = vars[6], s2 = vars[7], s3 = vars[8];
    expression omx = vars[9], omy = vars[10], omz = vars[11];
    expression W1 = vars[12], W2 = vars[13], W3 = vars[14];

    auto P = [&](int i) { return expression(heyoka::par[i]); };
    expression u1 = P(0), u2 = P(1), u3 = P(2);
    // P(3) is mass: kept in the 19-par layout for ABI compat with
    // pack_rw_pars, but unused (RW-only demo has no body thrust).
    expression Ixx = P(4), Iyy = P(5), Izz = P(6);
    expression Ixy = P(7), Ixz = P(8), Iyz = P(9);
    expression ixx = P(10), iyy = P(11), izz = P(12);
    expression ixy = P(13), ixz = P(14), iyz = P(15);
    expression J1 = P(16), J2 = P(17), J3 = P(18);

    // NOTE: no MRP->DCM here on purpose (translation is pure Kepler,
    // no body thrust). Building it only bloats the Taylor graph / JIT.
    expression sn2 = s1 * s1 + s2 * s2 + s3 * s3;

    // --- Translation: point-mass Kepler, no thrust force ---
    expression Rnorm = heyoka::sqrt(rx * rx + ry * ry + rz * rz);
    expression invR3 = 1.0 / (Rnorm * Rnorm * Rnorm);
    expression ax = -mu * rx * invR3;
    expression ay = -mu * ry * invR3;
    expression az = -mu * rz * invR3;

    // --- MRP kinematics ---
    expression B11 = 1.0 - sn2 + 2.0 * s1 * s1, B12 = 2.0 * (s1 * s2 - s3),
               B13 = 2.0 * (s1 * s3 + s2);
    expression B21 = 2.0 * (s2 * s1 + s3), B22 = 1.0 - sn2 + 2.0 * s2 * s2,
               B23 = 2.0 * (s2 * s3 - s1);
    expression B31 = 2.0 * (s3 * s1 - s2), B32 = 2.0 * (s3 * s2 + s1),
               B33 = 1.0 - sn2 + 2.0 * s3 * s3;
    expression s1dot = 0.25 * (B11 * omx + B12 * omy + B13 * omz);
    expression s2dot = 0.25 * (B21 * omx + B22 * omy + B23 * omz);
    expression s3dot = 0.25 * (B31 * omx + B32 * omy + B33 * omz);

    // --- Coupled hub + wheel dynamics ---
    // wheel momenta about spin axes (absolute): h_i = J_i*(W_i + om_i)
    expression h1 = J1 * (W1 + omx);
    expression h2 = J2 * (W2 + omy);
    expression h3 = J3 * (W3 + omz);
    // total angular momentum (hub + wheels)
    expression Hx = Ixx * omx + Ixy * omy + Ixz * omz + h1;
    expression Hy = Ixy * omx + Iyy * omy + Iyz * omz + h2;
    expression Hz = Ixz * omx + Iyz * omy + Izz * omz + h3;
    // hub Euler with wheel reaction -u (motor torque on wheels is +u)
    expression gx = -(omy * Hz - omz * Hy) - u1;
    expression gy = -(omz * Hx - omx * Hz) - u2;
    expression gz = -(omx * Hy - omy * Hx) - u3;
    expression wdotx = ixx * gx + ixy * gy + ixz * gz;
    expression wdoty = ixy * gx + iyy * gy + iyz * gz;
    expression wdotz = ixz * gx + iyz * gy + izz * gz;
    // wheel speeds: J_i dW_i = u_i
    expression W1dot = u1 / J1;
    expression W2dot = u2 / J2;
    expression W3dot = u3 / J3;

    return {{prime(rx) = vx, prime(ry) = vy, prime(rz) = vz,
             prime(vx) = ax, prime(vy) = ay, prime(vz) = az,
             prime(s1) = s1dot, prime(s2) = s2dot, prime(s3) = s3dot,
             prime(omx) = wdotx, prime(omy) = wdoty, prime(omz) = wdotz,
             prime(W1) = W1dot, prime(W2) = W2dot, prime(W3) = W3dot}};
}

// Run one tick: set motor torques, propagate dt, return new 15-state.
// The control loop (PD + allocation) runs in Basilisk Python; this C++ function
// only handles the dynamics propagation.
py::array_t<double> run_rw_tick(
    // Current state (15,)
    py::array_t<double, py::array::c_style | py::array::forcecast> state,
    // Motor torque command (3,)
    py::array_t<double, py::array::c_style | py::array::forcecast> u_motor,
    // Static parameters
    double mass,
    py::array_t<double, py::array::c_style | py::array::forcecast> I6,
    py::array_t<double, py::array::c_style | py::array::forcecast> invI6,
    py::array_t<double, py::array::c_style | py::array::forcecast> Js,
    // Propagation
    double dt,
    double tol, bool compact, double mu, double u_max = 0.2) {

    auto S = state.unchecked<1>();
    auto U = u_motor.unchecked<1>();
    auto I6a = I6.unchecked<1>();
    auto II6 = invI6.unchecked<1>();
    auto JS = Js.unchecked<1>();

    // Stock reactionWheelStateEffector saturates motor torque at +-u_max
    // (HR16: 0.2 Nm) internally; the logged motor message is the
    // allocator's unsaturated output. The Python mirror clips in
    // rw_allocate; the C++ tick must clip here too, else uncapped
    // torques drive a positive-feedback blowup in closed loop.
    double u0 = U(0) < -u_max ? -u_max : (U(0) > u_max ? u_max : U(0));
    double u1 = U(1) < -u_max ? -u_max : (U(1) > u_max ? u_max : U(1));
    double u2 = U(2) < -u_max ? -u_max : (U(2) > u_max ? u_max : U(2));

    // Build/reuse integrator (template cache; mu baked into the
    // compiled system so it is part of the key).
    struct TplKey {
        double tol;
        double mu;
        bool compact;
        bool operator==(const TplKey &o) const {
            return tol == o.tol && mu == o.mu && compact == o.compact;
        }
    };
    struct TplKeyHash {
        std::size_t operator()(const TplKey &k) const noexcept {
            std::size_t h = std::hash<double>{}(k.tol);
            h ^= std::hash<double>{}(k.mu) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>{}((int)k.compact);
            return h;
        }
    };
    static std::unordered_map<TplKey, heyoka::taylor_adaptive<double>, TplKeyHash> tpl_cache;
    TplKey key{tol, mu, compact};
    auto it = tpl_cache.find(key);
    if (it == tpl_cache.end()) {
        it = tpl_cache
                 .emplace(key, heyoka::taylor_adaptive<double>(
                                   build_rw_system(mu), {},
                                   heyoka::kw::tol = tol,
                                   heyoka::kw::compact_mode = compact,
                                   heyoka::kw::high_accuracy = false))
                 .first;
    }
    heyoka::taylor_adaptive<double> &ta = it->second;
    ta.set_time(0.0);
    double *ST = ta.get_state_data();
    double *PA = ta.get_pars_data();

    // Init state
    for (int i = 0; i < 15; ++i) ST[i] = S(i);
    // Motor torques (updated per tick, saturated)
    PA[0] = u0; PA[1] = u1; PA[2] = u2;
    // Static pars
    PA[3] = mass;
    for (int i = 0; i < 6; ++i) {
        PA[4 + i] = I6a(i);
        PA[10 + i] = II6(i);
    }
    PA[16] = JS(0); PA[17] = JS(1); PA[18] = JS(2);

    // --- Propagate ---
    ta.propagate_for(dt);

    // --- Shadow reset ---
    double *STD = ta.get_state_data();
    double n2 = STD[6] * STD[6] + STD[7] * STD[7] + STD[8] * STD[8];
    if (n2 > 1.0) {
        STD[6] /= -n2;
        STD[7] /= -n2;
        STD[8] /= -n2;
    }

    // --- Return new state ---
    auto result = py::array_t<double>({15}); // braces: scalar-init warning is benign
    auto r = result.mutable_unchecked<1>();
    for (int i = 0; i < 15; ++i) r(i) = STD[i];
    return result;
}

// Full closed-loop RW mission in one C++ call (no per-tick Python).
//
// Mirrors heyoka_rw.run_rw_mirror exactly:
//   e = mrp_diff(sigma, 0); Lr = -K*e - P*omega; u = clip(-Lr, +-u_max)
// with truth navigation, direct wheel allocation, and the same MRP
// shadow-set reset. One pybind call runs all n_steps ticks, so the
// 600 s demo pays zero per-tick Python/Basilisk-scheduler overhead.
// This is the "fully C++ heyoka module" counterpart to the per-tick
// run_rw_tick used inside the Basilisk pipeline.
py::dict run_rw_mission(
    py::array_t<double, py::array::c_style | py::array::forcecast> y0,
    double mass,
    py::array_t<double, py::array::c_style | py::array::forcecast> I6,
    py::array_t<double, py::array::c_style | py::array::forcecast> invI6,
    py::array_t<double, py::array::c_style | py::array::forcecast> Js,
    double dt, int n_steps, double K, double P, double u_max,
    double tol, bool compact, double mu) {

    auto Y0 = y0.unchecked<1>();
    auto I6a = I6.unchecked<1>();
    auto II6 = invI6.unchecked<1>();
    auto JS = Js.unchecked<1>();

    struct TplKey2 {
        double tol, mu;
        bool compact;
        bool operator==(const TplKey2 &o) const {
            return tol == o.tol && mu == o.mu && compact == o.compact;
        }
    };
    struct TplKeyHash2 {
        std::size_t operator()(const TplKey2 &k) const noexcept {
            std::size_t h = std::hash<double>{}(k.tol);
            h ^= std::hash<double>{}(k.mu) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>{}((int)k.compact);
            return h;
        }
    };
    static std::unordered_map<TplKey2, heyoka::taylor_adaptive<double>, TplKeyHash2> tpl_cache2;
    TplKey2 key{tol, mu, compact};
    auto it = tpl_cache2.find(key);
    if (it == tpl_cache2.end()) {
        it = tpl_cache2
                 .emplace(key, heyoka::taylor_adaptive<double>(
                                   build_rw_system(mu), {},
                                   heyoka::kw::tol = tol,
                                   heyoka::kw::compact_mode = compact,
                                   heyoka::kw::high_accuracy = false))
                 .first;
    }
    heyoka::taylor_adaptive<double> &ta = it->second;
    ta.set_time(0.0);
    double *ST = ta.get_state_data();
    double *PA = ta.get_pars_data();
    for (int i = 0; i < 15; ++i) ST[i] = Y0(i);
    PA[3] = mass;
    for (int i = 0; i < 6; ++i) {
        PA[4 + i] = I6a(i);
        PA[10 + i] = II6(i);
    }
    PA[16] = JS(0); PA[17] = JS(1); PA[18] = JS(2);

    const int N = n_steps;
    std::vector<double> Y((size_t)15 * (N + 1)), U((size_t)3 * (N + 1)), T(N + 1);
    for (int i = 0; i < 15; ++i) Y[(size_t)i * (N + 1) + 0] = ST[i];
    T[0] = 0.0;

    const double zero[3] = {0.0, 0.0, 0.0};
    for (int k = 1; k <= N; ++k) {
        double sg[3] = {ST[6], ST[7], ST[8]};
        double om[3] = {ST[9], ST[10], ST[11]};
        double e[3];
        mrp_diff_c(sg, zero, e);
        double Lr[3] = {-K * e[0] - P * om[0],
                        -K * e[1] - P * om[1],
                        -K * e[2] - P * om[2]};
        double u[3];
        for (int i = 0; i < 3; ++i) {
            double v = -Lr[i];
            u[i] = v < -u_max ? -u_max : (v > u_max ? u_max : v);
        }
        PA[0] = u[0]; PA[1] = u[1]; PA[2] = u[2];
        ta.propagate_for(dt);
        double *STD = ta.get_state_data();
        double n2 = STD[6] * STD[6] + STD[7] * STD[7] + STD[8] * STD[8];
        if (n2 > 1.0) {
            STD[6] /= -n2;
            STD[7] /= -n2;
            STD[8] /= -n2;
        }
        for (int i = 0; i < 15; ++i) Y[(size_t)i * (N + 1) + k] = STD[i];
        for (int i = 0; i < 3; ++i) U[(size_t)i * (N + 1) + k] = u[i];
        T[k] = ta.get_time();
    }

    py::dict out;
    auto to_arr = [](const std::vector<double> &v, std::vector<py::ssize_t> shape) {
        py::array_t<double> a(shape);
        std::copy(v.begin(), v.end(), static_cast<double *>(a.mutable_data()));
        return a;
    };
    out["Y"] = to_arr(Y, {15, (py::ssize_t)N + 1});
    out["u_motor"] = to_arr(U, {3, (py::ssize_t)N + 1});
    out["t"] = to_arr(T, {(py::ssize_t)N + 1});
    out["dt"] = dt;
    return out;
}

PYBIND11_MODULE(_rwdrive, m) {
    m.doc() = "C++ Basilisk SysModel: Heyoka 15-state MRP + RW dynamics.";
    m.def("run_rw_tick", &run_rw_tick,
          "Run one dynamics tick (set motor torques, propagate, return state).",
          py::arg("state"), py::arg("u_motor"), py::arg("mass"),
          py::arg("I6"), py::arg("invI6"), py::arg("Js"),
          py::arg("dt"), py::arg("tol"), py::arg("compact"), py::arg("mu"),
          py::arg("u_max") = 0.2);
    m.def("run_rw_mission", &run_rw_mission,
          "Run the full closed-loop RW mission in C++ (one call, no per-tick Python).",
          py::arg("y0"), py::arg("mass"), py::arg("I6"), py::arg("invI6"),
          py::arg("Js"), py::arg("dt"), py::arg("n_steps"), py::arg("K"),
          py::arg("P"), py::arg("u_max"), py::arg("tol"), py::arg("compact"),
          py::arg("mu"));
}
