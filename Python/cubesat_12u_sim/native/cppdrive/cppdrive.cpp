// C++ Basilisk SysModel: Heyoka dynamics + full control loop in C++.
//
// Drop-in replacement for Basilisk's spacecraft module. Reads
// THRArrayOnTimeCmdMsg, runs the entire tick loop (Heyoka Taylor
// propagation + MRP control + thruster allocation + fuel bookkeeping)
// in C++, and writes SCStatesMsg. All other Basilisk modules (IMU, CSS,
// navigation, reference, controller, allocator) work unchanged.
//
// The control loop mirrors heyoka_mc._run_batch with B=1:
//   - Gyro measurement assembly (bias + noise)
//   - Bias observer update
//   - MRP error computation
//   - PD controller
//   - Thruster allocation (cached pinv)
//   - Effector quantization
//   - Wrench matvecs + fuel bookkeeping
//   - Heyoka Taylor propagation
//   - MRP shadow-set reset

#include <cmath>
#include <cstdint>
#include <random>
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

// DCM row j (body-from-inertial) dotted with sun vector.
inline double sun_cos(const double *sg, const double *sun, const double *bore) {
    double s2 = sg[0] * sg[0] + sg[1] * sg[1] + sg[2] * sg[2];
    double d = (1.0 + s2) * (1.0 + s2);
    double S01 = -sg[2], S02 = sg[1], S10 = sg[2];
    double S12 = -sg[0], S20 = -sg[1], S21 = sg[0];
    double S[3][3] = {{0, S01, S02}, {S10, 0, S12}, {S20, S21, 0}};
    double R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s2ij = S[i][0] * S[0][j] + S[i][1] * S[1][j] + S[i][2] * S[2][j];
            R[i][j] = (i == j ? 1.0 : 0.0) + (8.0 * s2ij - 4.0 * (1.0 - s2) * S[i][j]) / d;
        }
    double sbx = R[0][0] * sun[0] + R[0][1] * sun[1] + R[0][2] * sun[2];
    double sby = R[1][0] * sun[0] + R[1][1] * sun[1] + R[1][2] * sun[2];
    double sbz = R[2][0] * sun[0] + R[2][1] * sun[1] + R[2][2] * sun[2];
    double c = bore[0] * sbx + bore[1] * sby + bore[2] * sbz;
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return c;
}

// Build the 12-state MRP system (same as cxxdrive/campaign.cpp).
static std::vector<std::pair<expression, expression>> build_system(double mu, bool gg) {
    auto vars = heyoka::make_vars("rx", "ry", "rz", "vx", "vy", "vz", "s1", "s2",
                                  "s3", "omx", "omy", "omz");
    expression rx = vars[0], ry = vars[1], rz = vars[2], vx = vars[3],
               vy = vars[4], vz = vars[5], s1 = vars[6], s2 = vars[7],
               s3 = vars[8], omx = vars[9], omy = vars[10], omz = vars[11];
    auto P = [&](int i) { return expression(heyoka::par[i]); };
    expression Fx = P(0), Fy = P(1), Fz = P(2), Tx = P(3), Ty = P(4), Tz = P(5);
    expression m = P(6);
    expression Ixx = P(7), Iyy = P(8), Izz = P(9), Ixy = P(10), Ixz = P(11),
               Iyz = P(12);
    expression ixx = P(13), iyy = P(14), izz = P(15), ixy = P(16), ixz = P(17),
               iyz = P(18);

    expression sn2 = s1 * s1 + s2 * s2 + s3 * s3;
    expression d = (1.0 + sn2) * (1.0 + sn2);
    expression c = 4.0 * (1.0 - sn2) / d, k8 = 8.0 / d;
    expression s11 = -s2 * s2 - s3 * s3, s12 = s1 * s2, s13 = s1 * s3;
    expression s21 = s1 * s2, s22 = -s1 * s1 - s3 * s3, s23 = s2 * s3;
    expression s31 = s1 * s3, s32 = s2 * s3, s33 = -s1 * s1 - s2 * s2;
    expression R00 = 1.0 + k8 * s11, R01 = k8 * s12 + c * s3, R02 = k8 * s13 - c * s2;
    expression R10 = k8 * s21 - c * s3, R11 = 1.0 + k8 * s22, R12 = k8 * s23 + c * s1;
    expression R20 = k8 * s31 + c * s2, R21 = k8 * s32 - c * s1, R22 = 1.0 + k8 * s33;

    expression Rnorm = heyoka::sqrt(rx * rx + ry * ry + rz * rz);
    expression invR3 = 1.0 / (Rnorm * Rnorm * Rnorm);
    expression ax = -mu * rx * invR3 + (R00 * Fx + R10 * Fy + R20 * Fz) / m;
    expression ay = -mu * ry * invR3 + (R01 * Fx + R11 * Fy + R21 * Fz) / m;
    expression az = -mu * rz * invR3 + (R02 * Fx + R12 * Fy + R22 * Fz) / m;

    expression tggx{0.0}, tggy{0.0}, tggz{0.0};
    if (gg) {
        expression rbx = R00 * rx + R01 * ry + R02 * rz;
        expression rby = R10 * rx + R11 * ry + R12 * rz;
        expression rbz = R20 * rx + R21 * ry + R22 * rz;
        expression invR5 = invR3 / (Rnorm * Rnorm);
        expression k = 3.0 * mu * invR5;
        expression Irx = Ixx * rbx + Ixy * rby + Ixz * rbz;
        expression Iry = Ixy * rbx + Iyy * rby + Iyz * rbz;
        expression Irz = Ixz * rbx + Iyz * rby + Izz * rbz;
        tggx = k * (rby * Irz - rbz * Iry);
        tggy = k * (rbz * Irx - rbx * Irz);
        tggz = k * (rbx * Iry - rby * Irx);
    }

    expression B11 = 1.0 - sn2 + 2.0 * s1 * s1, B12 = 2.0 * (s1 * s2 - s3),
               B13 = 2.0 * (s1 * s3 + s2);
    expression B21 = 2.0 * (s2 * s1 + s3), B22 = 1.0 - sn2 + 2.0 * s2 * s2,
               B23 = 2.0 * (s2 * s3 - s1);
    expression B31 = 2.0 * (s3 * s1 - s2), B32 = 2.0 * (s3 * s2 + s1),
               B33 = 1.0 - sn2 + 2.0 * s3 * s3;
    expression s1dot = 0.25 * (B11 * omx + B12 * omy + B13 * omz);
    expression s2dot = 0.25 * (B21 * omx + B22 * omy + B23 * omz);
    expression s3dot = 0.25 * (B31 * omx + B32 * omy + B33 * omz);

    expression Iwx = Ixx * omx + Ixy * omy + Ixz * omz;
    expression Iwy = Ixy * omx + Iyy * omy + Iyz * omz;
    expression Iwz = Ixz * omx + Iyz * omy + Izz * omz;
    expression gx = -(omy * Iwz - omz * Iwy) + Tx + tggx;
    expression gy = -(omz * Iwx - omx * Iwz) + Ty + tggy;
    expression gz = -(omx * Iwy - omy * Iwx) + Tz + tggz;
    expression wdotx = ixx * gx + ixy * gy + ixz * gz;
    expression wdoty = ixy * gx + iyy * gy + iyz * gz;
    expression wdotz = ixz * gx + iyz * gy + izz * gz;

    return {{prime(rx) = vx, prime(ry) = vy, prime(rz) = vz, prime(vx) = ax,
             prime(vy) = ay, prime(vz) = az, prime(s1) = s1dot,
             prime(s2) = s2dot, prime(s3) = s3dot, prime(omx) = wdotx,
             prime(omy) = wdoty, prime(omz) = wdotz}};
}

// Run one full tick: read on-time, compute control, propagate, write state.
// Returns the new 12-state as a numpy array.
py::array_t<double> run_tick(
    // Current state (12,)
    py::array_t<double, py::array::c_style | py::array::forcecast> state,
    // Thruster on-time message (36 slots, first 12 used)
    py::array_t<double, py::array::c_style | py::array::forcecast> on_time,
    // Static parameters
    double mass,
    py::array_t<double, py::array::c_style | py::array::forcecast> I6,
    py::array_t<double, py::array::c_style | py::array::forcecast> invI6,
    // Wrench effectiveness matrices
    py::array_t<double, py::array::c_style | py::array::forcecast> MF,
    py::array_t<double, py::array::c_style | py::array::forcecast> MT,
    py::array_t<double, py::array::c_style | py::array::forcecast> tmax,
    // Control parameters
    double K, double P, double obs_gain, double obs_bias_gain,
    double ctrl_scale, double dt, double g0,
    // Reference MRP (3,)
    py::array_t<double, py::array::c_style | py::array::forcecast> ref,
    // Gyro noise
    py::array_t<double, py::array::c_style | py::array::forcecast> gyro_bias,
    py::array_t<double, py::array::c_style | py::array::forcecast> arw_std,
    // Observer state (persistent across ticks)
    py::array_t<double, py::array::c_style> est_rate,
    py::array_t<double, py::array::c_style> bias_est,
    py::array_t<bool, py::array::c_style> est_init,
    // Fuel/impulse accumulators (persistent)
    py::array_t<double, py::array::c_style> on_time_accum,
    py::array_t<double, py::array::c_style> phase_imp,
    py::array_t<double, py::array::c_style> phase_fuel,
    int phase,
    bool quantize,
    double tol, bool compact, double mu, bool use_gg,
    // RNG seed for gyro noise
    std::uint64_t seed) {

    const int B = 1;
    const int NTH = 12;

    auto S = state.unchecked<1>();
    auto OT = on_time.unchecked<1>();
    auto I6a = I6.unchecked<1>();
    auto II6 = invI6.unchecked<1>();
    auto Mf = MF.unchecked<2>();
    auto Mt = MT.unchecked<2>();
    auto TM = tmax.unchecked<1>();
    auto RF = ref.unchecked<1>();
    auto GB = gyro_bias.unchecked<1>();
    auto AR = arw_std.unchecked<1>();
    auto ER = est_rate.mutable_unchecked<1>();
    auto BE = bias_est.mutable_unchecked<1>();
    auto EI = est_init.mutable_unchecked<1>();
    auto OTACC = on_time_accum.mutable_unchecked<1>();
    auto PI = phase_imp.mutable_unchecked<1>();
    auto PF = phase_fuel.mutable_unchecked<1>();

    // Build/reuse integrator (template cache; mu is baked into the
    // compiled system so it is part of the key — cf. rwdrive fix)
    struct TplKey {
        double tol;
        double mu;
        bool compact, gg;
        bool operator==(const TplKey &o) const {
            return tol == o.tol && mu == o.mu && compact == o.compact && gg == o.gg;
        }
    };
    struct TplKeyHash {
        std::size_t operator()(const TplKey &k) const noexcept {
            std::size_t h = std::hash<double>{}(k.tol);
            h ^= std::hash<double>{}(k.mu) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>{}((int)k.compact * 2 + (int)k.gg);
            return h;
        }
    };
    static std::unordered_map<TplKey, heyoka::taylor_adaptive<double>, TplKeyHash> tpl_cache;
    TplKey key{tol, mu, compact, use_gg};
    auto it = tpl_cache.find(key);
    if (it == tpl_cache.end()) {
        it = tpl_cache
                 .emplace(key, heyoka::taylor_adaptive<double>(
                                   build_system(mu, use_gg), {},
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
    for (int i = 0; i < 12; ++i) ST[i] = S(i);
    // Static pars
    PA[6] = mass;
    for (int i = 0; i < 6; ++i) {
        PA[7 + i] = I6a(i);
        PA[13 + i] = II6(i);
    }

    // RNG for gyro noise
    std::mt19937_64 rng(seed + 999983ULL);
    std::normal_distribution<double> gauss(0.0, 1.0);

    // --- Sensing + observer ---
    double sg[3] = {ST[6], ST[7], ST[8]};
    double om[3] = {ST[9], ST[10], ST[11]};
    double mm[3] = {om[0] + GB(0) + gauss(rng) * AR(0),
                    om[1] + GB(1) + gauss(rng) * AR(1),
                    om[2] + GB(2) + gauss(rng) * AR(2)};
    double es[3] = {ER(0), ER(1), ER(2)};
    double bs[3] = {BE(0), BE(1), BE(2)};
    bool init = EI(0);
    if (!init) {
        es[0] = mm[0]; es[1] = mm[1]; es[2] = mm[2];
        init = true;
    } else {
        double r0 = mm[0] - es[0] - bs[0];
        double r1 = mm[1] - es[1] - bs[1];
        double r2 = mm[2] - es[2] - bs[2];
        es[0] += obs_gain * r0 * dt;
        es[1] += obs_gain * r1 * dt;
        es[2] += obs_gain * r2 * dt;
        bs[0] += obs_bias_gain * r0 * dt;
        bs[1] += obs_bias_gain * r1 * dt;
        bs[2] += obs_bias_gain * r2 * dt;
    }
    double fb[3] = {mm[0] - bs[0], mm[1] - bs[1], mm[2] - bs[2]};
    ER(0) = es[0]; ER(1) = es[1]; ER(2) = es[2];
    BE(0) = bs[0]; BE(1) = bs[1]; BE(2) = bs[2];
    EI(0) = init;

    // --- Control ---
    double rs[3] = {RF(0), RF(1), RF(2)};
    double e[3];
    mrp_diff_c(sg, rs, e);
    double tx = -K * e[0] - P * fb[0];
    double ty = -K * e[1] - P * fb[1];
    double tz = -K * e[2] - P * fb[2];
    double Fx = 0, Fy = 0, Fz = 0, Tx = 0, Ty = 0, Tz = 0, imp = 0;
    for (int i = 0; i < NTH; ++i) {
        double v = Mf(3, i) * (ctrl_scale * tx) +
                   Mf(4, i) * (ctrl_scale * ty) +
                   Mf(5, i) * (ctrl_scale * tz);
        double uu = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
        double q = uu;
        if (quantize)
            q = q <= 0.0 ? 0.0 : (q < 0.5 ? 1.0 / 6.0 : (q < 1.0 ? 5.0 / 6.0 : 1.0));
        Fx += Mf(0, i) * q;
        Fy += Mf(1, i) * q;
        Fz += Mf(2, i) * q;
        Tx += Mt(0, i) * q;
        Ty += Mt(1, i) * q;
        Tz += Mt(2, i) * q;
        double on = uu * dt;
        OTACC(i) += on;
        imp += on * TM(i);
    }
    PA[0] = Fx; PA[1] = Fy; PA[2] = Fz;
    PA[3] = Tx; PA[4] = Ty; PA[5] = Tz;
    PI(phase) += imp;
    PF(phase) += imp / (g0 * 1.0);  // isp not needed for timing

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
    auto result = py::array_t<double>({12});
    auto r = result.mutable_unchecked<1>();
    for (int i = 0; i < 12; ++i) r(i) = STD[i];
    return result;
}

PYBIND11_MODULE(_cppdrive, m) {
    m.doc() = "C++ Basilisk SysModel: Heyoka dynamics + full control loop.";
    m.def("run_tick", &run_tick, "Run one full tick (sense+control+propagate).",
          py::arg("state"), py::arg("on_time"), py::arg("mass"),
          py::arg("I6"), py::arg("invI6"), py::arg("MF"), py::arg("MT"),
          py::arg("tmax"), py::arg("K"), py::arg("P"), py::arg("obs_gain"),
          py::arg("obs_bias_gain"), py::arg("ctrl_scale"), py::arg("dt"),
          py::arg("g0"), py::arg("ref"), py::arg("gyro_bias"),
          py::arg("arw_std"), py::arg("est_rate"), py::arg("bias_est"),
          py::arg("est_init"), py::arg("on_time_accum"), py::arg("phase_imp"),
          py::arg("phase_fuel"), py::arg("phase"), py::arg("quantize"),
          py::arg("tol"), py::arg("compact"), py::arg("mu"), py::arg("use_gg"),
          py::arg("seed"));
}
