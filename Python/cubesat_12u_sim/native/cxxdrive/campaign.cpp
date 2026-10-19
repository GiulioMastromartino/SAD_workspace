// Full closed-loop Monte Carlo chunk driver on the Heyoka C++ API.
//
// Same 12-state MRP system, same PD/observer/allocator/quantizer/fuel logic
// as heyoka_attitude + heyoka_mc, but the whole tick loop (including Taylor
// stepping) runs in C++: per tick Python makes exactly ONE call here.
// Dispersions come pre-sampled from Python (single source of truth);
// per-tick gyro noise uses C++ <random> (streams differ from numpy, so
// parity vs the batch backend is distributional, validated on fuel means).
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
    // S^2 rows needed for R rows dotted with sun: compute full S then S2.
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

// Build the 12-state MRP system (variable order matches heyoka_attitude:
// rx,ry,rz,vx,vy,vz,s1,s2,s3,omx,omy,omz; 19 pars like pack_pars).
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

py::dict run_chunk(
    // per-lane dispersions, row-major
    py::array_t<double, py::array::c_style | py::array::forcecast> r0,
    py::array_t<double, py::array::c_style | py::array::forcecast> v0,
    py::array_t<double, py::array::c_style | py::array::forcecast> sigma0,
    py::array_t<double, py::array::c_style | py::array::forcecast> omega0,
    py::array_t<double, py::array::c_style | py::array::forcecast> mass,
    py::array_t<double, py::array::c_style | py::array::forcecast> I6,
    py::array_t<double, py::array::c_style | py::array::forcecast> invI6,
    py::array_t<double, py::array::c_style | py::array::forcecast> MF,
    py::array_t<double, py::array::c_style | py::array::forcecast> MT,
    py::array_t<double, py::array::c_style | py::array::forcecast> tmax,
    py::array_t<double, py::array::c_style | py::array::forcecast> isp,
    py::array_t<double, py::array::c_style | py::array::forcecast> gyro_bias,
    py::array_t<double, py::array::c_style | py::array::forcecast> arw_std,
    py::array_t<std::uint64_t, py::array::c_style | py::array::forcecast> seeds,
    // shared
    py::array_t<double, py::array::c_style | py::array::forcecast> P_nom,
    py::array_t<double, py::array::c_style | py::array::forcecast> REFS,
    py::array_t<std::int64_t, py::array::c_style | py::array::forcecast> PH,
    py::array_t<double, py::array::c_style | py::array::forcecast> sun_dirs,
    py::array_t<double, py::array::c_style | py::array::forcecast> bore,
    double K, double P, double obs_gain, double obs_bias_gain,
    double ctrl_scale, double dt, int n_steps, int log_every, double g0,
    double rate_thresh, double slew_thresh_deg, double cone_deg, bool quantize,
    int quant_levels, double tol, bool compact, double mu, bool use_gg) {
    const int B = (int)r0.shape(0);
    const int NTH = 12;
    const int L = n_steps / log_every + 1;

    auto R0 = r0.unchecked<2>(), V0 = v0.unchecked<2>(), S0 = sigma0.unchecked<2>(),
         O0 = omega0.unchecked<2>();
    auto MA = mass.unchecked<1>();
    auto I6a = I6.unchecked<2>();
    auto II6 = invI6.unchecked<2>();
    auto Mf = MF.unchecked<3>();
    auto Mt = MT.unchecked<3>();
    auto TM = tmax.unchecked<2>();
    auto SP = isp.unchecked<1>();
    auto GB = gyro_bias.unchecked<2>(), AR = arw_std.unchecked<2>();
    auto SD = seeds.unchecked<1>();
    auto Pn = P_nom.unchecked<2>();
    auto RF = REFS.unchecked<3>();
    auto PHA = PH.unchecked<1>();
    auto SUN = sun_dirs.unchecked<2>();
    auto BO = bore.unchecked<1>();

    // Process-local template cache: same (B, tol, compact, gg) reuses the
    // compiled integrator across chunks (reset below); first build JITs once
    // (disk-cache shared across worker processes).
    struct TplKey {
        int B;
        double tol;
        bool compact, gg;
        bool operator==(const TplKey &o) const {
            return B == o.B && tol == o.tol && compact == o.compact && gg == o.gg;
        }
    };
    struct TplKeyHash {
        std::size_t operator()(const TplKey &k) const noexcept {
            std::size_t h = std::hash<int>{}(k.B);
            h ^= std::hash<double>{}(k.tol) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>{}((int)k.compact * 2 + (int)k.gg);
            return h;
        }
    };
    static std::unordered_map<TplKey, heyoka::taylor_adaptive_batch<double>, TplKeyHash> tpl_cache;
    TplKey key{B, tol, compact, use_gg};
    auto it = tpl_cache.find(key);
    if (it == tpl_cache.end()) {
        it = tpl_cache
                 .emplace(key, heyoka::taylor_adaptive_batch<double>(
                                   build_system(mu, use_gg), {},
                                   (std::uint32_t)B, heyoka::kw::tol = tol,
                                   heyoka::kw::compact_mode = compact,
                                   heyoka::kw::high_accuracy = false))
                 .first;
    }
    heyoka::taylor_adaptive_batch<double> &ta = it->second;
    ta.set_time(0.0);
    double *ST = ta.get_state_data();
    double *PA = ta.get_pars_data();
    // init state (var-major) and static pars
    for (int j = 0; j < B; ++j) {
        ST[0 * B + j] = R0(j, 0);
        ST[1 * B + j] = R0(j, 1);
        ST[2 * B + j] = R0(j, 2);
        ST[3 * B + j] = V0(j, 0);
        ST[4 * B + j] = V0(j, 1);
        ST[5 * B + j] = V0(j, 2);
        ST[6 * B + j] = S0(j, 0);
        ST[7 * B + j] = S0(j, 1);
        ST[8 * B + j] = S0(j, 2);
        ST[9 * B + j] = O0(j, 0);
        ST[10 * B + j] = O0(j, 1);
        ST[11 * B + j] = O0(j, 2);
        PA[0 * B + j] = PA[1 * B + j] = PA[2 * B + j] = 0.0;
        PA[3 * B + j] = PA[4 * B + j] = PA[5 * B + j] = 0.0;
        PA[6 * B + j] = MA(j);
        for (int i = 0; i < 6; ++i) {
            PA[(7 + i) * B + j] = I6a(j, i);
            PA[(13 + i) * B + j] = II6(j, i);
        }
    }
    ta.set_time(0.0);

    std::vector<std::mt19937_64> rngs;
    for (int j = 0; j < B; ++j) rngs.emplace_back(SD(j) + 999983ULL);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::vector<double> est(B * 3, 0.0), bias(B * 3, 0.0);
    std::vector<char> einit(B, 0);
    std::vector<double> on_time(B * 12, 0.0), pimp(B * 3, 0.0), pfuel(B * 3, 0.0);
    std::vector<double> log_om((size_t)L * B), log_se((size_t)L * B),
        log_sun((size_t)L * B);
    std::vector<double> sg(3 * B), om(3 * B);

    int li = 0;
    for (int k = 0; k <= n_steps; ++k) {
        // read current state
        for (int j = 0; j < B; ++j) {
            sg[3 * j + 0] = ST[6 * B + j];
            sg[3 * j + 1] = ST[7 * B + j];
            sg[3 * j + 2] = ST[8 * B + j];
            om[3 * j + 0] = ST[9 * B + j];
            om[3 * j + 1] = ST[10 * B + j];
            om[3 * j + 2] = ST[11 * B + j];
        }
        if (k % log_every == 0) {
            for (int j = 0; j < B; ++j) {
                const double *sjj = &sg[3 * j];
                double e[3], rs[3] = {RF(k, j, 0), RF(k, j, 1), RF(k, j, 2)};
                mrp_diff_c(sjj, rs, e);
                double se = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
                double ox = om[3 * j], oy = om[3 * j + 1], oz = om[3 * j + 2];
                log_om[(size_t)li * B + j] = std::sqrt(ox * ox + oy * oy + oz * oz);
                log_se[(size_t)li * B + j] = se;
                double sun[3] = {SUN(li, 0), SUN(li, 1), SUN(li, 2)};
                double bo[3] = {BO(0), BO(1), BO(2)};
                log_sun[(size_t)li * B + j] = std::acos(sun_cos(sjj, sun, bo));
            }
            ++li;
        }
        if (k == n_steps) break;
        int ph = PHA(k);
        for (int j = 0; j < B; ++j) {
            double mm[3] = {om[3 * j] + GB(j, 0) + gauss(rngs[j]) * AR(j, 0),
                            om[3 * j + 1] + GB(j, 1) + gauss(rngs[j]) * AR(j, 1),
                            om[3 * j + 2] + GB(j, 2) + gauss(rngs[j]) * AR(j, 2)};
            double *es = &est[3 * j], *bs = &bias[3 * j];
            if (!einit[j]) {
                es[0] = mm[0];
                es[1] = mm[1];
                es[2] = mm[2];
                einit[j] = 1;
            } else {
                double r0 = mm[0] - es[0] - bs[0], r1 = mm[1] - es[1] - bs[1],
                       r2 = mm[2] - es[2] - bs[2];
                es[0] += obs_gain * r0 * dt;
                es[1] += obs_gain * r1 * dt;
                es[2] += obs_gain * r2 * dt;
                bs[0] += obs_bias_gain * r0 * dt;
                bs[1] += obs_bias_gain * r1 * dt;
                bs[2] += obs_bias_gain * r2 * dt;
            }
            double fb[3] = {mm[0] - bs[0], mm[1] - bs[1], mm[2] - bs[2]};
            double rs[3] = {RF(k, j, 0), RF(k, j, 1), RF(k, j, 2)};
            double sg_j[3] = {sg[3 * j], sg[3 * j + 1], sg[3 * j + 2]};
            double e[3];
            mrp_diff_c(sg_j, rs, e);
            double tx = -K * e[0] - P * fb[0], ty = -K * e[1] - P * fb[1],
                   tz = -K * e[2] - P * fb[2];
            double Fx = 0, Fy = 0, Fz = 0, Tx = 0, Ty = 0, Tz = 0, imp = 0;
            for (int i = 0; i < NTH; ++i) {
                double v = Pn(i, 3) * (ctrl_scale * tx) +
                           Pn(i, 4) * (ctrl_scale * ty) +
                           Pn(i, 5) * (ctrl_scale * tz);
                double uu = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
                double q = uu;
                if (quantize) {
                    if (quant_levels > 1) {
                        // Uniform mid-tread quantization with quant_levels
                        // levels spanning [0, 1]. This is a STUDY knob, not a
                        // hardware claim: it answers "how fine must the throttle
                        // resolution be for the residual floor to drop below the
                        // success threshold". quant_levels <= 1 keeps the legacy
                        // measured staircase below, so the default call
                        // reproduces committed results bit-identically.
                        double n = (double)(quant_levels - 1);
                        q = std::floor(uu * n + 0.5) / n;
                        if (q < 0.0) q = 0.0;
                        if (q > 1.0) q = 1.0;
                    } else {
                        // Measured Basilisk thrusterDynamicEffector behavior
                        // (TEST_Thruster, cutoffFrequency=1000,
                        // useMinPulseTime=False): commanded duty maps to
                        // achieved duty as {0, 1/6, 5/6, 1}. See
                        // actuators.quantize_duty_cycle.
                        q = q <= 0.0 ? 0.0 : (q < 0.5 ? 1.0 / 6.0 : (q < 1.0 ? 5.0 / 6.0 : 1.0));
                    }
                }
                Fx += Mf(j, 0, i) * q;
                Fy += Mf(j, 1, i) * q;
                Fz += Mf(j, 2, i) * q;
                Tx += Mt(j, 0, i) * q;
                Ty += Mt(j, 1, i) * q;
                Tz += Mt(j, 2, i) * q;
                double on = uu * dt;
                on_time[(size_t)j * NTH + i] += on;
                imp += on * TM(j, i);
            }
            PA[0 * B + j] = Fx;
            PA[1 * B + j] = Fy;
            PA[2 * B + j] = Fz;
            PA[3 * B + j] = Tx;
            PA[4 * B + j] = Ty;
            PA[5 * B + j] = Tz;
            pimp[(size_t)j * 3 + ph] += imp;
            pfuel[(size_t)j * 3 + ph] += imp / (SP(j) * g0);
        }
        ta.propagate_for(dt);
        // shadow reset
        double *STD = ta.get_state_data();
        for (int j = 0; j < B; ++j) {
            double n2 = STD[6 * B + j] * STD[6 * B + j] +
                        STD[7 * B + j] * STD[7 * B + j] +
                        STD[8 * B + j] * STD[8 * B + j];
            if (n2 > 1.0) {
                STD[6 * B + j] /= -n2;
                STD[7 * B + j] /= -n2;
                STD[8 * B + j] /= -n2;
            }
        }
    }

    py::dict out;
    auto to_arr = [](const std::vector<double> &v, std::vector<py::ssize_t> shape) {
        py::array_t<double> a(shape);
        std::copy(v.begin(), v.end(), static_cast<double *>(a.mutable_data()));
        return a;
    };
    out["omega_norm"] = to_arr(log_om, {(py::ssize_t)L, B});
    out["sigma_err"] = to_arr(log_se, {(py::ssize_t)L, B});
    out["sun_angle"] = to_arr(log_sun, {(py::ssize_t)L, B});
    out["on_time"] = to_arr(on_time, {B, NTH});
    out["phase_imp"] = to_arr(pimp, {B, 3});
    out["phase_fuel"] = to_arr(pfuel, {B, 3});
    return out;
}

PYBIND11_MODULE(_cxxdrive, m) {
    m.doc() = "Full C++ Heyoka campaign chunk driver.";
    m.def("run_chunk", &run_chunk, "Run one closed-loop batch chunk in C++.",
          py::arg("r0"), py::arg("v0"), py::arg("sigma0"), py::arg("omega0"),
          py::arg("mass"), py::arg("I6"), py::arg("invI6"), py::arg("MF"),
          py::arg("MT"), py::arg("tmax"), py::arg("isp"), py::arg("gyro_bias"),
          py::arg("arw_std"), py::arg("seeds"), py::arg("P_nom"), py::arg("REFS"),
          py::arg("PH"), py::arg("sun_dirs"), py::arg("bore"), py::arg("K"),
          py::arg("P"), py::arg("obs_gain"), py::arg("obs_bias_gain"),
          py::arg("ctrl_scale"), py::arg("dt"), py::arg("n_steps"),
          py::arg("log_every"), py::arg("g0"), py::arg("rate_thresh"),
          py::arg("slew_thresh_deg"), py::arg("cone_deg"), py::arg("quantize"),
          py::arg("quant_levels"), py::arg("tol"), py::arg("compact"), py::arg("mu"), py::arg("use_gg"));
}
