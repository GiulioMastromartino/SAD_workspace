// Batched NMPC shooting cost + gradient on the Heyoka C++ Taylor path.
//
// Same energy cost as nmpc.ScipyShootingBackend (deadbanded semi-major error,
// radial-velocity damping, fuel, x5 terminal emphasis), but the rollout runs
// on heyoka::taylor_adaptive_batch<double> — the same C++ integrator family
// as cxxdrive/campaign.cpp — instead of numpy RK4.
//
// Layout follows campaign.cpp: process-local template cache keyed by lane
// count (JIT ~7 s cold, ~0.17 s warm from disk cache, ~0.01 s mem-cache),
// state/pars buffers filled via get_state_data()/get_pars_data(), one
// propagate_for(dt) per horizon stage.
//
// Two entry points because scipy calls func/jac separately:
//   cost_batch(X0, Z, ...) -> costs[B]          (B lanes, usually B=1)
//   grad_batch(x0, z0, ...) -> grad[3N]         (forward differences, computed
//       as ONE expanded batch of 3N+1 lanes, not 3N+1 Python round-trips)
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <heyoka/heyoka.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using heyoka::expression;
using heyoka::prime;

// 6-state Kepler + ECI accel pars (same as heyoka_propagate.build_orbit_system).
static std::vector<std::pair<expression, expression>> build_system(double mu) {
    auto vars = heyoka::make_vars("rx", "ry", "rz", "vx", "vy", "vz");
    expression rx = vars[0], ry = vars[1], rz = vars[2], vx = vars[3],
               vy = vars[4], vz = vars[5];
    expression ax = expression(heyoka::par[0]), ay = expression(heyoka::par[1]),
               az = expression(heyoka::par[2]);
    expression Rnorm = heyoka::sqrt(rx * rx + ry * ry + rz * rz);
    expression invR3 = 1.0 / (Rnorm * Rnorm * Rnorm);
    return {{prime(rx) = vx, prime(ry) = vy, prime(rz) = vz,
             prime(vx) = -mu * rx * invR3 + ax,
             prime(vy) = -mu * ry * invR3 + ay,
             prime(vz) = -mu * rz * invR3 + az}};
}

struct TplKey {
    int lanes;
    std::uint64_t tolbits;
    bool operator==(const TplKey &o) const {
        return lanes == o.lanes && tolbits == o.tolbits;
    }
};
struct TplKeyHash {
    std::size_t operator()(const TplKey &k) const noexcept {
        std::size_t h = std::hash<int>{}(k.lanes);
        h ^= std::hash<std::uint64_t>{}(k.tolbits) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

static heyoka::taylor_adaptive_batch<double> &batch_template(int lanes, double tol,
                                                             double mu) {
    static std::unordered_map<TplKey, heyoka::taylor_adaptive_batch<double>,
                              TplKeyHash>
        cache;
    TplKey key{lanes, 0};
    static_assert(sizeof(double) == sizeof(std::uint64_t), "");
    std::uint64_t tb = 0;
    __builtin_memcpy(&tb, &tol, sizeof(tb));
    key.tolbits = tb;
    auto it = cache.find(key);
    if (it == cache.end()) {
        it = cache
                 .emplace(key, heyoka::taylor_adaptive_batch<double>(
                                   build_system(mu), {}, (std::uint32_t)lanes,
                                   heyoka::kw::tol = tol,
                                   heyoka::kw::compact_mode = true,
                                   heyoka::kw::high_accuracy = false))
                 .first;
    }
    return it->second;
}

inline double semi_major_axis_c(const double *r, const double *v, double mu) {
    double rn = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    if (rn < 1.0) rn = 1.0;
    double denom = 2.0 / rn - (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / mu;
    if (std::fabs(denom) < 1e-15) return 1e30;
    return 1.0 / denom;
}

struct CostCfg {
    double dt, q_pos, q_vel, r_fuel, db_a, a_max;
    double mu, r_orbit, tol;
    int N;
};

// Stage-cost accumulation for one propagated stage across all lanes.
// r/v point at var-major state data (ST[0*B+j]..ST[5*B+j]).
inline void accumulate_stage(const double *ST, int lanes,
                             const double * const *Ustage,  // per-lane (3,) or nullptr
                             const CostCfg &c, double *costs, bool terminal_only) {
    static const double EA = 50e3, EVR = 50.0;
    double a_scale = c.a_max > 1e-12 ? c.a_max : 1e-12;
    for (int j = 0; j < lanes; ++j) {
        double r[3] = {ST[0 * lanes + j], ST[1 * lanes + j], ST[2 * lanes + j]};
        double v[3] = {ST[3 * lanes + j], ST[4 * lanes + j], ST[5 * lanes + j]};
        double e_a = semi_major_axis_c(r, v, c.mu) - c.r_orbit;
        if (std::fabs(e_a) < c.db_a) e_a = 0.0;
        if (terminal_only) {
            double e_term = e_a / EA;
            costs[j] += 5.0 * c.q_pos * e_term * e_term;
            continue;
        }
        double rn = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        if (rn < 1.0) rn = 1.0;
        double v_r = (v[0] * r[0] + v[1] * r[1] + v[2] * r[2]) / rn;
        const double *uk = Ustage[j];
        double a_n2 = (uk[0] * uk[0] + uk[1] * uk[1] + uk[2] * uk[2]) /
                      (a_scale * a_scale);
        costs[j] += c.q_pos * (e_a / EA) * (e_a / EA) +
                    c.q_vel * (v_r / EVR) * (v_r / EVR) + c.r_fuel * a_n2;
    }
}

// Roll out B lanes for N stages; Uall[lane][3N] row-major.
// Per-stage accels are written into the batch pars before each propagate.
static void rollout_cost(int lanes, const double *X0, const double *Uall,
                         const CostCfg &c, double *costs) {
    auto &ta = batch_template(lanes, c.tol, c.mu);
    ta.set_time(0.0);
    double *ST = ta.get_state_data();
    double *PA = ta.get_pars_data();
    for (int j = 0; j < lanes; ++j) {
        for (int i = 0; i < 6; ++i) ST[i * lanes + j] = X0[j * 6 + i];
        costs[j] = 0.0;
    }
    std::vector<const double *> Ustage(lanes);
    for (int k = 0; k < c.N; ++k) {
        for (int j = 0; j < lanes; ++j) {
            Ustage[j] = Uall + (j * c.N + k) * 3;
            PA[0 * lanes + j] = Ustage[j][0];
            PA[1 * lanes + j] = Ustage[j][1];
            PA[2 * lanes + j] = Ustage[j][2];
        }
        ta.propagate_for(c.dt);
        accumulate_stage(ta.get_state_data(), lanes, Ustage.data(), c, costs, false);
    }
    // terminal energy emphasis (uses final state, no extra propagate)
    accumulate_stage(ta.get_state_data(), lanes, Ustage.data(), c, costs, true);
}

// Z (B,3N) row-major, X0 (B,6); out costs (B,).
void cost_batch(py::array_t<double, py::array::c_style | py::array::forcecast> X0,
                py::array_t<double, py::array::c_style | py::array::forcecast> Z,
                double dt, int N, double q_pos, double q_vel, double r_fuel,
                double db_a, double a_max, double norm_penalty, double mu,
                double r_orbit, double tol,
                py::array_t<double, py::array::c_style> out) {
    (void)norm_penalty;  // hard norm constraint lives in scipy, not in the cost
    auto X = X0.unchecked<2>(), ZZ = Z.unchecked<2>();
    auto O = out.mutable_unchecked<1>();
    const int B = (int)ZZ.shape(0);
    CostCfg c{dt, q_pos, q_vel, r_fuel, db_a, a_max, mu, r_orbit, tol, N};
    std::vector<double> Xv(B * 6), Uv(B * (size_t)3 * N);
    for (int b = 0; b < B; ++b) {
        for (int i = 0; i < 6; ++i) Xv[b * 6 + i] = X(b, i);
        for (int i = 0; i < 3 * N; ++i) Uv[b * 3 * N + i] = ZZ(b, i);
    }
    std::vector<double> costs(B);
    rollout_cost(B, Xv.data(), Uv.data(), c, costs.data());
    for (int b = 0; b < B; ++b) O(b) = costs[b];
}

// Forward-difference gradient at z0 (3N,) via ONE expanded batch of 3N+1
// lanes (base + one perturbation per var). Out grad (3N,).
void grad_batch(py::array_t<double, py::array::c_style | py::array::forcecast> x0,
                py::array_t<double, py::array::c_style | py::array::forcecast> z0,
                double dt, int N, double q_pos, double q_vel, double r_fuel,
                double db_a, double a_max, double norm_penalty, double mu,
                double r_orbit, double tol,
                py::array_t<double, py::array::c_style> out) {
    (void)norm_penalty;
    auto X = x0.unchecked<1>(), Z = z0.unchecked<1>();
    auto O = out.mutable_unchecked<1>();
    CostCfg c{dt, q_pos, q_vel, r_fuel, db_a, a_max, mu, r_orbit, tol, N};
    const int n = 3 * N;
    const int L = n + 1;
    double h = 1e-7 * (a_max > 1e-12 ? a_max : 1e-12);
    if (h < 1e-12) h = 1e-12;
    std::vector<double> Xv(L * 6), Uv(L * (size_t)n);
    for (int l = 0; l < L; ++l)
        for (int i = 0; i < 6; ++i) Xv[l * 6 + i] = X(i);
    for (int i = 0; i < n; ++i) Uv[i] = Z(i);  // lane 0 = base
    for (int p = 0; p < n; ++p) {              // lanes 1..n = perturbations
        for (int i = 0; i < n; ++i) Uv[(p + 1) * n + i] = Z(i);
        Uv[(p + 1) * n + p] += h;
    }
    std::vector<double> costs(L);
    rollout_cost(L, Xv.data(), Uv.data(), c, costs.data());
    for (int p = 0; p < n; ++p) O(p) = (costs[p + 1] - costs[0]) / h;
}

PYBIND11_MODULE(_nmpcbatch, m) {
    m.doc() = "Batched NMPC shooting cost + gradient on the Heyoka C++ path.";
    m.def("cost_batch", &cost_batch, "Shooting costs for B candidates.",
          py::arg("X0"), py::arg("Z"), py::arg("dt"), py::arg("N"),
          py::arg("q_pos"), py::arg("q_vel"), py::arg("r_fuel"),
          py::arg("db_a"), py::arg("a_max"), py::arg("norm_penalty"),
          py::arg("mu"), py::arg("r_orbit"), py::arg("tol"), py::arg("out"));
    m.def("grad_batch", &grad_batch, "Forward-diff gradient at z0.",
          py::arg("x0"), py::arg("z0"), py::arg("dt"), py::arg("N"),
          py::arg("q_pos"), py::arg("q_vel"), py::arg("r_fuel"),
          py::arg("db_a"), py::arg("a_max"), py::arg("norm_penalty"),
          py::arg("mu"), py::arg("r_orbit"), py::arg("tol"), py::arg("out"));
}
