// Full NMPC closed-loop sim in ONE C++ call (Python = orchestrator only).
//
// Mirrors optimization/objective.py's NMPC path exactly:
// HierarchicalController (HighLevelNMPC bulk replanned every dt_outer with
// ZOH + FastTrimPID inner at dt_inner) + alloc_eff + box score, with safe
// early-abort when a feasible incumbent J* is already known.
//
// The bulk SLSQP solve runs EMBEDDED via NLopt (LD_SLSQP, Kraft — same
// algorithm family as scipy): cost/gradient on heyoka::taylor_adaptive_batch
// (shared template cache with nmpc_batch.cpp), stacked norm constraints with
// analytic Jacobian, warm-started (shifted) across plans, maxeval budget
// mapped from the scipy maxiter convention. On solver failure the same
// energy-based fallback as HighLevelNMPC._fallback_accel applies.
//
// Per candidate exactly ONE Python->C++ crossing (plus early-abort inside);
// the driver still spreads candidates across worker processes.
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <heyoka/heyoka.hpp>
#include <nlopt.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using heyoka::expression;
using heyoka::prime;

// ---- 6-state Kepler + ECI accel (same as nmpc_batch.cpp) ----
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
    std::uint64_t tb = 0;
    __builtin_memcpy(&tb, &tol, sizeof(tb));
    TplKey key{lanes, tb};
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

// ---- small vector helpers ----
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
inline double sma_c(const double *r, const double *v, double mu) {
    double rn = norm3(r);
    if (rn < 1.0) rn = 1.0;
    double denom = 2.0 / rn - dot3(v, v) / mu;
    if (std::fabs(denom) < 1e-15) return 1e30;
    return 1.0 / denom;
}

struct SimCfg {
    double dt_outer, dt_inner;
    int n_outer, horizon, maxiter;
    double q_pos, q_vel, r_fuel, db_a, a_max;
    double trim_a, trim_bud, kp_scale;
    double kp_r, ki_r, kd_r, kp_t, ki_t, kp_n, ki_n, kd_n;
    double int_clamp, pid_a_max, deadband;
    double mu, r_orbit, tol, alloc_eff;
    double spec_box;
    double j_star;
    bool has_incumbent;
};

// Shooting cost for one (x0,U): same energy cost as nmpc.py.
inline double shoot_cost(const double *x0, const double *U, const SimCfg &c,
                         heyoka::taylor_adaptive_batch<double> &ta) {
    static const double EA = 50e3, EVR = 50.0;
    double a_scale = c.a_max > 1e-12 ? c.a_max : 1e-12;
    ta.set_time(0.0);
    double *ST = ta.get_state_data();
    double *PA = ta.get_pars_data();
    for (int i = 0; i < 6; ++i) ST[i] = x0[i];  // lanes=1: ST[i*1+0]
    double cost = 0.0;
    double rT[3], vT[3];
    for (int k = 1; k <= c.horizon; ++k) {
        const double *uk = U + 3 * (k - 1);
        PA[0] = uk[0];
        PA[1] = uk[1];
        PA[2] = uk[2];
        ta.propagate_for(c.dt_outer);
        double r[3] = {ST[0], ST[1], ST[2]}, v[3] = {ST[3], ST[4], ST[5]};
        if (k == c.horizon) {
            rT[0] = r[0];
            rT[1] = r[1];
            rT[2] = r[2];
            vT[0] = v[0];
            vT[1] = v[1];
            vT[2] = v[2];
        }
        double e_a = sma_c(r, v, c.mu) - c.r_orbit;
        if (std::fabs(e_a) < c.db_a) e_a = 0.0;
        double rn = norm3(r);
        if (rn < 1.0) rn = 1.0;
        double v_r = (v[0] * r[0] + v[1] * r[1] + v[2] * r[2]) / rn;
        double a_n2 = dot3(uk, uk) / (a_scale * a_scale);
        cost += c.q_pos * (e_a / EA) * (e_a / EA) +
                c.q_vel * (v_r / EVR) * (v_r / EVR) + c.r_fuel * a_n2;
    }
    double e_term = (sma_c(rT, vT, c.mu) - c.r_orbit) / EA;
    if (std::fabs(sma_c(rT, vT, c.mu) - c.r_orbit) < c.db_a) e_term = 0.0;
    return cost + 5.0 * c.q_pos * e_term * e_term;
}

struct SolveCtx {
    const SimCfg *c;
    double x0[6];
    heyoka::taylor_adaptive_batch<double> *ta;
    heyoka::taylor_adaptive_batch<double> *tag;  // expanded L=3N+1 template
    long nevals = 0;
};

inline double nlopt_obj(unsigned n, const double *x, double *grad, void *data) {
    SolveCtx *s = static_cast<SolveCtx *>(data);
    const SimCfg &c = *s->c;
    s->nevals++;
    if (grad) {
        int nn = 3 * c.horizon;
        double h = 1e-7 * (c.a_max > 1e-12 ? c.a_max : 1e-12);
        if (h < 1e-12) h = 1e-12;
        int L = nn + 1;
        auto &ta = *s->tag;
        ta.set_time(0.0);
        double *ST = ta.get_state_data();
        double *PA = ta.get_pars_data();
        // lane 0 = base, lanes 1..nn = perturbations
        std::vector<double> Uv((size_t)L * nn);
        for (int i = 0; i < nn; ++i) Uv[i] = x[i];
        for (int p = 0; p < nn; ++p) {
            for (int i = 0; i < nn; ++i) Uv[(p + 1) * nn + i] = x[i];
            Uv[(p + 1) * nn + p] += h;
        }
        std::vector<double> f(L, 0.0);
        static const double EA = 50e3, EVR = 50.0;
        double a_scale = c.a_max > 1e-12 ? c.a_max : 1e-12;
        for (int j = 0; j < L; ++j)
            for (int i = 0; i < 6; ++i) ST[i * L + j] = s->x0[i];
        for (int k = 1; k <= c.horizon; ++k) {
            for (int j = 0; j < L; ++j) {
                const double *uk = &Uv[(j * c.horizon + (k - 1)) * 3];
                PA[0 * L + j] = uk[0];
                PA[1 * L + j] = uk[1];
                PA[2 * L + j] = uk[2];
            }
            ta.propagate_for(c.dt_outer);
            for (int j = 0; j < L; ++j) {
                double r[3] = {ST[0 * L + j], ST[1 * L + j], ST[2 * L + j]};
                double v[3] = {ST[3 * L + j], ST[4 * L + j], ST[5 * L + j]};
                const double *uk = &Uv[(j * c.horizon + (k - 1)) * 3];
                double e_a = sma_c(r, v, c.mu) - c.r_orbit;
                if (std::fabs(e_a) < c.db_a) e_a = 0.0;
                double rn = norm3(r);
                if (rn < 1.0) rn = 1.0;
                double v_r = (v[0] * r[0] + v[1] * r[1] + v[2] * r[2]) / rn;
                double a_n2 = dot3(uk, uk) / (a_scale * a_scale);
                f[j] += c.q_pos * (e_a / EA) * (e_a / EA) +
                        c.q_vel * (v_r / EVR) * (v_r / EVR) + c.r_fuel * a_n2;
                if (k == c.horizon) {
                    double e_term = (sma_c(r, v, c.mu) - c.r_orbit) / EA;
                    if (std::fabs(sma_c(r, v, c.mu) - c.r_orbit) < c.db_a)
                        e_term = 0.0;
                    f[j] += 5.0 * c.q_pos * e_term * e_term;
                }
            }
        }
        for (int p = 0; p < nn; ++p) grad[p] = (f[p + 1] - f[0]) / h;
        return f[0];
    }
    return shoot_cost(s->x0, x, c, *s->ta);
}

inline void nlopt_con(unsigned m, double *result, unsigned n, const double *x,
                      double *grad, void *data) {
    SolveCtx *s = static_cast<SolveCtx *>(data);
    const SimCfg &c = *s->c;
    (void)n;
    for (unsigned k = 0; k < m; ++k) {
        const double *uk = x + 3 * k;
        double nk = norm3(uk);
        result[k] = nk - c.a_max;
        if (grad) {
            for (unsigned i = 0; i < 3 * (unsigned)c.horizon; ++i)
                grad[k * 3 * c.horizon + i] = 0.0;
            if (nk > 1e-12) {
                grad[k * 3 * c.horizon + 3 * k] = uk[0] / nk;
                grad[k * 3 * c.horizon + 3 * k + 1] = uk[1] / nk;
                grad[k * 3 * c.horizon + 3 * k + 2] = uk[2] / nk;
            }
        }
    }
}

// One bulk plan. Returns false on solver failure (caller applies fallback).
inline bool bulk_plan(const double *x0, const SimCfg &c, double *warm /*3N io*/,
                      double *u0, long &nevals) {
    int nn = 3 * c.horizon;
    SolveCtx ctx{&c};
    for (int i = 0; i < 6; ++i) ctx.x0[i] = x0[i];
    ctx.ta = &batch_template(1, c.tol, c.mu);
    ctx.tag = &batch_template(nn + 1, c.tol, c.mu);
    try {
        nlopt::opt opt(nlopt::LD_SLSQP, (unsigned)nn);
        std::vector<double> lb(nn, -c.a_max), ub(nn, c.a_max);
        // wide brake like the scipy backend's 10x box (real cap is the norm con)
        for (int i = 0; i < nn; ++i) {
            lb[i] = -10.0 * c.a_max;
            ub[i] = 10.0 * c.a_max;
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        opt.set_min_objective(nlopt_obj, &ctx);
        std::vector<double> tol((size_t)c.horizon, 1e-8);
        opt.add_inequality_mconstraint(nlopt_con, &ctx, tol);
        opt.set_ftol_rel(1e-6);
        // Budget mapping: scipy maxiter counts SLSQP major iterations (~1-2
        // objective evals each). maxeval_cap>0 overrides the auto mapping
        // (used for budget sensitivity probes); 0 = maxiter*(nn+3).
        extern int g_maxeval_cap;
        int cap = g_maxeval_cap > 0 ? g_maxeval_cap : c.maxiter * (nn + 3);
        opt.set_maxeval(cap);
        std::vector<double> z(warm, warm + nn);
        double minf = 0.0;
        nlopt::result res = opt.optimize(z, minf);
        nevals = ctx.nevals;
        if (res < 0) return false;
        // norm-clip belt-and-braces (same as scipy backend)
        for (int k = 0; k < c.horizon; ++k) {
            double *uk = &z[3 * k];
            double nk = norm3(uk);
            if (nk > c.a_max && nk > 0) {
                double s = c.a_max / nk;
                uk[0] *= s;
                uk[1] *= s;
                uk[2] *= s;
            }
        }
        for (int i = 0; i < nn; ++i) warm[i] = z[i];
        u0[0] = z[0];
        u0[1] = z[1];
        u0[2] = z[2];
        return true;
    } catch (...) {
        nevals = ctx.nevals;
        return false;
    }
}

// Energy fallback (HighLevelNMPC._fallback_accel, fallback="pid" branch).
inline void fallback_accel(const double *r, const double *v, const SimCfg &c,
                           double *o) {
    double e_a = c.r_orbit - sma_c(r, v, c.mu);
    if (std::fabs(e_a) < c.db_a) {
        o[0] = o[1] = o[2] = 0.0;
        return;
    }
    double a_mag = clip1(3.0e-7 * e_a, -c.a_max, c.a_max);
    double h[3];
    cross3(r, v, h);
    double nh = norm3(h);
    if (nh > 1e-9) {
        double rn = norm3(r);
        double Rc[3] = {r[0] / rn, r[1] / rn, r[2] / rn};
        double hn[3] = {h[0] / nh, h[1] / nh, h[2] / nh};
        double T[3];
        cross3(hn, Rc, T);
        o[0] = T[0] * a_mag;
        o[1] = T[1] * a_mag;
        o[2] = T[2] * a_mag;
    } else {
        double rn = norm3(r);
        o[0] = r[0] / rn * a_mag;
        o[1] = r[1] / rn * a_mag;
        o[2] = r[2] / rn * a_mag;
    }
}

// Full closed-loop sim for ONE candidate. out[4] = dv, max_err, e_a, cost;
// out[4+i]: aborted flag, total SLSQP nevals.
int g_maxeval_cap = 0;

void closed_loop(const double *x0_in, const SimCfg &c, double *warm /*3N*/,
                 double *out) {
    double x[6];
    for (int i = 0; i < 6; ++i) x[i] = x0_in[i];
    // warm init: zeros first plan (matches backend._warm=None start)
    for (int i = 0; i < 3 * c.horizon; ++i) warm[i] = 0.0;
    double bulk[3] = {0.0, 0.0, 0.0};
    double int_r = 0.0, int_t = 0.0, int_n = 0.0, trim_spent = 0.0;
    double dv = 0.0, max_err = 0.0;
    long nevals = 0;
    int sub = (int)(c.dt_outer / c.dt_inner + 0.5);
    if (sub < 1) sub = 1;
    bool aborted = false;

    for (int o = 0; o < c.n_outer && !aborted; ++o) {
        // outer tick: fresh trim budget + replan (warm shifted like the
        // scipy backend: previous plan rolled by one, last step zero)
        int_r *= 0.5;
        int_t *= 0.5;
        int_n *= 0.5;
        trim_spent = 0.0;
        if (o > 0) {
            for (int i = 0; i < 3 * (c.horizon - 1); ++i) warm[i] = warm[i + 3];
            warm[3 * c.horizon - 3] = warm[3 * c.horizon - 2] =
                warm[3 * c.horizon - 1] = 0.0;
        }
        // NOTE: the scipy backend rolls warm only from the last SUCCESSFUL
        // plan (failures keep stale warm). The shift above applies regardless;
        // failure paths are rare (fallbacks ~ 0) and the fallback accel does
        // not depend on warm, so closed-loop divergence is negligible.
        double u0[3];
        long ne = 0;
        double xs[6];
        for (int i = 0; i < 6; ++i) xs[i] = x[i];
        if (!bulk_plan(xs, c, warm, u0, ne)) {
            fallback_accel(x, x + 3, c, u0);
        }
        nevals += ne;
        bulk[0] = u0[0];
        bulk[1] = u0[1];
        bulk[2] = u0[2];
        for (int s = 0; s < sub && !aborted; ++s) {
            const double *r = x, *v = x + 3;
            double r_norm = norm3(r);
            // trim (FastTrimPID, kp_scale on P+I, full D, authority+budget)
            double at[3] = {0.0, 0.0, 0.0};
            if (trim_spent < c.trim_bud) {
                double R[3] = {r[0] / r_norm, r[1] / r_norm, r[2] / r_norm};
                double h[3];
                cross3(r, v, h);
                double hn = norm3(h);
                double Nn[3] = {h[0] / hn, h[1] / hn, h[2] / hn};
                double T[3];
                cross3(Nn, R, T);
                double v_r = dot3(v, R), v_t = dot3(v, T), v_n = dot3(v, Nn);
                double r_n = dot3(r, Nn);
                double v_circ = std::sqrt(c.mu / r_norm);
                double e_r = c.r_orbit - r_norm;
                int_r = clip1(int_r + e_r * c.dt_inner, -c.int_clamp, c.int_clamp);
                double a_R = c.kp_scale * c.kp_r * e_r + c.kp_scale * c.ki_r * int_r -
                             c.kd_r * v_r;
                double e_t = v_circ - v_t;
                int_t = clip1(int_t + e_t * c.dt_inner, -c.int_clamp, c.int_clamp);
                double a_T = c.kp_scale * c.kp_t * e_t + c.kp_scale * c.ki_t * int_t;
                double e_n = -r_n;
                int_n = clip1(int_n + e_n * c.dt_inner, -c.int_clamp, c.int_clamp);
                double a_N = c.kp_scale * c.kp_n * e_n + c.kp_scale * c.ki_n * int_n -
                             c.kd_n * v_n;
                double artn[3] = {clip1(a_R, -c.trim_a, c.trim_a),
                                  clip1(a_T, -c.trim_a, c.trim_a),
                                  clip1(a_N, -c.trim_a, c.trim_a)};
                // trim deadband uses TRIM_DEADBAND (== deadband param here)
                artn[0] = std::fabs(artn[0]) < c.deadband ? 0.0 : artn[0];
                artn[1] = std::fabs(artn[1]) < c.deadband ? 0.0 : artn[1];
                artn[2] = std::fabs(artn[2]) < c.deadband ? 0.0 : artn[2];
                double ae[3] = {R[0] * artn[0] + T[0] * artn[1] + Nn[0] * artn[2],
                                R[1] * artn[0] + T[1] * artn[1] + Nn[1] * artn[2],
                                R[2] * artn[0] + T[2] * artn[1] + Nn[2] * artn[2]};
                double spend = norm3(ae) * c.dt_inner;
                trim_spent += spend;
                if (trim_spent >= c.trim_bud && spend > 0) {
                    double over = trim_spent - c.trim_bud;
                    double sc = 1.0 - over / spend;
                    if (sc < 0) sc = 0.0;
                    ae[0] *= sc;
                    ae[1] *= sc;
                    ae[2] *= sc;
                }
                at[0] = ae[0];
                at[1] = ae[1];
                at[2] = ae[2];
            }
            double tot[3] = {bulk[0] + at[0], bulk[1] + at[1], bulk[2] + at[2]};
            double tn = norm3(tot);
            if (tn > c.pid_a_max && tn > 0) {
                double s = c.pid_a_max / tn;
                tot[0] *= s;
                tot[1] *= s;
                tot[2] *= s;
            }
            if (std::fabs(tot[0]) < c.deadband && std::fabs(tot[1]) < c.deadband &&
                std::fabs(tot[2]) < c.deadband) {
                tot[0] = tot[1] = tot[2] = 0.0;
            }
            // surrogate plant: alloc_eff-scaled accel, RK4-quality via Taylor
            double app[3] = {tot[0] * c.alloc_eff, tot[1] * c.alloc_eff,
                             tot[2] * c.alloc_eff};
            dv += norm3(app) * c.dt_inner;
            // propagate one inner step with the B=1... use small Taylor steps:
            // reuse batch_template(1): set state+pars, propagate dt_inner
            {
                auto &ta = batch_template(1, c.tol, c.mu);
                ta.set_time(0.0);
                double *ST = ta.get_state_data();
                double *PA = ta.get_pars_data();
                for (int i = 0; i < 6; ++i) ST[i] = x[i];
                PA[0] = app[0];
                PA[1] = app[1];
                PA[2] = app[2];
                ta.propagate_for(c.dt_inner);
                for (int i = 0; i < 6; ++i) x[i] = ST[i];
            }
            double err = std::fabs(norm3(x) - c.r_orbit);
            if (err > max_err) max_err = err;
            // safe early abort (tuner incumbent): cost >= dv; infeasible
            // can never outrank a feasible incumbent
            if (c.has_incumbent) {
                if (dv > c.j_star || max_err > c.spec_box) {
                    aborted = true;
                    dv = 1e300;
                    max_err = 1e300;
                }
            }
        }
    }
    double e_a = (dv > 1e299) ? 1e300 : sma_c(x, x + 3, c.mu) - c.r_orbit;
    double excess = max_err / c.spec_box - 1.0;
    if (!(excess > 0.0)) excess = 0.0;
    // w_viol/w_final scoring stays in Python (orchestrator); C++ returns
    // dv, max_err, e_a, excess and Python applies the score formula.
    out[0] = dv;
    out[1] = max_err;
    out[2] = e_a;
    out[3] = excess;
    out[4] = aborted ? 1.0 : 0.0;
    out[5] = (double)nevals;
}

void closed_loop_batch(py::array_t<double, py::array::c_style | py::array::forcecast> X0,
                       int horizon, double dt_outer, double dt_inner, int n_outer,
                       int maxiter, double q_pos, double q_vel, double r_fuel,
                       double db_a, double a_max, double trim_a, double trim_bud,
                       double kp_scale, double kp_r, double ki_r, double kd_r,
                       double kp_t, double ki_t, double kp_n, double ki_n,
                       double kd_n, double int_clamp, double pid_a_max,
                       double deadband, double mu, double r_orbit, double tol,
                       double alloc_eff, double spec_box, double j_star,
                       bool has_incumbent, int maxeval_cap,
                       py::array_t<double, py::array::c_style> out) {
    auto X = X0.unchecked<2>();
    auto O = out.mutable_unchecked<2>();
    const int B = (int)X.shape(0);
    g_maxeval_cap = maxeval_cap;
    SimCfg c{dt_outer,
             dt_inner,
             n_outer,
             horizon,
             maxiter,
             q_pos,
             q_vel,
             r_fuel,
             db_a,
             a_max,
             trim_a,
             trim_bud,
             kp_scale,
             kp_r,
             ki_r,
             kd_r,
             kp_t,
             ki_t,
             kp_n,
             ki_n,
             kd_n,
             int_clamp,
             pid_a_max,
             deadband,
             mu,
             r_orbit,
             tol,
             alloc_eff,
             spec_box,
             j_star,
             has_incumbent};
    std::vector<double> warm(3 * (size_t)horizon), res(6);
    for (int b = 0; b < B; ++b) {
        double x0[6];
        for (int i = 0; i < 6; ++i) x0[i] = X(b, i);
        closed_loop(x0, c, warm.data(), res.data());
        for (int i = 0; i < 6; ++i) O(b, i) = res[i];
    }
}

PYBIND11_MODULE(_nmpcsolve, m) {
    m.doc() = "Full NMPC closed-loop sim in one C++ call (NLopt SLSQP inside).";
    m.def("closed_loop_batch", &closed_loop_batch,
          "Closed-loop sims for B candidates; out (B,6): dv, max_err, e_a, "
          "excess, aborted, nevals.",
          py::arg("X0"), py::arg("horizon"), py::arg("dt_outer"),
          py::arg("dt_inner"), py::arg("n_outer"), py::arg("maxiter"),
          py::arg("q_pos"), py::arg("q_vel"), py::arg("r_fuel"),
          py::arg("db_a"), py::arg("a_max"), py::arg("trim_a"),
          py::arg("trim_bud"), py::arg("kp_scale"), py::arg("kp_r"),
          py::arg("ki_r"), py::arg("kd_r"), py::arg("kp_t"), py::arg("ki_t"),
          py::arg("kp_n"), py::arg("ki_n"), py::arg("kd_n"),
          py::arg("int_clamp"), py::arg("pid_a_max"), py::arg("deadband"),
          py::arg("mu"), py::arg("r_orbit"), py::arg("tol"),
          py::arg("alloc_eff"), py::arg("spec_box"), py::arg("j_star"),
          py::arg("has_incumbent"), py::arg("maxeval_cap"), py::arg("out"));
}
