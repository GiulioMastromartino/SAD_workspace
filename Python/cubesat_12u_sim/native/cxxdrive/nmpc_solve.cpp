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
    // q_pos / q_vel / r_fuel are gone: the cost is now the scorer's cost, so
    // there are no free tracking weights left to tune. db_a survives as the
    // deadband on the terminal energy term only.
    double db_a, a_max;
    double trim_a, trim_bud, kp_scale;
    double kp_r, ki_r, kd_r, kp_t, ki_t, kp_n, ki_n, kd_n;
    double int_clamp, pid_a_max, deadband;
    double mu, r_orbit, tol, alloc_eff;
    double spec_box;
    // Position-tracking cost: w_track is the cost of sitting exactly on the box
    // edge (m/s), box_frac is where the profile stops being linear, and
    // track_slope is the linear-region slope that makes w(spec_box) == w_track.
    double w_track, box_frac, track_slope;
    double w_final;
    // fuel: A = a_max*dt (full authority for one tick, in m/s), x0 the knee as
    // a fraction of a_max, g1 = g(1) the normalisation
    double fuel_A, fuel_x0, fuel_g1;
    double j_star;
    bool has_incumbent;
};

// Position tracking on RADIUS error: linear inside the inner 20% of the box,
// logarithmic from there out to the edge and beyond.
//
//   w(x) = s*x                      x <= x0   (x0 = 0.2)
//   w(x) = s*x0*(1 + ln(x/x0))      x >  x0
//
// with x = |r_err|/box and s = w_edge / (x0*(1 + ln(1/x0))), so w(box) = w_edge
// exactly and both the value and the slope are continuous at the join.
//
// Why this shape, given the alternatives failed:
//  * A one-sided hinge (penalty only outside the box) is degenerate: inside the
//    box the cost is pure fuel, so coasting is free. Because a plan only sees
//    H*dt_outer (~40 min) while the violation accrues over the 12 h closed
//    loop, the planner never sees the penalty it is dodging -- measured box
//    error reached 1437 km.
//  * Pure log(1+x) is always-on but its slope decays as 1/x, so near the edge
//    the pull toward centre becomes weak.
//  * Linear everywhere is always-on and strong, but unbounded: 4x the edge
//    would cost 36 m/s, more than the fuel it saves.
// This composite is steep where being steep is cheap (near centre) and gentle
// where the state is already bad and fuel is better spent on recovery.
//
// w_track defaults to 20 m/s = 67% of a 30 m/s fuel budget: sitting on the box
// edge is worth two thirds of the fuel it took to get there.
inline double track_w(double x, double x0, double s) {
    if (x <= x0) return s * x;
    return s * x0 * (1.0 + std::log(x / x0));
}

// d/dx of track_w, i.e. slope per unit of r_err/box.
inline double track_dw(double x, double x0, double s) {
    if (x <= x0) return s;
    return s * x0 / x;
}

// Fuel: quadratic near the centre, exponential past a knee.
//
//   x = |u| / a_max
//   g(x) = x^2                    x <= x0      (the "square" region)
//   g(x) = x0^2 * exp(x - x0)     x >  x0      (the "exp" region)
//   J_k = A * g(x) / g(1),   A = a_max*dt
//
// so full authority for one tick costs exactly a_max*dt -- the same m/s the
// scorer counts for that burn.
//
// Why this shape:
//  * |u| (L1, the exact dv) is NOT differentiable at u = 0 and the warm start IS
//    u = 0. Central-differencing that kink gave SLSQP noise and it stalled at
//    the iteration limit.
//  * Pure |u|^2 is smooth but has no knee: it is quadratic all the way, so it
//    under-prices a large burn relative to spreading the same dv into several
//    smaller ones. That biases the plan toward bang-bang.
//  * Pure exp(|u|^2) is smooth but grows as e^(x^2) -- far too steep, and it
//    is *not* what was asked for.
//  * This one: quadratic where the action is cheap and the gradient is gentle,
//    exponential beyond the knee where a big single burn should be punished.
//    Both branches are smooth at 0 (the quadratic branch has zero gradient
//    there, which is exactly right for a warm start) and value-continuous at
//    the knee.
//
// The knee is a fraction of a_max, so it moves with the vehicle's authority
// rather than being an absolute thrust number.
inline double fuel_g(double x, double x0) {
    if (x <= x0) return x * x;
    return x0 * x0 * std::exp(x - x0);
}

// d/dx of fuel_g.
inline double fuel_dg(double x, double x0) {
    if (x <= x0) return 2.0 * x;
    return x0 * x0 * std::exp(x - x0);
}

inline double fuel_cost(const double *u, double a_scale, double A, double x0,
                        double g1) {
    double an = norm3(u);
    if (!(an > 0.0)) return 0.0;
    double x = an / a_scale;
    if (!(x < 20.0)) x = 20.0;   // exp(19.5) ~ 3e8: bounded, no overflow
    return A * fuel_g(x, x0) / g1;
}

inline double shoot_cost(const double *x0, const double *U, const SimCfg &c,
                         heyoka::taylor_adaptive_batch<double> &ta) {
    static const double EA = 50e3;
    ta.set_time(0.0);
    double *ST = ta.get_state_data();
    double *PA = ta.get_pars_data();
    for (int i = 0; i < 6; ++i) ST[i] = x0[i];  // lanes=1: ST[i*1+0]
    double cost = 0.0;
    double e_term = 0.0;
    const bool track = (c.w_track > 0.0 && c.spec_box > 1.0);
    double a_scale = c.a_max > 1e-12 ? c.a_max : 1e-12;
    for (int k = 1; k <= c.horizon; ++k) {
        const double *uk = U + 3 * (k - 1);
        PA[0] = uk[0];
        PA[1] = uk[1];
        PA[2] = uk[2];
        ta.propagate_for(c.dt_outer);
        double r[3] = {ST[0], ST[1], ST[2]}, v[3] = {ST[3], ST[4], ST[5]};
        // fuel: exp of the squared normalised thrust (see fuel_cost)
        cost += fuel_cost(uk, a_scale, c.fuel_A, c.fuel_x0, c.fuel_g1);
        if (track) {
            cost += track_w(std::fabs(norm3(r) - c.r_orbit) / c.spec_box,
                            c.box_frac, c.track_slope);
        }
        if (k == c.horizon) {
            double ea = sma_c(r, v, c.mu) - c.r_orbit;
            e_term = ea / EA;
            if (std::fabs(ea) < c.db_a) e_term = 0.0;
        }
    }
    return cost + c.w_final * e_term * e_term;
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
    double a_scale = c.a_max > 1e-12 ? c.a_max : 1e-12;
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
        static const double EA = 50e3;
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
                // Must mirror shoot_cost() term for term. A gradient of a
                // different function is worse than no gradient: SLSQP would
                // descend on something the scorer does not measure.
                f[j] += fuel_cost(uk, a_scale, c.fuel_A, c.fuel_x0, c.fuel_g1);
                if (c.w_track > 0.0 && c.spec_box > 1.0) {
                    f[j] += track_w(std::fabs(norm3(r) - c.r_orbit) /
                                        c.spec_box,
                                    c.box_frac, c.track_slope);
                }
                if (k == c.horizon) {
                    double ea = sma_c(r, v, c.mu) - c.r_orbit;
                    double e_term = ea / EA;
                    if (std::fabs(ea) < c.db_a) e_term = 0.0;
                    f[j] += c.w_final * e_term * e_term;
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

// Fallback for when the bulk SLSQP plan fails. Now targets RADIUS error rather
// than semi-major-axis error, matching the cost: an energy-driven fallback
// drives e_a to zero, which does not bring |r| back inside the box, so it would
// be optimising against a different objective than the planner above it.
//
// Gain 3e-7 m/s^2 per m of radius error, i.e. a 20 km error commands the full
// 6e-3 m/s^2 -- an order below the bus limit, so it coasts in gently rather
// than fighting for authority.
inline void fallback_accel(const double *r, const double *v, const SimCfg &c,
                           double *o) {
    (void)v;
    double rn0 = norm3(r);
    if (rn0 < 1.0) rn0 = 1.0;
    double rerr = c.r_orbit - rn0;
    double dead = 0.1 * c.spec_box;
    if (std::fabs(rerr) < dead) {
        o[0] = o[1] = o[2] = 0.0;
        return;
    }
    double a_mag = clip1(3.0e-7 * rerr, -c.a_max, c.a_max);
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

// Full closed-loop sim for ONE candidate. out[9]: dv, max_err, e_a, excess,
// aborted flag, total SLSQP nevals, dv_bulk, dv_trim, fallbacks.
// (trim_frac derives from dv_bulk/dv_trim exactly like
// HierarchicalController.fuel_split; plans attempted = out outer count.)
int g_maxeval_cap = 0;

// Cross-lane warm-start: 0 = each lane cold-starts (the original behaviour,
// and what the scipy backend does), 1 = lanes within one batch inherit the
// previous lane's converged plan.
//
// This exists because a Monte Carlo ensemble scores one candidate against many
// runs that share a controller and differ only in the injected error, so lane
// k+1's optimum is close to lane k's. Zeroing `warm` per lane throws that away
// and pays a cold SLSQP start every time.
//
// It is OFF by default because it changes results, not just speed: a warm start
// can only change the answer if SLSQP terminates early, so the cost it returns
// belongs to a slightly different optimiser run. Enabling it is therefore an
// explicit decision (see the `warm_share` entry point) and every cost computed
// with it on must say so.
// MEASURED NEGATIVE -- kept off deliberately, do not enable without re-measuring.
//
// The hypothesis was that a Monte Carlo ensemble scores one candidate against
// many runs sharing a controller, so lane k+1's optimum is close to lane k's
// and should need far fewer SLSQP iterations warm-started from it. Measured on
// the 250-run design, sharing made things *worse*:
//
//     H   nevals cold -> warm     dv shift     wall
//     2      6900 -> 7142          5.4%       0.97x
//     4      7498 -> 7885          3.3%       0.95x
//     6      8754 -> 8795          3.0%       1.01x
//
// The reason is the roll at the top of the outer loop: by the time a lane
// finishes, its buffer has been shifted left by one step n_outer times, so what
// the next lane inherits is a heavily-degraded plan rather than a converged
// one. Reusing it perturbs dv by 3-5% (changing the objective the optimiser is
// chasing) without saving any iterations.
//
// The switch remains so the experiment is reproducible, not because it helps.
int g_warm_share = 0;

void closed_loop(const double *x0_in, const SimCfg &c, double *warm /*3N*/,
                 double *out, bool warm_valid) {
    double x[6];
    for (int i = 0; i < 6; ++i) x[i] = x0_in[i];
    // Cold start zeroes the buffer (matches backend._warm=None start). A shared
    // warm start keeps whatever the previous lane converged to.
    if (!warm_valid) {
        for (int i = 0; i < 3 * c.horizon; ++i) warm[i] = 0.0;
    }
    double bulk[3] = {0.0, 0.0, 0.0};
    double int_r = 0.0, int_t = 0.0, int_n = 0.0, trim_spent = 0.0;
    double dv = 0.0, max_err = 0.0;
    double dv_bulk = 0.0, dv_trim = 0.0;
    long nevals = 0, fallbacks = 0, plans = 0;
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
        plans++;
        if (!bulk_plan(xs, c, warm, u0, ne)) {
            fallbacks++;
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
            // fuel bookkeeping mirrors HierarchicalController exactly:
            // bulk/trim counted pre-combination (and pre-alloc_eff), while dv
            // below counts the post-everything applied accel.
            dv_bulk += norm3(bulk) * c.dt_inner;
            dv_trim += norm3(at) * c.dt_inner;
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
    // The outer w_viol hinge stays in Python (orchestrator); C++ returns
    // dv, max_err, e_a, excess and Python applies the score formula.
    out[0] = dv;
    out[1] = max_err;
    out[2] = e_a;
    out[3] = excess;
    out[4] = aborted ? 1.0 : 0.0;
    out[5] = (double)nevals;
    out[6] = dv_bulk;
    out[7] = dv_trim;
    out[8] = (double)fallbacks;
    out[9] = (double)plans;
}

void closed_loop_batch(py::array_t<double, py::array::c_style | py::array::forcecast> X0,
                       int horizon, double dt_outer, double dt_inner, int n_outer,
                       int maxiter, double db_a, double a_max, double trim_a,
                       double trim_bud, double kp_scale, double kp_r,
                       double ki_r, double kd_r, double kp_t, double ki_t,
                       double kp_n, double ki_n, double kd_n,
                       double int_clamp, double pid_a_max, double deadband,
                       double mu, double r_orbit, double tol, double alloc_eff,
                       double spec_box, double w_track, double box_frac,
                       double w_final, double fuel_A, double fuel_x0,
                       double fuel_g1, double j_star, bool has_incumbent,
                       int maxeval_cap,
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
             w_track,
             box_frac,
             // derived: the linear-region slope that makes w(spec_box) == w_track
             (box_frac > 1e-6 && box_frac < 1.0)
                 ? w_track / (box_frac * (1.0 + std::log(1.0 / box_frac)))
                 : w_track,
             w_final,
             fuel_A,
             fuel_x0,
             fuel_g1,
             j_star,
             has_incumbent};
    std::vector<double> warm(3 * (size_t)horizon), res(10);
    // With sharing on, lane 0 still cold-starts; every later lane inherits the
    // previous lane's converged plan. `warm` is already zero-initialised, so
    // lane 0 would be correct either way, but being explicit keeps the cold
    // path identical to the historical one.
    bool warm_valid = false;
    for (int b = 0; b < B; ++b) {
        double x0[6];
        for (int i = 0; i < 6; ++i) x0[i] = X(b, i);
        closed_loop(x0, c, warm.data(), res.data(), warm_valid);
        warm_valid = g_warm_share != 0;
        for (int i = 0; i < 10; ++i) O(b, i) = res[i];
    }
}

PYBIND11_MODULE(_nmpcsolve, m) {
    m.doc() = "Full NMPC closed-loop sim in one C++ call (NLopt SLSQP inside).";
    m.def("closed_loop_batch", &closed_loop_batch,
          "Closed-loop sims for B candidates; out (B,10): dv, max_err, e_a, "
          "excess, aborted, nevals, dv_bulk, dv_trim, fallbacks, plans.",
          py::arg("X0"), py::arg("horizon"), py::arg("dt_outer"),
          py::arg("dt_inner"), py::arg("n_outer"), py::arg("maxiter"),
          py::arg("db_a"), py::arg("a_max"), py::arg("trim_a"),
          py::arg("trim_bud"), py::arg("kp_scale"), py::arg("kp_r"),
          py::arg("ki_r"), py::arg("kd_r"), py::arg("kp_t"), py::arg("ki_t"),
          py::arg("kp_n"), py::arg("ki_n"), py::arg("kd_n"),
          py::arg("int_clamp"), py::arg("pid_a_max"), py::arg("deadband"),
          py::arg("mu"), py::arg("r_orbit"), py::arg("tol"),
          py::arg("alloc_eff"), py::arg("spec_box"),
          py::arg("w_track"), py::arg("box_frac"), py::arg("w_final"),
          py::arg("fuel_A"), py::arg("fuel_x0"),
          py::arg("fuel_g1"),
          py::arg("j_star"),
          py::arg("has_incumbent"), py::arg("maxeval_cap"), py::arg("out"));
    // Off by default; enabling it changes the optimiser's start point and so
    // can change the returned cost, not just the time to reach it.
    m.def("set_warm_share", [](int on) { g_warm_share = on ? 1 : 0; },
          "Share the SLSQP warm start across lanes within one batch. Off by "
          "default: this changes results as well as speed.",
          py::arg("enabled"));
    m.def("get_warm_share", []() { return g_warm_share; },
          "Whether cross-lane warm sharing is currently enabled.");
}
