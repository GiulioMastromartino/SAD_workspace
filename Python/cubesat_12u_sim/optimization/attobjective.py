"""Cost of one attitude-controller vector: a Monte Carlo campaign in one call.

One candidate is one point in a 5-D box (K, P, observer gains, control scale).
Scoring it means running the *whole three-phase mission* -- detumble, slew,
sun-pointing -- once per run of the frozen design in :mod:`.attdesign`, in a
single ``run_chunk`` call whose batch lanes are the Monte Carlo runs.

**The per-run scalar.** Each run reports fuel per phase, settle times against
the mission's own thresholds, and the sun-cone verdict. The scalar is

    J_i = fuel_total_g
        + W_TIME * (d_settle/T_det + s_settle/T_slew)
        + W_SUN * max(0, ss_max/cone - 1)^2

Settle times are normalised by their phase budgets, so each is 0..1 and capped
at 1.0 when the run never settles -- the cap is what makes "never settles" a
plateau rather than an unbounded cost, and the feasibility flag is what ranks
it. The sun term is a hinge on the steady-state worst sun angle, zero inside
the cone. Thresholds (0.1 deg/s, 2.5 deg, 15 deg cone) are the mission's own,
not the tuner's invention.

**Reduction: mean plus weighted tails**, identical in form to the orbit
pipeline so the two reports read the same way:

    J = mean(J_i) + w_hi * mean(J | worst 10%) - w_lo * mean(J | best 10%)
      + W_INFEAS * infeasible_fraction

A run is feasible iff it reports ``success`` -- final rate under twice the
detumble threshold *and* the steady-state sun angle inside the cone. Infeasible
runs are excluded from both decile means and priced separately, so no candidate
can buy a cheap worst decile by letting its hard runs fail.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

from cubesat_12u_sim.optimization.attdesign import AttBundle
from cubesat_12u_sim.optimization.attparams import ATT_SPACE
from cubesat_12u_sim.uncertainty.monte_carlo import MonteCarloConfig

PENALTY_INFEASIBLE = 1e9


@dataclass(frozen=True)
class AttWeights:
    """Weights of the attitude Monte Carlo reduction.

    ``w_time`` prices a phase used in full: settling at the very end of the
    phase costs ``w_time`` grams-equivalent, settling instantly costs nothing.
    ``w_sun`` prices leaving the keep-in cone quadratically. ``w_hi``/``w_lo``
    weight the worst-/best-decile means. Defaults are calibrated against the
    baseline ensemble at default gains (see the sweep log); they make a full
    phase of settling cost roughly what the phase's fuel costs, so neither
    term dominates the other before the search starts.
    """

    w_time: float = 5.0
    w_sun: float = 50.0
    w_hi: float = 0.5
    w_lo: float = 0.5
    w_infeasible: float = 1e3
    tail_frac: float = 0.1

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class AttStats:
    """Monte Carlo summary of one candidate."""

    cost: float
    fuel_mean: float
    fuel_p50: float
    fuel_p90: float
    fuel_min: float
    fuel_max: float
    tail_hi: float
    tail_lo: float
    det_settle_mean: float
    slew_settle_mean: float
    sun_max_mean: float
    n_infeasible: int
    feasible_frac: float
    fuel: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))

    def as_dict(self) -> dict:
        d = asdict(self)
        d.pop("fuel")  # raw ensemble stays in memory, not in the JSON
        return d


@dataclass
class AttCandidateResult:
    cost: float
    stats: AttStats
    theta: list
    params: dict

    def as_dict(self) -> dict:
        return {"cost": self.cost, "stats": self.stats.as_dict(),
                "theta": self.theta, "params": self.params}


def _dead_stats(n: int) -> AttStats:
    inf = float("inf")
    return AttStats(PENALTY_INFEASIBLE, inf, inf, inf, inf, inf, inf, inf,
                    inf, inf, inf, n, 0.0)


def _kpis(om: np.ndarray, se: np.ndarray, sang: np.ndarray,
          phase_fuel: np.ndarray, times: np.ndarray,
          mc: MonteCarloConfig) -> dict:
    """Per-run KPI vectors from the raw ``run_chunk`` arrays.

    ``om``/``se``/``sang`` are (B, L); ``phase_fuel`` is (B, 3) in kg.
    Logic mirrors ``_results_from_arrays`` exactly (same thresholds, same caps)
    but returns lean float vectors instead of per-run dicts with timeseries --
    a 150-candidate generation would otherwise churn 150 x n_runs timeseries
    arrays through IPC for no benefit.
    """
    B = om.shape[0]
    dm = times <= mc.t_detumble
    sm = ((times > mc.t_detumble)
          & (times < mc.t_detumble + mc.t_slew))
    steady = times >= (mc.t_total - 40.0)
    t_dm, t_sm = times[dm], times[sm]

    d_set = np.zeros(B)
    fin_rate = np.zeros(B)
    for j in range(B):
        ab = np.where(om[j][dm] >= mc.rate_settle_thresh_deg_s)[0]
        d_set[j] = (t_dm[0] if len(ab) == 0 else
                    (mc.t_detumble if ab[-1] == len(om[j][dm]) - 1
                     else t_dm[ab[-1] + 1]))
        fin_rate[j] = om[j][dm][-1]

    s_set = np.zeros(B)
    fin_slew = np.zeros(B)
    for j in range(B):
        e = np.degrees(4.0 * np.arctan(se[j][sm]))
        av = np.where(e >= mc.slew_settle_thresh_deg)[0]
        s_set[j] = (0.0 if len(av) == 0 else
                    (mc.t_slew if av[-1] == len(e) - 1
                     else t_sm[av[-1] + 1] - mc.t_detumble))
        fin_slew[j] = e[-1] if len(e) else 0.0

    ss_max = np.array([sang[j][steady].max() if np.any(steady) else sang[j][-1]
                       for j in range(B)])
    inside = ss_max <= mc.sun_cone_angle_deg
    success = (fin_rate < mc.rate_settle_thresh_deg_s * 2.0) & inside
    fuel = phase_fuel.sum(axis=1) * 1000.0  # total propellant [g]
    return {"fuel": fuel, "d_set": d_set, "s_set": s_set, "ss_max": ss_max,
            "fin_rate": fin_rate, "fin_slew": fin_slew,
            "success": success}


def run_scores(kpis: dict, mc: MonteCarloConfig,
               weights: AttWeights) -> np.ndarray:
    """Per-run cost: fuel + normalised settle times + sun-cone hinge."""
    time_term = (kpis["d_set"] / mc.t_detumble
                 + kpis["s_set"] / mc.t_slew)
    excess = np.maximum(0.0, kpis["ss_max"] / mc.sun_cone_angle_deg - 1.0)
    return (kpis["fuel"] + weights.w_time * time_term
            + weights.w_sun * excess ** 2)


def reduce_runs(kpis: dict, mc: MonteCarloConfig,
                weights: AttWeights) -> tuple[float, AttStats]:
    """Collapse one candidate's run set into ``(cost, AttStats)``."""
    n = int(kpis["fuel"].size)
    if n == 0:
        return PENALTY_INFEASIBLE, _dead_stats(0)
    costs = run_scores(kpis, mc, weights)
    feasible = np.isfinite(costs) & kpis["success"]
    ok = feasible & np.isfinite(kpis["fuel"])
    if not np.any(ok):
        return PENALTY_INFEASIBLE, _dead_stats(n)

    srt = np.sort(costs[ok])
    k = max(1, int(round(weights.tail_frac * n)))
    tail_hi = float(srt[-k:].mean())
    tail_lo = float(srt[:k].mean())
    infeas_frac = 1.0 - float(ok.sum()) / float(n)
    cost = (float(costs[ok].mean()) + weights.w_hi * tail_hi
            - weights.w_lo * tail_lo + weights.w_infeasible * infeas_frac)
    f = kpis["fuel"][ok]
    return cost, AttStats(
        cost=cost, fuel_mean=float(f.mean()),
        fuel_p50=float(np.percentile(f, 50)),
        fuel_p90=float(np.percentile(f, 90)),
        fuel_min=float(f.min()), fuel_max=float(f.max()),
        tail_hi=tail_hi, tail_lo=tail_lo,
        det_settle_mean=float(kpis["d_set"][ok].mean()),
        slew_settle_mean=float(kpis["s_set"][ok].mean()),
        sun_max_mean=float(kpis["ss_max"][ok].mean()),
        n_infeasible=int(n - int(ok.sum())),
        feasible_frac=float(ok.sum()) / float(n),
        fuel=kpis["fuel"].copy())


def evaluate_candidate(theta: np.ndarray, bundle: AttBundle,
                       mc_cfg: MonteCarloConfig, *,
                       dt: float, log_every: int,
                       g0: float, tol: float, quantize: bool,
                       quant_levels: int = 0,
                       weights: AttWeights | None = None) -> AttCandidateResult:
    """Score one controller vector over the whole frozen ensemble.

    One ``run_chunk`` call: the bundle's B lanes are the Monte Carlo runs, the
    5 decoded scalars are the controller. A failing call marks the candidate
    infeasible rather than raising, exactly like the orbit pipeline.

    ``quant_levels`` selects the throttle model: 0 keeps the legacy measured
    staircase ({0, 1/6, 5/6, 1}, the Basilisk effector behaviour), >1 uses
    uniform mid-tread quantization with that many levels. It exists to answer
    "how fine must the resolution be", not as a tuning knob.
    """
    import heyoka  # noqa: F401  extensions need libheyoka's symbols first
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.native import cxxdrive as _cxx

    weights = weights or AttWeights()
    theta = np.clip(np.asarray(theta, float).ravel(), *ATT_SPACE.bounds())
    p = ATT_SPACE.from_theta(theta)

    try:
        out = _cxx.run_chunk(
            np.ascontiguousarray(bundle.r0), np.ascontiguousarray(bundle.v0),
            np.ascontiguousarray(bundle.sigma0),
            np.ascontiguousarray(bundle.omega0),
            np.ascontiguousarray(bundle.mass),
            np.ascontiguousarray(bundle.I6),
            np.ascontiguousarray(bundle.invI6),
            np.ascontiguousarray(bundle.MF), np.ascontiguousarray(bundle.MT),
            np.ascontiguousarray(bundle.tmax), np.ascontiguousarray(bundle.isp),
            np.ascontiguousarray(bundle.gyro_bias),
            np.ascontiguousarray(bundle.gyro_arw),
            np.ascontiguousarray(np.asarray(bundle.seeds, dtype=np.uint64)),
            np.ascontiguousarray(bundle.P_nom),
            np.ascontiguousarray(bundle.REFS),
            np.ascontiguousarray(bundle.PH),
            np.ascontiguousarray(bundle.sun_dirs),
            np.ascontiguousarray(bundle.bore),
            float(p["K"]), float(p["P"]),
            float(p["obs_gain"]), float(p["obs_bias_gain"]),
            float(p["ctrl_scale"]), float(dt), int(bundle.n_steps),
            int(log_every), float(g0),
            float(mc_cfg.rate_settle_thresh_deg_s),
            float(mc_cfg.slew_settle_thresh_deg),
            float(mc_cfg.sun_cone_angle_deg),
            bool(quantize), int(quant_levels), float(tol), False,
            float(C.MU_EARTH), bool(C.USE_GRAVITY_GRADIENT))
    except Exception:
        n = bundle.n_runs
        return AttCandidateResult(
            cost=float(PENALTY_INFEASIBLE), stats=_dead_stats(n),
            theta=[float(x) for x in theta], params=p)

    om = np.degrees(np.asarray(out["omega_norm"], float)).T
    se = np.asarray(out["sigma_err"], float).T
    sang = np.degrees(np.asarray(out["sun_angle"], float)).T
    pf = np.asarray(out["phase_fuel"], float)
    times = np.arange(om.shape[1]) * float(log_every) * float(dt)
    kpis = _kpis(om, se, sang, pf, times, mc_cfg)
    cost, stats = reduce_runs(kpis, mc_cfg, weights)
    return AttCandidateResult(cost=float(cost), stats=stats,
                              theta=[float(x) for x in theta], params=p)


__all__ = ["AttWeights", "AttStats", "AttCandidateResult", "evaluate_candidate",
           "reduce_runs", "run_scores", "PENALTY_INFEASIBLE"]
