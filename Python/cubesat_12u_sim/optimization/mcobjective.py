"""Cost of one hypercube vector: 250 Monte Carlo runs through the full C++ pipeline.

One candidate is one point in a 16-dimensional box (6 NMPC + 9 PID + horizon).
Scoring it means running the *whole* station-keeping loop 250 times — once per
run of the frozen design in :mod:`.mcdesign` — and reducing those 250 outcomes
to a single scalar for CMA-ES.

**Both arms are run, and both feed the cost.** The NMPC arm rides the C++
closed-loop kernel (``_nmpcsolve.closed_loop_batch``: NLopt-SLSQP inside,
warm-started, one crossing per vehicle group) and the PID arm rides the fused
batch evaluator (``_pidbatch.pid_batch_eval``, exact to 1e-14 of the numpy
reference). Neither arm uses the numpy path here: 250 runs x 150 candidates is
37.5k closed-loop sims per generation, which only the C++ kernels can afford.

**Reduction: mean plus weighted tails.** ``mean(dv)`` alone would let a
controller win by being cheap on the easy injections and ruinous on the hard
ones. The reported cost of one arm is

    J = mean(J_i) + w_hi * mean(J | worst 10%) - w_lo * mean(J | best 10%)
      + W_INFEAS * infeasible_fraction

where ``J_i`` is the per-run score (dv, box penalty, terminal-energy penalty).
The worst-decile term prices the tail risk a mean hides. The best-decile term is
*subtracted*: it rewards a candidate that is uniformly good rather than one that
is occasionally excellent, because the quantity being bought here is a
station-keeping controller that has to work every day, not one that wins on the
median launch. Infeasible runs are excluded from both decile means and priced
separately by ``W_INFEAS``, so no candidate can lower its worst-decile cost by
pushing its hard runs outside the box.

Everything in :class:`ArmStats` is reported, not just ``cost``: the mean, the
percentiles, both decile means, the infeasible count and the box excursion are
all things a reader of the report will want in order to check the reduction
against the raw ensemble.
"""
from __future__ import annotations

import os
import traceback
from dataclasses import asdict, dataclass, field

import numpy as np

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.optimization.mcdesign import MCDesign, VehicleGroup
from cubesat_12u_sim.optimization.params import (HYPER_SPACE, NMPC_SPACE,
                                   PID_SPACE)

PENALTY_INFEASIBLE = 1e9

# Default fuel amplitude (a multiple of a_max*dt, i.e. "full authority for one
# tick, in m/s"). 1.0 is the natural calibration. Now that fuel_scale is a
# hypercube coordinate this is only the DEFAULT and the space's lower bound
# starts at 0.3, so most of the search will move away from it. Kept because the
# kernel and the numpy reference both need a fixed value when a caller does not
# supply one.
FUEL_SCALE = 1.0

# Position-tracking shape the planner optimises. Mirrors objective.TRACK_EDGE /
# TRACK_FRAC (20 m/s at the box edge = 67% of a 30 m/s fuel budget; linear
# inside 20% of the box, logarithmic beyond). Duplicated rather than imported
# because objective imports this module's sibling objective, and a literal that
# must agree with one other literal is cheaper than the cycle.
TRACK_EDGE = 20.0
TRACK_FRAC = 0.2

# Fuel-cost amplitude, as a multiple of ``a_max*dt`` (full authority for one
# tick, in m/s). 1.0 means "one tick at full authority costs the same m/s the
# scorer counts", which is the natural calibration.
#
# It is a single number because the fuel shape deliberately departs from dv
# below its knee -- it is x^2 there, so it charges only ~0.12-0.6x what the
# scorer charges for moderate thrust. Left at 1.0, the optimiser responds to
# that discount by overspending: a full 30-generation run settled at 71.9 m/s of
# NMPC dv where the expected optimum is ~13 m/s. Raising this number restores
# the balance without changing the shape.
#
# The measured fuel/feasibility trade-off that made this a search coordinate in
# the first place, recorded here so the number does not have to be re-derived:
#
#   scale   dv [m/s]   box max [km]   feasible
#     1.0      76.1         22.0       1.00
#     4.0      50.5        658         0.50
#    16.0      18.0        690         0.04
#
# Below ~90% feasibility the +1000*infeasible_fraction term outruns any fuel
# saving, so the useful part of that curve is the top of it -- which is what the
# 90-100% band is for.

# Set by mc_progress-style debugging; off in normal use. The per-arm try/except
# deliberately swallows exceptions so one bad group cannot abort a generation,
# which also means a systematic wiring error shows up only as "everything is
# infeasible". This flag makes the loop observable when that happens -- it is
# how the missing w_track parameter here was found.
_TRACE = bool(int(os.environ.get("MCTRACE", "0")))

# Order the PID kernel expects; must match GAIN_KEYS in pid_batch.cpp.
GAIN_KEYS = ("kp_r", "ki_r", "kd_r", "kp_t", "ki_t", "kp_n", "ki_n", "kd_n")


@dataclass(frozen=True)
class TailWeights:
    """Weights of the Monte Carlo reduction.

    ``w_hi``/``w_lo`` weight the worst-/best-decile means; ``w_nmpc``/``w_pid``
    mix the two arms. The defaults make the arms co-equal and price the tail at
    half the mean — the usual risk-averse choice, under which a candidate that
    is 10% better on the mean but several times worse in its worst decile does
    not win.
    """

    w_hi: float = 0.5
    w_lo: float = 0.5
    w_nmpc: float = 0.5
    w_pid: float = 0.5
    w_infeasible: float = 1e3
    tail_frac: float = 0.1

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class ArmStats:
    """Monte Carlo summary of one arm at one candidate."""

    cost: float
    dv_mean: float
    dv_p50: float
    dv_p90: float
    dv_p99: float
    dv_min: float
    dv_max: float
    tail_hi: float          # mean cost over the worst decile
    tail_lo: float          # mean cost over the best decile
    box_mean: float         # mean max |r err| over feasible runs [m]
    box_max: float          # worst max |r err| over all runs [m]
    n_infeasible: int
    feasible_frac: float
    dv: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))
    # per-run max |r err| [m]. Kept alongside `dv` because a per-run table is
    # what a report needs to show *why* a candidate is feasible or not: the
    # aggregate box_mean and box_max cannot say which individual runs left the
    # station-keeping box.
    box_per_run: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))

    def as_dict(self) -> dict:
        d = asdict(self)
        # the raw ensembles stay in memory, not in the sweep JSON: 50 floats per
        # candidate x 50 generations is noise in the results file, and the
        # exporter rebuilds the winner's table by re-evaluating it.
        d.pop("dv")
        d.pop("box_per_run")
        return d


@dataclass
class CandidateResult:
    """Everything one hypercube vector produced."""

    cost: float
    horizon: int
    nmpc: ArmStats
    pid: ArmStats
    theta: list
    params: dict

    def as_dict(self) -> dict:
        return {"cost": self.cost, "horizon": self.horizon,
                "nmpc": self.nmpc.as_dict(), "pid": self.pid.as_dict(),
                "theta": self.theta, "params": self.params}


def _dead_arm(n: int) -> ArmStats:
    inf = float("inf")
    return ArmStats(PENALTY_INFEASIBLE, inf, inf, inf, inf, inf, inf, inf, inf,
                    inf, inf, n, 0.0,
                    dv=np.full(n, inf), box_per_run=np.full(n, inf))


def _over_budget_candidate(theta: np.ndarray, horizon: int,
                          n_runs: int) -> CandidateResult:
    """A rejected draw: horizon too expensive to simulate inside the budget."""
    return CandidateResult(
        cost=float(PENALTY_INFEASIBLE * 2), horizon=horizon,
        nmpc=_dead_arm(n_runs), pid=_dead_arm(n_runs),
        theta=[float(x) for x in theta],
        params={"_rejected": "horizon over budget"})


# Per-lane closed-loop cost, measured on the reference machine at H=2/4/6
# (28/76/158 ms) and extrapolated. The trend is superlinear because the SLSQP
# gradient is an expanded 3N+1-lane batch, so cost grows faster than H; a linear
# fit would badly under-budget H=12.
_PER_H = (9.0, 2.5)   # ms = _PER_H[0]*H + _PER_H[1]*H^2, per lane


def est_nmpc_s(horizon: int, n_runs: int) -> float:
    """Predicted NMPC wall time [s] for ``n_runs`` runs at ``horizon``."""
    a, b = _PER_H
    ms = a * horizon + b * horizon * horizon
    return ms * n_runs / 1000.0


# ---------------------------------------------------------------------------
# reduction
# ---------------------------------------------------------------------------

def run_scores(dv: np.ndarray, box: np.ndarray, ea: np.ndarray,
               spec_box: float, w_viol: float, w_final: float) -> np.ndarray:
    """Per-run cost: dv + box penalty + terminal-energy penalty.

    Identical to :func:`optimization.objective.score` per run, so a candidate
    scored here and the same candidate scored by the single-run path agree.
    """
    excess = np.maximum(0.0, box / spec_box - 1.0)
    return dv + w_viol * excess ** 2 + w_final * (ea / 50e3) ** 2


def reduce_arm(dv: np.ndarray, box: np.ndarray, ea: np.ndarray, spec_box: float,
               w_viol: float, w_final: float,
               weights: TailWeights) -> tuple[float, ArmStats]:
    """Collapse one arm's run set into ``(cost, ArmStats)``.

    Infeasible runs are excluded from both decile means and priced by
    ``w_infeasible * infeasible_fraction`` instead.
    """
    n = int(dv.size)
    if n == 0:
        return PENALTY_INFEASIBLE, _dead_arm(0)

    costs = run_scores(dv, box, ea, spec_box, w_viol, w_final)
    excess = np.maximum(0.0, box / spec_box - 1.0)
    feasible = np.isfinite(costs) & (excess <= 0.0)
    ok = feasible & np.isfinite(dv)
    if not np.any(ok):
        return PENALTY_INFEASIBLE, _dead_arm(n)

    srt = np.sort(costs[ok])
    k = max(1, int(round(weights.tail_frac * n)))
    tail_hi = float(srt[-k:].mean())
    tail_lo = float(srt[:k].mean())
    infeas_frac = 1.0 - float(ok.sum()) / float(n)

    cost = (float(costs[ok].mean())
            + weights.w_hi * tail_hi
            - weights.w_lo * tail_lo
            + weights.w_infeasible * infeas_frac)

    stats = ArmStats(
        cost=cost,
        dv_mean=float(dv[ok].mean()),
        dv_p50=float(np.percentile(dv[ok], 50)),
        dv_p90=float(np.percentile(dv[ok], 90)),
        dv_p99=float(np.percentile(dv[ok], 99)),
        dv_min=float(dv[ok].min()),
        dv_max=float(dv[ok].max()),
        tail_hi=tail_hi,
        tail_lo=tail_lo,
        box_mean=float(box[ok].mean()),
        box_max=float(np.max(np.where(np.isfinite(box), box, 0.0))),
        n_infeasible=int(n - int(ok.sum())),
        feasible_frac=float(ok.sum()) / float(n),
        dv=dv.copy(),
        box_per_run=box.copy())
    return cost, stats


# ---------------------------------------------------------------------------
# C++ evaluators, one call per vehicle group
# ---------------------------------------------------------------------------

def _nmpc_group(x0: np.ndarray, params: dict, horizon: int, vg: VehicleGroup,
                dt_outer: float, dt_inner: float, n_outer: int,
                maxiter: int, spec_box: float, tol: float = 1e-12,
                w_track: float = 20.0, box_frac: float = 0.2,
                w_final: float = 0.05) -> np.ndarray:
    """...

    ``fuel_scale`` is read from ``params`` (a hypercube coordinate), NOT taken as
    a keyword. An earlier version accepted it as a keyword and then gave the
    params value precedence via ``.get`` -- which made the keyword dead, so any
    caller sweeping it silently swept nothing.
    """
    from cubesat_12u_sim.control.adjoint import fuel_params
    fA, fx0, fg1 = fuel_params(C.NMPC_A_MAX, dt_outer)
    fA *= float(params.get("fuel_scale", FUEL_SCALE))
    """This vehicle group's runs through the full C++ closed loop.

    ``x0`` is one lane per run. Returns ``(n, 10)``: dv, max_err, e_a, excess,
    aborted, nevals, dv_bulk, dv_trim, fallbacks, plans — the schema of
    :func:`optimization.nmpcsolve.closed_loop`.

    The vehicle build enters twice. ``a_max`` stays at the *commanded*
    saturation (that is a property of the control law, not of the bus), while
    the realised authority is folded into ``alloc_eff`` together with the
    allocator ratio: a heavier bus with proportionally larger thrusters ends up
    with the same accelerations, and one with mismatched thrusters does not.

    ``tol`` is the Heyoka Taylor tolerance, and it is the single biggest
    performance knob here. Measured on the 250-run design at H=4:

    ===========  ===========  =========  ==============================
    tol          s/candidate  speedup    dv mean (vs 1e-12)
    ===========  ===========  =========  ==============================
    1e-12        18.9         1.00x      36.88 (reference)
    1e-10        15.7         1.20x      36.96
    1e-8         13.2         1.43x      37.03
    1e-6         9.9          1.90x      36.96
    ===========  ===========  =========  ==============================

    The *mean* dv barely moves (<0.5%), which is what the cost mostly sees, but
    individual runs shift by up to ~4 m/s. Since the cost also reads the
    worst-decile mean, loosening the tolerance perturbs the tails too. The
    default therefore stays at the config value used by the committed staged
    sweep, keeping new results comparable with it; ``--tol`` exposes the
    trade-off for anyone willing to accept noisier runs for wall time.
    """
    import heyoka  # noqa: F401  the extensions need libheyoka's symbols first
    from cubesat_12u_sim.native.cxxdrive import _nmpcsolve

    out = np.zeros((x0.shape[0], 10))
    _nmpcsolve.closed_loop_batch(
        np.ascontiguousarray(x0, dtype=float), int(horizon),
        float(dt_outer), float(dt_inner), int(n_outer), int(maxiter),
        float(params["db_a"]),
        float(C.NMPC_A_MAX), float(params["trim_a_max"]),
        float(params["trim_dv_budget"]), 0.2,
        float(C.PID_KP_R), float(C.PID_KI_R), float(C.PID_KD_R),
        float(C.PID_KP_T), float(C.PID_KI_T),
        float(C.PID_KP_N), float(C.PID_KI_N), float(C.PID_KD_N),
        float(C.PID_INT_CLAMP), float(C.PID_A_MAX), float(C.PID_DEADBAND),
        float(C.MU_EARTH), float(C.R_ORBIT), float(tol),
        # thrust/mass scaling folded into the applied-accel factor: the kernel
        # multiplies the commanded accel by this before propagating
        float(vg.alloc_eff * vg.a_max / C.PID_A_MAX), float(spec_box),
        # Same weights the scorer uses: L1 fuel against the box edge at 30% of
        # a 30 m/s budget, linear inside 20% of the box and log beyond.
        float(w_track), float(box_frac), float(w_final),
        float(fA), float(fx0), float(fg1),
        # j_star / has_incumbent drive the early abort; both stay off because a
        # Monte Carlo candidate needs every run's dv to build its distribution,
        # not a truncated bound on the incumbent
        float("inf"), False, 0, out)
    return out


def _pid_group(x0: np.ndarray, params: dict, vg: VehicleGroup, dt: float,
               n_steps: int, spec_box: float, w_viol: float,
               w_final: float) -> np.ndarray:
    """This vehicle group's runs through the fused PID batch kernel -> ``(n, 4)``.

    Schema: dv, max_abs_r_err, final_e_a, cost. Here ``a_max`` *is* the
    realised vehicle authority (the PID saturates on its own gain limits, not on
    a separate command ceiling), so the vehicle dispersion enters directly.

    The kernel batches over *candidates* against a single initial state, not
    over Monte Carlo runs — its batch axis is the wrong shape for this use. So
    the group's runs are looped here, one call per lane. That is ~0.2 ms per
    run, i.e. ~50 ms for a whole 250-run ensemble, against ~40 s for the NMPC
    arm: the shape mismatch costs nothing measurable. ``alloc_eff`` and
    ``a_max`` are shared by the group, so no lane needs its own call for those.
    """
    import heyoka  # noqa: F401
    from cubesat_12u_sim.native.cxxdrive import _pidbatch

    n = int(x0.shape[0])
    P = np.ascontiguousarray(
        np.array([params[k] for k in GAIN_KEYS] + [params["box_r"]], float)[None, :])
    out = np.zeros((n, 4))
    for i in range(n):
        _pidbatch.pid_batch_eval(
            P, np.ascontiguousarray(x0[i], dtype=float),
            float(dt), int(n_steps), float(spec_box), float(w_viol),
            float(w_final), float(vg.alloc_eff), float(C.MU_EARTH),
            float(C.R_ORBIT), float(vg.a_max), float(C.PID_INT_CLAMP),
            float(C.PID_DEADBAND), out[i:i + 1])
    return out


# ---------------------------------------------------------------------------
# candidate evaluation
# ---------------------------------------------------------------------------

def evaluate_candidate(theta: np.ndarray, design: MCDesign, *,
                       dt_outer: float, dt_inner: float, t_end: float,
                       spec_box: float, w_viol: float, w_final: float,
                       maxiter: int, weights: TailWeights | None = None,
                       pid_dt: float = C.DT_CONTROL,
                       max_candidate_s: float = 900.0,
                       tol: float = C.HEYOKA_TOL,
                       w_track: float = 20.0,
                       box_frac: float = 0.2) -> CandidateResult:
    """Score one hypercube vector over the whole Monte Carlo design.

    Both arms are evaluated over the identical design and combined as
    ``w_nmpc * J_nmpc + w_pid * J_pid``, so one scalar drives CMA-ES while the
    per-arm statistics stay visible in the report.

    A failing group is marked infeasible rather than raising: a candidate whose
    parameters make the solver diverge should rank last, exactly like every
    other bad candidate, not abort the generation.
    """
    weights = weights or TailWeights()
    theta = np.clip(np.asarray(theta, float).ravel(), *HYPER_SPACE.bounds())
    p = HYPER_SPACE.from_theta(theta)
    horizon = int(round(p["horizon"]))

    # A candidate that picks a horizon it cannot afford to simulate is not a
    # candidate, it is an outage. Past this budget one draw would dominate the
    # generation's wall time, so it is marked infeasible and ranks last rather
    # than stalling every worker behind it. At the default 0.5-day horizon all
    # of NMPC_HORIZONS fits, so this only bites if the horizon is pushed hard.
    if est_nmpc_s(horizon, design.n_runs) > max_candidate_s:
        return _over_budget_candidate(theta, horizon, design.n_runs)

    n_runs = design.n_runs
    n_outer = int(round(t_end / dt_outer))
    n_steps_pid = int(round(t_end / pid_dt))

    nmpc_params = {k: p[k] for k in NMPC_SPACE.names}
    pid_params = {k: p[k] for k in PID_SPACE.names}

    dv_n = np.full(n_runs, np.inf)
    box_n = np.full(n_runs, np.inf)
    ea_n = np.full(n_runs, np.inf)
    dv_p = np.full(n_runs, np.inf)
    box_p = np.full(n_runs, np.inf)
    ea_p = np.full(n_runs, np.inf)

    for g, vg in enumerate(design.vehicles):
        idx = design.group_runs(g)
        if idx.size == 0:
            continue
        x0 = design.x0[idx]
        if __debug__ and _TRACE:
            print(f"  [trace] group {g} lanes {idx.size} horizon {horizon} "
                  f"n_outer {n_outer} pid_steps {n_steps_pid}", flush=True)

        try:
            o = _nmpc_group(x0, nmpc_params, horizon, vg, dt_outer, dt_inner,
                            n_outer, maxiter, spec_box, tol, w_track,
                            box_frac, w_final)
            dv_n[idx], box_n[idx], ea_n[idx] = o[:, 0], o[:, 1], o[:, 2]
        except Exception:
            if _TRACE:
                traceback.print_exc()
            pass    # stays inf -> infeasible

        try:
            q = _pid_group(x0, pid_params, vg, pid_dt, n_steps_pid, spec_box,
                           w_viol, w_final)
            dv_p[idx], box_p[idx], ea_p[idx] = q[:, 0], q[:, 1], q[:, 2]
        except Exception:
            if _TRACE:
                traceback.print_exc()
            pass

    cost_n, stats_n = reduce_arm(dv_n, box_n, ea_n, spec_box, w_viol, w_final,
                                 weights)
    cost_p, stats_p = reduce_arm(dv_p, box_p, ea_p, spec_box, w_viol, w_final,
                                 weights)

    total = weights.w_nmpc * cost_n + weights.w_pid * cost_p
    return CandidateResult(cost=float(total), horizon=horizon, nmpc=stats_n,
                           pid=stats_p, theta=[float(x) for x in theta],
                           params=p)


__all__ = ["ArmStats", "CandidateResult", "TailWeights", "evaluate_candidate",
           "reduce_arm", "run_scores", "PENALTY_INFEASIBLE"]