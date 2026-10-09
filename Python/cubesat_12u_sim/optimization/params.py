"""Log-space parameter spaces for the ACS tuning sweep.

Gains and thresholds span several orders of magnitude, so every parameter is
optimised as its natural log. That turns multiplicative bounds into additive
ones, keeps all values positive, and gives CMA-ES a roughly isotropic search
space instead of one dominated by the largest-magnitude coordinate.

Each space also carries a *default* — the current hand-tuned value in
``config.py`` — which doubles as the seed mean for CMA-ES and as the fallback
when a stage is skipped.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from cubesat_12u_sim.core import config as C


@dataclass(frozen=True)
class Param:
    """One optimisable scalar, specified by its natural-log bounds.

    ``choices`` turns the parameter into a *discrete* one: the optimiser still
    proposes a continuous log value, but it is snapped to the nearest allowed
    value before use. That is the only honest way to put a structural knob like
    the NMPC horizon inside the same hypercube as the gains — CMA-ES then sees
    a plateaued landscape rather than a smooth one, which is a real cost and is
    documented at the call site rather than hidden.
    """

    name: str
    lo_log: float
    hi_log: float
    default_log: float
    unit: str = ""
    choices: tuple = ()

    def __post_init__(self) -> None:
        if not (self.lo_log <= self.default_log <= self.hi_log):
            raise ValueError(
                f"{self.name}: default log {self.default_log} outside "
                f"[{self.lo_log}, {self.hi_log}]")
        if self.choices:
            logs = [math.log(c) for c in self.choices]
            if min(logs) < self.lo_log - 1e-12 or max(logs) > self.hi_log + 1e-12:
                raise ValueError(
                    f"{self.name}: choices {self.choices} escape the log bounds "
                    f"[{self.lo_log}, {self.hi_log}]")
            if not any(abs(math.log(c) - self.default_log) < 1e-12
                       for c in self.choices):
                raise ValueError(
                    f"{self.name}: default is not one of the choices")


class ParamSpace:
    """An ordered, named set of log-space parameters."""

    def __init__(self, params: list[Param]):
        self.params = list(params)

    def __len__(self) -> int:
        return len(self.params)

    @property
    def names(self) -> list[str]:
        return [p.name for p in self.params]

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        lb = np.array([p.lo_log for p in self.params])
        ub = np.array([p.hi_log for p in self.params])
        return lb, ub

    def seed_theta(self) -> np.ndarray:
        return np.array([p.default_log for p in self.params])

    def random_theta(self, rng: np.random.Generator) -> np.ndarray:
        lb, ub = self.bounds()
        return rng.uniform(lb, ub)

    def from_theta(self, theta) -> dict[str, float]:
        """Decode a log-space vector into a dict of physical values.

        A :class:`Param` with ``choices`` snaps to the nearest choice in *log*
        space, which is the metric the search actually moves in; snapping in
        linear space would bias against the large choices.
        """
        theta = np.clip(np.asarray(theta, float).ravel(), *self.bounds())
        out: dict[str, float] = {}
        for p, t in zip(self.params, theta):
            v = float(math.exp(t))
            if p.choices:
                v = float(min(p.choices, key=lambda c: abs(math.log(c) - t)))
            out[p.name] = v
        return out

    def to_theta(self, values: dict[str, float]) -> np.ndarray:
        """Encode physical values into a log-space vector (defaults fill gaps)."""
        out = self.seed_theta()
        lb, ub = self.bounds()
        for i, p in enumerate(self.params):
            if p.name in values:
                out[i] = math.log(max(float(values[p.name]), 1e-300))
        return np.clip(out, lb, ub)

    def describe(self) -> str:
        lb, ub = self.bounds()
        lines = []
        for i, p in enumerate(self.params):
            if p.choices:
                lines.append(f"  {i:2d} {p.name:<22} discrete {p.choices} {p.unit} "
                             f"(default {math.exp(p.default_log):.3g})")
            else:
                lines.append(f"  {i:2d} {p.name:<22} "
                             f"[{math.exp(lb[i]):.3g}, {math.exp(ub[i]):.3g}] {p.unit} "
                             f"(default {math.exp(p.default_log):.3g})")
        return "\n".join(lines)


def _p(name, lo, hi, default, unit=""):
    """Param from *physical* bounds/default (more readable than raw logs)."""
    return Param(name, math.log(lo), math.log(hi), math.log(default), unit)


def _d(name, choices, default, unit=""):
    """Discrete param from a set of allowed values."""
    return Param(name, math.log(min(choices)), math.log(max(choices)),
                 math.log(default), unit, tuple(choices))


# --------------------------------------------------------------------------
# PID arm: OrbitPID RTN gains + its coast threshold (box_r).
# Bounds are ~2 decades either side of the hand-tuned value; anything further
# is either unstable (too high) or indistinguishable from zero (too low).
# --------------------------------------------------------------------------
PID_SPACE = ParamSpace([
    _p("kp_r", 1e-7, 1e-4, C.PID_KP_R, "1/s^2"),
    _p("ki_r", 1e-12, 1e-6, C.PID_KI_R, "1/s^3"),
    _p("kd_r", 1e-4, 1e-1, C.PID_KD_R, "1/s"),
    _p("kp_t", 1e-5, 1e-1, C.PID_KP_T, "1/s"),
    _p("ki_t", 1e-9, 1e-3, C.PID_KI_T, "1/s^2"),
    _p("kp_n", 1e-7, 1e-4, C.PID_KP_N, "1/s^2"),
    _p("ki_n", 1e-12, 1e-6, C.PID_KI_N, "1/s^3"),
    _p("kd_n", 1e-4, 1e-1, C.PID_KD_N, "1/s"),
    # box_r = the PID's own coast threshold. Lower bound is 0 in effect (the
    # config default, "always correct"); the upper bound is generous because a
    # wide box lets the natural epicycle ride for free, which is exactly the
    # trade-off the sweep is meant to expose.
    _p("box_r", 1e-2, 1e5, 25e3, "m"),
])

# --------------------------------------------------------------------------
# NMPC arm: inner-trim authority/budget + the box-planning margin.
#
# NOTE: the cost weights (q_pos, q_vel, r_fuel) are deliberately GONE from this
# space. The planner's internal objective is now the scorer's objective
# (see nmpc_solve.cpp shoot_cost): fuel as L1 dv, position as radius error
# against a box fraction, terminal energy as a small tie-breaker. Leaving
# tracking weights in the search space is what produced the previous run's
# pathology -- the optimiser inflated q_vel 329x to buy radial-velocity
# accuracy that nothing scores, and drove q_pos to 0.013 so position error
# stopped mattering, leaving the run riding the station-keeping box at 99.3% of
# its limit.
#
# What is left is genuinely structural:
#   trim_a_max     - authority of the inner PID trim that the bulk plan fights
#   trim_dv_budget - how much dv the trim may spend per outer tick
#   db_a           - deadband on the terminal energy term only
#
# trim_dv_budget is capped at 5e-2 m/s/tick (3.6 m/s over the 0.5-day window)
# because a larger budget makes the hierarchy degenerate: the trim saturates and
# cancels the bulk plan every tick, so `bulk` contributes 0.00 and the radius
# error becomes whatever the trim produces. Measured at the old default of
# 0.5 m/s/tick: bulk 0.00 / trim 36.00 and a 443 km box error that was
# bit-identical for every w_track from 9 to 40 -- i.e. no amount of tuning the
# cost could recover it. The span now matches trim_a_max, which is the same
# order of magnitude and already capped.
#
# The position-tracking shape is FIXED, not searched: linear inside 20% of the
# box, logarithmic out to the edge and beyond, with w(edge) = 20 m/s = 67% of a
# 30 m/s fuel budget. It is always strictly positive away from dead centre, so
# the planner can never find coasting free -- which a one-sided hinge allowed,
# and which made the closed loop drift to 1437 km of box error.
# --------------------------------------------------------------------------
NMPC_SPACE = ParamSpace([
    _p("trim_a_max", 1e-4, 5e-2, C.TRIM_A_MAX, "m/s^2"),
    _p("trim_dv_budget", 1e-4, 5e-2, C.TRIM_DV_BUDGET, "m/s"),
    _p("db_a", 50.0, 5e4, C.NMPC_DB_A, "m"),
    # Fuel-cost amplitude, as a multiple of a_max*dt. 1.0 = "one tick at full
    # authority costs the m/s the scorer counts", the natural calibration.
    #
    # This is a search coordinate rather than a fixed constant because it is the
    # only knob that moves the NMPC along the fuel/feasibility trade-off, and in
    # the band that actually matters -- 90-100% feasible -- that trade is a real
    # one. Below 90% the +1000*infeasible_fraction term swamps any fuel saving
    # (each 2% of feasibility costs ~20, against a whole-run fuel budget of
    # 30-90), so the optimiser will not linger there and the coordinate is
    # effectively gated for free.
    #
    # Span is ~1.5 decades either side of 1.0: enough to reach both the
    # expensive-and-safe and cheap-and-sloppy regimes, not so wide that a full
    # decade of it is never evaluated.
    _p("fuel_scale", 0.3, 3.0, 1.0, "-"),
])

# Joint space: both arms at once, so the reported head-to-head compares two
# tuned controllers rather than a tuned controller against a hand-tuned one.
JOINT_SPACE = ParamSpace(NMPC_SPACE.params + PID_SPACE.params)

NMPC_HORIZONS = (2, 4, 6, 8, 12)

# Horizon values the *global sweep* is allowed to choose from.
#
# Capped at H=6 for two measured reasons, not taste:
#
# 1. **Cost.** The closed-loop cost is superlinear in H because the SLSQP
#    gradient is an expanded 3N+1-lane Taylor batch: measured 28/76/158 ms per
#    lane at H=2/4/6, rising to ~420 ms at H=12. With 250 runs per candidate, an
#    H=12 draw costs a worker ~105 s against ~19 s for H=4.
# 2. **Straggler cost.** CMA-ES draws the horizon as a free coordinate, so a
#    population ends up mixed — a real 25-generation run produced a mix of
#    H2:142 H4:938 H6:1253 H8:991 H12:426. Under an equal-count shard the
#    generation's wall clock is set by whichever worker drew the longest horizon,
#    so the tail candidates buy no extra information but hold the generation
#    open several times over.
#
# The staged sweep keeps the full (2, 4, 6, 8, 12) set: it runs one horizon at a
# time and is not latency-bound, so the long-horizon candidates cost wall time
# without stranding a pool.
MC_HORIZONS = (2, 4, 6)

# --------------------------------------------------------------------------
# Global hypercube: every tunable of the NMPC+PID pipeline in one box, with
# the horizon as a discrete coordinate.
#
# The horizon changes the NMPC decision-vector size, which is why the staged
# sweep treated it as a structural sweep rather than a gain. On the *batch* C++
# path it is only a runtime argument, so a single search can optimise it
# alongside everything else. The price is a plateaued coordinate: CMA-ES
# proposes a continuous value, :meth:`ParamSpace.from_theta` snaps it to the
# nearest horizon, and every proposal between two horizons scores identically.
# That is a genuine loss of gradient information, paid for by not having to
# guess the horizon in advance. The allowed values are capped at
# :data:`MC_HORIZONS` for cost and straggler reasons documented there.
# --------------------------------------------------------------------------
HYPER_SPACE = ParamSpace(NMPC_SPACE.params + PID_SPACE.params +
                         # H=4 as the default, NOT H=6: the stage where the
                         # default must sit strictly inside the bounds that
                         # _bounds() hands to cma (which shrink them by 1e-12 to
                         # stay strictly outside the domain). A default sitting on
                         # an edge -- log(6) is the upper bound -- makes the seed
                         # vector violate the box it created, and CMAEvolutionStrategy
                         # raises "argument of inverse must be within the given
                         # bounds" before the first ask().
                         [_d("horizon", MC_HORIZONS, 4, "steps")])

# Index at which the horizon sits, so evaluators can split a theta without
# hard-coding 6/9/16.
HYPER_NMPC_SLICE = slice(0, len(NMPC_SPACE))
HYPER_PID_SLICE = slice(len(NMPC_SPACE), len(NMPC_SPACE) + len(PID_SPACE))
HYPER_HORIZON_IDX = len(NMPC_SPACE) + len(PID_SPACE)
