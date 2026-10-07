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

from .. import config as C


@dataclass(frozen=True)
class Param:
    """One optimisable scalar, specified by its natural-log bounds."""

    name: str
    lo_log: float
    hi_log: float
    default_log: float
    unit: str = ""

    def __post_init__(self) -> None:
        if not (self.lo_log <= self.default_log <= self.hi_log):
            raise ValueError(
                f"{self.name}: default log {self.default_log} outside "
                f"[{self.lo_log}, {self.hi_log}]")


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
        """Decode a log-space vector into a dict of physical values."""
        theta = np.clip(np.asarray(theta, float).ravel(),
                        *self.bounds())
        return {p.name: float(math.exp(t)) for p, t in zip(self.params, theta)}

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
            lines.append(f"  {i:2d} {p.name:<22} "
                         f"[{math.exp(lb[i]):.3g}, {math.exp(ub[i]):.3g}] {p.unit} "
                         f"(default {math.exp(p.default_log):.3g})")
        return "\n".join(lines)


def _p(name, lo, hi, default, unit=""):
    """Param from *physical* bounds/default (more readable than raw logs)."""
    return Param(name, math.log(lo), math.log(hi), math.log(default), unit)


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
# NMPC arm: cost weights + energy deadband + inner-trim authority/budget.
# Horizon is structural and swept separately (it changes the problem size,
# not just the gains) — same treatment as the phase-count sweep in the
# reference implementation.
# --------------------------------------------------------------------------
NMPC_SPACE = ParamSpace([
    _p("q_pos", 1e-2, 1e3, C.NMPC_Q_POS, "-"),
    _p("q_vel", 1e-3, 1e2, C.NMPC_Q_VEL, "-"),
    _p("r_fuel", 1e-3, 1e3, C.NMPC_R_FUEL, "-"),
    _p("db_a", 50.0, 5e4, C.NMPC_DB_A, "m"),
    _p("trim_a_max", 1e-4, 5e-2, C.TRIM_A_MAX, "m/s^2"),
    _p("trim_dv_budget", 1e-3, 50.0, C.TRIM_DV_BUDGET, "m/s"),
])

# Joint space: both arms at once, so the reported head-to-head compares two
# tuned controllers rather than a tuned controller against a hand-tuned one.
JOINT_SPACE = ParamSpace(NMPC_SPACE.params + PID_SPACE.params)

NMPC_HORIZONS = (2, 4, 6, 8, 12)
