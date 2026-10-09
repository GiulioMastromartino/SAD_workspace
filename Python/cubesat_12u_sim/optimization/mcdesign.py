"""Monte Carlo design: what the 250 runs per candidate actually vary.

Two levels of dispersion, both sampled **once** and reused for every candidate
in every generation:

* **injection** (per run): launch-vehicle velocity/position error. This is the
  perturbation the station-keeping controller exists to reject, and it is the
  only per-run freedom the C++ batch entry points expose — they take one
  initial state per lane.
* **vehicle** (per group): build tolerances that change how much of a
  commanded acceleration the bus actually delivers — thruster scale, bus mass,
  and the allocator's achieved/commanded ratio.

The vehicle level is per *group* rather than per run because ``alloc_eff`` and
``a_max`` are call scalars in both C++ kernels, not per-lane arrays. Grouping
is therefore free: one call per group with all the group's runs as batch lanes.
``n_vehicles * n_injections == n_runs``.

**Common random numbers are the whole point.** Every candidate is scored
against the identical 250-run design, so a cost difference between two
candidates is a property of the controllers and not of which launcher error
they happened to be handed. Re-drawing the design per candidate would bury the
signal under sampling noise: with a per-run dv spread of a few m/s, the
standard error of a 250-run mean is large enough to dominate the differences
between neighbouring candidates, and CMA-ES would be fitting noise. Fixed
design also means the tail statistics (worst/best 10%) compare like with like.

The design is therefore built once from ``seed``, passed by value into every
worker, and never regenerated inside an evaluation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from cubesat_12u_sim.core import config as C


@dataclass(frozen=True)
class MCConfig:
    """Dispersion levels for the search-time Monte Carlo.

    Defaults are sized for a 12U in LEO/GTO-class injection: the velocity error
    is dominated by a *bias* (the vehicle is delivered slow, which is the
    scenario the controller must actually null), with a spread around it from
    injection repeatability. Position error is small by comparison because a
    radial offset mostly trades against velocity through the vis-viva
    relation.

    ``sigma_*`` are relative (fractional) standard deviations.
    """

    n_runs: int = 250
    n_vehicles: int = 25
    seed: int = 20261007

    # injection
    dv_bias: float = -0.005        # nominal velocity error (fraction)
    sigma_dv: float = 2.0e-3       # injection repeatability (fraction of V)
    dr_sigma: float = 5.0e-4       # position error (fraction of R)
    dv_dir_sigma: float = 1.0e-4   # velocity direction error [rad]

    # vehicle build tolerances
    sigma_mass: float = 0.05       # bus dry mass
    sigma_thrust: float = 0.05     # thruster max thrust, per unit
    sigma_alloc: float = 0.05      # allocator achieved/commanded ratio
    alloc_eff_nom: float = 0.5     # nominal ratio (see objective.alloc_eff)

    @property
    def n_injections(self) -> int:
        """Runs per vehicle group."""
        return self.n_runs // self.n_vehicles


@dataclass(frozen=True)
class VehicleGroup:
    """One build of the bus, shared by every run in its lane group."""

    mass_frac: float       # mass / nominal
    thrust_frac: float     # thruster scale / nominal
    alloc_eff: float       # achieved / commanded accel
    a_max: float           # realisable per-axis accel [m/s^2]

    def as_dict(self) -> dict:
        return {"mass_frac": self.mass_frac, "thrust_frac": self.thrust_frac,
                "alloc_eff": self.alloc_eff, "a_max": self.a_max}


@dataclass
class MCDesign:
    """The frozen set of runs every candidate is evaluated against."""

    cfg: MCConfig
    x0: np.ndarray                 # (n_runs, 6) initial ECI states
    vehicles: list[VehicleGroup] = field(default_factory=list)
    # group index per run, so a candidate's lanes can be grouped for the batched
    # C++ calls (one call per group, since alloc_eff/a_max are call scalars)
    run_group: np.ndarray = field(default_factory=lambda: np.zeros(0, int))

    @property
    def n_runs(self) -> int:
        return int(self.x0.shape[0])

    def group_runs(self, g: int) -> np.ndarray:
        """Indices of the runs belonging to vehicle group ``g``."""
        return np.flatnonzero(self.run_group == g)

    def summary(self) -> dict:
        al = np.array([v.alloc_eff for v in self.vehicles])
        am = np.array([v.a_max for v in self.vehicles])
        return {"n_runs": self.n_runs, "n_vehicles": len(self.vehicles),
                "alloc_eff": {"mean": float(al.mean()), "std": float(al.std()),
                              "min": float(al.min()), "max": float(al.max())},
                "a_max": {"mean": float(am.mean()), "std": float(am.std()),
                          "min": float(am.min()), "max": float(am.max())},
                "dv_frac": {"mean": float(((np.linalg.norm(self.x0[:, 3:6], axis=1)
                                            / C.V_CIRC) - 1.0).mean()),
                            "std": float(((np.linalg.norm(self.x0[:, 3:6], axis=1)
                                           / C.V_CIRC) - 1.0).std())},
                "dr_frac_std": float((np.linalg.norm(self.x0[:, 0:3], axis=1)
                                      / C.R_ORBIT).std())}


def _rtn_axes(r: np.ndarray, v: np.ndarray):
    """Radial / along-track / normal unit vectors for a state pair."""
    rn = float(np.linalg.norm(r))
    R = r / rn
    h = np.cross(r, v)
    N = h / float(np.linalg.norm(h))
    T = np.cross(N, R)
    return R, T, N


def build_design(cfg: MCConfig = MCConfig()) -> MCDesign:
    """Sample the frozen design: ``n_vehicles`` builds x ``n_injections`` runs.

    Layout is group-major: run ``k`` of vehicle ``g`` sits at index
    ``g * n_injections + k``, so each group's runs are a contiguous slice and the
    batched C++ call can be handed ``x0[a:b]`` with no gather.
    """
    if cfg.n_vehicles > cfg.n_runs:
        raise ValueError("n_vehicles must not exceed n_runs")
    if cfg.n_runs % cfg.n_vehicles:
        raise ValueError(f"n_runs={cfg.n_runs} is not a multiple of "
                         f"n_vehicles={cfg.n_vehicles}")

    rng = np.random.default_rng(cfg.seed)
    n_inj = cfg.n_runs // cfg.n_vehicles

    # ---- vehicle builds -------------------------------------------------
    vehicles: list[VehicleGroup] = []
    for _ in range(cfg.n_vehicles):
        mass = 1.0 + cfg.sigma_mass * rng.standard_normal()
        thrust = 1.0 + cfg.sigma_thrust * rng.standard_normal()
        # the allocator ratio is a property of the thruster set, not the bus
        eff = cfg.alloc_eff_nom + cfg.sigma_alloc * rng.standard_normal()
        eff = float(np.clip(eff, 0.2, 0.8))
        # commanded accel converts to realised accel through BOTH the
        # available thrust and the bus mass; keeping them separate is what lets
        # a heavy vehicle with oversized thrusters look different from a light
        # one with small ones, even at equal a_max.
        a_max = C.PID_A_MAX * thrust / mass
        vehicles.append(VehicleGroup(float(mass), float(thrust), eff,
                                     float(a_max)))

    # ---- injection errors ----------------------------------------------
    x0 = np.zeros((cfg.n_runs, 6))
    run_group = np.repeat(np.arange(cfg.n_vehicles), n_inj)
    for i in range(cfg.n_runs):
        R0, T0, N0 = _rtn_axes(C.R0_ECI, C.V0_ECI)

        dv_frac = cfg.dv_bias + cfg.sigma_dv * rng.standard_normal()
        dr_frac = cfg.dr_sigma * rng.standard_normal()
        # small velocity-direction error, expressed as an angle per axis and
        # applied in RTN so it is independent of the magnitude error
        dtheta = cfg.dv_dir_sigma * rng.standard_normal(2)

        v = (C.V_CIRC * (1.0 + dv_frac)) * (T0 + dtheta[0] * R0 + dtheta[1] * N0)
        r = C.R_ORBIT * (1.0 + dr_frac) * (R0 + dtheta[0] * T0
                                           + dtheta[1] * N0)
        x0[i, 0:3] = r
        x0[i, 3:6] = v

    return MCDesign(cfg=cfg, x0=x0, vehicles=vehicles, run_group=run_group)


def propellant_g(dv: float, mass_frac: float = 1.0) -> float:
    """Propellant mass [g] for a given dv at nominal Isp.

    Kept beside the design because ``mass_frac`` is a build property that does
    not change the *dynamics* the search sees — a thruster delivers the same
    force whatever the bus weighs — but does change what the fuel weighs, so
    reporting dv alone would hide it.
    """
    mass = C.MASS * mass_frac
    return float(dv * mass / (C.ISP_JET * 9.80665) * 1000.0)


__all__ = ["MCConfig", "MCDesign", "VehicleGroup", "build_design",
           "propellant_g"]