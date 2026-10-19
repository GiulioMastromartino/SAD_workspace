"""Frozen Monte Carlo design for the attitude-loop sweep.

Same common-random-numbers discipline as the orbit pipeline: the dispersions
are sampled once from fixed seeds and every candidate is scored against the
identical ensemble, so a cost difference between two candidates is a property
of the controllers, not of which dispersions they happened to be handed.

Two things make this cheaper than it looks:

1. **Everything but the gains is static.** Dispersions (from fixed seeds),
   wrench setups, REFS/PH tables, sun directions and the nominal allocator are
   identical for every candidate. The worker precomputes that whole bundle
   once at init (``build_bundle``); per candidate it swaps 5 scalars and makes
   one ``run_chunk`` call. Without this, each candidate would re-run
   ``sample_dispersions`` + ``wrench_setup`` over all lanes for no reason.

2. **Determinism is total.** Dispersions come from ``sample_dispersions(seed)``
   (numpy, fixed seeds) *and* the per-tick gyro noise comes from C++
   ``mt19937_64`` seeded by ``seeds + 999983``. Both are functions of the seed
   alone, so the ensemble is bit-identical for every candidate in every
   generation. Re-drawing per candidate would bury the signal under sampling
   noise, exactly as in the orbit pipeline.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from cubesat_12u_sim.backends.heyoka.heyoka_attitude import (
    precompute_sun_refs, sun_direction_at, wrench_setup)
from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.core.actuators import allocation_matrices
from cubesat_12u_sim.uncertainty.monte_carlo import (
    MonteCarloConfig, sample_dispersions)


@dataclass
class AttBundle:
    """The static per-worker argument bundle for ``run_chunk``.

    Every array is built once from the frozen seeds; ``evaluate_candidate``
    only overwrites the 5 controller scalars per call.
    """

    n_runs: int
    seeds: list[int]
    disps: list = field(default_factory=list)
    r0: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    v0: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    sigma0: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    omega0: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    mass: np.ndarray = field(default_factory=lambda: np.zeros(0))
    I6: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    invI6: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    MF: np.ndarray = field(default_factory=lambda: np.zeros((0, 0, 0)))
    MT: np.ndarray = field(default_factory=lambda: np.zeros((0, 0, 0)))
    tmax: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    isp: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    gyro_arw: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    P_nom: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    REFS: np.ndarray = field(default_factory=lambda: np.zeros((0, 0, 0)))
    PH: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int64))
    sun_dirs: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    bore: np.ndarray = field(default_factory=lambda: np.zeros(3))
    n_steps: int = 0
    log_every: int = 1

    def summary(self) -> dict:
        return {"n_runs": self.n_runs, "n_steps": self.n_steps,
                "seeds": [int(s) for s in self.seeds]}


def build_bundle(n_runs: int = 50, base_seed: int = 20261007,
                 mc_cfg: MonteCarloConfig | None = None,
                 task_step: float = 0.02,
                 log_step: float = 1.0) -> AttBundle:
    """Sample the frozen ensemble and precompute every static argument.

    ``task_step``/``log_step`` are explicit (not taken from ``mc_cfg``) because
    the search resolution is a property of the sweep, while ``mc_cfg`` carries
    the mission: durations, targets, thresholds and dispersion levels.
    """
    mc_cfg = mc_cfg or MonteCarloConfig()
    seeds = [int(base_seed + i) for i in range(n_runs)]
    dt = float(task_step)
    n_steps = int(round(mc_cfg.t_total / dt))
    log_every = max(1, int(round(float(log_step) / dt)))

    disps = [sample_dispersions(int(s), mc_cfg) for s in seeds]
    sun_refs = precompute_sun_refs(mc_cfg, np.asarray(mc_cfg.sun_roll_ref,
                                                      float), dt)
    sigma_init = np.array([np.asarray(d["sigma0"], float) for d in disps])
    k_det = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                float(mc_cfg.t_detumble), side="left"))
    k_slew = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                 float(mc_cfg.t_detumble + mc_cfg.t_slew),
                                 side="left"))
    REFS = np.empty((n_steps + 1, n_runs, 3))
    REFS[:k_det] = sigma_init[None, :, :]
    REFS[k_det:k_slew] = np.asarray(mc_cfg.sigma_slew, float)[None, None, :]
    REFS[k_slew:] = np.asarray(sun_refs[k_slew:])[:, None, :]
    PH = np.zeros(n_steps + 1, dtype=np.int64)
    PH[k_det:k_slew] = 1
    PH[k_slew:] = 2

    ws_all = [wrench_setup(d) for d in disps]
    P_nom = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR,
                                C.THRUSTER_TMAX)
    sun_dirs = np.array([sun_direction_at(float(k * dt))
                         for k in range(0, n_steps + 1, log_every)])

    return AttBundle(
        n_runs=n_runs, seeds=seeds, disps=disps,
        r0=np.array([d["r0"] for d in disps], float),
        v0=np.array([d["v0"] for d in disps], float),
        sigma0=np.array([d["sigma0"] for d in disps], float),
        omega0=np.array([d["omega0"] for d in disps], float),
        mass=np.array([float(d["mass"]) for d in disps]),
        I6=np.array([w["I6"] for w in ws_all]),
        invI6=np.array([w["invI6"] for w in ws_all]),
        MF=np.stack([w["M_F"] for w in ws_all]),
        MT=np.stack([w["M_T"] for w in ws_all]),
        tmax=np.stack([w["tmax"] for w in ws_all]),
        isp=np.array([float(d["thruster_isp"]) for d in disps]),
        gyro_bias=np.array([d["gyro_bias"] for d in disps], float),
        gyro_arw=np.array([d["gyro_arw"] for d in disps], float),
        P_nom=np.stack(P_nom),
        REFS=np.ascontiguousarray(REFS), PH=PH, sun_dirs=sun_dirs,
        bore=np.asarray(mc_cfg.sun_pointing_axis_b, float),
        n_steps=n_steps, log_every=log_every)


__all__ = ["AttBundle", "build_bundle"]
