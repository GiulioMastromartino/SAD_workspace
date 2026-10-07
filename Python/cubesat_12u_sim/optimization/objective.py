"""Closed-loop evaluation of a controller parameter vector.

Design mirrors the reference tuning work: a cheap deterministic propagator for
the search, the full simulation reserved for validating the winner.

**Search propagator** is ``_rk4_step_orbit`` (pure numpy, 6-state [r, v]) — a
2-day arc costs ~36 ms, negligible next to the NMPC solve. The full
heyoka/12-state thruster path is used only by :func:`validate_candidate`, so a
sweep of thousands of evaluations never pays integrator-compile cost and stays
reproducible (no adaptive-solver state to leak between candidates).

**Objective** is fuel subject to a station-keeping box::

    J = dv
      + W_VIOL  * max(0, max_t|r_err| / spec_box - 1)^2
      + W_FINAL * (e_a(T) / 50 km)^2

with a hard feasibility flag reported alongside. The box term is a
specification constraint, not a preference: a candidate that leaves the box is
never selected as best, only ranked by how badly.

Symmetry note: both arms get a coast threshold as an optimised parameter
(``box_r`` for the PID, ``db_a`` for NMPC) *and* are judged against the same
external ``spec_box``, so neither arm can satisfy the spec by simply
suppressing thrust.

Gains are injected per instance rather than through ``config`` because the
controller reads ``C.PID_K*`` module-globally, which is not safe to vary across
concurrent worker processes.
"""
from __future__ import annotations

import hashlib
from dataclasses import asdict, dataclass

import numpy as np

from .. import config as C
from ..actuators import allocation_pinv, jets_to_wrench
from ..controller import OrbitPID, rtn_frame
from ..hierarchical import FastTrimPID, HierarchicalController
from ..nmpc import (NMPCConfig, _rk4_step_orbit, semi_major_axis)

from .params import NMPC_SPACE, PID_SPACE, ParamSpace

PENALTY_INFEASIBLE = 1e9
GAIN_KEYS = ("kp_r", "ki_r", "kd_r", "kp_t", "ki_t", "kp_n", "ki_n", "kd_n")


@dataclass(frozen=True)
class EvalConfig:
    """Resolution and constraint settings for one closed-loop evaluation.

    Search uses a coarse horizon so a candidate costs ~2 s; validation uses the
    fine resolution from ``run_nmpc_compare`` so the reported numbers are the
    ones that would actually ship.
    """

    t_days: float = 0.5
    dt_outer: float = 600.0
    dt_inner: float = 60.0
    v_frac: float = -0.005          # initial velocity injection
    spec_box: float = 25e3          # [m] station-keeping box half-width
    w_viol: float = 1e3             # box penalty weight
    w_final: float = 0.05           # terminal energy-error weight
    maxiter: int = 50               # SLSQP iteration cap during search
    alloc_eff: float = 0.5          # commanded/achieved accel ratio, see below
    seed: int = 0

    @property
    def t_end(self) -> float:
        return self.t_days * 86400.0

    def fingerprint(self) -> str:
        payload = "|".join(f"{k}={v}" for k, v in sorted(asdict(self).items()))
        return hashlib.md5(payload.encode()).hexdigest()[:12]

    def with_dt(self, dt_outer: float, dt_inner: float) -> "EvalConfig":
        from dataclasses import replace
        return replace(self, dt_outer=dt_outer, dt_inner=dt_inner)


# Each arm is searched at the resolution it would actually be flown at: the PID
# is a 10 s zero-order-hold loop (C.DT_CONTROL), the long-horizon NMPC replans
# at 600 s with 60 s trim ticks. Sharing one resolution between them would
# penalise whichever arm is run away from its own control period.
SEARCH_CFG = EvalConfig()
PID_SEARCH_CFG = SEARCH_CFG.with_dt(C.DT_CONTROL, C.DT_CONTROL)
VALIDATE_CFG = EvalConfig(t_days=2.0, dt_outer=600.0, dt_inner=60.0,
                          maxiter=C.NMPC_MAXITER)


@dataclass
class Result:
    cost: float
    dv: float
    max_abs_r_err: float
    final_e_a: float
    feasible: bool
    notes: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


# ---------------------------------------------------------------------------
# controllers
# ---------------------------------------------------------------------------

class TunedOrbitPID(OrbitPID):
    """OrbitPID with instance-local gains.

    The control law is OrbitPID's unchanged; only the ``C.PID_K*`` lookups are
    redirected to ``self.gains`` so several parameter sets can be evaluated
    inside one process (and across worker processes) without touching global
    config.
    """

    def __init__(self, gains: dict, box_r: float):
        super().__init__(box_r=float(box_r), deadband=C.PID_DEADBAND)
        self.gains = {k: float(gains[k]) for k in GAIN_KEYS}

    def accel_command(self, r, v, dt):
        g = self.gains
        r_norm = float(np.linalg.norm(r))
        if self.box_r > 0.0 and abs(r_norm - C.R_ORBIT) < self.box_r:
            self.int_r *= 0.9
            self.int_t *= 0.9
            self.int_n *= 0.9
            return np.zeros(3)

        R_h, T, N = rtn_frame(r, v)
        v_r = float(v @ R_h)
        v_t = float(v @ T)
        v_n = float(v @ N)
        r_n = float(r @ N)
        v_circ = float(np.sqrt(C.MU_EARTH / r_norm))

        e_r = C.R_ORBIT - r_norm
        prev = (self.int_r, self.int_t, self.int_n)
        self.int_r = np.clip(self.int_r + e_r * dt, -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_R = g["kp_r"] * e_r + g["ki_r"] * self.int_r - g["kd_r"] * v_r

        e_t = v_circ - v_t
        self.int_t = np.clip(self.int_t + e_t * dt, -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_T = g["kp_t"] * e_t + g["ki_t"] * self.int_t

        e_n = -r_n
        self.int_n = np.clip(self.int_n + e_n * dt, -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_N = g["kp_n"] * e_n + g["ki_n"] * self.int_n - g["kd_n"] * v_n

        a_rtn = np.clip(np.array([a_R, a_T, a_N]), -C.PID_A_MAX, C.PID_A_MAX)
        if np.all(np.abs(a_rtn) < self.deadband):
            self.int_r, self.int_t, self.int_n = prev
            return np.zeros(3)
        a_rtn[np.abs(a_rtn) < self.deadband] = 0.0
        return R_h * a_rtn[0] + T * a_rtn[1] + N * a_rtn[2]


def make_pid(params: dict) -> TunedOrbitPID:
    return TunedOrbitPID(params, box_r=params["box_r"])


def make_nmpc(params: dict, cfg: EvalConfig, horizon: int,
              backend: str = "numpy") -> HierarchicalController:
    from ..nmpc import ScipyShootingBackend
    ncfg = NMPCConfig(dt_outer=cfg.dt_outer, horizon=int(horizon),
                      q_pos=params["q_pos"], q_vel=params["q_vel"],
                      r_fuel=params["r_fuel"], db_a=params["db_a"],
                      maxiter=cfg.maxiter)
    trim = FastTrimPID(authority=params["trim_a_max"],
                       dv_budget=params["trim_dv_budget"])
    if backend == "cpp":
        be = ScipyShootingBackend(use_cpp=True)
    else:
        be = ScipyShootingBackend(use_cpp=False)
    return HierarchicalController(ncfg, trim, backend=be)


# ---------------------------------------------------------------------------
# closed-loop rollout
# ---------------------------------------------------------------------------

def measure_alloc_eff(n_samples: int = 64, seed: int = 0) -> float:
    """Measure the jet allocator's mean |achieved| / |commanded| force ratio.

    Reported in the results JSON so the surrogate's fidelity is visible rather
    than assumed. Currently 0.5 for this geometry.
    """
    rng = np.random.default_rng(seed)
    ratios = []
    for _ in range(n_samples):
        e = rng.normal(size=3)
        e /= np.linalg.norm(e)
        F = e * (C.PID_A_MAX * C.MASS)
        Fa, _ = jets_to_wrench(allocation_pinv(F, np.zeros(3)))
        ratios.append(float(np.linalg.norm(Fa) / np.linalg.norm(F)))
    return float(np.mean(ratios))


def initial_state(v_frac: float) -> np.ndarray:
    return np.concatenate([C.R0_ECI.copy(), C.V0_ECI.copy() * (1.0 + v_frac)])


def simulate(accel_fn, cfg: EvalConfig) -> tuple[np.ndarray, float, float]:
    """Run the closed loop; return ``(final x, total dv, max |r_err|)``.

    ``accel_fn`` returns the *commanded* ECI acceleration. The commanded value
    is scaled by ``cfg.alloc_eff`` before it is applied and counted, because
    the real 12-jet allocator does not deliver what it is asked for: the
    least-squares throttle solution for a pure force command has negative
    components (the jets are one-way, u >= 0), and clipping them to zero leaves
    exactly half the requested thrust. Tuned against the raw command, the
    optimiser would size every gain for a vehicle with twice the real
    authority, and the tuned result would fail the full-simulation check.
    """
    x = initial_state(cfg.v_frac)
    dt = cfg.dt_inner
    n_steps = int(round(cfg.t_end / dt))
    eff = cfg.alloc_eff
    dv = 0.0
    max_abs_r_err = 0.0
    for _ in range(n_steps):
        a = np.asarray(accel_fn(x[0:3], x[3:6], dt), float) * eff
        if not np.all(np.isfinite(a)):
            return x, float("inf"), float("inf")
        dv += float(np.linalg.norm(a)) * dt
        x = _rk4_step_orbit(x, a, dt)
        if not np.all(np.isfinite(x)):
            return x, float("inf"), float("inf")
        err = abs(float(np.linalg.norm(x[0:3])) - C.R_ORBIT)
        if err > max_abs_r_err:
            max_abs_r_err = err
    return x, dv, max_abs_r_err


def score(dv: float, max_abs_r_err: float, final_e_a: float,
          cfg: EvalConfig) -> Result:
    if not np.isfinite(dv) or not np.isfinite(max_abs_r_err):
        return Result(PENALTY_INFEASIBLE, float("inf"), float("inf"),
                      float("inf"), False, "non-finite")
    excess = max(0.0, max_abs_r_err / cfg.spec_box - 1.0)
    cost = (dv
            + cfg.w_viol * excess ** 2
            + cfg.w_final * (final_e_a / 50e3) ** 2)
    return Result(cost=float(cost), dv=float(dv),
                  max_abs_r_err=float(max_abs_r_err),
                  final_e_a=float(final_e_a),
                  feasible=bool(excess == 0.0),
                  notes="" if excess == 0.0 else
                  f"box exceeded by {excess * cfg.spec_box / 1e3:.2f} km")


def _evaluate(make_ctrl, cfg: EvalConfig, cache: dict | None, key) -> Result:
    if cache is not None and key in cache:
        return cache[key]
    x, dv, max_err = simulate(make_ctrl().accel_command, cfg)
    res = score(dv, max_err, semi_major_axis(x[0:3], x[3:6]) - C.R_ORBIT, cfg)
    if cache is not None:
        cache[key] = res
    return res


def evaluate_pid(theta, cfg: EvalConfig, cache: dict | None = None,
                 space: ParamSpace = PID_SPACE) -> Result:
    params = space.from_theta(theta)
    key = ("pid", cfg.fingerprint(), np.asarray(theta, float).round(9).tobytes())
    return _evaluate(lambda: make_pid(params), cfg, cache, key)


def evaluate_nmpc(theta, cfg: EvalConfig, horizon: int, cache: dict | None = None,
                  space: ParamSpace = NMPC_SPACE,
                  backend: str = "numpy") -> Result:
    params = space.from_theta(theta)
    key = ("nmpc", backend, cfg.fingerprint(), int(horizon),
           np.asarray(theta, float).round(9).tobytes())
    return _evaluate(lambda: make_nmpc(params, cfg, horizon, backend=backend),
                     cfg, cache, key)


def evaluate_joint(theta, cfg: EvalConfig, horizon: int,
                   cache: dict | None = None, nmpc_backend: str = "numpy"):
    """Evaluate both arms from one concatenated vector (NMPC params first).

    Returns ``(nmpc_result, pid_result)``. The NMPC result is the objective;
    the PID result rides along so the joint stage reports a tuned-vs-tuned
    comparison rather than tuned-vs-default.
    """
    theta = np.asarray(theta, float)
    split = len(NMPC_SPACE)
    return (evaluate_nmpc(theta[:split], cfg, horizon, cache,
                           backend=nmpc_backend),
            evaluate_pid(theta[split:], cfg, cache))
