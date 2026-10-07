"""Modular NMPC high-level controller (scipy POC, swappable backend).

Hierarchy:
  outer NMPC @ NMPC_DT_OUTER (default 10 s): solves a short-horizon optimal
    control problem for bulk ECI accel (+ small torque) and holds u0 for the
    whole outer window (zero-order hold).
  inner fast PID @ NMPC_DT_INNER (default 1 s, see hierarchical.py): trims
    tracking error under TRIM_A_MAX / TRIM_DV_BUDGET.

Modularity contract (so you can drop in casadi/acados later without touching
callers): any solver must subclass :class:`SolverBackend` and implement
``solve(x0, ref) -> NMPCSolution``. :class:`HighLevelNMPC` only talks to the
backend through that method + :class:`NMPCConfig`. To add casadi: create
``CasadiShootingBackend(SolverBackend)`` with the same signature and pass
``backend="casadi"`` (or an instance) to HighLevelNMPC.

POC prediction model (scipy backend): 6-state Kepler + ECI accel control
(translation is what burns station-keeping fuel); attitude is tracked by the
existing MRP PD / fast trim and enters only through the torque channel +
allocation feasibility check. The control vector is still 6-D [a_eci(3),
tau_body(3)] so the interface is already full 6-DOF coupled; a future backend
can add MRP states to the rollout without changing callers.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from . import config as C


@dataclass
class NMPCConfig:
    dt_outer: float = C.NMPC_DT_OUTER
    horizon: int = C.NMPC_HORIZON
    q_pos: float = C.NMPC_Q_POS  # weight on deadbanded semi-major error (energy)
    q_vel: float = C.NMPC_Q_VEL  # weight on radial-velocity damping
    r_fuel: float = C.NMPC_R_FUEL
    db_a: float = C.NMPC_DB_A  # [m] coast deadband on |a_osc - R_ORBIT|
    a_max: float = C.NMPC_A_MAX
    tau_max: float = C.NMPC_TAU_MAX
    solver: str = C.NMPC_SOLVER
    maxiter: int = C.NMPC_MAXITER
    fallback: str = C.NMPC_FALLBACK  # "pid" | "zero"
    mode: str = "orbit"  # "orbit" (6-state) only for now; "full" reserved


@dataclass
class NMPCSolution:
    a_eci: np.ndarray  # (3,) first-step bulk accel actually applied
    tau_body: np.ndarray  # (3,) first-step torque (small authority)
    u_seq: np.ndarray  # (N,3) planned accel sequence (diagnostics/MPC warm start)
    pred_traj: np.ndarray  # (N+1,6) predicted [r,v] (diagnostics)
    cost: float
    success: bool
    info: dict = field(default_factory=dict)


def _kepler_accel(r: np.ndarray) -> np.ndarray:
    R = float(np.linalg.norm(r))
    return -C.MU_EARTH / R**3 * r


def _rk4_step_orbit(x: np.ndarray, a_eci: np.ndarray, dt: float) -> np.ndarray:
    """One RK4 step for 6-state [r,v] with piecewise-constant ECI accel."""
    r, v = x[0:3], x[3:6]

    def f(xx):
        rr, vv = xx[0:3], xx[3:6]
        return np.concatenate([vv, _kepler_accel(rr) + a_eci])

    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def semi_major_axis(r: np.ndarray, v: np.ndarray) -> float:
    """Non-oscillating Kepler energy metric (same as SemiMajorPID.semi_major).

    a = 1/(2/|r| - |v|^2/mu). Constant on unperturbed arcs, so feedback on it
    never fights the natural radial epicycle (the failure mode of raw
    radius-based costs, which burn forever circularizing).
    """
    rn = max(float(np.linalg.norm(r)), 1.0)
    v2 = float(np.asarray(v, float) @ np.asarray(v, float))
    denom = 2.0 / rn - v2 / C.MU_EARTH
    if abs(denom) < 1e-15:
        return float("inf")
    return 1.0 / denom


def _reference_state(r0: np.ndarray) -> tuple[np.ndarray, float]:
    """Circular-orbit reference at the TARGET shell (fixed, not current r)."""
    r_norm = max(float(np.linalg.norm(r0)), 1.0)
    v_circ = float(np.sqrt(C.MU_EARTH / C.R_ORBIT))  # fixed target, not moving target
    return r0 / r_norm * C.R_ORBIT, v_circ


class SolverBackend(ABC):
    """Modularity contract for NMPC solvers.

    Implement ``solve``; keep signature stable so runners/hierarchical code
    never import scipy/casadi/acados directly.
    """

    @abstractmethod
    def solve(self, x0: np.ndarray, cfg: NMPCConfig) -> NMPCSolution:
        ...


class ScipyShootingBackend(SolverBackend):
    """Single-shooting scipy POC: decision = accel sequence (N,3).

    Energy cost per stage k (never fights the epicycle):
      q_pos*(db(e_a)/50km)^2 + q_vel*(v_radial/50m/s)^2 + r_fuel*|a_k/a_max|^2,
    where e_a = a_osc(r_k,v_k) - R_ORBIT with deadband db_a (coast inside),
    v_radial = v_k.R_hat_k (eccentricity damping only). Plus x5 terminal
    energy weight. Dynamics: RK4 Kepler rollout. Box |a| <= a_max.
    Warm-started (shifted) for speed.
    """

    def __init__(self):
        self._warm: np.ndarray | None = None

    def solve(self, x0: np.ndarray, cfg: NMPCConfig) -> NMPCSolution:
        from scipy.optimize import minimize

        N, dt = int(cfg.horizon), float(cfg.dt_outer)
        x0 = np.asarray(x0, float).ravel()
        assert x0.size == 6, f"orbit-mode x0 must be 6-state, got {x0.shape}"

        if self._warm is not None and self._warm.shape == (N, 3):
            u_init = np.roll(self._warm, -1, axis=0)
            u_init[-1, :] = 0.0
        else:
            u_init = np.zeros((N, 3))
        z0 = u_init.ravel()

        bounds = [(-cfg.a_max, cfg.a_max)] * (3 * N)

        def rollout(U: np.ndarray) -> np.ndarray:
            traj = np.zeros((N + 1, 6))
            traj[0] = x0
            for k in range(N):
                traj[k + 1] = _rk4_step_orbit(traj[k], U[k], dt)
            return traj

        _EA, _EVR = 50e3, 50.0  # normalization scales (m, m/s)
        a_scale = max(cfg.a_max, 1e-12)

        def cost(z: np.ndarray) -> float:
            U = z.reshape(N, 3)
            traj = rollout(U)
            c = 0.0
            for k in range(1, N + 1):
                rk, vk = traj[k, 0:3], traj[k, 3:6]
                a_osc = semi_major_axis(rk, vk)
                e_a = a_osc - C.R_ORBIT
                if abs(e_a) < cfg.db_a:
                    e_a = 0.0  # coast deadband: no reason to burn inside
                rn = max(float(np.linalg.norm(rk)), 1.0)
                v_r = float(vk @ (rk / rn))  # radial velocity (damping only)
                a_n2 = float(U[k - 1] @ U[k - 1]) / a_scale**2
                c += (cfg.q_pos * (e_a / _EA) ** 2 + cfg.q_vel * (v_r / _EVR) ** 2
                      + cfg.r_fuel * a_n2)
            # terminal energy emphasis (restore shell energy by horizon end)
            aT = semi_major_axis(traj[N, 0:3], traj[N, 3:6])
            e_term = (aT - C.R_ORBIT) / _EA
            if abs(aT - C.R_ORBIT) < cfg.db_a:
                e_term = 0.0
            return float(c + 5.0 * cfg.q_pos * e_term**2)

        res = minimize(cost, z0, method=cfg.solver,
                       bounds=bounds,
                       options={"maxiter": cfg.maxiter, "disp": False})
        success = bool(res.success)
        U_opt = res.x.reshape(N, 3) if success else np.zeros((N, 3))
        if success:
            self._warm = U_opt.copy()
        traj = rollout(U_opt)
        return NMPCSolution(
            a_eci=np.array(U_opt[0], float),
            tau_body=np.zeros(3),  # outer torque: detumbled hold (inner trim adds damping)
            u_seq=U_opt,
            pred_traj=traj,
            cost=float(res.fun) if res.fun is not None else float("inf"),
            success=success,
            info={"nit": getattr(res, "nit", -1), "status": getattr(res, "status", -1)},
        )


_BACKENDS: dict[str, type[SolverBackend]] = {
    "scipy": ScipyShootingBackend,
    # "casadi": CasadiShootingBackend,  # future: same solve() signature
    # "acados": AcadosBackend,          # future: same solve() signature
}


class HighLevelNMPC:
    """Outer-loop NMPC with warm start + safe fallback.

    Duck-compatible with OrbitPID: exposes ``accel_command(r, v, dt)`` so
    existing ``propagate_closedloop*`` harnesses work unchanged. Internally
    replans every ``dt_outer`` and ZOHs u0 between replans; on solver failure
    falls back to zero (coast) or a tiny radial PD (``fallback="pid"``).
    """

    def __init__(self, cfg: NMPCConfig | None = None,
                 backend: SolverBackend | str | None = None):
        self.cfg = cfg or NMPCConfig()
        if backend is None or isinstance(backend, str):
            self.backend = _BACKENDS[backend or "scipy"]()
        else:
            self.backend = backend  # custom (casadi/acados) instance
        self._last_a = np.zeros(3)
        self._last_tau = np.zeros(3)
        self._ticks_since_plan = 9999
        self._last_cost = float("inf")
        self._last_success = False
        self.n_plans = 0
        self.n_fallbacks = 0

    def reset(self):
        self._last_a = np.zeros(3)
        self._last_tau = np.zeros(3)
        self._ticks_since_plan = 9999
        if hasattr(self.backend, "_warm"):
            self.backend._warm = None  # type: ignore[attr-defined]
        self.n_plans = 0
        self.n_fallbacks = 0

    def _fallback_accel(self, r: np.ndarray, v: np.ndarray) -> np.ndarray:
        if self.cfg.fallback == "zero":
            return np.zeros(3)
        # tiny safe energy-based fallback (same order as SemiMajorPID Kp, not full PID)
        a_osc = semi_major_axis(np.asarray(r, float), np.asarray(v, float))
        e_a = C.R_ORBIT - a_osc
        if abs(e_a) < self.cfg.db_a:
            return np.zeros(3)
        R_hat = np.asarray(r, float) / max(float(np.linalg.norm(r)), 1.0)
        # transverse energy correction direction ~ velocity-transverse; use radial
        # projection here (safe, small); full fix comes from next successful plan
        a_mag = np.clip(3.0e-7 * e_a, -self.cfg.a_max, self.cfg.a_max)
        # apply along transverse unit if available, else radial
        v = np.asarray(v, float)
        h = np.cross(r, v)
        nh = float(np.linalg.norm(h))
        if nh > 1e-9:
            R_hat_c = r / max(float(np.linalg.norm(r)), 1.0)
            T_hat = np.cross(h / nh, R_hat_c)
            return T_hat * a_mag
        return R_hat * a_mag

    def plan(self, r: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Always solve once (explicit outer scheduling by HierarchicalController).

        Returns the clipped first-step bulk accel (ZOH by caller). Never raises:
        on failure returns the safe fallback and counts it.
        """
        r = np.asarray(r, float)
        v = np.asarray(v, float)
        x0 = np.concatenate([r, v])
        try:
            sol = self.backend.solve(x0, self.cfg)
        except Exception:
            sol = NMPCSolution(np.zeros(3), np.zeros(3),
                               np.zeros((self.cfg.horizon, 3)),
                               np.zeros((self.cfg.horizon + 1, 6)),
                               float("inf"), False, {"exception": True})
        self._last_cost = sol.cost
        self._last_success = sol.success
        if sol.success:
            self._last_a = np.clip(sol.a_eci, -self.cfg.a_max, self.cfg.a_max)
            self._last_tau = np.clip(sol.tau_body, -self.cfg.tau_max, self.cfg.tau_max)
        else:
            self.n_fallbacks += 1
            self._last_a = self._fallback_accel(r, v)
            self._last_tau = np.zeros(3)
        self._ticks_since_plan = 0
        self.n_plans += 1
        return self._last_a.copy()

    def accel_command(self, r: np.ndarray, v: np.ndarray, dt: float) -> np.ndarray:
        r = np.asarray(r, float)
        v = np.asarray(v, float)
        # standalone (non-hierarchical) use: replan at outer rate, ZOH between.
        # HierarchicalController does NOT use this path (it calls plan()
        # explicitly to avoid double-counter bugs); this stays for compat.
        steps_per_plan = max(1, int(round(self.cfg.dt_outer / max(dt, 1e-9))))
        if self._ticks_since_plan >= steps_per_plan:
            self.plan(r, v)
        else:
            self._ticks_since_plan += 1
        # output deadband (fuel saving, mirrors OrbitPID chatter guard)
        a = self._last_a.copy()
        if np.all(np.abs(a) < C.PID_DEADBAND):
            return np.zeros(3)
        a[np.abs(a) < C.PID_DEADBAND] = 0.0
        return a

    @property
    def last_torque(self) -> np.ndarray:
        return self._last_tau.copy()

    @property
    def stats(self) -> dict:
        return {"plans": self.n_plans, "fallbacks": self.n_fallbacks,
                "last_cost": self._last_cost, "last_success": self._last_success}
