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

from cubesat_12u_sim.control import adjoint
from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.tuning.profiling import profiler as _prof


@dataclass
class NMPCConfig:
    dt_outer: float = C.NMPC_DT_OUTER
    horizon: int = C.NMPC_HORIZON
    # The objective IS the scorer's objective -- there are no free tracking
    # weights. See nmpc_solve.cpp shoot_cost for why the previous
    # q_pos/q_vel/r_fuel form was wrong as an optimisation target.
    # Position-tracking cost: linear inside w_track_frac of the box, log beyond,
    # with w(edge) == w_track. w_track = 20 m/s = 67% of a 30 m/s fuel budget:
    # sitting on the box edge must cost two thirds of the fuel that got you
    # there, or the planner has no reason to leave it.
    w_track: float = 20.0    # [m/s] cost of sitting exactly on the box edge
    w_track_frac: float = 0.2   # where the profile stops being linear
    w_final: float = 0.05    # terminal energy weight
    box_ref: float = 25e3    # [m] scored box half-width
    db_a: float = C.NMPC_DB_A  # [m] deadband on the terminal |a_osc - R_ORBIT|
    a_max: float = C.NMPC_A_MAX
    tau_max: float = C.NMPC_TAU_MAX
    solver: str = C.NMPC_SOLVER
    maxiter: int = C.NMPC_MAXITER
    fallback: str = C.NMPC_FALLBACK  # "pid" | "zero"
    mode: str = "orbit"  # "orbit" (6-state) only for now; "full" reserved
    record_pred_traj: bool = False
    """Compute the predicted trajectory for every solve.

    Off by default because nothing consumes it: ``NMPCSolution.pred_traj`` is
    written and never read, yet the rollout costs N RK4 steps *per solve*,
    immediately after the solver finished using Taylor on the cpp path. Measured
    at ~0.28 ms per solve at H=12, which is ~9 % of solve time and ~5-8 % of a
    full sweep. Turn it on when debugging a single plan, not in a sweep.
    """


@dataclass
class NMPCSolution:
    a_eci: np.ndarray  # (3,) first-step bulk accel actually applied
    tau_body: np.ndarray  # (3,) first-step torque (small authority)
    u_seq: np.ndarray  # (N,3) planned accel sequence (diagnostics/MPC warm start)
    pred_traj: np.ndarray | None  # (N+1,6) predicted [r,v]; None unless
    #   NMPCConfig.record_pred_traj, since nothing reads it and it costs a
    #   rollout per solve
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


def clip_norm(a: np.ndarray, a_max: float) -> np.ndarray:
    """Scale a 3-vector down to |a| <= a_max if it exceeds it.

    The actuator limit is a thrust *magnitude* (one common Isp along a single
    commanded direction), not a per-axis box: np.clip(a, -a_max, a_max) lets a
    diagonal command reach sqrt(3)*a_max, i.e. 1.73x the physical capability.
    Use this wherever a commanded acceleration is saturated.
    """
    a = np.asarray(a, float)
    if a_max <= 0.0:
        return np.zeros_like(a)
    n = float(np.linalg.norm(a))
    if n <= a_max or n == 0.0:
        return a.copy()
    return a * (a_max / n)


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

    Cost per stage k, identical to what the tuner scores:

        fuel_w * (exp((|a_k|/a_max)^2) - 1)  +  track_w(|r_k|_err / box_ref)

    i.e. L1 fuel (the same dv the scorer counts) plus radius-error tracking
    that is linear inside 20% of the box and logarithmic out to the edge and
    beyond -- never zero, so coasting is never free. Plus a small w_final
    terminal energy term, deadbanded by db_a.

    Rollout: numpy RK4 by default; ``use_cpp=True`` evaluates cost and
    gradient through the Heyoka C++ batch path instead (same cost function,
    Taylor prediction at tol=1e-12 rather than RK4 — the more accurate of the
    two, and the same family as the validation simulator).

    Constraint: |a_k| <= a_max as a true norm (one inequality per stage, with
    analytic Jacobian), NOT a per-axis box — a box admits diagonal commands at
    sqrt(3)*a_max, i.e. 1.73x the jet capability. Warm-started (shifted) for
    speed.
    """

    def __init__(self, use_cpp: bool = False, grad: str = "adjoint"):
        self._warm: np.ndarray | None = None
        self.use_cpp = bool(use_cpp)
        # "adjoint": exact discrete-adjoint gradient, 1 forward + 1 backward
        #   sweep instead of 3N+1 perturbed rollouts. "fd": let scipy
        #   finite-difference, which is the reference the adjoint is tested
        #   against. Ignored when use_cpp is set, since grad_batch supplies the
        #   gradient either way.
        self.grad = str(grad)

    def solve(self, x0: np.ndarray, cfg: NMPCConfig) -> NMPCSolution:
        from scipy.optimize import NonlinearConstraint, minimize

        _p = _prof()
        with _p.section("solve"):
            return self._solve(x0, cfg)

    def _solve(self, x0: np.ndarray, cfg: NMPCConfig) -> NMPCSolution:
        from scipy.optimize import NonlinearConstraint, minimize

        _p = _prof()
        N, dt = int(cfg.horizon), float(cfg.dt_outer)
        x0 = np.asarray(x0, float).ravel()
        assert x0.size == 6, f"orbit-mode x0 must be 6-state, got {x0.shape}"

        if self._warm is not None and self._warm.shape == (N, 3):
            u_init = np.roll(self._warm, -1, axis=0)
            u_init[-1, :] = 0.0
        else:
            u_init = np.zeros((N, 3))
        z0 = u_init.ravel()

        # Wide box purely as a numerical brake against runaway variables; the
        # real feasibility boundary is the norm constraint below.
        box = 10.0 * cfg.a_max
        bounds = [(-box, box)] * (3 * N)

        def _norm_c(U: np.ndarray, k: int) -> float:
            return float(np.linalg.norm(U[k]))

        def _norm_jac(U: np.ndarray, k: int) -> np.ndarray:
            J = np.zeros((1, 3 * N))
            nk = float(np.linalg.norm(U[k]))
            if nk > 1e-12:
                J[0, 3 * k:3 * k + 3] = U[k] / nk
            return J

        constraints = [
            NonlinearConstraint(lambda z, k=k: _norm_c(z.reshape(N, 3), k),
                                -np.inf, cfg.a_max,
                                jac=lambda z, k=k: _norm_jac(z.reshape(N, 3), k))
            for k in range(N)
        ]

        def rollout(U: np.ndarray) -> np.ndarray:
            traj = np.zeros((N + 1, 6))
            traj[0] = x0
            for k in range(N):
                traj[k + 1] = _rk4_step_orbit(traj[k], U[k], dt)
            return traj

        _EA, _EVR = 50e3, 50.0  # normalization scales (m, m/s)
        a_scale = max(cfg.a_max, 1e-12)
        # Three counters, because the three gradient strategies do fundamentally
        # different amounts of work per iteration and reporting one number for
        # them is misleading:
        #   cost_evals  cost() calls (SLSQP line-search trials, ~4.6/iter)
        #   grad_evals  explicit gradient calls
        #   grad_lanes  rollouts inside one gradient call: 3N+1 for finite
        #               differences (numpy), 3N+1 batched on cpp, 0 for the adjoint
        _stats = {"cost_evals": 0, "grad_evals": 0, "grad_lanes": 0}
        _cache: dict = {"z": None, "states": None}
        _cache_on = [self.grad == "adjoint" and not self.use_cpp]

        def _forward(U: np.ndarray) -> np.ndarray:
            traj = np.zeros((N + 1, 6))
            traj[0] = x0
            for k in range(N):
                traj[k + 1] = _rk4_step_orbit(traj[k], U[k], dt)
            return traj

        def cost(z: np.ndarray) -> float:
            _stats["cost_evals"] += 1
            U = z.reshape(N, 3)
            # delegate to the single source of truth so the value the adjoint
            # differentiates cannot drift from the value being optimised
            c = adjoint.shooting_cost(x0, U, dt, C.MU_EARTH, C.R_ORBIT,
                                      cfg.w_track, cfg.w_track_frac,
                                      cfg.w_final, cfg.box_ref, cfg.db_a,
                                      cfg.a_max)
            if _cache_on[0]:
                # SLSQP asks for fun and jac at the same point in a line
                # search, so cache the rollout: the adjoint would otherwise
                # redo the forward pass. Only worth it when the adjoint is
                # actually going to read it.
                _cache["z"] = z.copy()
                _cache["states"] = _forward(U)
            return c

        def adjoint_grad(z: np.ndarray) -> np.ndarray:
            """Exact discrete-adjoint gradient: 1 forward + 1 backward sweep.

            Replaces scipy's finite differences, which need 3N+1 perturbed
            rollouts per iteration. On the numpy backend that is the whole
            solver cost; on cpp it is 89 % of the Taylor lanes.
            """
            _stats["grad_evals"] += 1
            U = z.reshape(N, 3)
            if _cache["states"] is None or not np.array_equal(_cache["z"], z):
                _cache["states"] = _forward(U)
                _cache["z"] = z.copy()
            return adjoint.states_cost_grad(
                _cache["states"], U, dt, C.MU_EARTH, C.R_ORBIT, cfg.w_track,
                cfg.w_track_frac, cfg.w_final, cfg.box_ref, cfg.db_a,
                cfg.a_max)

        # C++ Heyoka path: cost + forward-diff gradient + stacked norm
        # constraints, each in one batch call (4 C++ crossings per SLSQP
        # iteration instead of 2+2N Python ones). Falls back to the Python
        # closures when unavailable or disabled.
        jac_fn = None
        # Timed apart from the solve itself: this is where the first Heyoka
        # Taylor build happens, so it carries the ~7 s cold JIT that the README
        # notes and that has nothing to do with steady-state solve cost.
        with _p.section("jac_setup"):
            if self.use_cpp:
                try:
                    from cubesat_12u_sim.optimization.nmpcbatch import make_funcs
                    funcs = make_funcs(x0, cfg)
                    if funcs[0] is not None:
                        from scipy.optimize import NonlinearConstraint as _NLC
                        _c_cost, _c_jac, con_vals, con_jac = funcs
                        # Wrap, don't just swap: without this the cost counter
                        # reads 0 on the cpp path, because make_funcs replaces
                        # the Python closure that increments it. The gradient
                        # then looks free when it is really 3N+1 Taylor lanes
                        # evaluated inside one crossing.
                        def cost(z, _c=_c_cost):
                            _stats["cost_evals"] += 1
                            return _c(z)

                        if _c_jac is not None:
                            def jac_fn(z, _c=_c_jac):
                                _stats["grad_evals"] += 1
                                return _c(z)
                        else:
                            jac_fn = None
                        _stats["grad_lanes"] = 3 * N + 1
                        constraints = [_NLC(con_vals, -np.inf, cfg.a_max,
                                            jac=con_jac)]
                except Exception:
                    jac_fn = None

        jac = jac_fn
        if jac is None and self.grad == "adjoint":
            jac = adjoint_grad

        with _p.section("minimize"):
            res = minimize(cost, z0, method=cfg.solver,
                           bounds=bounds,
                           constraints=constraints,
                           jac=jac,
                           options={"maxiter": cfg.maxiter, "disp": False})
        nit = max(int(getattr(res, "nit", 0)), 0)
        if jac is None:
            # scipy finite-differences: one gradient per iteration, each costing
            # 3N+1 perturbed rollouts. Counted here because scipy never calls a
            # jac closure on this path, so the counter would read zero and the
            # report would show the adjoint as having removed nothing.
            _stats["grad_evals"] = nit
            _stats["grad_lanes"] = (3 * N + 1) * nit
        _p.count("cost_evals", _stats["cost_evals"])
        _p.count("grad_evals", _stats["grad_evals"])
        _p.count("grad_lanes", _stats["grad_lanes"])
        _p.count("nit", nit)
        success = bool(res.success)
        _p.count("success" if success else "failure", 1)
        U_opt = res.x.reshape(N, 3) if success else np.zeros((N, 3))
        # Belt-and-braces: the constraint is exact to solver tolerance, but the
        # applied first step must never exceed the physical cap regardless.
        U_opt = np.array([clip_norm(u, cfg.a_max) for u in U_opt])
        if success:
            self._warm = U_opt.copy()
        # Diagnostic only, and see NMPCConfig.record_pred_traj for why this is
        # off by default.
        traj = rollout(U_opt) if cfg.record_pred_traj else None
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
        _p = _prof()
        with _p.section("outer_plan"):
            try:
                sol = self.backend.solve(x0, self.cfg)
            except Exception:
                _p.count("plan_exception", 1)
                sol = NMPCSolution(np.zeros(3), np.zeros(3),
                                   np.zeros((self.cfg.horizon, 3)),
                                   np.zeros((self.cfg.horizon + 1, 6)),
                                   float("inf"), False, {"exception": True})
            self._last_cost = sol.cost
            self._last_success = sol.success
            if sol.success:
                self._last_a = clip_norm(sol.a_eci, self.cfg.a_max)
                self._last_tau = np.clip(sol.tau_body, -self.cfg.tau_max,
                                         self.cfg.tau_max)
            else:
                self.n_fallbacks += 1
                with _p.section("outer_fallback"):
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
