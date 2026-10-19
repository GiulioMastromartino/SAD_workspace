"""Hierarchical controller: NMPC outer (bulk) + fast PID inner (small trim).

Outer @ dt_outer (10 s): HighLevelNMPC plans bulk ECI accel, ZOH between plans.
Inner @ dt_inner (1 s): FastTrimPID corrects residual RTN error with TINY
authority (TRIM_A_MAX saturation + TRIM_DV_BUDGET latch + deadband).

Duck-compatible with OrbitPID/SemiMajorPID: ``accel_command(r, v, dt)``,
``throttle_command(r, v, euler, dt)``, ``reset()`` — so existing
``propagate_closedloop`` / ``propagate_closedloop_orbit`` harnesses and the
run_* comparison scripts work without modification. Torque path makes it
6-DOF: outer tau (small hold) + inner rate-damping trim, allocated together
with force through the same 12-jet pinv (allocation_pinv).
"""
from __future__ import annotations

import numpy as np

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.control.controller import rtn_frame
from cubesat_12u_sim.core.dynamics import euler_to_dcm
from cubesat_12u_sim.core.actuators import allocation_pinv, jets_to_wrench
from cubesat_12u_sim.control.nmpc import HighLevelNMPC, NMPCConfig, clip_norm
from cubesat_12u_sim.tuning.profiling import profiler as _prof


class FastTrimPID:
    """Inner trim PID with hard propulsion budget.

    Same RTN structure as OrbitPID but: (1) gains default to 20% of orbit PID
    (gentle trim, not bulk recovery), (2) output saturated to TRIM_A_MAX,
    (3) cumulative |a|*dt budget per outer window (TRIM_DV_BUDGET) — once
    exhausted trim latches to zero until the next outer plan (prevents the
    inner loop from spending the fuel the NMPC saved).
    """

    def __init__(self, authority: float | None = None,
                 dv_budget: float | None = None,
                 kp_scale: float = 0.2):
        self.authority = float(authority if authority is not None else C.TRIM_A_MAX)
        self.dv_budget = float(dv_budget if dv_budget is not None else C.TRIM_DV_BUDGET)
        self.kp_scale = float(kp_scale)
        self.deadband = float(C.TRIM_DEADBAND)
        self.int_r = 0.0
        self.int_t = 0.0
        self.int_n = 0.0
        self.dv_spent_window = 0.0

    def reset(self):
        self.int_r = self.int_t = self.int_n = 0.0
        self.dv_spent_window = 0.0

    def reset_window(self):
        """Called by outer loop each replan: fresh trim budget."""
        self.dv_spent_window = 0.0
        self.int_r *= 0.5
        self.int_t *= 0.5
        self.int_n *= 0.5

    def accel_command(self, r: np.ndarray, v: np.ndarray, dt: float) -> np.ndarray:
        if self.dv_spent_window >= self.dv_budget:
            _prof().count("latched")
            return np.zeros(3)  # budget exhausted: coast
        R, T, N = rtn_frame(np.asarray(r, float), np.asarray(v, float))
        r_norm = float(np.linalg.norm(r))
        v_r = float(v @ R)
        v_t = float(v @ T)
        v_n = float(v @ N)
        r_n = float(r @ N)
        v_circ = float(np.sqrt(C.MU_EARTH / r_norm))

        e_r = C.R_ORBIT - r_norm
        self.int_r = np.clip(self.int_r + e_r * dt, -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_R = self.kp_scale * C.PID_KP_R * e_r - C.PID_KD_R * v_r

        e_t = v_circ - v_t
        self.int_t = np.clip(self.int_t + e_t * dt, -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_T = self.kp_scale * C.PID_KP_T * e_t

        e_n = -r_n
        self.int_n = np.clip(self.int_n + e_n * dt, -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_N = self.kp_scale * C.PID_KP_N * e_n - C.PID_KD_N * v_n

        a_rtn = np.clip(np.array([a_R, a_T, a_N]), -self.authority, self.authority)
        a_rtn[np.abs(a_rtn) < self.deadband] = 0.0
        a_eci = R * a_rtn[0] + T * a_rtn[1] + N * a_rtn[2]
        # budget bookkeeping (count what we are about to spend)
        self.dv_spent_window += float(np.linalg.norm(a_eci) * dt)
        if self.dv_spent_window >= self.dv_budget:
            # overspent this tick: scale back proportionally, then latch off
            over = self.dv_spent_window - self.dv_budget
            scale = max(0.0, 1.0 - over / max(float(np.linalg.norm(a_eci) * dt), 1e-12))
            a_eci = a_eci * scale
        return a_eci


class HierarchicalController:
    """NMPC bulk + fast-PID trim, OrbitPID-compatible interface.

    Usage mirrors OrbitPID::
        ctrl = HierarchicalController()  # outer 10 s / inner 1 s from config
        a = ctrl.accel_command(r, v, dt_inner)   # call every inner tick
        u, F, tau = ctrl.throttle_command(r, v, euler, dt_inner)

    ``dt`` passed per call determines outer scheduling: every
    ``dt_outer/dt`` inner ticks a replan is triggered and the trim window
    budget is refreshed. For the 3-year orbit-only mode pass the scaled
    dts (e.g. outer 600 s / inner 60 s) — same code, fewer solves.
    """

    def __init__(self, nmpc_cfg: NMPCConfig | None = None,
                 trim: FastTrimPID | None = None,
                 tau_damp: float | None = None,
                 backend=None):
        if backend is not None:
            self.nmpc = HighLevelNMPC(nmpc_cfg or NMPCConfig(), backend=backend)
        else:
            self.nmpc = HighLevelNMPC(nmpc_cfg or NMPCConfig())
        self.trim = trim or FastTrimPID()
        self.tau_damp = float(tau_damp if tau_damp is not None else C.TRIM_TAU_MAX)
        self._bulk_a = np.zeros(3)
        self._tick = 0
        self.dv_bulk = 0.0
        self.dv_trim = 0.0

    @property
    def dt_outer(self) -> float:
        return float(self.nmpc.cfg.dt_outer)

    def reset(self):
        self.nmpc.reset()
        self.trim.reset()
        self._bulk_a = np.zeros(3)
        self._tick = 0
        self.dv_bulk = 0.0
        self.dv_trim = 0.0

    def _is_outer_tick(self, dt: float) -> bool:
        steps = max(1, int(round(self.dt_outer / max(dt, 1e-9))))
        return (self._tick % steps) == 0

    def accel_command(self, r: np.ndarray, v: np.ndarray, dt: float) -> np.ndarray:
        r = np.asarray(r, float)
        v = np.asarray(v, float)
        _p = _prof()
        outer = self._is_outer_tick(dt)
        _p.count("outer_ticks" if outer else "inner_ticks", 1)
        with _p.section("tick"):
            if outer:
                self.trim.reset_window()
                # explicit solve, no hidden counters. Timed separately from the
                # trim below: the bulk plan is where essentially all of the
                # control CPU goes, and lumping it in with the trim hides that.
                with _p.section("plan"):
                    self._bulk_a = self.nmpc.plan(r, v)
            with _p.section("trim"):
                a_trim = self.trim.accel_command(r, v, dt)
            a_total = self._bulk_a + a_trim
        # global jet-capability cap as a norm, matching the physical thrust
        # limit (a per-axis clip would allow sqrt(3)x the capability)
        a_total = clip_norm(a_total, C.PID_A_MAX)
        self.dv_bulk += float(np.linalg.norm(self._bulk_a) * dt)
        self.dv_trim += float(np.linalg.norm(a_trim) * dt)
        self._tick += 1
        if np.all(np.abs(a_total) < self.trim.deadband):
            return np.zeros(3)
        return a_total

    def torque_command(self, euler: np.ndarray, omega: np.ndarray) -> np.ndarray:
        """Small rate-damping torque trim (detumbled hold for station-keeping).

        Bulk torque comes from NMPC (currently zero hold); here only damp
        residual body rate so the 12-jet allocation sees a 6-DOF wrench.
        Saturated to TRIM_TAU_MAX — inner loop cannot waste fuel on attitude.
        """
        omega = np.asarray(omega, float)
        tau = -0.05 * omega  # gentle damping (cf. ACS_P=0.4 bulk, here 8x smaller)
        return np.clip(tau, -C.TRIM_TAU_MAX, C.TRIM_TAU_MAX)

    def throttle_command(self, r: np.ndarray, v: np.ndarray, euler: np.ndarray,
                         dt: float, omega: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        _p = _prof()
        with _p.section("throttle"):
            a_eci = self.accel_command(r, v, dt)
            F_eci = C.MASS * a_eci
            R_bi = euler_to_dcm(*np.asarray(euler, float))
            F_body_des = R_bi @ F_eci
            tau_des = self.torque_command(euler,
                                          np.zeros(3) if omega is None else omega)
            # add outer NMPC torque hold
            tau_des = tau_des + self.nmpc.last_torque
            with _p.section("pinv"):
                u = allocation_pinv(F_body_des, tau_des)
            with _p.section("wrench"):
                F_body, tau_body = jets_to_wrench(u)
        return u, F_body, tau_body

    @property
    def fuel_split(self) -> dict:
        return {"dv_bulk": self.dv_bulk, "dv_trim": self.dv_trim,
                "dv_total": self.dv_bulk + self.dv_trim,
                "trim_frac": self.dv_trim / max(self.dv_bulk + self.dv_trim, 1e-12),
                **self.nmpc.stats}
