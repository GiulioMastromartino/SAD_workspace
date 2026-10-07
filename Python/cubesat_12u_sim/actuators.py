"""Actuators: variable-thrust jets + optional reaction wheels."""
from __future__ import annotations
import numpy as np
from . import config as C


def jets_to_wrench(u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Throttle vector u in [0,1]^N -> (F_body [N], tau_body [N m]).

    Clips to [0,1]; variable (continuous) thrust assumed (PWM-averaged).
    """
    u = np.clip(np.asarray(u, float), 0.0, 1.0)
    thrust = u * C.THRUSTER_TMAX                      # [N] per jet
    F = (thrust[:, None] * C.THRUSTER_DIR).sum(axis=0)
    tau = np.cross(C.THRUSTER_POS, thrust[:, None] * C.THRUSTER_DIR).sum(axis=0)
    return F, tau


def allocation_pinv(F_des: np.ndarray, tau_des: np.ndarray) -> np.ndarray:
    """Least-squares throttle allocation for desired wrench (may saturate; clips to [0,1]).

    Builds 6xN effectiveness matrix A = [d_i; r_i x d_i] * Tmax_i, solves A u = [F; tau].
    """
    d = C.THRUSTER_DIR          # (N,3)
    r = C.THRUSTER_POS          # (N,3)
    m = np.cross(r, d)          # (N,3) moment per unit thrust
    A = np.vstack([ (d * C.THRUSTER_TMAX[:, None]).T,
                    (m * C.THRUSTER_TMAX[:, None]).T ])  # (6,N)
    b = np.concatenate([F_des, tau_des])
    u, *_ = np.linalg.lstsq(A, b, rcond=None)
    return np.clip(u, 0.0, 1.0)


def allocation_matrices(pos: np.ndarray, dirs: np.ndarray,
                        tmax: np.ndarray) -> np.ndarray:
    """Precompute the 12x6 pseudoinverse allocation matrix for fixed geometry.

    The geometry never changes within a run, so solve once with
    ``pinv(A)`` and apply per tick via :func:`allocate_cached` (a matvec
    instead of a least-squares solve: ~0.08 ms -> ~0.005 ms per call).
    """
    d = np.asarray(dirs, float)
    r = np.asarray(pos, float)
    m = np.cross(r, d)
    tm = np.asarray(tmax, float)[:, None]
    A = np.vstack([(d * tm).T, (m * tm).T])  # (6, N)
    return np.linalg.pinv(A)  # (N, 6)


def allocate_cached(P: np.ndarray, F_des: np.ndarray,
                    tau_des: np.ndarray) -> np.ndarray:
    """Throttle allocation with a precomputed pseudoinverse (see above)."""
    b = np.concatenate([np.asarray(F_des, float), np.asarray(tau_des, float)])
    return np.clip(np.asarray(P, float) @ b, 0.0, 1.0)


def quantize_duty_cycle(u: np.ndarray) -> np.ndarray:
    """Emulate Basilisk thrusterDynamicEffector on-time quantization.

    Measured steady-state behavior (TEST_Thruster, cutoffFrequency=1000,
    useMinPulseTime=False, task dt 0.02 and 0.01 s) maps commanded duty
    ``f`` (on-time / task period) to achieved duty as a 3-level staircase
    ``{0, 1/6, 5/6, 1}`` with edges at half period and full period::

        f <= 0        -> 0
        0 < f < 0.5   -> 1/6
        0.5 <= f < 1  -> 5/6
        f >= 1        -> 1

    Small commands are therefore over-actuated ~8x (e.g. 2% -> 16.7%),
    which drives the limit-cycle chatter and fuel waste seen in the
    Basilisk Monte Carlo. Fuel bookkeeping must still use the COMMANDED
    throttle (as the allocator log does); only the dynamics wrench uses
    the quantized value. See ``jets_wrench_quantized``.
    """
    u = np.asarray(u, float)
    q = np.zeros_like(u)
    q[(u > 0.0) & (u < 0.5)] = 1.0 / 6.0
    q[(u >= 0.5) & (u < 1.0)] = 5.0 / 6.0
    q[u >= 1.0] = 1.0
    return np.clip(q, 0.0, 1.0)


def jets_wrench_quantized(u: np.ndarray, pos: np.ndarray, dirs: np.ndarray,
                          tmax: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Throttle -> (F, tau) through the effector quantizer (truth model)."""
    uq = quantize_duty_cycle(np.clip(np.asarray(u, float), 0.0, 1.0))
    thrust = uq * np.asarray(tmax, float)
    dirs_a = np.asarray(dirs, float)
    pos_a = np.asarray(pos, float)
    F = (thrust[:, None] * dirs_a).sum(axis=0)
    tau = np.cross(pos_a, thrust[:, None] * dirs_a).sum(axis=0)
    return F, tau


class ReactionWheels:
    """3 orthogonal body-aligned wheels: first-order torque response + momentum saturation."""

    def __init__(self):
        self.h = np.zeros(3)  # stored angular momentum [N m s]

    def step(self, tau_cmd: np.ndarray, dt: float) -> np.ndarray:
        """Advance one step; returns actual reaction torque applied to bus (-dh/dt)."""
        tau_cmd = np.clip(np.asarray(tau_cmd, float), -C.RW_MAX_TORQUE, C.RW_MAX_TORQUE)
        # first-order actuator: tau_act += (tau_cmd - tau_act) * dt/tau ; integrate h
        # simplified: assume tau_act ~= tau_cmd for dt >> tau, else lag via single state
        if not hasattr(self, "_tau"):
            self._tau = np.zeros(3)
        alpha = dt / (C.RW_TAU + dt)
        self._tau += alpha * (tau_cmd - self._tau)
        self.h += self._tau * dt
        # saturate momentum (simple clip, no desat logic here)
        n = np.linalg.norm(self.h)
        if n > C.RW_MAX_MOMENTUM:
            self.h *= C.RW_MAX_MOMENTUM / n
        return -self._tau  # reaction on bus
