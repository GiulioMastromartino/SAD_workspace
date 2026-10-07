"""Propagation harness (scipy.solve_ivp; heyoka-compatible RHS layout)."""
from __future__ import annotations
import numpy as np
from scipy.integrate import solve_ivp
from . import config as C
from .dynamics import pack_state, state_derivative
from .actuators import jets_to_wrench


def propagate(t_end: float = C.T_END_DEFAULT, dt_log: float = C.DT_LOG,
              u_jet: np.ndarray | None = None,
              y0: np.ndarray | None = None,
              h_wheels: np.ndarray | None = None):
    """Propagate free-fall (default u=0) trajectory.

    u_jet: constant throttle vector (N,) in [0,1]; None -> zeros (pure free-fall).
    Returns scipy solve_ivp result with y = [r, v, euler, omega].
    """
    n_thr = len(C.THRUSTER_TMAX)
    if u_jet is None:
        u_jet = np.zeros(n_thr)
    F_body, tau_body = jets_to_wrench(u_jet)
    if y0 is None:
        y0 = pack_state(C.R0_ECI, C.V0_ECI, C.EULER0, C.OMEGA0_BODY)

    def rhs(t, y):
        return state_derivative(t, y, F_body, tau_body, h_wheels)

    t_eval = np.arange(0.0, t_end + dt_log, dt_log)
    sol = solve_ivp(rhs, (0.0, t_end), y0, method="DOP853",
                    t_eval=t_eval, rtol=C.RTOL, atol=C.ATOL)
    # gimbal-lock warning
    theta = sol.y[7, :]
    if np.any(np.abs(np.abs(theta) - np.pi / 2) < np.radians(1.0)):
        print("[warn] trajectory passed within 1 deg of Euler singularity (theta=+-90 deg).")
    return sol
