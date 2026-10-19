"""Quaternion PD controller (reference).

The gains are scaled per step so the explicit PD torque is algebraically
equivalent to a backward-Euler (implicit) update of the linearized per-axis
attitude/rate loop.  This is the Python twin of ``quatpd_step`` in the C++
module and must stay in parity (see ``tests/test_cpp_parity.py``).
"""
import numpy as np
from ..core.config import (ACS_K_DETUMBLE, ACS_P_DETUMBLE, ACS_KI_DETUMBLE,
                           ACS_K_SLEW, ACS_P_SLEW, ACS_KI_SLEW,
                           ACS_K_TRACK, ACS_P_TRACK, ACS_KI_TRACK,
                           ACS_SLEW_QUAT, INERTIA, IMPLICIT_PD, IMPLICIT_TRACK_ONLY)
from ..core.dynamics import quat_error_vec

# module-level integral state
_INTEGRAL = {'detumble': np.zeros(3), 'slew': np.zeros(3), 'track': np.zeros(3)}


def reset_pd():
    """Clear the reference controller's integral state."""
    for v in _INTEGRAL.values():
        v[:] = 0.0


def _effective_gains(K, P, I, dt):
    """Backward-Euler equivalent gains for one axis (D is also the Ki scale)."""
    D = 1.0 + dt * P / I + 0.5 * dt * dt * K / I
    return K / D, (P + 0.5 * dt * K) / D, D


def quat_pd(q_current, omega, q_desired, mode='detumble', dt=0.1,
            inertia=None, implicit_dt=None, implicit_track_only=None):
    if mode == 'detumble':
        K, P, Ki = ACS_K_DETUMBLE, ACS_P_DETUMBLE, ACS_KI_DETUMBLE
    elif mode == 'slew':
        K, P, Ki = ACS_K_SLEW, ACS_P_SLEW, ACS_KI_SLEW
    else:
        K, P, Ki = ACS_K_TRACK, ACS_P_TRACK, ACS_KI_TRACK

    e = quat_error_vec(q_desired, q_current)
    _INTEGRAL[mode] = _INTEGRAL[mode] + e * dt
    _INTEGRAL[mode] = np.clip(_INTEGRAL[mode], -0.5, 0.5)

    if implicit_dt is None:
        implicit_dt = IMPLICIT_PD
    if implicit_track_only is None:
        implicit_track_only = IMPLICIT_TRACK_ONLY
    omega = np.asarray(omega, float)
    if not implicit_dt or dt <= 0.0 or (implicit_track_only and mode != 'track'):
        return -K * e - P * omega - Ki * _INTEGRAL[mode]

    I = np.diag(INERTIA) if inertia is None else np.asarray(inertia, float).reshape(3)
    I = np.where(I > 0.0, I, 1.0)
    Ke = np.empty(3); Pe = np.empty(3); Kie = np.empty(3)
    for i in range(3):
        Ke[i], Pe[i], D = _effective_gains(K, P, I[i], dt)
        Kie[i] = Ki / D
    return -Ke * e - Pe * omega - Kie * _INTEGRAL[mode]


def mission_phase(state, t, params=None):
    # returns (phase, q_desired, mode)
    omega = state[10:13]
    rate = float(np.linalg.norm(omega))
    if t < 5.0 or rate > 0.01:
        return 'detumble', np.array([0,0,0,1.0]), 'detumble'
    # slew
    q = state[6:10]
    qerr = np.array([0,0,0,0])  # placeholder
    from ..core.dynamics import quat_mul, quat_conj
    qe = quat_mul(ACS_SLEW_QUAT, quat_conj(q))
    if qe[3] < 0: qe = -qe
    ang = 2*np.arctan2(np.linalg.norm(qe[:3]), abs(qe[3]))
    if ang > 0.05 or rate > 0.02:
        return 'slew', ACS_SLEW_QUAT, 'slew'
    # track
    from ..core.sun_model import sun_direction, sun_pointing_quaternion
    sd = sun_direction(t)
    qsp = sun_pointing_quaternion(sd)
    return 'track', qsp, 'track'
