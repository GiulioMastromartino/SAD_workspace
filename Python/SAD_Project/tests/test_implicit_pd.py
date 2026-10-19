"""Implicit (backward-Euler equivalent) PD scaling: stability and C++ parity."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import sad_py
from SAD_Project.core import config as C
from SAD_Project.control.controller import quat_pd, reset_pd, _effective_gains


def spectral_radius(K, P, I, dt):
    """Spectral radius of the linearized per-axis implicit update."""
    D = 1.0 + dt * P / I + 0.5 * dt * dt * K / I
    a = 0.5 * dt * dt * K / (I * D)
    b = 0.5 * dt / D
    c = dt * K / (I * D)
    d = 1.0 / D
    M = np.array([[1.0 - a, b], [-c, d]])
    return float(max(abs(np.linalg.eigvals(M))))


def legacy_radius(P, I, dt):
    """Spectral radius of the legacy explicit damping map."""
    return abs(1.0 - dt * P / I)


def test_implicit_is_unconditionally_stable():
    rng = np.random.default_rng(0)
    inertias = np.diag(C.INERTIA)
    for dt in (0.05, 0.1, 0.5, 1.0, 2.0, 5.0):
        for I in inertias:
            for K in np.logspace(-3, 2, 9):
                for P in np.logspace(-3, 2, 9):
                    rho = spectral_radius(K, P, I, dt)
                    assert rho < 1.0 + 1e-12, (K, P, I, dt, rho)


def test_legacy_limit_cycle_at_dt_one():
    # z axis is the tightest: P > 2I/dt is unstable.
    I = np.diag(C.INERTIA)[2]
    P = C.ACS_P_TRACK
    assert legacy_radius(P, I, 1.0) > 1.0
    assert spectral_radius(C.ACS_K_TRACK, P, I, 1.0) < 1.0


def test_effective_gain_formula():
    K, P, I, dt = 0.9, 2.9, 0.03333333333333333, 1.0
    Ke, Pe, D = _effective_gains(K, P, I, dt)
    assert np.isclose(D, 1.0 + dt * P / I + 0.5 * dt * dt * K / I)
    assert np.isclose(Ke, K / D)
    assert np.isclose(Pe, (P + 0.5 * dt * K) / D)
    # dt -> 0 recovers the continuous gains
    Ke0, Pe0, _ = _effective_gains(K, P, I, 1e-9)
    assert np.isclose(Ke0, K, rtol=1e-6)
    assert np.isclose(Pe0, P, rtol=1e-6)


def _cpp_torque(dt, implicit, mode='track', track_only=True):
    qp = sad_py.QuatPDParams()
    if mode == 'track':
        qp.track_K = C.ACS_K_TRACK
        qp.track_P = C.ACS_P_TRACK
        qp.track_Ki = C.ACS_KI_TRACK
        cpp_mode = sad_py.CtrlMode.Track
        qd = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    elif mode == 'slew':
        qp.slew_K = C.ACS_K_SLEW
        qp.slew_P = C.ACS_P_SLEW
        qp.slew_Ki = C.ACS_KI_SLEW
        cpp_mode = sad_py.CtrlMode.Slew
        qd = C.ACS_SLEW_QUAT
    else:
        qp.detumble_K = C.ACS_K_DETUMBLE
        qp.detumble_P = C.ACS_P_DETUMBLE
        qp.detumble_Ki = C.ACS_KI_DETUMBLE
        cpp_mode = sad_py.CtrlMode.Detumble
        qd = np.array([0.0, 0.0, 0.0, 1.0])
    qp.inertia = np.diag(C.INERTIA)
    qp.implicit_dt = implicit
    qp.implicit_track_only = track_only
    qs = sad_py.QuatPDState(); sad_py.quatpd_init(qs, qp)
    qi = sad_py.QuatPDInput()
    q0 = np.array([0.02, -0.01, 0.03, 0.9993])
    qi.q_current = q0 / np.linalg.norm(q0)
    qi.omega = np.array([0.04, -0.02, 0.01])
    qi.q_desired = qd
    qi.mode = cpp_mode
    qo = sad_py.QuatPDOutput()
    sad_py.quatpd_step(qi, qs, qo, dt)
    return np.array(qo.torque_b), qi


def test_cpp_python_parity_implicit():
    for dt in (1.0, 0.1):
        cpp, qi = _cpp_torque(dt, True)
        reset_pd()
        py = quat_pd(qi.q_current, qi.omega, qi.q_desired, mode='track', dt=dt,
                     inertia=np.diag(C.INERTIA), implicit_dt=True)
        assert np.allclose(cpp, py, atol=1e-12, rtol=0.0), (dt, cpp, py)


def test_cpp_python_parity_legacy():
    for dt in (1.0, 0.1):
        cpp, qi = _cpp_torque(dt, False)
        reset_pd()
        py = quat_pd(qi.q_current, qi.omega, qi.q_desired, mode='track', dt=dt,
                     inertia=np.diag(C.INERTIA), implicit_dt=False)
        assert np.allclose(cpp, py, atol=1e-12, rtol=0.0), (dt, cpp, py)


def test_track_only_gating():
    # Slew is untouched by default (legacy) and scaled when explicitly enabled.
    legacy, qi = _cpp_torque(0.1, False, mode='slew', track_only=True)
    gated, _ = _cpp_torque(0.1, True, mode='slew', track_only=True)
    scaled, _ = _cpp_torque(0.1, True, mode='slew', track_only=False)
    assert np.allclose(gated, legacy, atol=0.0, rtol=0.0)
    assert not np.allclose(scaled, legacy)

    reset_pd()
    py_scaled = quat_pd(qi.q_current, qi.omega, qi.q_desired, mode='slew', dt=0.1,
                        inertia=np.diag(C.INERTIA), implicit_dt=True,
                        implicit_track_only=False)
    assert np.allclose(scaled, py_scaled, atol=1e-12, rtol=0.0)
