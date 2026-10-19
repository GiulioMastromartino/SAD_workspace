"""Heyoka mirror of Basilisk's scenarioAttitudeFeedbackRW (baseline config).

15-state MRP system with reaction-wheel momentum states::

    [r(3), v(3), sigma(3), omega(3), Omega_wheels(3)]

Wheels are body-axis-aligned (Honeywell HR16 equivalent, Js per-wheel as a
parameter). Balanced-wheel model (no friction/jitter), matching the demo's
``BalancedWheels`` baseline (``useJitterSimple=False, useRWVoltageIO=False``):

- wheel: ``Js_i * dW_i = u_i`` (motor torque command),
- hub: ``I*dw = -w x (I*w + h) - u + Tgg_off``, ``h_i = Js_i*(W_i + w_i)``.
- No gravity-gradient module (the demo attaches Earth gravity force only).

Closed loop mirrors the demo: inertial identity reference, PD ``Lr = -K*e -
P*w`` with the demo gains, motor torque = clip(Lr, +-u_max) (what
``rwMotorTorque`` does for three orthogonal body-aligned wheels). RWs are
continuous actuators: NO thruster quantizer here. Navigation is truth
(``simpleNav`` equivalent).
"""
from __future__ import annotations

import numpy as np
import heyoka as hy

from cubesat_12u_sim.core import config as C

STATE_NAMES = ["rx", "ry", "rz", "vx", "vy", "vz",
               "s1", "s2", "s3", "omx", "omy", "omz",
               "W1", "W2", "W3"]

# pars: [u1,u2,u3, m, Ixx,Iyy,Izz,Ixy,Ixz,Iyz,
#        invIxx,invIyy,invIzz,invIxy,invIxz,invIyz, Js1,Js2,Js3] = 19
N_PARS_RW = 19

# Demo constants (scenarioAttitudeFeedbackRW baseline).
RW_K = 3.5
RW_P = 30.0
RW_U_MAX = 0.2          # HR16 max motor torque [Nm]
RW_JS = 0.07957747154594767  # HR16 rotor inertia [kg m^2]
RW_OMEGA0_RPM = (100.0, 200.0, 300.0)


def pack_rw_pars(u_motor: np.ndarray, mass: float, inertia: np.ndarray,
                 js: np.ndarray | None = None) -> list[float]:
    """Build the 22-par vector for the RW system."""
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import inertia_pars
    I6, invI6 = inertia_pars(np.asarray(inertia, float))
    if js is None:
        js = np.full(3, RW_JS)
    return (list(map(float, np.asarray(u_motor, float).ravel()))
            + [float(mass)] + I6 + invI6
            + list(map(float, np.asarray(js, float).ravel())))


def build_rw_system():
    """Return (system, var_tuple) for the 15-state hub+RW EOM."""
    (rx, ry, rz, vx, vy, vz, s1, s2, s3, omx, omy, omz,
     W1, W2, W3) = hy.make_vars(*STATE_NAMES)
    u1, u2, u3 = (hy.par[i] for i in range(3))
    m = hy.par[3]
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = (hy.par[4 + i] for i in range(6))
    ixx, iyy, izz, ixy, ixz, iyz = (hy.par[10 + i] for i in range(6))
    J1, J2, J3 = (hy.par[16 + i] for i in range(3))

    mu = float(C.MU_EARTH)

    # NOTE: no MRP->DCM here on purpose. Translation is pure Kepler
    # (no body thrust in the RW-only demo), so building the DCM
    # expressions only bloats the Taylor graph / JIT without affecting
    # dynamics. Attitude enters solely via MRP kinematics below.
    sn2 = s1 * s1 + s2 * s2 + s3 * s3

    # --- Translation: point-mass Kepler, no thrust force (RW-only demo) ---
    Rnorm = hy.sqrt(rx * rx + ry * ry + rz * rz)
    invR3 = 1.0 / (Rnorm * Rnorm * Rnorm)
    ax = -mu * rx * invR3
    ay = -mu * ry * invR3
    az = -mu * rz * invR3

    # --- MRP kinematics ---
    B11 = 1.0 - sn2 + 2.0 * s1 * s1
    B12 = 2.0 * (s1 * s2 - s3)
    B13 = 2.0 * (s1 * s3 + s2)
    B21 = 2.0 * (s2 * s1 + s3)
    B22 = 1.0 - sn2 + 2.0 * s2 * s2
    B23 = 2.0 * (s2 * s3 - s1)
    B31 = 2.0 * (s3 * s1 - s2)
    B32 = 2.0 * (s3 * s2 + s1)
    B33 = 1.0 - sn2 + 2.0 * s3 * s3
    s1dot = 0.25 * (B11 * omx + B12 * omy + B13 * omz)
    s2dot = 0.25 * (B21 * omx + B22 * omy + B23 * omz)
    s3dot = 0.25 * (B31 * omx + B32 * omy + B33 * omz)

    # --- Coupled hub + wheel dynamics (body axes = wheel axes) ---
    # wheel momenta about spin axes (absolute): h_i = J_i*(W_i + om_i)
    h1 = J1 * (W1 + omx)
    h2 = J2 * (W2 + omy)
    h3 = J3 * (W3 + omz)
    # total angular momentum (hub + wheels)
    Hx = Ixx * omx + Ixy * omy + Ixz * omz + h1
    Hy = Ixy * omx + Iyy * omy + Iyz * omz + h2
    Hz = Ixz * omx + Iyz * omy + Izz * omz + h3
    # hub Euler with wheel reaction -u (motor torque on wheels is +u)
    gx = -(omy * Hz - omz * Hy) - u1
    gy = -(omz * Hx - omx * Hz) - u2
    gz = -(omx * Hy - omy * Hx) - u3
    wdotx = ixx * gx + ixy * gy + ixz * gz
    wdoty = ixy * gx + iyy * gy + iyz * gz
    wdotz = ixz * gx + iyz * gy + izz * gz
    # wheel speeds: J_i dW_i = u_i
    W1dot = u1 / J1
    W2dot = u2 / J2
    W3dot = u3 / J3

    system = [
        (rx, vx), (ry, vy), (rz, vz),
        (vx, ax), (vy, ay), (vz, az),
        (s1, s1dot), (s2, s2dot), (s3, s3dot),
        (omx, wdotx), (omy, wdoty), (omz, wdotz),
        (W1, W1dot), (W2, W2dot), (W3, W3dot),
    ]
    return system


def make_rw_integrator(y0: np.ndarray, u_motor=None, mass: float | None = None,
                       inertia: np.ndarray | None = None,
                       js: np.ndarray | None = None) -> hy.taylor_adaptive:
    """Create a Taylor integrator for the 15-state RW system."""
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import inertia_pars  # noqa: F401 (re-export check)
    if u_motor is None:
        u_motor = np.zeros(3)
    if mass is None:
        mass = 750.0
    if inertia is None:
        inertia = np.diag([900.0, 800.0, 600.0])
    system = build_rw_system()
    return hy.taylor_adaptive(
        system, list(map(float, np.asarray(y0, float).ravel())),
        pars=pack_rw_pars(u_motor, mass, inertia, js),
        tol=float(C.HEYOKA_TOL), high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY),
    )


def pack_rw_state(r, v, sigma, omega, wheel_speeds) -> np.ndarray:
    return np.concatenate([np.asarray(r, float), np.asarray(v, float),
                           np.asarray(sigma, float), np.asarray(omega, float),
                           np.asarray(wheel_speeds, float)])


def rw_allocate(torque: np.ndarray, u_max: float = RW_U_MAX) -> np.ndarray:
    """Motor torques for orthogonal body-aligned wheels.

    Mirrors ``rwMotorTorque`` for the demo's identity ``controlAxes_B``:
    the wheel torque is the NEGATION of the bus torque demand (to impress
    ``Lr`` on the bus, the wheels are driven with ``-Lr`` and the bus feels
    the reaction), verified sign-for-sign against stock first-tick outputs.
    Saturated by the effector at ``+-u_max`` (``rwMotorTorque`` itself does
    not clip: stock passes 0.38 Nm with u_max 0.2 straight through).
    """
    return np.clip(-np.asarray(torque, float), -u_max, u_max)


def run_rw_mirror(duration: float = 600.0, dt: float = 0.1,
                   y0: np.ndarray | None = None,
                   K: float = RW_K, P: float = RW_P,
                   mass: float = 750.0,
                   inertia: np.ndarray | None = None,
                   js: np.ndarray | None = None) -> dict:
    """Closed-loop Heyoka mirror of the stock RW demo (identity target).

    PD ``Lr = -K*e - P*w`` (exact small-angle form of mrpFeedback with
    Ki off and zero reference rate), truth navigation, direct wheel
    allocation. Returns per-log-step series.
    """
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff

    if inertia is None:
        inertia = np.diag([900.0, 800.0, 600.0])
    if y0 is None:
        raise ValueError("y0 (15-state incl. wheel speeds) is required")
    ta = make_rw_integrator(y0, np.zeros(3), mass, inertia, js)
    I6inv = np.linalg.inv(np.asarray(inertia, float))
    _ = I6inv
    n = int(round(duration / dt))
    ts = np.zeros(n + 1)
    Y = np.zeros((15, n + 1))
    U = np.zeros((3, n + 1))
    Y[:, 0] = np.asarray(y0, float)
    zero3 = np.zeros(3)
    for k in range(1, n + 1):
        st = Y[:, k - 1]
        e = mrp_diff(st[6:9], zero3)
        Lr = -K * e - P * st[9:12]
        u = rw_allocate(Lr)
        ta.pars[0], ta.pars[1], ta.pars[2] = map(float, u)
        ta.propagate_for(dt)
        snew = np.asarray(ta.state, float)
        if float(snew[6:9] @ snew[6:9]) > 1.0:  # shadow reset
            snew[6:9] = -snew[6:9] / float(snew[6:9] @ snew[6:9])
            ta.state[6:9] = snew[6:9]
        Y[:, k] = snew
        U[:, k] = u
        ts[k] = ta.time
    return {"t": ts, "Y": Y, "u_motor": U, "dt": dt}
