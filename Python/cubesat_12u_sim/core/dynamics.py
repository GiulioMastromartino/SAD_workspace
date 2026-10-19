"""6-DOF dynamics: Keplerian free-fall translation + rigid-body Euler-angle attitude."""
from __future__ import annotations
import numpy as np
from cubesat_12u_sim.core import config as C


def euler_to_dcm(phi: float, theta: float, psi: float) -> np.ndarray:
    """Body-from-inertial DCM for 3-2-1 (ZYX): R_bi = Rx(phi) Ry(theta) Rz(psi)."""
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cs, ss = np.cos(psi), np.sin(psi)
    Rx = np.array([[1, 0, 0], [0, cp, sp], [0, -sp, cp]])
    Ry = np.array([[ct, 0, -st], [0, 1, 0], [st, 0, ct]])
    Rz = np.array([[cs, ss, 0], [-ss, cs, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def euler_kinematics(phi: float, theta: float, omega: np.ndarray) -> np.ndarray:
    """Map body rate [p,q,r] -> Euler rates [phidot, thetadot, psidot] (3-2-1).

    Raises near gimbal lock (|cos(theta)| ~ 0); caller may clamp.
    """
    p, q, r = omega
    ct = np.cos(theta)
    st = np.sin(theta)
    if abs(ct) < 1e-8:
        ct = np.copysign(1e-8, ct)  # regularised inversion, warn in propagate if needed
    sp, cp = np.sin(phi), np.cos(phi)
    phidot = p + (q * sp + r * cp) * st / ct
    thetadot = q * cp - r * sp
    psidot = (q * sp + r * cp) / ct
    return np.array([phidot, thetadot, psidot])


def gravity_accel(r_eci: np.ndarray) -> np.ndarray:
    """Point-mass (+optional J2) gravity acceleration in ECI."""
    R = float(np.linalg.norm(r_eci))
    a = -C.MU_EARTH / R**3 * r_eci
    if C.USE_J2:
        # Simplified J2 (ECI, equatorial); off by default for pure Keplerian baseline.
        x, y, z = r_eci
        k = 1.5 * C.J2 * (C.R_EARTH / R) ** 2
        zx = (z / R) ** 2
        a = a.copy()
        a[0] += C.MU_EARTH / R**3 * x * k * (5 * zx - 1)
        a[1] += C.MU_EARTH / R**3 * y * k * (5 * zx - 1)
        a[2] += C.MU_EARTH / R**3 * z * k * (5 * zx - 3)
    return a


def gravity_gradient_torque(r_eci: np.ndarray, R_bi: np.ndarray) -> np.ndarray:
    """tau_gg = 3 mu / R^5 * (r_b x I r_b), r_b = R_bi r_eci."""
    R = float(np.linalg.norm(r_eci))
    r_b = R_bi @ r_eci
    return 3.0 * C.MU_EARTH / R**5 * np.cross(r_b, C.INERTIA @ r_b)


def state_derivative(t: float, y: np.ndarray, F_body: np.ndarray, tau_body: np.ndarray,
                     h_wheels: np.ndarray | None = None) -> np.ndarray:
    """Full 12-state derivative. y = [r(3), v(3), euler(3), omega(3)]."""
    r = y[0:3]
    v = y[3:6]
    phi, theta, psi = y[6:9]
    omega = y[9:12]

    R_bi = euler_to_dcm(phi, theta, psi)
    R_ib = R_bi.T

    # Translation: Kepler + rotated jet force (free-fall => no aero)
    a_grav = gravity_accel(r)
    a_jet = R_ib @ (F_body / C.MASS)
    rdot = v
    vdot = a_grav + a_jet

    # Rotation: Euler eq. I dw = -w x (I w + h) + tau_jets + tau_gg (+ wheel reaction already in tau)
    h = np.zeros(3) if h_wheels is None else h_wheels
    tau_gg = gravity_gradient_torque(r, R_bi) if C.USE_GRAVITY_GRADIENT else np.zeros(3)
    wdot = C.INV_INERTIA @ (-np.cross(omega, C.INERTIA @ omega + h) + tau_body + tau_gg)
    eulerdot = euler_kinematics(phi, theta, omega)
    return np.concatenate([rdot, vdot, eulerdot, wdot])


def pack_state(r, v, euler, omega) -> np.ndarray:
    return np.concatenate([np.asarray(r, float), np.asarray(v, float),
                           np.asarray(euler, float), np.asarray(omega, float)])


def unpack_state(y: np.ndarray):
    return y[0:3], y[3:6], y[6:9], y[9:12]
