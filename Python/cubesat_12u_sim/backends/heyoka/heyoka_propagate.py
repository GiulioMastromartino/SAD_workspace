"""Orbit propagation with heyoka (Taylor adaptive) + closed-loop PID station-keeping.

Mirrors dynamics.py exactly (same 12-state EOM, same 3-2-1 Euler DCM), but the
body-frame jet wrench [F_body, tau_body] enters as heyoka *parameters*, so the
PID can update thrust at every control tick without rebuilding the integrator:

    ta.pars[0:3] = F_body   # [N]
    ta.pars[3:6] = tau_body # [N m]
    ta.propagate_for(DT_CONTROL)

NOTE: Euler 3-2-1 has a singularity at theta = +/-90 deg (1/cos term). Keep the
closed-loop demo detumbled / away from pitch lock, same as the scipy version.
"""
from __future__ import annotations
import numpy as np
import heyoka as hy
from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.core.dynamics import pack_state
from cubesat_12u_sim.core.actuators import jets_to_wrench
from cubesat_12u_sim.control.controller import OrbitPID

STATE_NAMES = ["rx", "ry", "rz", "vx", "vy", "vz",
               "phi", "theta", "psi", "omx", "omy", "omz"]


def build_system():
    """Return (system, var_tuple) for the 12-state EOM with 6 thrust parameters."""
    rx, ry, rz, vx, vy, vz, phi, theta, psi, omx, omy, omz = hy.make_vars(*STATE_NAMES)
    Fx, Fy, Fz, Tx, Ty, Tz = (hy.par[i] for i in range(6))

    mu = float(C.MU_EARTH)
    m = float(C.MASS)
    Ix, Iy, Iz = float(C.IX), float(C.IY), float(C.IZ)

    cp, sp = hy.cos(phi), hy.sin(phi)
    ct, st = hy.cos(theta), hy.sin(theta)
    cs, ss = hy.cos(psi), hy.sin(psi)
    # R_bi = Rx(phi) Ry(theta) Rz(psi), exactly as in dynamics.euler_to_dcm
    R00 = ct * cs
    R01 = ct * ss
    R02 = -st
    R10 = -cp * ss + sp * st * cs
    R11 = cp * cs + sp * st * ss
    R12 = sp * ct
    R20 = sp * ss + cp * st * cs
    R21 = -sp * cs + cp * st * ss
    R22 = cp * ct

    # --- Translation: Kepler + jet force rotated to ECI ---
    Rnorm = hy.sqrt(rx * rx + ry * ry + rz * rz)
    invR3 = 1.0 / (Rnorm * Rnorm * Rnorm)
    ax = -mu * rx * invR3 + (R00 * Fx + R10 * Fy + R20 * Fz) / m
    ay = -mu * ry * invR3 + (R01 * Fx + R11 * Fy + R21 * Fz) / m
    az = -mu * rz * invR3 + (R02 * Fx + R12 * Fy + R22 * Fz) / m

    # --- Gravity-gradient torque (optional, baked at build time) ---
    rbx = R00 * rx + R01 * ry + R02 * rz
    rby = R10 * rx + R11 * ry + R12 * rz
    rbz = R20 * rx + R21 * ry + R22 * rz
    if C.USE_GRAVITY_GRADIENT:
        invR5 = invR3 / (Rnorm * Rnorm)
        k = 3.0 * mu * invR5
        tggx = k * (rby * (Iz * rbz) - rbz * (Iy * rby))
        tggy = k * (rbz * (Ix * rbx) - rbx * (Iz * rbz))
        tggz = k * (rbx * (Iy * rby) - rby * (Ix * rbx))
    else:
        tggx, tggy, tggz = 0.0, 0.0, 0.0

    # --- Attitude: Euler kinematics + Euler equation ---
    q_sp_r_cp = omy * sp + omz * cp
    phidot = omx + q_sp_r_cp * st / ct
    thetadot = omy * cp - omz * sp
    psidot = q_sp_r_cp / ct
    # -w x (I w) expanded for diagonal inertia
    wdotx = (omy * omz * (Iy - Iz) + Tx + tggx) / Ix
    wdoty = (omz * omx * (Iz - Ix) + Ty + tggy) / Iy
    wdotz = (omx * omy * (Ix - Iy) + Tz + tggz) / Iz

    system = [
        (rx, vx), (ry, vy), (rz, vz),
        (vx, ax), (vy, ay), (vz, az),
        (phi, phidot), (theta, thetadot), (psi, psidot),
        (omx, wdotx), (omy, wdoty), (omz, wdotz),
    ]
    return system, (rx, ry, rz, vx, vy, vz, phi, theta, psi, omx, omy, omz)


def make_integrator(y0: np.ndarray, u_jet: np.ndarray | None = None) -> hy.taylor_adaptive:
    """Create a heyoka integrator from a 12-state vector (thrust from throttle u)."""
    if u_jet is None:
        u_jet = np.zeros(len(C.THRUSTER_TMAX))
    F_body, tau_body = jets_to_wrench(u_jet)
    system, _ = build_system()
    return hy.taylor_adaptive(
        system, list(map(float, y0)),
        pars=list(map(float, np.concatenate([F_body, tau_body]))),
        tol=float(C.HEYOKA_TOL), high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY),
    )


def propagate_heyoka(t_end: float = C.T_END_DEFAULT, dt_log: float = C.DT_LOG,
                     u_jet: np.ndarray | None = None,
                     y0: np.ndarray | None = None) -> dict:
    """Open-loop (constant throttle) propagation with heyoka. Returns log dict."""
    if y0 is None:
        y0 = pack_state(C.R0_ECI, C.V0_ECI, C.EULER0, C.OMEGA0_BODY)
    ta = make_integrator(y0, u_jet)
    n = int(round(t_end / dt_log))
    t = np.zeros(n + 1)
    y = np.zeros((12, n + 1))
    y[:, 0] = y0
    for k in range(1, n + 1):
        ta.propagate_for(dt_log)
        t[k] = ta.time
        y[:, k] = ta.state
    return {"t": t, "y": y}


def propagate_closedloop(t_end: float = 7200.0, dt_control: float = C.DT_CONTROL,
                         y0: np.ndarray | None = None,
                         pid: OrbitPID | None = None) -> dict:
    """Closed-loop PID station-keeping with heyoka.

    Each control tick: read (r, v, euler) -> PID throttle -> set pars -> step.
    Logs state, throttle, desired accel and fuel use (dv).
    """
    if y0 is None:
        y0 = pack_state(C.R0_ECI, C.V0_ECI, C.EULER0, C.OMEGA0_BODY)
    if pid is None:
        pid = OrbitPID()
    ta = make_integrator(y0, np.zeros(len(C.THRUSTER_TMAX)))
    n = int(round(t_end / dt_control))
    t = np.zeros(n + 1)
    y = np.zeros((12, n + 1))
    u_hist = np.zeros((len(C.THRUSTER_TMAX), n + 1))
    a_hist = np.zeros((3, n + 1))
    y[:, 0] = y0
    dv = 0.0
    for k in range(1, n + 1):
        r, v, euler = y[0:3, k - 1], y[3:6, k - 1], y[6:9, k - 1]
        u, F_body, tau_body = pid.throttle_command(r, v, euler, dt_control)
        for i, val in enumerate(np.concatenate([F_body, tau_body])):
            ta.pars[i] = float(val)
        ta.propagate_for(dt_control)
        t[k] = ta.time
        y[:, k] = ta.state
        u_hist[:, k] = u
        a_hist[:, k] = F_body / C.MASS  # achieved body accel (for fuel bookkeeping)
        dv += float(np.linalg.norm(F_body) / C.MASS * dt_control)
    return {"t": t, "y": y, "u": u_hist, "a_body": a_hist, "dv": dv,
            "dt_control": dt_control}


# ---------------------------------------------------------------------------
# Orbit-only (6-state) system for multi-year runs.
# No attitude states -> no Euler singularity during long tumbles, ~2x faster.
# Thrust enters directly as ECI acceleration command (ideal allocation assumed;
# a posteriori check |a| against the jets' capability in the runner script).
# ---------------------------------------------------------------------------

def build_orbit_system():
    rx, ry, rz, vx, vy, vz = hy.make_vars("rx", "ry", "rz", "vx", "vy", "vz")
    ax_cmd, ay_cmd, az_cmd = (hy.par[i] for i in range(3))
    mu = float(C.MU_EARTH)
    Rnorm = hy.sqrt(rx * rx + ry * ry + rz * rz)
    invR3 = 1.0 / (Rnorm * Rnorm * Rnorm)
    return ([(rx, vx), (ry, vy), (rz, vz),
             (vx, -mu * rx * invR3 + ax_cmd),
             (vy, -mu * ry * invR3 + ay_cmd),
             (vz, -mu * rz * invR3 + az_cmd)])


def propagate_closedloop_orbit(t_end: float, dt_control: float,
                               r0: np.ndarray | None = None,
                               v0: np.ndarray | None = None,
                               pid: OrbitPID | None = None,
                               tol: float | None = None) -> dict:
    """Multi-year-capable closed-loop orbit keeping (translation only, heyoka).

    Uses OrbitPID.accel_command (needs only r, v). Returns per-tick logs plus
    cumulative dv / impulse / propellant (rocket eq., config ISP_JET).
    """
    if r0 is None:
        r0 = C.R0_ECI.copy()
    if v0 is None:
        v0 = C.V0_ECI.copy()
    if pid is None:
        pid = OrbitPID()
    system = build_orbit_system()
    ta = hy.taylor_adaptive(system,
                            list(map(float, np.concatenate([r0, v0]))),
                            pars=[0.0, 0.0, 0.0],
                            tol=float(tol or C.HEYOKA_TOL),
                            high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY))
    n = int(round(t_end / dt_control))
    t = np.zeros(n + 1)
    y = np.zeros((6, n + 1))
    a_hist = np.zeros((3, n + 1))
    dv_hist = np.zeros(n + 1)
    y[:, 0] = np.concatenate([r0, v0])
    dv = 0.0
    for k in range(1, n + 1):
        r, v = y[0:3, k - 1], y[3:6, k - 1]
        a_eci = pid.accel_command(r, v, dt_control)
        ta.pars[0], ta.pars[1], ta.pars[2] = map(float, a_eci)
        ta.propagate_for(dt_control)
        t[k] = ta.time
        y[:, k] = ta.state
        a_hist[:, k] = a_eci
        dv += float(np.linalg.norm(a_eci) * dt_control)
        dv_hist[k] = dv
    m0 = float(C.MASS)
    prop = m0 * (1.0 - np.exp(-dv_hist / (C.ISP_JET * C.G0)))  # [kg] Tsiolkovsky
    return {"t": t, "y": y, "a_eci": a_hist, "dv": dv_hist,
            "impulse": m0 * dv_hist, "propellant": prop,
            "dt_control": dt_control}
