"""MRP-based Heyoka dynamics + closed-loop attitude propagator (Phases 1-2).

Phase 1: 12-state ``[r(3), v(3), sigma(3), omega(3)]`` Taylor system using MRPs
(no Euler singularity at theta=+-90 deg). Thrust wrench + mass + inertia enter
as runtime parameters so dispersions / control can update them per tick without
rebuilding the integrator.

Par layout (19): [Fx,Fy,Fz,Tx,Ty,Tz, m, Ixx,Iyy,Izz,Ixy,Ixz,Iyz,
                  invIxx,invIyy,invIzz,invIxy,invIxz,invIyz]

Phase 2: :func:`propagate_closedloop_mrp` mirrors
``monte_carlo.run_single_mission`` (3-phase guidance, MRP PD, nominal
allocation + dispersed truth wrench, gyro bias/ARW + observer, fuel
accounting, same KPI dict) without Basilisk.
"""
from __future__ import annotations

import numpy as np
import heyoka as hy

from cubesat_12u_sim.core import config as C

STATE_NAMES = ["rx", "ry", "rz", "vx", "vy", "vz",
               "s1", "s2", "s3", "omx", "omy", "omz"]

N_PARS = 19


def inertia_pars(inertia: np.ndarray) -> tuple[list[float], list[float]]:
    """Flatten full inertia + its inverse into par vectors (6 each)."""
    I = np.asarray(inertia, float).reshape(3, 3)
    invI = np.linalg.inv(I)
    idx = [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)]
    return ([float(I[i, j]) for i, j in idx],
            [float(invI[i, j]) for i, j in idx])


def pack_pars(F_body: np.ndarray, tau_body: np.ndarray,
              mass: float, inertia: np.ndarray) -> list[float]:
    """Build the 19-par vector for given wrench + mass properties."""
    I6, invI6 = inertia_pars(inertia)
    return (list(map(float, np.asarray(F_body, float).ravel()))
            + list(map(float, np.asarray(tau_body, float).ravel()))
            + [float(mass)] + I6 + invI6)


def wrench_setup(disp: dict) -> dict:
    """One-time per-run wrench constants (inertia + effectiveness matrices).

    The dispersed geometry never changes mid-run, so the full inertia,
    its inverse, and the per-jet force/torque effectiveness matrices are
    baked once; per tick the wrench is two small matvecs (no ``cross`` or
    ``inv`` calls in the hot loop).
    """
    inertia = np.asarray(disp["inertia"], float)
    I6, invI6 = inertia_pars(inertia)
    dirs = np.asarray(disp["thruster_dir"], float)
    pos = np.asarray(disp["thruster_pos"], float)
    tmax = np.asarray(disp["thruster_tmax"], float)
    return {
        "mass": float(disp["mass"]),
        "isp": float(disp["thruster_isp"]),
        "tmax": tmax,
        "I6": I6,
        "invI6": invI6,
        "M_F": (dirs * tmax[:, None]).T.copy(),          # (3, N) F per unit u
        "M_T": (np.cross(pos, dirs) * tmax[:, None]).T.copy(),  # (3, N)
    }


def fast_wrench_pars(u: np.ndarray, ws: dict, quantize: bool,
                     F_out=None, T_out=None) -> tuple[np.ndarray, np.ndarray]:
    """Throttle -> (F, tau) via matvecs; optional effector quantizer."""
    from cubesat_12u_sim.core.actuators import quantize_duty_cycle

    uu = np.clip(np.asarray(u, float), 0.0, 1.0)
    uq = quantize_duty_cycle(uu) if quantize else uu
    F = ws["M_F"] @ uq
    T = ws["M_T"] @ uq
    return F, T


def pack_pars_parts(F_body: np.ndarray, tau_body: np.ndarray, mass: float,
                    I6: list[float], invI6: list[float]) -> list[float]:
    """19-par vector from precomputed inertia parts (no ``inv`` per tick)."""
    return (list(map(float, np.asarray(F_body, float).ravel()))
            + list(map(float, np.asarray(tau_body, float).ravel()))
            + [float(mass)] + list(I6) + list(invI6))


def build_mrp_system():
    """Return (system, var_tuple) for the 12-state MRP EOM."""
    rx, ry, rz, vx, vy, vz, s1, s2, s3, omx, omy, omz = hy.make_vars(*STATE_NAMES)
    Fx, Fy, Fz, Tx, Ty, Tz = (hy.par[i] for i in range(6))
    m = hy.par[6]
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = (hy.par[7 + i] for i in range(6))
    ixx, iyy, izz, ixy, ixz, iyz = (hy.par[13 + i] for i in range(6))

    mu = float(C.MU_EARTH)

    # --- MRP -> DCM (body-from-inertial), R = I + (8 S^2 - 4(1-s2) S)/(1+s2)^2
    sn2 = s1 * s1 + s2 * s2 + s3 * s3
    d = (1.0 + sn2) * (1.0 + sn2)
    c = 4.0 * (1.0 - sn2) / d
    k8 = 8.0 / d
    # S rows
    # S2 = S@S entries
    s11 = -s2 * s2 - s3 * s3
    s12 = s1 * s2
    s13 = s1 * s3
    s21 = s1 * s2
    s22 = -s1 * s1 - s3 * s3
    s23 = s2 * s3
    s31 = s1 * s3
    s32 = s2 * s3
    s33 = -s1 * s1 - s2 * s2
    R00 = 1.0 + k8 * s11 - c * 0.0
    R01 = k8 * s12 - c * (-s3)
    R02 = k8 * s13 - c * (s2)
    R10 = k8 * s21 - c * (s3)
    R11 = 1.0 + k8 * s22 - c * 0.0
    R12 = k8 * s23 - c * (-s1)
    R20 = k8 * s31 - c * (-s2)
    R21 = k8 * s32 - c * (s1)
    R22 = 1.0 + k8 * s33 - c * 0.0

    # --- Translation: Kepler + body thrust rotated to ECI (F_N = R^T F_B)
    Rnorm = hy.sqrt(rx * rx + ry * ry + rz * rz)
    invR3 = 1.0 / (Rnorm * Rnorm * Rnorm)
    ax = -mu * rx * invR3 + (R00 * Fx + R10 * Fy + R20 * Fz) / m
    ay = -mu * ry * invR3 + (R01 * Fx + R11 * Fy + R21 * Fz) / m
    az = -mu * rz * invR3 + (R02 * Fx + R12 * Fy + R22 * Fz) / m

    # --- Gravity-gradient torque with full inertia
    rbx = R00 * rx + R01 * ry + R02 * rz
    rby = R10 * rx + R11 * ry + R12 * rz
    rbz = R20 * rx + R21 * ry + R22 * rz
    Irx = Ixx * rbx + Ixy * rby + Ixz * rbz
    Iry = Ixy * rbx + Iyy * rby + Iyz * rbz
    Irz = Ixz * rbx + Iyz * rby + Izz * rbz
    if C.USE_GRAVITY_GRADIENT:
        invR5 = invR3 / (Rnorm * Rnorm)
        k = 3.0 * mu * invR5
        tggx = k * (rby * Irz - rbz * Iry)
        tggy = k * (rbz * Irx - rbx * Irz)
        tggz = k * (rbx * Iry - rby * Irx)
    else:
        tggx, tggy, tggz = 0.0, 0.0, 0.0

    # --- MRP kinematics: sdot = 0.25 * B(s) * omega
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

    # --- Euler equation: I wdot = -w x (I w) + T + Tgg, via inverse inertia
    Iwx = Ixx * omx + Ixy * omy + Ixz * omz
    Iwy = Ixy * omx + Iyy * omy + Iyz * omz
    Iwz = Ixz * omx + Iyz * omy + Izz * omz
    # -w x (Iw)
    gx = -(omy * Iwz - omz * Iwy) + Tx + tggx
    gy = -(omz * Iwx - omx * Iwz) + Ty + tggy
    gz = -(omx * Iwy - omy * Iwx) + Tz + tggz
    wdotx = ixx * gx + ixy * gy + ixz * gz
    wdoty = ixy * gx + iyy * gy + iyz * gz
    wdotz = ixz * gx + iyz * gy + izz * gz

    system = [
        (rx, vx), (ry, vy), (rz, vz),
        (vx, ax), (vy, ay), (vz, az),
        (s1, s1dot), (s2, s2dot), (s3, s3dot),
        (omx, wdotx), (omy, wdoty), (omz, wdotz),
    ]
    return system, (rx, ry, rz, vx, vy, vz, s1, s2, s3, omx, omy, omz)


def make_mrp_integrator(y0: np.ndarray, F_body=None, tau_body=None,
                        mass: float | None = None,
                        inertia: np.ndarray | None = None) -> hy.taylor_adaptive:
    """Create a Taylor integrator for the MRP system."""
    if F_body is None:
        F_body = np.zeros(3)
    if tau_body is None:
        tau_body = np.zeros(3)
    if mass is None:
        mass = float(C.MASS)
    if inertia is None:
        inertia = C.INERTIA
    system, _ = build_mrp_system()
    return hy.taylor_adaptive(
        system, list(map(float, np.asarray(y0, float).ravel())),
        pars=pack_pars(F_body, tau_body, mass, inertia),
        tol=float(C.HEYOKA_TOL), high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY),
    )


def gyro_noise_std(arw: np.ndarray) -> np.ndarray:
    """Per-sample gyro white-noise std matching Basilisk ImuSensor behavior.

    Measured (see ``/tmp/imu_cal.py``): with ``PMatrixGyro = ARW``, Basilisk
    outputs approx ``N(0, ARW)`` per sample INDEPENDENT of task rate
    (0.01°/s at dt = 0.01..1.0 s), i.e. it uses the ARW number directly as
    the per-sample std rather than the textbook ``ARW/sqrt(dt)``. Mirror
    that here for parity; the textbook scaling would overstate IMU chatter
    ~7x at the 50 Hz MC rate (+0.5 g fuel bias).
    """
    return np.asarray(arw, float)


def pack_mrp_state(r, v, sigma, omega) -> np.ndarray:
    return np.concatenate([np.asarray(r, float), np.asarray(v, float),
                           np.asarray(sigma, float), np.asarray(omega, float)])


# ---------------------------------------------------------------------------
# Pure-numpy helpers (mirror Basilisk conventions for parity)
# ---------------------------------------------------------------------------

def mrp_to_dcm(sigma: np.ndarray) -> np.ndarray:
    """MRP -> body-from-inertial DCM (numpy mirror of build_mrp_system)."""
    s = np.asarray(sigma, float)
    s2 = float(s @ s)
    d = (1.0 + s2) ** 2
    S = np.array([[0.0, -s[2], s[1]], [s[2], 0.0, -s[0]], [-s[1], s[0], 0.0]])
    return np.eye(3) + (8.0 * (S @ S) - 4.0 * (1.0 - s2) * S) / d


def mrp_to_dcm_batch(sigmas: np.ndarray) -> np.ndarray:
    """Batched MRP -> DCM for (N,3) input; returns (N,3,3)."""
    s = np.asarray(sigmas, float)
    s2 = np.einsum("ij,ij->i", s, s)
    d = (1.0 + s2) ** 2
    S = np.zeros((s.shape[0], 3, 3))
    S[:, 0, 1] = -s[:, 2]
    S[:, 0, 2] = s[:, 1]
    S[:, 1, 0] = s[:, 2]
    S[:, 1, 2] = -s[:, 0]
    S[:, 2, 0] = -s[:, 1]
    S[:, 2, 1] = s[:, 0]
    S2 = S @ S
    I = np.eye(3)[None, :, :]
    return I + (8.0 * S2 - 4.0 * (1.0 - s2)[:, None, None] * S) / d[:, None, None]


def mrp_from_dcm(R: np.ndarray) -> np.ndarray:
    """Body-from-inertial DCM -> short-rotation MRP (Shepperd quaternion).

    Matches Basilisk ``C2MRP`` exactly (same off-diagonal sign convention,
    verified to ~1e-16 on random rotations).
    """
    R = np.asarray(R, float)
    tr = float(np.trace(R))
    if tr >= 1.0:
        q0 = 0.5 * np.sqrt(1.0 + tr)
        q1 = 0.25 * (R[1, 2] - R[2, 1]) / q0
        q2 = 0.25 * (R[2, 0] - R[0, 2]) / q0
        q3 = 0.25 * (R[0, 1] - R[1, 0]) / q0
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        q1 = 0.5 * np.sqrt(1.0 + 2.0 * R[0, 0] - tr)
        q0 = 0.25 * (R[1, 2] - R[2, 1]) / q1
        q2 = 0.25 * (R[1, 0] + R[0, 1]) / q1
        q3 = 0.25 * (R[0, 2] + R[2, 0]) / q1
    elif R[1, 1] >= R[2, 2]:
        q2 = 0.5 * np.sqrt(1.0 + 2.0 * R[1, 1] - tr)
        q0 = 0.25 * (R[2, 0] - R[0, 2]) / q2
        q1 = 0.25 * (R[1, 0] + R[0, 1]) / q2
        q3 = 0.25 * (R[2, 1] + R[1, 2]) / q2
    else:
        q3 = 0.5 * np.sqrt(1.0 + 2.0 * R[2, 2] - tr)
        q0 = 0.25 * (R[0, 1] - R[1, 0]) / q3
        q1 = 0.25 * (R[0, 2] + R[2, 0]) / q3
        q2 = 0.25 * (R[2, 1] + R[1, 2]) / q3
    if q0 < 0.0:  # shortest rotation (positive scalar part)
        q0, q1, q2, q3 = -q0, -q1, -q2, -q3
    s = np.array([q1, q2, q3]) / (1.0 + q0)
    n2 = float(s @ s)
    if n2 > 1.0:
        s = -s / n2
    return s


def precompute_sun_refs(mc_cfg, roll_ref: np.ndarray, dt: float) -> np.ndarray:
    """Sun-pointing reference MRP for every control tick (no Basilisk calls).

    Same basis construction as ``sun_model.sun_pointing_sigma`` with the
    constrained boresight from ``config.SUN_POINTING_AXIS_B`` (body +X),
    converted with :func:`mrp_from_dcm`. Validated against ``C2MRP`` in tests.
    Only sun-phase ticks are computed (detumble/slew refs are trivial);
    earlier entries stay zero and are never read via ``ref_at``.
    """
    from cubesat_12u_sim.core import config as Cfg

    n = int(round(mc_cfg.t_total / dt))
    out = np.zeros((n + 1, 3))
    t_end_phase = float(mc_cfg.t_detumble + mc_cfg.t_slew)
    k0 = int(np.searchsorted(np.arange(n + 1) * dt, t_end_phase, side="left"))
    if k0 > n:
        return out
    bore = np.asarray(Cfg.SUN_POINTING_AXIS_B, float)
    k = int(np.flatnonzero(np.abs(bore) > 1e-9)[0])
    m = (k + 1) % 3
    nn = (k + 2) % 3
    ref = np.asarray(roll_ref, float)
    kk = np.arange(k0, n + 1)
    th = float(Cfg.SUN_PHASE0) + (2.0 * np.pi / float(Cfg.SUN_YEAR)) * (kk * dt)
    suns = np.stack([np.cos(th), np.sin(th), np.zeros_like(th)], axis=1)
    dots = suns @ ref
    projs = ref[None, :] - dots[:, None] * suns
    projs = projs / np.linalg.norm(projs, axis=1, keepdims=True)
    basis = np.zeros((len(kk), 3, 3))
    basis[:, k, :] = suns
    basis[:, m, :] = projs
    basis[:, nn, :] = np.cross(suns, projs)
    for i, q in enumerate(kk):
        out[q] = mrp_from_dcm(basis[i].T)
    return out


def mrp_diff(sigma_BN: np.ndarray, sigma_RN: np.ndarray) -> np.ndarray:
    """Error MRP sigma_BR from body and reference MRPs (short rotation)."""
    b = np.asarray(sigma_BN, float)
    r = np.asarray(sigma_RN, float)
    b2, r2 = float(b @ b), float(r @ r)
    num = b * (1.0 - r2) - r * (1.0 - b2) + 2.0 * np.cross(b, r)
    den = 1.0 + b2 * r2 + 2.0 * float(b @ r)
    e = num / den
    if float(e @ e) > 1.0:  # shadow set -> short rotation
        e = -e / float(e @ e)
    return e


def mrp_diff_batch(sigma_BN: np.ndarray, sigma_RN: np.ndarray) -> np.ndarray:
    """Batched MRP difference for (B,3) inputs; returns (B,3)."""
    b = np.asarray(sigma_BN, float)
    r = np.asarray(sigma_RN, float)
    b2 = np.einsum("ij,ij->i", b, b)
    r2 = np.einsum("ij,ij->i", r, r)
    num = (b * (1.0 - r2)[:, None] - r * (1.0 - b2)[:, None]
           + 2.0 * np.cross(b, r))
    den = 1.0 + b2 * r2 + 2.0 * np.einsum("ij,ij->i", b, r)
    e = num / den[:, None]
    n2 = np.einsum("ij,ij->i", e, e)
    over = n2 > 1.0
    e[over] = -e[over] / n2[over, None]
    return e


def sun_direction_at(t: float) -> np.ndarray:
    """Analytic Sun unit vector (mirrors SunEphemeris.directionAt)."""
    th = float(C.SUN_PHASE0) + (2.0 * np.pi / float(C.SUN_YEAR)) * float(t)
    return np.array([np.cos(th), np.sin(th), 0.0])


def guidance_sigma(t: float, mc_cfg, sigma_init: np.ndarray,
                   sun_roll_ref: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """3-phase reference MRP + reference rate (mirrors MissionGuidance)."""
    from cubesat_12u_sim.core.sun_model import sun_pointing_sigma
    if t < mc_cfg.t_detumble:
        return np.asarray(sigma_init, float), np.zeros(3)
    if t < (mc_cfg.t_detumble + mc_cfg.t_slew):
        return np.asarray(mc_cfg.sigma_slew, float), np.zeros(3)
    s = sun_pointing_sigma(sun_direction_at(t) * float(C.VIZ_SUN_DISTANCE),
                           np.asarray(sun_roll_ref, float))
    return np.asarray(s, float), np.array([0.0, 0.0, 2.0 * np.pi / float(C.SUN_YEAR)])


def jets_wrench_disp(u: np.ndarray, pos: np.ndarray, dirs: np.ndarray,
                     tmax: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Throttle -> (F, tau) with dispersed geometry (truth model)."""
    u = np.clip(np.asarray(u, float), 0.0, 1.0)
    thrust = u * np.asarray(tmax, float)
    F = (thrust[:, None] * np.asarray(dirs, float)).sum(axis=0)
    tau = np.cross(np.asarray(pos, float),
                   thrust[:, None] * np.asarray(dirs, float)).sum(axis=0)
    return F, tau


def allocate_nominal(F_des: np.ndarray, tau_des: np.ndarray) -> np.ndarray:
    """Nominal-geometry pinv allocation (flight software side)."""
    from cubesat_12u_sim.core.actuators import allocation_pinv
    return allocation_pinv(np.asarray(F_des, float), np.asarray(tau_des, float))


# ---------------------------------------------------------------------------
# Phase 2: closed-loop MRP propagator mirroring run_single_mission
# ---------------------------------------------------------------------------

def propagate_closedloop_mrp(seed: int, mc_cfg=None, disp: dict | None = None,
                             task_step: float | None = None,
                             log_step: float | None = None,
                             loop_delay_ticks: int = 0,
                             quantize_thrusters: bool = True,
                             ctrl_every: int = 1) -> dict:
    """Closed-loop 3-phase mission with Heyoka MRP dynamics.

    Returns the same result dict shape as ``monte_carlo.run_single_mission``
    (run_idx/seed/success/dispersions/kpis/timeseries) so statistics, plots
    and NPZ savers can be reused unchanged.

    ``loop_delay_ticks`` emulates Basilisk task phasing: the spacecraft
    integrates first in each task while the freshly computed wrench only
    reaches the dynamics ``loop_delay_ticks`` later (Basilisk ≈ 2:
    demand(state_k) affects state_{k+2}). Fuel is always accounted from the
    commanded (undelayed) throttle, matching the allocator log.

    ``quantize_thrusters`` (default True) routes the dynamics wrench through
    :func:`actuators.quantize_duty_cycle`, emulating the measured
    ``thrusterDynamicEffector`` 3-level staircase (small commands over-actuate
    ~8x). Fuel still books the commanded throttle. Set False for the ideal
    proportional-thruster screening model.

    ``ctrl_every`` decimates the guidance/PD/allocation to every Nth dynamics
    tick (default 1 = every tick (Basilisk parity), ~30x plant-bandwidth
    margin). Sensing and the bias observer still run every tick (stability);
    the wrench is held (ZOH) between control ticks and fuel books
    ``u * ctrl_every * dt`` per control update.
    """
    from cubesat_12u_sim.core import config as Cfg
    from Basilisk.utilities import RigidBodyKinematics as rk

    if mc_cfg is None:
        from cubesat_12u_sim.uncertainty.monte_carlo import MonteCarloConfig
        mc_cfg = MonteCarloConfig()
    if disp is None:
        from cubesat_12u_sim.uncertainty.monte_carlo import sample_dispersions
        disp = sample_dispersions(seed, mc_cfg)
    dt = float(task_step or mc_cfg.task_step)
    dlog = float(log_step or mc_cfg.log_step)

    mass = float(disp["mass"])
    inertia = np.asarray(disp["inertia"], float)
    sigma = np.asarray(disp["sigma0"], float)
    omega = np.asarray(disp["omega0"], float)
    r = np.asarray(disp["r0"], float)
    v = np.asarray(disp["v0"], float)

    y0 = pack_mrp_state(r, v, sigma, omega)
    ta = make_mrp_integrator(y0, np.zeros(3), np.zeros(3), mass, inertia)

    # Gyro + observer state (mirrors MissionNavigation)
    rng = np.random.default_rng(seed + 999983)
    bias_true = np.asarray(disp["gyro_bias"], float)
    arw = np.asarray(disp["gyro_arw"], float)
    bias_est = np.zeros(3)
    est_rate = None
    sigma_init = np.asarray(disp["sigma0"], float)
    roll_ref = np.asarray(mc_cfg.sun_roll_ref, float)

    n_steps = int(round(mc_cfg.t_total / dt))
    log_every = max(1, int(round(dlog / dt)))
    times, omegas, sigmas, sigma_errs = [], [], [], []
    on_time = np.zeros(len(Cfg.THRUSTER_TMAX))
    phase_imp = {"detumble": 0.0, "slew": 0.0, "sunpoint": 0.0}
    phase_fuel = {"detumble": 0.0, "slew": 0.0, "sunpoint": 0.0}
    isp, g0 = float(disp["thruster_isp"]), float(Cfg.G0)
    dbg_torque: list[np.ndarray] = []
    dbg_on: list[np.ndarray] = []
    from collections import deque
    from cubesat_12u_sim.core.actuators import allocate_cached, allocation_matrices
    delay_q: deque = deque(maxlen=max(1, int(loop_delay_ticks) + 1))
    for _ in range(int(loop_delay_ticks)):
        delay_q.append((np.zeros(3), np.zeros(3)))
    P_nom = allocation_matrices(  # controller assumes nominal geometry
        Cfg.THRUSTER_POS, Cfg.THRUSTER_DIR, Cfg.THRUSTER_TMAX)
    t_ctrl = float(dt * int(ctrl_every))  # ZOH control period
    sigma_slew = np.asarray(mc_cfg.sigma_slew, float)
    sun_refs = precompute_sun_refs(mc_cfg, roll_ref, dt)
    ws = wrench_setup(disp)
    tmax_arr = ws["tmax"]
    isp = ws["isp"]

    def ref_at(step: int) -> np.ndarray:
        t = step * dt
        if t < mc_cfg.t_detumble:
            return sigma_init
        if t < mc_cfg.t_detumble + mc_cfg.t_slew:
            return sigma_slew
        return sun_refs[step]

    F_hold = np.zeros(3)
    T_hold = np.zeros(3)

    for k in range(n_steps + 1):
        t = k * dt
        if k % log_every == 0:
            times.append(t)
            omegas.append(omega.copy())
            sigmas.append(sigma.copy())
            sigma_errs.append(mrp_diff(sigma, ref_at(k)))
        if k == n_steps:
            break
        # --- sensing + observer every tick (cheap, keeps 50 Hz stability) ---
        # NOTE: gyro_noise_std mirrors measured Basilisk ImuSensor scaling.
        meas = omega + bias_true + rng.normal(0.0, 1.0, 3) * gyro_noise_std(arw)
        if est_rate is None:
            est_rate = meas.copy()
        else:
            resid = meas - est_rate - bias_est
            est_rate += float(Cfg.ACS_OBSERVER_GAIN) * resid * dt
            bias_est += float(Cfg.ACS_OBSERVER_BIAS_GAIN) * resid * dt
        fb_omega = meas - bias_est
        # --- guidance + PD + allocation at the decimated control rate ---
        if k % int(ctrl_every) == 0:
            e = mrp_diff(sigma, ref_at(k))
            torque_cmd = -float(Cfg.ACS_K) * e - float(Cfg.ACS_P) * fb_omega
            u = allocate_cached(P_nom, np.zeros(3),
                                float(Cfg.THRUSTER_CONTROL_SCALE) * torque_cmd)
            if quantize_thrusters:
                F_hold, T_hold = fast_wrench_pars(
                    u, ws, True)
            else:
                F_hold, T_hold = fast_wrench_pars(
                    u, ws, False)
            on = np.clip(u, 0.0, 1.0) * t_ctrl
            on_time += on
            dbg_torque.append(torque_cmd.copy())
            dbg_on.append(on.copy())
            tick_imp = float(np.sum(on * tmax_arr))
            tick_fuel = tick_imp / (isp * g0)
            ph = ("detumble" if t < mc_cfg.t_detumble
                  else "slew" if t < mc_cfg.t_detumble + mc_cfg.t_slew else "sunpoint")
            phase_imp[ph] += tick_imp
            phase_fuel[ph] += tick_fuel
        F_true, T_true = F_hold, T_hold
        # --- step dynamics with the (possibly delayed) wrench; fuel books
        # the commanded throttle, like the allocator log does
        if int(loop_delay_ticks) > 0:
            delay_q.append((F_true.copy(), T_true.copy()))
            F_app, T_app = delay_q.popleft()
        else:
            F_app, T_app = F_true, T_true
        pars = pack_pars_parts(F_app, T_app, mass, ws["I6"], ws["invI6"])
        for i, val in enumerate(pars):
            ta.pars[i] = float(val)
        ta.propagate_for(dt)
        st = np.asarray(ta.state, float)
        r, v, sigma, omega = st[0:3], st[3:6], st[6:9], st[9:12]
        if float(sigma @ sigma) > 1.0:
            sigma = -sigma / float(sigma @ sigma)
            ta.state[6:9] = sigma

    times_a = np.asarray(times)
    omega_hist = np.asarray(omegas)
    sigma_hist = np.asarray(sigmas)
    sigma_err_norm = np.linalg.norm(np.asarray(sigma_errs), axis=1)
    omega_norm = np.degrees(np.linalg.norm(omega_hist, axis=1))

    # Sun angle (same convention as monte_carlo)
    bore = np.asarray(mc_cfg.sun_pointing_axis_b, float)
    sun_ang = np.zeros(len(times_a))
    for j, tj in enumerate(times_a):
        dcm = np.asarray(rk.MRP2C(sigma_hist[j]), float)
        sb = dcm @ sun_direction_at(float(tj))
        sun_ang[j] = np.degrees(np.arccos(np.clip(float(bore @ sb), -1.0, 1.0)))

    # --- KPIs (identical logic to monte_carlo, incl. boundary fix)
    dmask = times_a <= mc_cfg.t_detumble
    dt_t, dt_r = times_a[dmask], omega_norm[dmask]
    above = np.where(dt_r >= mc_cfg.rate_settle_thresh_deg_s)[0]
    if len(above) == 0:
        d_settle = float(dt_t[0])
    elif above[-1] == len(dt_r) - 1:
        d_settle = float(mc_cfg.t_detumble)
    else:
        d_settle = float(dt_t[above[-1] + 1])

    smask = (times_a > mc_cfg.t_detumble) & (times_a < mc_cfg.t_detumble + mc_cfg.t_slew)
    st_t, st_e = times_a[smask], sigma_err_norm[smask]
    sang = np.degrees(4.0 * np.arctan(st_e))
    abv = np.where(sang >= mc_cfg.slew_settle_thresh_deg)[0]
    if len(abv) == 0:
        s_settle = 0.0
    elif abv[-1] == len(sang) - 1:
        s_settle = float(mc_cfg.t_slew)
    else:
        s_settle = float(st_t[abv[-1] + 1] - mc_cfg.t_detumble)
    final_slew = float(sang[-1]) if len(sang) else 0.0

    steady = sun_ang[times_a >= (mc_cfg.t_detumble + mc_cfg.t_slew + mc_cfg.t_sunpoint - 40.0)]
    final_sun = float(sun_ang[-1])
    ss_max = float(np.max(steady)) if len(steady) else final_sun
    inside = bool(ss_max <= mc_cfg.sun_cone_angle_deg)
    success = bool((dt_r[-1] < mc_cfg.rate_settle_thresh_deg_s * 2.0) and inside)

    det_f = phase_fuel["detumble"] * 1000.0
    sl_f = phase_fuel["slew"] * 1000.0
    sp_f = phase_fuel["sunpoint"] * 1000.0
    tot_imp = phase_imp["detumble"] + phase_imp["slew"] + phase_imp["sunpoint"]
    return {
        "run_idx": 1, "seed": seed, "success": success,
        "dispersions": {
            "mass_kg": mass,
            "inertia_diag": [float(inertia[0, 0]), float(inertia[1, 1]), float(inertia[2, 2])],
            "com_offset_mm": (np.asarray(disp["com_offset"], float) * 1000.0).tolist(),
            "thruster_isp_s": isp,
            "omega0_deg_s": np.degrees(np.asarray(disp["omega0"], float)).tolist(),
            "sigma0": np.asarray(disp["sigma0"], float).tolist(),
        },
        "kpis": {
            "detumble_settle_time_s": d_settle,
            "final_detumble_rate_deg_s": float(dt_r[-1]),
            "slew_settle_time_s": s_settle,
            "final_slew_error_deg": final_slew,
            "final_sun_angle_deg": final_sun,
            "steady_state_max_sun_angle_deg": ss_max,
            "inside_sun_cone": inside,
            "detumble_propellant_g": det_f,
            "slew_propellant_g": sl_f,
            "sunpoint_propellant_g": sp_f,
            "total_propellant_g": det_f + sl_f + sp_f,
            "total_impulse_ns": tot_imp,
            "delta_v_m_s": float(tot_imp / mass),
            "thruster_duty_cycles": (on_time / mc_cfg.t_total).tolist(),
        },
        "timeseries": {
            "time": times_a, "omega_norm_deg_s": omega_norm,
            "sigma_err_norm": sigma_err_norm, "sun_angle_deg": sun_ang,
        },
        "signals": {
            "torque_demand": np.asarray(dbg_torque),
            "on_time": np.asarray(dbg_on),
        },
    }
