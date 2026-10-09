"""Central parameter file: 12U CubeSat, Euler angles, gyros, variable-thrust jets, free-fall at 12000 km."""
from __future__ import annotations
from pathlib import Path

import numpy as np

# ---------------- Earth / Orbit ----------------
MU_EARTH: float = 3.986004418e14      # [m^3/s^2] GM Earth
R_EARTH: float = 6378.137e3           # [m] equatorial radius
ALTITUDE: float = 12000e3             # [m] free-fall orbit altitude above surface
R_ORBIT: float = R_EARTH + ALTITUDE   # [m] ~= 18378.137 km circular orbit radius
# Circular Keplerian velocity / period at that radius (free-fall, no drag/J2 by default)
V_CIRC: float = float(np.sqrt(MU_EARTH / R_ORBIT))          # [m/s] ~= 4657 m/s
T_ORBIT: float = float(2.0 * np.pi * np.sqrt(R_ORBIT**3 / MU_EARTH))  # [s] ~= 24816 s (~6.9 h)

# Initial orbital state (ECI, equatorial circular, start on +X axis, velocity +Y)
R0_ECI = np.array([R_ORBIT, 0.0, 0.0])
V0_ECI = np.array([0.0, V_CIRC, 0.0])

# ---------------- 12U CubeSat bus ----------------
# 12U standard ~ 200 x 200 x 300 mm class, up to ~24 kg. Nominal values below.
MASS: float = 20.0                    # [kg]
LX, LY, LZ = 0.20, 0.20, 0.30         # [m] body dimensions (x, y, z)
# Solid rectangular-prism inertia about centre of mass
IX = MASS / 12.0 * (LY**2 + LZ**2)
IY = MASS / 12.0 * (LX**2 + LZ**2)
IZ = MASS / 12.0 * (LX**2 + LY**2)
INERTIA = np.diag([IX, IY, IZ])       # [kg m^2]
INV_INERTIA = np.linalg.inv(INERTIA)

# ---------------- Attitude: Euler angles (3-2-1 / ZYX yaw-pitch-roll) ----------------
# euler = [phi (roll, x), theta (pitch, y), psi (yaw, z)], radians.
# DCM body-from-inertial: R_bi = Rx(phi) @ Ry(theta) @ Rz(psi)
# Singularity at theta = +/- pi/2 (pitch). Integrator clamps/warns there.
EULER0 = np.radians(np.array([10.0, 5.0, 0.0]))  # initial attitude [rad]
OMEGA0_BODY = np.radians(np.array([1.0, 0.5, -0.5]))  # initial body rate [rad/s]

# ---------------- Variable-thrust jets (RCS) ----------------
# N thrusters, throttle u_i in [0, 1] continuous (throttlable, e.g. cold-gas / monoprop in PWM-averaged sense).
# Each thruster: position r_i in body frame [m], unit direction d_i (exhaust thrust axis in body), Tmax_i [N].
# Default: 12-thruster 6-DOF set, 1.0 N max each, face-mounted with moment arm for torque authority.
TMAX_JET: float = 1.0  # [N] per thruster
ISP_JET: float = 220.0   # [s] assumed monoprop Isp (propellant bookkeeping only)
G0: float = 9.80665      # [m/s^2]
# positions: +/- faces offset to edges so thrust gives torque
_hx, _hy, _hz = LX / 2, LY / 2, LZ / 2
THRUSTER_POS = np.array([
    [+_hx, +_hy, 0.0], [+_hx, -_hy, 0.0],   # +X face, push -X
    [-_hx, +_hy, 0.0], [-_hx, -_hy, 0.0],   # -X face, push +X
    [+_hx, 0.0, +_hz], [-_hx, 0.0, +_hz],   # +Z face, push -Z
    [+_hx, 0.0, -_hz], [-_hx, 0.0, -_hz],   # -Z face, push +Z
    [0.0, +_hy, +_hz], [0.0, +_hy, -_hz],   # +Y face, push -Y
    [0.0, -_hy, +_hz], [0.0, -_hy, -_hz],   # -Y face, push +Y (folded to 12 below: keep 12)
])
# Trim/pad to exactly 12 entries with directions below (kept explicit for clarity)
THRUSTER_POS = np.array([
    [ _hx,  _hy, 0.0], [ _hx, -_hy, 0.0],
    [-_hx,  _hy, 0.0], [-_hx, -_hy, 0.0],
    [ _hx, 0.0,  _hz], [-_hx, 0.0,  _hz],
    [ _hx, 0.0, -_hz], [-_hx, 0.0, -_hz],
    [0.0,  _hy,  _hz], [0.0,  _hy, -_hz],
    [0.0, -_hy,  _hz], [0.0, -_hy, -_hz],
])
THRUSTER_DIR = np.array([
    [-1, 0, 0], [-1, 0, 0],
    [+1, 0, 0], [+1, 0, 0],
    [0, 0, -1], [0, 0, -1],
    [0, 0, +1], [0, 0, +1],
    [0, -1, 0], [0, -1, 0],
    [0, +1, 0], [0, +1, 0],
], dtype=float)
THRUSTER_TMAX = np.full(12, TMAX_JET)

# Reaction wheels (optional gyroscopic actuators, 3 orthogonal, body-aligned)
RW_MAX_TORQUE = 0.05    # [N m]
RW_MAX_MOMENTUM = 0.5   # [N m s]
RW_TAU = 0.2            # [s] first-order response time constant

# ---------------- Rate gyroscopes (sensors) ----------------
GYRO_BIAS0 = np.radians(np.array([0.05, -0.03, 0.02]))  # [rad/s] turn-on bias
GYRO_ARW = np.radians(np.array([0.01, 0.01, 0.01]))     # [rad/s/sqrt(Hz)] angle random walk density
GYRO_BIAS_TAU = 300.0    # [s] bias random-walk correlation time
GYRO_BIAS_WALK = np.radians(np.array([0.001, 0.001, 0.001]))  # [rad/s/sqrt(Hz)] bias instability drive
GYRO_RATE_HZ = 100.0     # [Hz] sampling rate

# ---------------- Attitude control ----------------
ACS_K: float = 0.04             # [rad/s] MRP proportional gain
ACS_P: float = 0.4              # [N m s] body-rate feedback gain
ACS_KI: float = -1.0            # [N m] negative disables integral feedback
ACS_INTEGRAL_LIMIT: float = 0.05  # [N m] integral windup limit
ACS_SLEW_SIGMA = np.array([0.0, 0.0, -1.0])  # 90 deg inertial slew target
THRUSTER_CONTROL_SCALE: float = 2.0
ACS_OBSERVER_GAIN: float = 50.0
ACS_OBSERVER_BIAS_GAIN: float = 0.5
VIZ_SPACECRAFT_NAME: str = "12uSpacecraft"
# Bus + two deployed solar wings, generated as an OBJ in metres at true scale.
VIZ_MODEL_KEY: str = "PANELS"
VIZ_PANEL_MODEL_KEY: str = "PANELS"
VIZ_PANEL_DEPLOY: float = 0.5   # [m] wing length outboard of each +/-X bus face
VIZ_PANEL_THICKNESS: float = 0.01  # [m] wing plate thickness
# Visualization geometry only; INERTIA above is never derived from these.
VIZ_MODEL_DIR: str = str(Path(__file__).resolve().parents[1] / "assets" / "models")
# One shared asset: the geometry depends only on the bus dimensions, not on the
# scene, so the detumble/slew scenes reuse the same file.
VIZ_PANEL_MODEL_FILENAME: str = "12u_panelled_bus.obj"
# Vizard primitive shapes accepted by vizSupport.createCustomModel(modelPath=...)
VIZ_PRIMITIVE_MODELS: tuple[str, ...] = ("CUBE", "CYLINDER", "SPHERE")
# Vizard built-in CAD dictionary keys, matched case-insensitively
VIZ_BUILTIN_MODELS: dict[str, str] = {
    "bsksat": "bskSat",
    "3usat": "3USat",
    "6usat": "6USat",
}
# Uniform display size for the Planet / Solar System views only. The spacecraft
# local view is drawn at true scale, so this must NOT also be folded into the
# createCustomModel() scale or the bus is rendered oversize while the thruster,
# CSS and IMU overlays stay at their true body-frame offsets.
VIZ_DISPLAY_SCALE: float = 10.0
# Segments used for the osculating orbit line. Recomputed per step at a fixed
# count, so it is cheap; the flown trajectory is the expensive one and is
# opt-in via the --trajectory-line flag.
VIZ_ORBIT_LINE_SEGMENTS: int = 200
# GenericSensor cone marking the IMU, placed on the +Z bus face pointing +Z
VIZ_IMU_POSITION_B = (0.0, 0.0, 0.5 * LZ)
VIZ_IMU_NORMAL_B = (0.0, 0.0, 1.0)
VIZ_IMU_FOV_DEG: float = 30.0
VIZ_IMU_SIZE: float = 0.05  # [m] cone size, comparable to the 0.2 m bus
# 2-way live-stream endpoint; BSK binds reqComAddress:reqPortNumber and Vizard
# is launched pointing at the same endpoint, so keep this the single source.
VIZ_COM_PROTOCOL: str = "tcp"
VIZ_COM_ADDRESS: str = "127.0.0.1"
VIZ_COM_PORT: str = "5556"
VIZ_APP_PATH: str = "/Applications/Vizard.app"

# ---------------- Sun and sun-pointing orientation constraint ----------------
# Constrained boresight: the body axis the ACS drives onto the Sun and the axis
# the keep-in cone is built about (body +X). The solar wings lie in the body
# XY plane (normal +Z, see VIZ_PANEL_NORMAL_B below), so this boresight is
# in the wing plane, not the wing normal -- deliberate: +X hosts css1 which
# measures the sun-pointing error.
# Constrained boresight: the body axis the ACS drives onto the Sun and the axis
# the keep-in cone is built about.
SUN_POINTING_AXIS_B = (1.0, 0.0, 0.0)
# Panel normal for the generated OBJ. Deliberately independent of the constrained
# boresight above; set it equal to SUN_POINTING_AXIS_B to have the array face the
# Sun as well. With the defaults the wings lie in the body XY plane (normal +Z)
# while the constraint is applied about +X.
VIZ_PANEL_NORMAL_B = (0.0, 0.0, 1.0)
# Body axis the solar wings deploy along. Must be a different axis from
# VIZ_PANEL_NORMAL_B; the wing plane contains it, so the panel normal is exactly
# VIZ_PANEL_NORMAL_B.
VIZ_PANEL_DEPLOY_AXIS_B = (1.0, 0.0, 0.0)
# Inertial reference the remaining roll is pinned to, projected perpendicular to
# the Sun line. Purely a roll convention: it fixes the twist about the boresight.
SUN_ROLL_REFERENCE_N = (1.0, 0.0, 0.0)
# displayName of the Basilisk Sun body, which is the name Vizard registers and
# the toBodyName that createConeInOut / createPointLine must reference
VIZ_SUN_BODY_NAME: str = "sun"
# [m] Sun distance used for both the ephemeris and the CSS sun vector (~1 AU).
# The body mu comes from Basilisk, so the orbital rate is derived from it and the
# two cannot disagree.
VIZ_SUN_DISTANCE: float = 1.496e11
VIZ_SUN_CONE_ANGLE_DEG: float = 15.0  # [deg] half-angle of the allowed misalignment
VIZ_SUN_CONE_HEIGHT: float = 2.0      # [m] cone length, bus is 0.3 m and span 1.2 m
VIZ_SUN_CONE_COLOR: str = "lime"
VIZ_SUN_LINE_COLOR: str = "yellow"
# The Earth-centred frame sees the Sun sweep a full circle once per year, so the
# Sun is given a circular apparent orbit. One tropical year [s].
SUN_YEAR: float = 365.25 * 86400.0
# 0 -> Sun along inertial +Y at t=0, matching the direction this scenario has
# always assumed. Set to pi/2 to start the Sun along +X instead.
SUN_PHASE0: float = 0.5 * np.pi
# When true the ACS reference tracks the moving Sun, so every maneuver ends
# satisfying the keep-in cone. Set False to restore the previous inertial-fixed
# targets (ACS_SLEW_SIGMA / hold), which do not satisfy the cone.
ACS_SUN_POINTING: bool = True

# ---------------- Orbit-keeping PID (RTN frame) ----------------
# Simple decoupled PID: radial (R), transverse/along-track (T), normal (N).
# a_R = Kp_R*(Rdes-R) - Kd_R*v_radial + Ki_R*int(Rdes-R)
# a_T = Kp_T*(VcircDes-v_T) + Ki_T*int(...)      (velocity-form, no Kd)
# a_N = Kp_N*(-r_N) - Kd_N*v_N + Ki_N*int(-r_N)  (equatorial: drive out-of-plane to 0)
PID_KP_R: float = 2.0e-6    # [1/s^2]
PID_KD_R: float = 2.0e-3    # [1/s]
PID_KI_R: float = 1.0e-9    # [1/s^3]
PID_KP_T: float = 1.0e-3    # [1/s]  (on transverse-velocity error)
PID_KI_T: float = 1.0e-6    # [1/s^2]
PID_KP_N: float = 2.0e-6    # [1/s^2]
PID_KD_N: float = 2.0e-3    # [1/s]
PID_KI_N: float = 1.0e-9    # [1/s^3]
PID_INT_CLAMP: float = 5.0e4  # [m s] integral anti-windup clamp (R/N); velocity-loop uses m
PID_A_MAX: float = 0.05     # [m/s^2] per-axis saturation (~1 N on 20 kg bus)
PID_DEADBAND: float = 1.0e-5  # [m/s^2] commands below this are set to 0 (fuel saving)
DT_CONTROL: float = 10.0    # [s] PID / thruster zero-order-hold period

# ---------------- Heyoka integration ----------------
HEYOKA_TOL: float = 1e-12   # Taylor adaptive tolerance
HEYOKA_HIGH_ACCURACY: bool = False

# ---------------- Hierarchical NMPC + fast-PID (outer 10 s / inner 1 s) ----------------
# High-level NMPC plans bulk ECI accel (+ torque) over a short horizon;
# low-level fast PID only trims model/disturbance error under a tiny authority
# + cumulative dv budget. Modular: SolverBackend in nmpc.py (scipy now,
# casadi/acados later) without touching this file.
NMPC_DT_OUTER: float = 10.0   # [s] outer replan period (matches DT_CONTROL)
NMPC_DT_INNER: float = 1.0    # [s] fast-PID trim period
NMPC_HORIZON: int = 8         # outer steps predicted (HORIZON*DT_OUTER ~ 80 s)
NMPC_Q_POS: float = 1.0       # normalized weight on deadbanded (e_a/50km)^2 (energy, not radius)
NMPC_Q_VEL: float = 0.1       # normalized weight on (v_radial/50m/s)^2 (eccentricity damping)
NMPC_R_FUEL: float = 0.5      # normalized weight on |a/a_max|^2 (raise = save fuel, must beat tracking when close)
NMPC_DB_A: float = 500.0      # [m] semi-major-axis deadband: no tracking cost inside (coast, like SemiMajorPID)
NMPC_A_MAX: float = PID_A_MAX  # [m/s^2] per-axis, same jet capability as PID
NMPC_TAU_MAX: float = 0.02    # [N m] outer torque authority (small; jets do bulk force)
TRIM_A_MAX: float = 0.005     # [m/s^2] inner trim saturation (10% of PID_A_MAX)
TRIM_TAU_MAX: float = 0.005   # [N m] inner torque trim saturation
TRIM_DV_BUDGET: float = 5e-3  # [m/s] cumulative trim dv per outer window, then trim latches off.
# Was 0.5. That budget let the inner trim saturate and cancel the NMPC bulk plan
# every 600 s tick (measured bulk 0.00 / trim 36.00 m/s), so the closed loop ran
# trim-only and the station-keeping box error was set by the trim rather than by
# the optimiser. At 5e-3 the trim can spend ~0.4 m/s over a 0.5-day window, i.e.
# it supplements the plan instead of overriding it. See NMPC_SPACE in
# optimization/params.py; that span is 1e-4..5e-2.
TRIM_DEADBAND: float = 1.0e-5  # [m/s^2] inner deadband (same as PID_DEADBAND)
NMPC_SOLVER: str = "SLSQP"    # scipy.optimize.minimize method for ScipyShootingBackend
NMPC_MAXITER: int = 100       # per-solve iteration cap (POC real-time guard)
NMPC_FALLBACK: str = "pid"    # on solver failure: "pid" (zero-order hold safe) or "zero"

# ---------------- Simulation defaults ----------------
USE_GRAVITY_GRADIENT: bool = True   # only disturbance at 12000 km free-fall (drag ~ 0)
USE_J2: bool = False                # keep off for pure Keplerian free-fall baseline
J2: float = 1.08262668e-3
DT_LOG: float = 1.0                 # [s] output sampling
T_END_DEFAULT: float = 3600.0       # [s] 1 h demo (full orbit = T_ORBIT)
RTOL, ATOL = 1e-9, 1e-12
