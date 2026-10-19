"""6U CubeSat parameters."""
import numpy as np

MU_EARTH = 3.986004418e14
R_EARTH = 6378.137e3
# 600 km Sun-synchronous LEO (6U mission orbit)
ALTITUDE = 600e3
R_ORBIT = R_EARTH + ALTITUDE
V_CIRC = float(np.sqrt(MU_EARTH / R_ORBIT))
T_ORBIT = float(2.0 * np.pi * np.sqrt(R_ORBIT**3 / MU_EARTH))
MASS = 8.0
LX, LY, LZ = 0.10, 0.20, 0.30
IX = MASS/12.0*(LY**2+LZ**2)
IY = MASS/12.0*(LX**2+LZ**2)
IZ = MASS/12.0*(LX**2+LY**2)
INERTIA = np.diag([IX,IY,IZ])
INV_INERTIA = np.linalg.inv(INERTIA)

# Standard 12-thruster 6-DOF set (ported from cubesat_12u_sim):
# 4 on the +/-X faces, 4 on the +/-Z faces, 4 on the +/-Y faces, mounted at the
# face edges so every thruster pair produces a moment arm.  Rank 6, full +/-
# authority on Fx/Fy/Fz/Tx/Ty/Tz.
_hx,_hy,_hz = LX/2, LY/2, LZ/2
THRUSTER_POS = np.array([
    [ _hx,  _hy, 0.0], [ _hx, -_hy, 0.0],   # +X face, push -X
    [-_hx,  _hy, 0.0], [-_hx, -_hy, 0.0],   # -X face, push +X
    [ _hx, 0.0,  _hz], [-_hx, 0.0,  _hz],   # +Z face, push -Z
    [ _hx, 0.0, -_hz], [-_hx, 0.0, -_hz],   # -Z face, push +Z
    [0.0,  _hy,  _hz], [0.0,  _hy, -_hz],   # +Y face, push -Y
    [0.0, -_hy,  _hz], [0.0, -_hy, -_hz],   # -Y face, push +Y
], dtype=float)
THRUSTER_DIR = np.array([
    [-1,0,0],[-1,0,0],
    [ 1,0,0],[ 1,0,0],
    [0,0,-1],[0,0,-1],
    [0,0, 1],[0,0, 1],
    [0,-1,0],[0,-1,0],
    [0, 1,0],[0, 1,0],
], dtype=float)
THRUSTER_TMAX = np.full(12, 0.5)  # 0.5 N for the 8 kg 6U bus (12U reference uses 1.0 N)
ISP_JET = 220.0
G0 = 9.80665

GYRO_BIAS0 = np.array([0.05,-0.03,0.02])*1e-3  # rad/s (scaled)
GYRO_ARW = np.array([0.01,0.01,0.01])*1e-3
STAR_SIGMA = 1e-4
STAR_RATE_CAP = 0.1
STAR_DROPOUT = 0.05

ACS_K_DETUMBLE = 5.339150251994063
ACS_P_DETUMBLE = 0.8828143280025442
ACS_KI_DETUMBLE = 0.027321708852311766
ACS_K_SLEW = 1.286717625102853
ACS_P_SLEW = 0.4457756848994122
ACS_KI_SLEW = 0.0031478207674284194
ACS_K_TRACK = 4.409346198161378
ACS_P_TRACK = 49.85938825119385
ACS_KI_TRACK = 0.00010264766213601192

# Step-aware PD: scale gains so the explicit update is backward-Euler equivalent
# at the current dt. Keeps the track loop stable at dt=1.0. See
# diagnostics/IMPLICIT_PD_PROPOSAL.md.
IMPLICIT_PD = True
# Detumble/slew run at dt=0.1 and are already stable; scaling only the track
# phase avoids slowing the slew (and the actuator-authority timeouts).
IMPLICIT_TRACK_ONLY = True

SUN_KEEP_IN_CONE_DEG = 15.0

# Optimizer cost shaping.  Errors inside the box/cone are quadratic; outside
# they grow exponentially so the optimizer strongly avoids violations.
ORBIT_BOX_KM = 5.0
COST_BOX_WEIGHT = 1.0
COST_CONE_WEIGHT = 1.0
COST_EXP_RATE = 1.0
ACS_SLEW_QUAT = np.array([0,0,np.sin(np.pi/4),np.cos(np.pi/4)])  # 90 deg about Z
ACS_TAU_MAX = 0.1  # body torque command limit (Nm); avoids allocator saturation limit cycles
# Orbit injection dispersion used by the Monte Carlo campaigns.
DISP_R_KM = 1.0       # +/- km on each ECI position axis
DISP_V_MPS = 0.5      # +/- m/s on each ECI velocity axis
R0_ECI = np.array([R_ORBIT,0.0,0.0])
V0_ECI = np.array([0.0,V_CIRC,0.0])
EULER0 = np.radians(np.array([10.0,5.0,0.0]))
OMEGA0_BODY = np.radians(np.array([1.0,0.5,-0.5]))
HEYOKA_TOL = 1e-12
DT = 0.1
TRACK_DT = 1.0  # integration step during the track phase (s); implicit PD keeps it stable
TS = 278.2200065567522  # time budget for detumble+slew (seconds)
T_END = TS + 2.0 * T_ORBIT  # total mission: detumble+slew+2 orbits track
