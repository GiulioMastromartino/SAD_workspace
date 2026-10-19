"""Simple orbit-keeping PID in the RTN frame -> desired inertial thrust.

Loop (run at DT_CONTROL):
  1. Build RTN frame from current (r, v).
  2. Radial PID on altitude error, transverse PI on circular-velocity error,
     normal PID on out-of-plane error -> desired acceleration a_RTN.
  3. Saturate + deadband -> a_eci -> F_eci = m * a_eci.
  4. Rotate into body frame with current Euler attitude, allocate to the
     variable-thrust jets (allocation_pinv), return throttle vector.
"""
from __future__ import annotations
import numpy as np
from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.core.dynamics import euler_to_dcm
from cubesat_12u_sim.core.actuators import allocation_pinv, jets_to_wrench


def rtn_frame(r: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    R = np.asarray(r, float) / np.linalg.norm(r)
    h = np.cross(r, v)
    N = h / np.linalg.norm(h)
    T = np.cross(N, R)
    return R, T, N


class SemiMajorPID:
    """Long-horizon station-keeping on the (non-oscillating) semi-major axis.

    a_osc = 1 / (2/r - v^2/mu) is constant along a Keplerian orbit, so feeding
    back on it never fights the natural epicycle (the failure mode of raw
    RTN PIDs, which chatter on radial/transverse oscillations and waste fuel).
      a_T = Kp_a * (ades - a_osc) + Ki_a * int(...)   (transverse, energy)
      a_R = -Kd_r * v_radial                           (weak epicycle damping)
    Integrators accumulate only while |ades - a| > db_a (no windup); output
    deadband gives a true coast once recovered.
    """

    def __init__(self, kp_a: float = 3.0e-7, ki_a: float = 0.0,
                 kd_r: float = 5.0e-4, db_a: float = 500.0,
                 deadband: float | None = None):
        self.kp_a = float(kp_a)    # [1/s^2]
        self.ki_a = float(ki_a)    # [1/s^3]
        self.kd_r = float(kd_r)    # [1/s]
        self.db_a = float(db_a)    # [m] a-error deadband
        self.deadband = float(deadband) if deadband is not None else float(C.PID_DEADBAND)
        self.int_a = 0.0

    def reset(self):
        self.int_a = 0.0

    def semi_major(self, r: np.ndarray, v: np.ndarray) -> float:
        rn, v2 = float(np.linalg.norm(r)), float(v @ v)
        return 1.0 / (2.0 / rn - v2 / C.MU_EARTH)

    def accel_command(self, r: np.ndarray, v: np.ndarray, dt: float) -> np.ndarray:
        R, T, N = rtn_frame(r, v)
        a_osc = self.semi_major(r, v)
        e_a = C.R_ORBIT - a_osc  # ades = R_ORBIT for circular target
        if abs(e_a) > self.db_a:
            self.int_a = np.clip(self.int_a + e_a * dt,
                                 -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_T = self.kp_a * e_a + self.ki_a * self.int_a
        v_r = float(v @ R)
        a_R = -self.kd_r * v_r
        a_rtn = np.clip(np.array([a_R, a_T, 0.0]), -C.PID_A_MAX, C.PID_A_MAX)
        a_rtn[np.abs(a_rtn) < self.deadband] = 0.0
        return R * a_rtn[0] + T * a_rtn[1]  # N component always 0 (equatorial)

class OrbitPID:
    """Decoupled RTN PID for circular-orbit maintenance. See config PID_* gains.

    box_r: optional station-keeping box half-width [m]. If > 0 and the craft is
      inside |R - Rdes| < box_r, thrust is cut (zeros) and integrators leak off.
      Standard box strategy: drift freely inside, correct at the edges.
    """

    def __init__(self, box_r: float = 0.0, deadband: float | None = None,
                 err_db_r: float = 0.0, err_db_t: float = 0.0, err_db_n: float = 0.0):
        self.box_r = float(box_r)
        self.deadband = float(deadband) if deadband is not None else float(C.PID_DEADBAND)
        # error deadbands: gate P+I below these, DAMPING ALWAYS stays on
        self.err_db_r = float(err_db_r)  # [m] on radial position error
        self.err_db_t = float(err_db_t)  # [m/s] on transverse-velocity error
        self.err_db_n = float(err_db_n)  # [m] on out-of-plane error
        self.int_r = 0.0   # integral of radial position error [m s]
        self.int_t = 0.0   # integral of transverse-velocity error [m]
        self.int_n = 0.0   # integral of out-of-plane error [m s]

    def reset(self):
        self.int_r = self.int_t = self.int_n = 0.0

    def accel_command(self, r: np.ndarray, v: np.ndarray, dt: float) -> np.ndarray:
        """Desired inertial acceleration [m/s^2] (RTN PID + saturation + deadband)."""
        r_norm = float(np.linalg.norm(r))
        if self.box_r > 0.0 and abs(r_norm - C.R_ORBIT) < self.box_r:
            # inside the box: coast, leak off integrators (no chatter)
            self.int_r *= 0.9
            self.int_t *= 0.9
            self.int_n *= 0.9
            return np.zeros(3)
        R, T, N = rtn_frame(r, v)
        v_r = float(v @ R)  # radial velocity [m/s]
        v_t = float(v @ T)                                # transverse velocity [m/s]
        r_n = float(r @ N)                                # out-of-plane position (signed ~0)
        v_n = float(v @ N)                                # out-of-plane velocity [m/s]
        v_circ = float(np.sqrt(C.MU_EARTH / r_norm))

        # --- P+I gated by error deadbands (don't fight the natural epicycle);
        # --- D damping always on; integrators accumulate only when gated in ---
        e_r = C.R_ORBIT - r_norm
        g_r = 0.0 if abs(e_r) < self.err_db_r else 1.0
        int_r_old, int_t_old, int_n_old = self.int_r, self.int_t, self.int_n
        self.int_r = np.clip(self.int_r + g_r * e_r * dt,
                             -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_R = g_r * (C.PID_KP_R * e_r + C.PID_KI_R * self.int_r) - C.PID_KD_R * v_r

        e_t = v_circ - v_t
        g_t = 0.0 if abs(e_t) < self.err_db_t else 1.0
        self.int_t = np.clip(self.int_t + g_t * e_t * dt,
                             -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_T = g_t * (C.PID_KP_T * e_t + C.PID_KI_T * self.int_t)

        e_n = -r_n
        g_n = 0.0 if abs(e_n) < self.err_db_n else 1.0
        self.int_n = np.clip(self.int_n + g_n * e_n * dt,
                             -C.PID_INT_CLAMP, C.PID_INT_CLAMP)
        a_N = g_n * (C.PID_KP_N * e_n + C.PID_KI_N * self.int_n) - C.PID_KD_N * v_n

        a_rtn = np.clip(np.array([a_R, a_T, a_N]), -C.PID_A_MAX, C.PID_A_MAX)
        if np.all(np.abs(a_rtn) < self.deadband):
            # chatter guard: coast AND roll back this step's integral buildup
            self.int_r = int_r_old
            self.int_t = int_t_old
            self.int_n = int_n_old
            return np.zeros(3)
        a_rtn[np.abs(a_rtn) < self.deadband] = 0.0
        return R * a_rtn[0] + T * a_rtn[1] + N * a_rtn[2]  # back to ECI

    def throttle_command(self, r: np.ndarray, v: np.ndarray, euler: np.ndarray,
                         dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Full step: (r, v, euler) -> (throttle u, F_body_actual, tau_body_actual).

        Desired ECI force is mapped to the body frame with the *current*
        attitude, then allocated to jets (saturation included).
        """
        a_eci = self.accel_command(r, v, dt)
        F_eci = C.MASS * a_eci
        R_bi = euler_to_dcm(*euler)
        F_body_des = R_bi @ F_eci
        u = allocation_pinv(F_body_des, np.zeros(3))  # orbit-keeping; no torque demand
        F_body, tau_body = jets_to_wrench(u)
        return u, F_body, tau_body
