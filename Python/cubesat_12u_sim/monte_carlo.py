"""Monte Carlo simulation suite for the 12U CubeSat mission with 3 thruster maneuvers.

Maneuvers:
  1. Detumble: Null initial tip-off body rates after separation.
  2. Slew: Reorient spacecraft to target inertial attitude.
  3. Sun-pointing: Align and track the primary boresight to the Sun vector,
     maintaining attitude inside the 15-degree keep-in cone.

All maneuvers use ONLY the 12 variable-thrust RCS jets (no reaction wheels).
Dispersions:
  - Spacecraft mass and inertia tensor (with off-diagonal cross products)
  - Center of Mass (CoM) offset / shift (thruster torque arm variation)
  - Thruster max thrust, canting / alignment errors, and Isp
  - Initial tip-off angular rates and attitude dispersion
  - Initial orbit position and velocity injection errors
  - Sensor errors: Gyro bias, ARW noise, and Coarse Sun Sensor errors
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
import time
from typing import Any

import numpy as np

from . import config as C
from .actuators import allocation_pinv
from .sun_model import SunEphemeris, sun_pointing_sigma


@dataclass
class MonteCarloConfig:
    """Parameters and dispersion limits for the 12U CubeSat Monte Carlo campaign."""

    # Mission maneuver durations [s]
    t_detumble: float = 80.0
    t_slew: float = 80.0
    t_sunpoint: float = 140.0

    # Simulation time stepping
    task_step: float = 0.02   # [s] 50 Hz control and thruster update
    log_step: float = 0.5     # [s] trajectory recording interval

    # Maneuver targets & constraints
    sigma_slew: tuple[float, float, float] = (0.0, 0.0, -1.0)  # 180 deg slew target (|sigma|=1 -> 4*atan(1))
    sun_roll_ref: tuple[float, float, float] = C.SUN_ROLL_REFERENCE_N
    sun_pointing_axis_b: tuple[float, float, float] = C.SUN_POINTING_AXIS_B
    sun_cone_angle_deg: float = C.VIZ_SUN_CONE_ANGLE_DEG  # 15 deg keep-in cone

    # Performance criteria thresholds
    rate_settle_thresh_deg_s: float = 0.1   # detumble criteria [deg/s]
    slew_settle_thresh_deg: float = 2.5     # slew convergence criteria [deg]

    # Dispersions (standard deviations)
    mass_std_pct: float = 5.0               # Spacecraft mass [%]
    inertia_std_pct: float = 5.0            # Principal inertia [%]
    inertia_cross_std: float = 0.005        # Products of inertia [kg m^2]
    com_offset_std_m: float = 0.005         # Center of mass offset [m] (5 mm)
    thrust_std_pct: float = 5.0             # Individual thruster max thrust [%]
    thrust_align_std_deg: float = 1.0       # Thruster nozzle alignment error [deg]
    isp_std_pct: float = 5.0                # Thruster specific impulse [%]
    omega0_std_deg_s: float = 1.5           # Initial tip-off rates per axis [deg/s]
    sigma0_std: float = 0.08                # Initial attitude MRP dispersion
    pos0_std_m: float = 5000.0              # Orbit injection position dispersion [m]
    vel0_std_m_s: float = 5.0               # Orbit injection velocity dispersion [m/s]
    gyro_bias_std_deg_s: float = 0.02       # Initial gyro turn-on bias [deg/s]
    gyro_arw_std_pct: float = 10.0          # Gyro ARW variation [%]
    css_scale_std_pct: float = 3.0          # CSS scale factor dispersion [%]
    css_bias_std: float = 0.0003            # CSS bias dispersion
    css_noise_std: float = 0.002            # CSS noise dispersion

    @property
    def t_total(self) -> float:
        return self.t_detumble + self.t_slew + self.t_sunpoint


def sample_dispersions(seed: int, mc_cfg: MonteCarloConfig) -> dict[str, Any]:
    """Sample perturbed parameters for a single Monte Carlo realization."""
    rng = np.random.default_rng(seed)

    # 1. Mass & Inertia
    mass_factor = 1.0 + rng.normal(0.0, mc_cfg.mass_std_pct / 100.0)
    mass = float(np.clip(C.MASS * mass_factor, 15.0, 30.0))

    # Principal moments of inertia + products
    ixx = C.IX * (1.0 + rng.normal(0.0, mc_cfg.inertia_std_pct / 100.0))
    iyy = C.IY * (1.0 + rng.normal(0.0, mc_cfg.inertia_std_pct / 100.0))
    izz = C.IZ * (1.0 + rng.normal(0.0, mc_cfg.inertia_std_pct / 100.0))
    ixy = rng.normal(0.0, mc_cfg.inertia_cross_std)
    ixz = rng.normal(0.0, mc_cfg.inertia_cross_std)
    iyz = rng.normal(0.0, mc_cfg.inertia_cross_std)

    inertia = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ], dtype=float)
    # Ensure positive definiteness
    evals, evecs = np.linalg.eigh(inertia)
    evals = np.clip(evals, 0.01, 10.0)
    inertia = evecs @ np.diag(evals) @ evecs.T

    # Center of mass offset
    com_offset = rng.normal(0.0, mc_cfg.com_offset_std_m, size=3)

    # 2. Thruster Cluster Dispersions (12 jets)
    n_thrusters = len(C.THRUSTER_POS)
    thrust_factors = 1.0 + rng.normal(0.0, mc_cfg.thrust_std_pct / 100.0, size=n_thrusters)
    thruster_tmax = np.clip(C.THRUSTER_TMAX * thrust_factors, 0.5, 1.5)

    isp_factor = 1.0 + rng.normal(0.0, mc_cfg.isp_std_pct / 100.0)
    thruster_isp = float(np.clip(C.ISP_JET * isp_factor, 150.0, 300.0))

    # Thruster positions relative to perturbed CoM: r' = r - com_offset
    thruster_pos = C.THRUSTER_POS - com_offset

    # Thruster direction canting / alignment error
    thruster_dir = np.zeros_like(C.THRUSTER_DIR)
    for i in range(n_thrusters):
        d_nom = C.THRUSTER_DIR[i] / np.linalg.norm(C.THRUSTER_DIR[i])
        # Generate perpendicular perturbation axis
        rand_vec = rng.normal(size=3)
        perp_axis = np.cross(d_nom, rand_vec)
        if np.linalg.norm(perp_axis) < 1e-6:
            rand_vec[0] += 1.0
            perp_axis = np.cross(d_nom, rand_vec)
        perp_axis /= np.linalg.norm(perp_axis)
        angle_rad = np.radians(rng.normal(0.0, mc_cfg.thrust_align_std_deg))
        # Small angle rotation: d = cos(a)*d_nom + sin(a)*(perp x d_nom)
        d_pert = np.cos(angle_rad) * d_nom + np.sin(angle_rad) * np.cross(perp_axis, d_nom)
        thruster_dir[i] = d_pert / np.linalg.norm(d_pert)

    # 3. Initial State Dispersions
    # Tip-off body rate [rad/s]
    omega0_rad_s = C.OMEGA0_BODY + np.radians(rng.normal(0.0, mc_cfg.omega0_std_deg_s, size=3))

    # Initial attitude MRP: nominal [0.1, -0.2, 0.3] + dispersion
    sigma0 = np.array([0.1, -0.2, 0.3]) + rng.normal(0.0, mc_cfg.sigma0_std, size=3)

    # Orbit injection position & velocity
    r0 = C.R0_ECI + rng.normal(0.0, mc_cfg.pos0_std_m, size=3)
    v0 = C.V0_ECI + rng.normal(0.0, mc_cfg.vel0_std_m_s, size=3)

    # 4. Sensor Dispersions
    gyro_bias = C.GYRO_BIAS0 + np.radians(rng.normal(0.0, mc_cfg.gyro_bias_std_deg_s, size=3))
    arw_factor = 1.0 + rng.normal(0.0, mc_cfg.gyro_arw_std_pct / 100.0, size=3)
    gyro_arw = np.clip(C.GYRO_ARW * arw_factor, 1e-6, 1e-1)

    css_scale = 1.0 + rng.normal(0.0, mc_cfg.css_scale_std_pct / 100.0, size=3)
    css_bias = 0.001 + rng.normal(0.0, mc_cfg.css_bias_std, size=3)
    css_noise = np.clip(0.01 + rng.normal(0.0, mc_cfg.css_noise_std, size=3), 0.001, 0.05)

    return {
        "mass": mass,
        "inertia": inertia,
        "com_offset": com_offset,
        "thruster_pos": thruster_pos,
        "thruster_dir": thruster_dir,
        "thruster_tmax": thruster_tmax,
        "thruster_isp": thruster_isp,
        "omega0": omega0_rad_s,
        "sigma0": sigma0,
        "r0": r0,
        "v0": v0,
        "gyro_bias": gyro_bias,
        "gyro_arw": gyro_arw,
        "css_scale": css_scale,
        "css_bias": css_bias,
        "css_noise": css_noise,
    }


def run_single_mission(
    run_idx: int,
    seed: int,
    mc_cfg: MonteCarloConfig | None = None,
) -> dict[str, Any]:
    """Execute a single Monte Carlo mission run through all 3 maneuvers."""
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.fswAlgorithms import attTrackingError, mrpFeedback
    from Basilisk.simulation import coarseSunSensor, imuSensor, spacecraft, thrusterDynamicEffector
    from Basilisk.utilities import (
        RigidBodyKinematics as rk,
        SimulationBaseClass,
        macros,
        simHelpers,
        simIncludeGravBody,
        simIncludeThruster,
    )

    if mc_cfg is None:
        mc_cfg = MonteCarloConfig()

    disp = sample_dispersions(seed, mc_cfg)

    sim = SimulationBaseClass.SimBaseClass()
    simTaskName = "missionTask"
    simProcess = sim.CreateNewProcess("missionProcess")
    taskStep = mc_cfg.task_step
    simProcess.addTask(sim.CreateNewTask(simTaskName, macros.sec2nano(taskStep)))

    # Sun ephemeris
    sunEphemeris = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    sunEphemeris.ModelTag = "sunEphemeris"
    sim.AddModelToTask(simTaskName, sunEphemeris)

    # Spacecraft Hub
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = f"12uCubeSat_MC_{run_idx}"
    scObject.hub.mHub = disp["mass"]
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]
    scObject.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(disp["inertia"].flatten())
    scObject.hub.r_CN_NInit = disp["r0"].tolist()
    scObject.hub.v_CN_NInit = disp["v0"].tolist()
    scObject.hub.sigma_BNInit = disp["sigma0"].tolist()
    scObject.hub.omega_BN_BInit = disp["omega0"].tolist()

    # Earth Gravity
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)
    sim.AddModelToTask(simTaskName, scObject)

    # Gyro / IMU Sensor
    imu = imuSensor.ImuSensor()
    imu.ModelTag = "imu"
    imu.RNGSeed = seed
    imu.senRotBias = disp["gyro_bias"]
    imu.PMatrixGyro = np.diag(disp["gyro_arw"])
    imu.PMatrixAccel = np.diag([1e-4, 1e-4, 1e-4])
    imu.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    sim.AddModelToTask(simTaskName, imu)

    # Rate Observer / Navigation Filter
    class MissionNavigation(sysModel.SysModel):
        def __init__(self, observerGain: float, biasGain: float):
            super().__init__()
            self.observerGain = observerGain
            self.biasGain = biasGain
            self.scStateInMsg = messaging.SCStatesMsgReader()
            self.imuInMsg = messaging.IMUSensorMsgReader()
            self.attOutMsg = messaging.NavAttMsg()
            self.payload = messaging.NavAttMsgPayload()
            self.estimatedRate = None
            self.biasEstimate = np.zeros(3)
            self.previousTime = None

        def Reset(self, CurrentSimNanos):
            self.payload.sigma_BN = [0.0, 0.0, 0.0]
            self.payload.omega_BN_B = [0.0, 0.0, 0.0]
            self.estimatedRate = None
            self.biasEstimate = np.zeros(3)
            self.previousTime = None
            self.attOutMsg.write(self.payload)

        def UpdateState(self, CurrentSimNanos):
            state = self.scStateInMsg()
            imuData = self.imuInMsg()
            measuredRate = np.asarray(imuData.AngVelPlatform, dtype=float)
            if self.estimatedRate is None:
                self.estimatedRate = measuredRate.copy()
            else:
                dt = (CurrentSimNanos - self.previousTime) * 1e-9
                residual = measuredRate - self.estimatedRate - self.biasEstimate
                self.estimatedRate += self.observerGain * residual * dt
                self.biasEstimate += self.biasGain * residual * dt
            feedbackRate = measuredRate - self.biasEstimate
            self.previousTime = CurrentSimNanos
            self.payload.timeTag = CurrentSimNanos * 1e-9
            self.payload.sigma_BN = state.sigma_BN
            self.payload.omega_BN_B = feedbackRate.tolist()
            self.attOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

    navigation = MissionNavigation(C.ACS_OBSERVER_GAIN, C.ACS_OBSERVER_BIAS_GAIN)
    navigation.ModelTag = "navigation"
    navigation.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    navigation.imuInMsg.subscribeTo(imu.sensorOutMsg)
    sim.AddModelToTask(simTaskName, navigation)

    # Coarse Sun Sensors
    css_sensors = []
    css_loggers = []
    css_normals = [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
    ]
    for idx, normal in enumerate(css_normals):
        css = coarseSunSensor.CoarseSunSensor()
        css.ModelTag = f"css{idx+1}"
        css.nHat_B = normal
        css.fov = np.deg2rad(80.0)
        css.scaleFactor = float(disp["css_scale"][idx])
        css.senNoiseStd = float(disp["css_noise"][idx])
        css.senBias = float(disp["css_bias"][idx])
        css.stateInMsg.subscribeTo(scObject.scStateOutMsg)
        css.sunInMsg.subscribeTo(sunEphemeris.sunOutMsg)
        sim.AddModelToTask(simTaskName, css)
        css_sensors.append(css)
        rec = css.cssDataOutMsg.recorder(macros.sec2nano(mc_cfg.log_step))
        sim.AddModelToTask(simTaskName, rec)
        css_loggers.append(rec)

    # Thruster Dynamic Effector (12 variable jets)
    thrusterFactory = simIncludeThruster.thrusterFactory()
    for pos, dirn, tmax in zip(disp["thruster_pos"], disp["thruster_dir"], disp["thruster_tmax"]):
        thrusterFactory.create(
            "TEST_Thruster",
            pos.tolist(),
            dirn.tolist(),
            MaxThrust=float(tmax),
            cutoffFrequency=1000.0,
            useMinPulseTime=False,
        )
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    thrusterFactory.addToSpacecraft("rcsThrusters", thrusterSet, scObject)

    # 3-Phase Mission Guidance Module
    class MissionGuidance(sysModel.SysModel):
        def __init__(
            self,
            ephemeris: SunEphemeris,
            t_detumble: float,
            t_slew: float,
            sigma_init: np.ndarray,
            sigma_slew: tuple[float, float, float],
            roll_ref: tuple[float, float, float],
        ):
            super().__init__()
            self.ephemeris = ephemeris
            self.t_detumble = float(t_detumble)
            self.t_slew = float(t_slew)
            self.sigma_init = np.asarray(sigma_init, dtype=float)
            self.sigma_slew = np.asarray(sigma_slew, dtype=float)
            self.roll_ref = np.asarray(roll_ref, dtype=float)
            self.attRefOutMsg = messaging.AttRefMsg()
            self.payload = messaging.AttRefMsgPayload()
            self.phase = "detumble"

        def Reset(self, CurrentSimNanos):
            self.payload.sigma_RN = self.sigma_init.tolist()
            self.payload.omega_RN_N = [0.0, 0.0, 0.0]
            self.payload.domega_RN_N = [0.0, 0.0, 0.0]
            self.phase = "detumble"
            self.attRefOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

        def UpdateState(self, CurrentSimNanos):
            t = CurrentSimNanos * 1e-9
            if t < self.t_detumble:
                self.phase = "detumble"
                ref_sigma = self.sigma_init
                ref_omega = np.zeros(3)
            elif t < (self.t_detumble + self.t_slew):
                self.phase = "slew"
                ref_sigma = self.sigma_slew
                ref_omega = np.zeros(3)
            else:
                self.phase = "sunpoint"
                sun_dir = self.ephemeris.directionAt(t)
                ref_sigma = sun_pointing_sigma(sun_dir * C.VIZ_SUN_DISTANCE, self.roll_ref)
                ref_omega = np.array([0.0, 0.0, self.ephemeris.angularRate])
            self.payload.sigma_RN = ref_sigma.tolist()
            self.payload.omega_RN_N = ref_omega.tolist()
            self.payload.domega_RN_N = [0.0, 0.0, 0.0]
            self.attRefOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

    guidance = MissionGuidance(
        sunEphemeris,
        mc_cfg.t_detumble,
        mc_cfg.t_slew,
        disp["sigma0"],
        mc_cfg.sigma_slew,
        mc_cfg.sun_roll_ref,
    )
    guidance.ModelTag = "missionGuidance"
    sim.AddModelToTask(simTaskName, guidance)

    # Attitude Tracking Error
    attitudeError = attTrackingError.attTrackingError()
    attitudeError.ModelTag = "attitudeTrackingError"
    attitudeError.attNavInMsg.subscribeTo(navigation.attOutMsg)
    attitudeError.attRefInMsg.subscribeTo(guidance.attRefOutMsg)
    sim.AddModelToTask(simTaskName, attitudeError)

    # MRP Feedback Controller (uses nominal inertia: dispersions are unknown
    # to the flight software, so this is the fair robustness test)
    vehicleConfigMessage = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(
            ISCPntB_B=C.INERTIA.flatten().tolist(),
            CoM_B=[0.0, 0.0, 0.0],
        )
    )
    controller = mrpFeedback.mrpFeedback()
    controller.ModelTag = "mrpFeedback"
    controller.K = C.ACS_K
    controller.P = C.ACS_P
    controller.Ki = C.ACS_KI
    controller.integralLimit = C.ACS_INTEGRAL_LIMIT
    controller.guidInMsg.subscribeTo(attitudeError.attGuidOutMsg)
    controller.vehConfigInMsg.subscribeTo(vehicleConfigMessage)
    sim.AddModelToTask(simTaskName, controller)

    # Thruster Allocator & High-Fidelity Impulse Accounting
    class ThrusterAllocatorAccounting(sysModel.SysModel):
        def __init__(self, step: float, isp: float, t_detumble: float, t_slew: float):
            super().__init__()
            self.step = step
            self.isp = isp
            self.g0 = C.G0
            self.t_detumble = t_detumble
            self.t_slew = t_slew
            self.cmdTorqueInMsg = messaging.CmdTorqueBodyMsgReader()
            self.thrOnTimeOutMsg = messaging.THRArrayOnTimeCmdMsg()
            self.thrOnTimePayload = messaging.THRArrayOnTimeCmdMsgPayload()

            # Accumulated metrics
            self.n_thr = len(disp["thruster_pos"])
            self.total_on_time = np.zeros(self.n_thr)
            self.phase_impulse = {"detumble": 0.0, "slew": 0.0, "sunpoint": 0.0}
            self.phase_propellant = {"detumble": 0.0, "slew": 0.0, "sunpoint": 0.0}

        def Reset(self, CurrentSimNanos):
            self.thrOnTimePayload.OnTimeRequest = np.zeros(self.n_thr).tolist()
            self.thrOnTimeOutMsg.write(self.thrOnTimePayload)
            self.total_on_time.fill(0.0)
            self.phase_impulse = {"detumble": 0.0, "slew": 0.0, "sunpoint": 0.0}
            self.phase_propellant = {"detumble": 0.0, "slew": 0.0, "sunpoint": 0.0}

        def UpdateState(self, CurrentSimNanos):
            t = CurrentSimNanos * 1e-9
            torque = np.asarray(self.cmdTorqueInMsg().torqueRequestBody, dtype=float)
            throttle = allocation_pinv(np.zeros(3), C.THRUSTER_CONTROL_SCALE * torque)
            on_times = np.clip(throttle, 0.0, 1.0) * self.step
            self.thrOnTimePayload.OnTimeRequest = on_times.tolist()
            self.thrOnTimeOutMsg.write(self.thrOnTimePayload, CurrentSimNanos, self.moduleID)

            # Accumulate on-times and impulse per tick
            self.total_on_time += on_times
            tick_impulse = np.sum(on_times * disp["thruster_tmax"])
            tick_fuel = tick_impulse / (self.isp * self.g0)

            if t < self.t_detumble:
                self.phase_impulse["detumble"] += tick_impulse
                self.phase_propellant["detumble"] += tick_fuel
            elif t < (self.t_detumble + self.t_slew):
                self.phase_impulse["slew"] += tick_impulse
                self.phase_propellant["slew"] += tick_fuel
            else:
                self.phase_impulse["sunpoint"] += tick_impulse
                self.phase_propellant["sunpoint"] += tick_fuel

    allocator = ThrusterAllocatorAccounting(taskStep, disp["thruster_isp"], mc_cfg.t_detumble, mc_cfg.t_slew)
    allocator.ModelTag = "thrusterAllocator"
    allocator.cmdTorqueInMsg.subscribeTo(controller.cmdTorqueOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(allocator.thrOnTimeOutMsg)
    sim.AddModelToTask(simTaskName, allocator)
    sim.AddModelToTask(simTaskName, thrusterSet, 2)

    # Loggers (sampled at log_step)
    logNanos = macros.sec2nano(mc_cfg.log_step)
    scStateLogger = scObject.scStateOutMsg.recorder(logNanos)
    attErrLogger = attitudeError.attGuidOutMsg.recorder(logNanos)
    thrOnTimeLogger = allocator.thrOnTimeOutMsg.recorder(logNanos)
    cmdTorqueLogger = controller.cmdTorqueOutMsg.recorder()
    sim.AddModelToTask(simTaskName, scStateLogger)
    sim.AddModelToTask(simTaskName, attErrLogger)
    sim.AddModelToTask(simTaskName, thrOnTimeLogger)
    sim.AddModelToTask(simTaskName, cmdTorqueLogger)

    # Execute simulation
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(mc_cfg.t_total))
    sim.ExecuteSimulation()

    # Post-process recorded timeseries
    times = scStateLogger.times() * macros.NANO2SEC
    omega_hist = np.asarray(scStateLogger.omega_BN_B, dtype=float)
    omega_norm_deg_s = np.degrees(np.linalg.norm(omega_hist, axis=1))

    sigma_hist = np.asarray(scStateLogger.sigma_BN, dtype=float)
    sigma_err_hist = np.asarray(attErrLogger.sigma_BR, dtype=float)
    sigma_err_norm = np.linalg.norm(sigma_err_hist, axis=1)

    # Calculate Sun pointing angle over time
    sun_angles_deg = np.zeros(len(times))
    boresight_axis = np.asarray(mc_cfg.sun_pointing_axis_b, dtype=float)
    for k, t_val in enumerate(times):
        sun_dir_N = sunEphemeris.directionAt(t_val)
        dcm_BN = rk.MRP2C(sigma_hist[k])
        # Body boresight in inertial frame is dcm_BN.T @ boresight_axis
        # Or inertial Sun in body frame is dcm_BN @ sun_dir_N
        sun_dir_B = dcm_BN @ sun_dir_N
        cos_ang = np.clip(np.dot(boresight_axis, sun_dir_B), -1.0, 1.0)
        sun_angles_deg[k] = np.degrees(np.arccos(cos_ang))

    # Evaluate KPIs
    # 1. Detumble settling time (when rate norm drops below threshold and stays below)
    detumble_mask = (times <= mc_cfg.t_detumble)
    detumble_times = times[detumble_mask]
    detumble_rates = omega_norm_deg_s[detumble_mask]
    above_thresh_indices = np.where(detumble_rates >= mc_cfg.rate_settle_thresh_deg_s)[0]
    if len(above_thresh_indices) == 0:
        detumble_settle_time = float(detumble_times[0])
    elif above_thresh_indices[-1] == len(detumble_rates) - 1:
        detumble_settle_time = float(mc_cfg.t_detumble)
    else:
        detumble_settle_time = float(detumble_times[above_thresh_indices[-1] + 1])

    # 2. Slew phase pointing error and settling time. Use strict < on the upper
    # bound: guidance switches reference at exactly t_detumble+t_slew, so the
    # sample at the boundary already belongs to sun-pointing (seen as a 0.97
    # MRP jump in the log) and must not score the slew.
    slew_mask = (times > mc_cfg.t_detumble) & (times < mc_cfg.t_detumble + mc_cfg.t_slew)
    slew_times = times[slew_mask]
    slew_errs = sigma_err_norm[slew_mask]
    # Exact MRP-to-angle conversion: angle = 4 * atan(||sigma_err||)
    slew_angle_err_deg = np.degrees(4.0 * np.arctan(slew_errs))
    above_slew_indices = np.where(slew_angle_err_deg >= mc_cfg.slew_settle_thresh_deg)[0]
    if len(above_slew_indices) == 0:
        slew_settle_time = 0.0
    elif above_slew_indices[-1] == len(slew_angle_err_deg) - 1:
        slew_settle_time = float(mc_cfg.t_slew)
    else:
        slew_settle_time = float(slew_times[above_slew_indices[-1] + 1] - mc_cfg.t_detumble)
    final_slew_err_deg = float(slew_angle_err_deg[-1]) if len(slew_angle_err_deg) > 0 else 0.0

    # 3. Sun pointing phase performance
    sunpoint_mask = (times > (mc_cfg.t_detumble + mc_cfg.t_slew))
    sun_angles_sunpoint = sun_angles_deg[sunpoint_mask]
    steady_state_mask = (times >= (mc_cfg.t_total - 40.0))
    steady_state_angles = sun_angles_deg[steady_state_mask]
    final_sun_angle_deg = float(sun_angles_deg[-1])
    steady_state_max_angle_deg = float(np.max(steady_state_angles)) if len(steady_state_angles) > 0 else final_sun_angle_deg
    inside_cone = bool(steady_state_max_angle_deg <= mc_cfg.sun_cone_angle_deg)

    # Propellant and Impulse
    detumble_fuel_g = allocator.phase_propellant["detumble"] * 1000.0
    slew_fuel_g = allocator.phase_propellant["slew"] * 1000.0
    sunpoint_fuel_g = allocator.phase_propellant["sunpoint"] * 1000.0
    total_fuel_g = detumble_fuel_g + slew_fuel_g + sunpoint_fuel_g
    total_impulse_ns = (
        allocator.phase_impulse["detumble"]
        + allocator.phase_impulse["slew"]
        + allocator.phase_impulse["sunpoint"]
    )
    thruster_duty_cycles = allocator.total_on_time / mc_cfg.t_total

    # Overall mission success: rate nulled after detumble and final
    # sun-pointing inside the keep-in cone. The intermediate slew target is
    # reported (final_slew_error_deg / slew_settle_time_s) but does not gate
    # success, since the sun-pointing phase recovers residual slew error.
    success = bool(
        (detumble_rates[-1] < mc_cfg.rate_settle_thresh_deg_s * 2.0)
        and inside_cone
    )

    return {
        "run_idx": run_idx,
        "seed": seed,
        "success": success,
        "dispersions": {
            "mass_kg": disp["mass"],
            "inertia_diag": [disp["inertia"][0, 0], disp["inertia"][1, 1], disp["inertia"][2, 2]],
            "com_offset_mm": (disp["com_offset"] * 1000.0).tolist(),
            "thruster_isp_s": disp["thruster_isp"],
            "omega0_deg_s": np.degrees(disp["omega0"]).tolist(),
            "sigma0": disp["sigma0"].tolist(),
        },
        "kpis": {
            "detumble_settle_time_s": detumble_settle_time,
            "final_detumble_rate_deg_s": float(detumble_rates[-1]),
            "slew_settle_time_s": slew_settle_time,
            "final_slew_error_deg": final_slew_err_deg,
            "final_sun_angle_deg": final_sun_angle_deg,
            "steady_state_max_sun_angle_deg": steady_state_max_angle_deg,
            "inside_sun_cone": inside_cone,
            "detumble_propellant_g": detumble_fuel_g,
            "slew_propellant_g": slew_fuel_g,
            "sunpoint_propellant_g": sunpoint_fuel_g,
            "total_propellant_g": total_fuel_g,
            "total_impulse_ns": total_impulse_ns,
            "delta_v_m_s": float(total_impulse_ns / disp["mass"]),
            "thruster_duty_cycles": thruster_duty_cycles.tolist(),
        },
        "timeseries": {
            "time": times,
            "omega_norm_deg_s": omega_norm_deg_s,
            "sigma_err_norm": sigma_err_norm,
            "sun_angle_deg": sun_angles_deg,
        },
        "signals": {
            # Per-tick (task_step) series for model-parity diagnostics.
            "torque_demand": np.asarray(cmdTorqueLogger.torqueRequestBody, dtype=float),
            "on_time": np.asarray(thrOnTimeLogger.OnTimeRequest, dtype=float)[:, :12],
        },
    }


def compute_monte_carlo_statistics(runs_data: list[dict[str, Any]]) -> dict[str, Any]:
    """Compute mean, std, median, 95th and 99th percentiles (3-sigma) across runs."""
    kpi_keys = [
        "detumble_settle_time_s",
        "final_detumble_rate_deg_s",
        "slew_settle_time_s",
        "final_slew_error_deg",
        "final_sun_angle_deg",
        "steady_state_max_sun_angle_deg",
        "detumble_propellant_g",
        "slew_propellant_g",
        "sunpoint_propellant_g",
        "total_propellant_g",
        "total_impulse_ns",
        "delta_v_m_s",
    ]

    stats: dict[str, Any] = {}
    n_runs = len(runs_data)
    successes = sum(1 for r in runs_data if r["success"])
    cone_compliances = sum(1 for r in runs_data if r["kpis"]["inside_sun_cone"])

    stats["summary"] = {
        "n_runs": n_runs,
        "success_rate_pct": float(successes / n_runs * 100.0),
        "sun_cone_compliance_pct": float(cone_compliances / n_runs * 100.0),
    }

    stats["kpis"] = {}
    for key in kpi_keys:
        vals = np.array([r["kpis"][key] for r in runs_data], dtype=float)
        stats["kpis"][key] = {
            "mean": float(np.mean(vals)),
            "std": float(np.std(vals)),
            "median": float(np.median(vals)),
            "min": float(np.min(vals)),
            "max": float(np.max(vals)),
            "p95": float(np.percentile(vals, 95.0)),
            "p99": float(np.percentile(vals, 99.0)),
        }

    # Thruster duty cycles across runs
    duty_cycles = np.array([r["kpis"]["thruster_duty_cycles"] for r in runs_data], dtype=float)
    stats["thruster_duty_cycles"] = {
        "mean_pct": (np.mean(duty_cycles, axis=0) * 100.0).tolist(),
        "std_pct": (np.std(duty_cycles, axis=0) * 100.0).tolist(),
        "max_pct": (np.max(duty_cycles, axis=0) * 100.0).tolist(),
    }

    return stats


def _run_single_star(args: tuple) -> dict[str, Any]:
    """Picklable worker for process-pool fan-out: (run_idx, seed, mc_cfg)."""
    run_idx, seed, mc_cfg = args
    return run_single_mission(run_idx=run_idx, seed=seed, mc_cfg=mc_cfg)


def run_monte_carlo(
    num_runs: int = 30,
    base_seed: int = 100,
    mc_cfg: MonteCarloConfig | None = None,
    verbose: bool = True,
    n_jobs: int = 1,
) -> dict[str, Any]:
    """Run full Monte Carlo simulation campaign across num_runs.

    ``n_jobs`` fans out over processes (Basilisk sims are independent and
    process-safe; results are bit-identical to serial, ordered by run_idx).
    """
    if mc_cfg is None:
        mc_cfg = MonteCarloConfig()

    if verbose:
        print("=" * 72)
        print(f"Starting 12U CubeSat Monte Carlo Simulation ({num_runs} runs, jobs={n_jobs})")
        print(f"Mission: Detumble ({mc_cfg.t_detumble:.0f}s) -> Slew ({mc_cfg.t_slew:.0f}s) -> Sun-Point ({mc_cfg.t_sunpoint:.0f}s)")
        print(f"Total mission duration: {mc_cfg.t_total:.0f}s | Actuators: 12 RCS Thrusters Only")
        print("=" * 72)

    t_start = time.time()
    runs_data: list[dict[str, Any]] = []
    if int(n_jobs) <= 1:
        for i in range(num_runs):
            seed = base_seed + i
            t0 = time.time()
            res = run_single_mission(run_idx=i + 1, seed=seed, mc_cfg=mc_cfg)
            runs_data.append(res)
            dt = time.time() - t0
            if verbose:
                k = res["kpis"]
                status = "PASS" if res["success"] else "FAIL"
                print(
                    f"[Run {i+1:2d}/{num_runs}] [{status}] "
                    f"Fuel={k['total_propellant_g']:.2f}g | "
                    f"SunAngle={k['final_sun_angle_deg']:.2f} deg | "
                    f"Detumble={k['detumble_settle_time_s']:.1f}s | "
                    f"Time={dt:.2f}s"
                )
    else:
        import concurrent.futures
        args = [(i + 1, base_seed + i, mc_cfg) for i in range(num_runs)]
        with concurrent.futures.ProcessPoolExecutor(max_workers=int(n_jobs)) as ex:
            for res in ex.map(_run_single_star, args):
                runs_data.append(res)
                if verbose:
                    k = res["kpis"]
                    status = "PASS" if res["success"] else "FAIL"
                    print(
                        f"[Run {res['run_idx']:2d}/{num_runs}] [{status}] "
                        f"Fuel={k['total_propellant_g']:.2f}g | "
                        f"SunAngle={k['final_sun_angle_deg']:.2f} deg | "
                        f"Detumble={k['detumble_settle_time_s']:.1f}s"
                    )
        runs_data.sort(key=lambda r: r["run_idx"])

    wall_time = time.time() - t_start
    stats = compute_monte_carlo_statistics(runs_data)

    if verbose:
        print("-" * 72)
        print(f"Completed {num_runs} runs in {wall_time:.2f}s ({wall_time/num_runs:.2f}s/run)")
        print(f"Mission Success Rate: {stats['summary']['success_rate_pct']:.1f}%")
        print(f"Sun Keep-In Cone Compliance: {stats['summary']['sun_cone_compliance_pct']:.1f}%")
        prop = stats["kpis"]["total_propellant_g"]
        print(f"Total Propellant: Mean={prop['mean']:.2f}g, 3-sigma (p99)={prop['p99']:.2f}g, Max={prop['max']:.2f}g")
        sun = stats["kpis"]["final_sun_angle_deg"]
        print(f"Final Sun Pointing Error: Mean={sun['mean']:.2f} deg, p99={sun['p99']:.2f} deg (limit: {mc_cfg.sun_cone_angle_deg} deg)")
        print("=" * 72)

    return {
        "config": asdict(mc_cfg),
        "stats": stats,
        "runs": runs_data,
        "wall_time_s": wall_time,
    }


def plot_monte_carlo(
    mc_results: dict[str, Any],
    output_dir: Path | str = "results_mc",
) -> list[str]:
    """Generate publication-quality diagnostic plots for the Monte Carlo mission."""
    import matplotlib.pyplot as plt

    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    generated_plots = []

    cfg = mc_results["config"]
    t_detumble = cfg["t_detumble"]
    t_slew = cfg["t_slew"]
    t_sunpoint = cfg["t_sunpoint"]
    t_total = t_detumble + t_slew + t_sunpoint
    cone_angle = cfg["sun_cone_angle_deg"]

    runs = mc_results["runs"]
    times = runs[0]["timeseries"]["time"]

    # -------------------------------------------------------------
    # Plot 1: Mission Trajectories (4 panels)
    # -------------------------------------------------------------
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    rates = np.array([r["timeseries"]["omega_norm_deg_s"] for r in runs])
    sigma_errs = np.array([r["timeseries"]["sigma_err_norm"] for r in runs])
    sun_angles = np.array([r["timeseries"]["sun_angle_deg"] for r in runs])

    # 1. Angular rate norm
    for i in range(len(runs)):
        axes[0].plot(times, rates[i], color="tab:blue", alpha=0.25, lw=0.8)
    axes[0].plot(times, np.mean(rates, axis=0), color="darkblue", lw=2.0, label="Mean Rate")
    axes[0].axhline(cfg["rate_settle_thresh_deg_s"], color="red", linestyle="--", lw=1.2, label=f"Threshold ({cfg['rate_settle_thresh_deg_s']} deg/s)")
    axes[0].set_ylabel("Body Rate Norm [deg/s]", fontsize=10)
    axes[0].set_title(f"12U CubeSat Whole Mission Monte Carlo ({len(runs)} Runs) — 3 Thruster Maneuvers", fontsize=12, fontweight="bold")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=9)

    # 2. Attitude error (MRP norm)
    for i in range(len(runs)):
        axes[1].plot(times, sigma_errs[i], color="tab:purple", alpha=0.25, lw=0.8)
    axes[1].plot(times, np.mean(sigma_errs, axis=0), color="indigo", lw=2.0, label="Mean MRP Error")
    axes[1].set_ylabel(r"Attitude Error $\|\Delta\sigma\|$", fontsize=10)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right", fontsize=9)

    # 3. Sun angle & keep-in cone
    axes[2].axhspan(0.0, cone_angle, color="lightgreen", alpha=0.3, label=f"Keep-In Cone (< {cone_angle}°)")
    axes[2].axhline(cone_angle, color="green", linestyle="--", lw=1.5)
    for i in range(len(runs)):
        axes[2].plot(times, sun_angles[i], color="tab:orange", alpha=0.25, lw=0.8)
    axes[2].plot(times, np.mean(sun_angles, axis=0), color="darkred", lw=2.0, label="Mean Sun Angle")
    axes[2].set_ylabel("Sun Angle [deg]", fontsize=10)
    axes[2].set_ylim(0.0, 180.0)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right", fontsize=9)

    # 4. Cumulative Propellant Consumption
    # Reconstruct approximate propellant histories from on-times and final values
    total_fuels = np.array([r["kpis"]["total_propellant_g"] for r in runs])
    for i, r in enumerate(runs):
        # Piecewise cumulative propellant approximation
        p_det = r["kpis"]["detumble_propellant_g"]
        p_slew = r["kpis"]["slew_propellant_g"]
        p_sun = r["kpis"]["sunpoint_propellant_g"]
        fuel_curve = np.zeros_like(times)
        m1 = times <= t_detumble
        m2 = (times > t_detumble) & (times <= t_detumble + t_slew)
        m3 = times > (t_detumble + t_slew)
        fuel_curve[m1] = p_det * (times[m1] / t_detumble)
        fuel_curve[m2] = p_det + p_slew * ((times[m2] - t_detumble) / t_slew)
        fuel_curve[m3] = p_det + p_slew + p_sun * ((times[m3] - t_detumble - t_slew) / t_sunpoint)
        axes[3].plot(times, fuel_curve, color="tab:cyan", alpha=0.25, lw=0.8)
    axes[3].set_ylabel("Propellant [g]", fontsize=10)
    axes[3].set_xlabel("Mission Time [s]", fontsize=11)
    axes[3].grid(True, alpha=0.3)

    # Vertical phase dividers and labels across all subplots
    for ax in axes:
        ax.axvline(t_detumble, color="black", linestyle=":", lw=1.5)
        ax.axvline(t_detumble + t_slew, color="black", linestyle=":", lw=1.5)

    axes[0].text(t_detumble / 2, axes[0].get_ylim()[1] * 0.85, "PHASE 1:\nDETUMBLE", ha="center", fontsize=9, fontweight="bold")
    axes[0].text(t_detumble + t_slew / 2, axes[0].get_ylim()[1] * 0.85, "PHASE 2:\nSLEW", ha="center", fontsize=9, fontweight="bold")
    axes[0].text(t_detumble + t_slew + t_sunpoint / 2, axes[0].get_ylim()[1] * 0.85, "PHASE 3:\nSUN-POINTING", ha="center", fontsize=9, fontweight="bold")

    fig.tight_layout()
    traj_path = str(out_path / "mc_mission_trajectories.png")
    fig.savefig(traj_path, dpi=200)
    plt.close(fig)
    generated_plots.append(traj_path)

    # -------------------------------------------------------------
    # Plot 2: KPI Distributions & Performance Histograms
    # -------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(11, 9))

    # A: Total Propellant Histogram
    ax = axes[0, 0]
    ax.hist(total_fuels, bins=12, color="teal", edgecolor="black", alpha=0.75)
    mean_fuel = np.mean(total_fuels)
    p99_fuel = np.percentile(total_fuels, 99.0)
    ax.axvline(mean_fuel, color="red", linestyle="-", lw=1.8, label=f"Mean: {mean_fuel:.2f} g")
    ax.axvline(p99_fuel, color="darkorange", linestyle="--", lw=1.8, label=f"99% (3-σ): {p99_fuel:.2f} g")
    ax.set_title("Total Mission Propellant Consumption", fontweight="bold")
    ax.set_xlabel("Propellant [g]")
    ax.set_ylabel("Count")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # B: Final Sun Pointing Error
    ax = axes[0, 1]
    final_sun = np.array([r["kpis"]["final_sun_angle_deg"] for r in runs])
    ax.hist(final_sun, bins=12, color="gold", edgecolor="black", alpha=0.75)
    ax.axvline(cone_angle, color="red", linestyle="--", lw=2.0, label=f"15° Cone Limit")
    mean_sun = np.mean(final_sun)
    ax.axvline(mean_sun, color="blue", linestyle="-", lw=1.8, label=f"Mean: {mean_sun:.2f}°")
    ax.set_title("Steady-State Sun Pointing Angle", fontweight="bold")
    ax.set_xlabel("Sun Angle [deg]")
    ax.set_ylabel("Count")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # C: Detumble Settling Time
    ax = axes[1, 0]
    detumble_times_all = np.array([r["kpis"]["detumble_settle_time_s"] for r in runs])
    ax.hist(detumble_times_all, bins=12, color="cornflowerblue", edgecolor="black", alpha=0.75)
    mean_det = np.mean(detumble_times_all)
    ax.axvline(mean_det, color="red", linestyle="-", lw=1.8, label=f"Mean: {mean_det:.1f} s")
    ax.set_title("Detumble Rate-Nulling Settling Time", fontweight="bold")
    ax.set_xlabel("Settling Time [s]")
    ax.set_ylabel("Count")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # D: Propellant Consumption per Phase
    ax = axes[1, 1]
    p_det_all = [r["kpis"]["detumble_propellant_g"] for r in runs]
    p_slew_all = [r["kpis"]["slew_propellant_g"] for r in runs]
    p_sun_all = [r["kpis"]["sunpoint_propellant_g"] for r in runs]
    box_data = [p_det_all, p_slew_all, p_sun_all]
    ax.boxplot(box_data, labels=["Detumble", "Slew", "Sun-Pointing"], patch_artist=True,
               boxprops=dict(facecolor="lightblue", color="blue"),
               medianprops=dict(color="red", lw=1.5))
    ax.set_title("Propellant Consumption by Phase", fontweight="bold")
    ax.set_ylabel("Propellant [g]")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    dist_path = str(out_path / "mc_performance_histograms.png")
    fig.savefig(dist_path, dpi=200)
    plt.close(fig)
    generated_plots.append(dist_path)

    # -------------------------------------------------------------
    # Plot 3: 12-Thruster Utilization & Duty Cycle Distribution
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 5))
    duty_pct = np.array([r["kpis"]["thruster_duty_cycles"] for r in runs]) * 100.0
    mean_duty = np.mean(duty_pct, axis=0)
    std_duty = np.std(duty_pct, axis=0)
    thruster_ids = [f"T{i+1}" for i in range(12)]

    bars = ax.bar(thruster_ids, mean_duty, yerr=std_duty, capsize=4, color="coral", edgecolor="black", alpha=0.85)
    ax.set_ylabel("Mean Duty Cycle [%]", fontsize=11)
    ax.set_xlabel("RCS Jet Identifier", fontsize=11)
    ax.set_title("12U CubeSat RCS Thruster Workload Distribution across Monte Carlo Runs", fontsize=12, fontweight="bold")
    ax.grid(True, alpha=0.3, axis="y")
    ax.set_ylim(0.0, float(np.max(mean_duty + std_duty)) * 1.35 + 0.05)

    for bar, val in zip(bars, mean_duty):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2.0, height + 0.02, f"{val:.2f}%", ha="center", va="bottom", fontsize=8)

    fig.tight_layout()
    thr_path = str(out_path / "mc_thruster_load.png")
    fig.savefig(thr_path, dpi=200)
    plt.close(fig)
    generated_plots.append(thr_path)

    return generated_plots


def save_monte_carlo_results(
    mc_results: dict[str, Any],
    output_dir: Path | str = "results_mc",
) -> None:
    """Save Monte Carlo statistics to JSON and NPZ files."""
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    # Save summary JSON
    summary_data = {
        "config": mc_results["config"],
        "stats": mc_results["stats"],
        "wall_time_s": mc_results["wall_time_s"],
    }
    with open(out_path / "mc_summary.json", "w") as f:
        json.dump(summary_data, f, indent=2)

    # Save raw arrays to NPZ
    runs = mc_results["runs"]
    times = runs[0]["timeseries"]["time"]
    rates = np.array([r["timeseries"]["omega_norm_deg_s"] for r in runs])
    sigma_errs = np.array([r["timeseries"]["sigma_err_norm"] for r in runs])
    sun_angles = np.array([r["timeseries"]["sun_angle_deg"] for r in runs])
    total_fuels = np.array([r["kpis"]["total_propellant_g"] for r in runs])
    duty_cycles = np.array([r["kpis"]["thruster_duty_cycles"] for r in runs])

    np.savez_compressed(
        out_path / "mc_results.npz",
        time=times,
        rates=rates,
        sigma_errs=sigma_errs,
        sun_angles=sun_angles,
        total_fuels=total_fuels,
        duty_cycles=duty_cycles,
    )
