"""Basilisk thruster pipeline with the NATIVE C++ Heyoka dynamics module.

Mirrors heyoka_dynamics.run_heyoka_pipeline exactly, except the dynamics is
a true compiled Basilisk SysModel (nativerw.HeyokaMCDynamics) instead of a
Python SysModel. All other modules (ephemeris, IMU, navigation, reference,
controller, allocator, CSS) are identical stock/Python glue.

Requires the native extension (see ../native_rw): imported lazily so this
module stays importable without it.

ISC License - see native_rw/LICENSE.
"""
from __future__ import annotations

import numpy as np

try:
    from nativerw import HeyokaMCDynamics as _SwigMCBase

    _MC_NATIVE = True
except Exception:
    _SwigMCBase = object  # type: ignore[assignment]
    _MC_NATIVE = False


def mc_available() -> bool:
    return bool(_MC_NATIVE)


class NativeMCSpacecraft(_SwigMCBase):
    """Drop-in ``HeyokaSpacecraft`` replacement with native C++ dynamics.

    A Python subclass of the native SWIG module, so it *is* a Basilisk
    ``SysModel`` to the scheduler. Hub/gravity scripting API is shared with
    ``cpp_native.NativeSpacecraft`` (no wheel fields here: the MC demo has
    no reaction wheels — never assign ``hub.wheelSpeedsInit``).
    """

    def __init__(self):
        if not _MC_NATIVE:
            raise RuntimeError("nativerw extension unavailable (see native_rw/)")
        super().__init__()
        from .cpp_native import NativeSpacecraft as _NS

        object.__setattr__(self, "hub", _NS._Hub(self))
        object.__setattr__(self, "gravField", _NS._GravField(self))

    def __setattr__(self, name, value):
        if name in ("hub", "gravField"):
            object.__setattr__(self, name, value)
        else:
            super().__setattr__(name, value)  # SWIG property routing

    @property
    def model(self):
        """Back-compat alias: this object already is the SysModel."""
        return self


def run_native_mc_pipeline(
    duration: float = 600.0,
    seed: int = 1,
    maneuver: str = "detumble",
    rate_feedback: str = "observer",
    task_step: float = 0.01,
) -> dict:
    """Full thruster pipeline with native C++ Heyoka dynamics.

    Same signature and result schema as
    ``heyoka_dynamics.run_heyoka_pipeline``.
    """
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback
    from Basilisk.simulation import coarseSunSensor, imuSensor
    from Basilisk.utilities import (
        SimulationBaseClass,
        macros,
        simIncludeGravBody,
    )

    from . import config as C
    from .actuators import allocation_pinv
    from .sun_model import SunEphemeris, SunPointingReference, sun_pointing_sigma

    simulation = SimulationBaseClass.SimBaseClass()
    simulationTaskName = "sensorTask"
    simulationProcess = simulation.CreateNewProcess("sensorProcess")
    simulationProcess.addTask(simulation.CreateNewTask(simulationTaskName, macros.sec2nano(task_step)))

    sunEphemeris = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    sunEphemeris.ModelTag = "sunEphemeris"
    simulation.AddModelToTask(simulationTaskName, sunEphemeris)

    if maneuver == "detumble":
        initial_sigma = np.array([0.1, -0.2, 0.3])
        target_sigma = initial_sigma
        initial_omega = C.OMEGA0_BODY
    elif maneuver == "slew":
        initial_sigma = np.zeros(3)
        target_sigma = C.ACS_SLEW_SIGMA
        initial_omega = C.OMEGA0_BODY
    elif maneuver == "sunpoint":
        initial_sigma = np.array([0.1, -0.2, 0.3])
        target_sigma = sun_pointing_sigma(
            np.array([0.0, C.VIZ_SUN_DISTANCE, 0.0]), C.SUN_ROLL_REFERENCE_N
        )
        initial_omega = np.zeros(3)
    else:
        raise ValueError(f"unknown maneuver {maneuver!r}")

    # Native dynamics (true C++ SysModel: zero Python per tick)
    scObject = NativeMCSpacecraft()
    scObject.ModelTag = C.VIZ_SPACECRAFT_NAME
    scObject.hub.mHub = C.MASS
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]
    I6 = C.INERTIA.flatten().tolist()
    scObject.hub.IHubPntBc_B = [I6[0:3], I6[3:6], I6[6:9]]
    scObject.hub.r_CN_NInit = C.R0_ECI
    scObject.hub.v_CN_NInit = C.V0_ECI
    scObject.hub.sigma_BNInit = initial_sigma
    scObject.hub.omega_BN_BInit = initial_omega
    MF = (C.THRUSTER_DIR * C.THRUSTER_TMAX[:, None]).T.copy()
    MT = (np.cross(C.THRUSTER_POS, C.THRUSTER_DIR) * C.THRUSTER_TMAX[:, None]).T.copy()
    scObject.setForceMatrix([float(v) for v in np.asarray(MF, float).ravel()])
    scObject.setTorqueMatrix([float(v) for v in np.asarray(MT, float).ravel()])
    scObject.setQuantize(True)
    scObject.setUseGravityGradient(bool(C.USE_GRAVITY_GRADIENT))
    scObject.setTol(float(C.HEYOKA_TOL))
    scObject.setMu(float(C.MU_EARTH))
    simulation.AddModelToTask(simulationTaskName, scObject)

    # Gravity (Vizard registration only — Kepler is inside Heyoka)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    sunBody = gravFactory.createSun()
    sunBody.planetBodyInMsg.subscribeTo(sunEphemeris.sunOutMsg)

    stateLogger = scObject.scStateOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, stateLogger)

    imu = imuSensor.ImuSensor()
    imu.ModelTag = "imu"
    imu.RNGSeed = seed
    imu.senRotBias = C.GYRO_BIAS0
    imu.PMatrixGyro = np.diag(C.GYRO_ARW)
    imu.PMatrixAccel = np.diag([1e-4, 1e-4, 1e-4])
    imu.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    simulation.AddModelToTask(simulationTaskName, imu)

    class NoisyImuNavigation(sysModel.SysModel):
        def __init__(self, feedbackMode: str, observerGain: float, biasGain: float):
            super().__init__()
            self.feedbackMode = feedbackMode
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
            if self.feedbackMode == "truth":
                feedbackRate = np.asarray(state.omega_BN_B, dtype=float)
            elif self.feedbackMode == "raw":
                feedbackRate = measuredRate
            else:
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
            self.payload.omega_BN_B = feedbackRate
            self.attOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

    navigation = NoisyImuNavigation(
        rate_feedback,
        C.ACS_OBSERVER_GAIN,
        C.ACS_OBSERVER_BIAS_GAIN,
    )
    navigation.ModelTag = "noisyImuNavigation"
    navigation.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    navigation.imuInMsg.subscribeTo(imu.sensorOutMsg)
    simulation.AddModelToTask(simulationTaskName, navigation)
    navigationLogger = navigation.attOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, navigationLogger)

    if C.ACS_SUN_POINTING:
        reference = SunPointingReference(sunEphemeris, C.SUN_ROLL_REFERENCE_N)
        reference.ModelTag = "sunPointingReference"
    else:
        reference = inertial3D.inertial3D()
        reference.ModelTag = "attitudeReference"
        reference.sigma_R0N = target_sigma
    simulation.AddModelToTask(simulationTaskName, reference)

    attitudeError = attTrackingError.attTrackingError()
    attitudeError.ModelTag = "attitudeTrackingError"
    attitudeError.attNavInMsg.subscribeTo(navigation.attOutMsg)
    attitudeError.attRefInMsg.subscribeTo(reference.attRefOutMsg)
    simulation.AddModelToTask(simulationTaskName, attitudeError)

    controller = mrpFeedback.mrpFeedback()
    controller.ModelTag = "mrpFeedback"
    controller.K = C.ACS_K
    controller.P = C.ACS_P
    controller.Ki = C.ACS_KI
    controller.integralLimit = C.ACS_INTEGRAL_LIMIT
    controller.guidInMsg.subscribeTo(attitudeError.attGuidOutMsg)
    vehicleConfigMessage = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(
            ISCPntB_B=C.INERTIA.flatten().tolist(),
            CoM_B=[0.0, 0.0, 0.0],
        )
    )
    controller.vehConfigInMsg.subscribeTo(vehicleConfigMessage)
    simulation.AddModelToTask(simulationTaskName, controller)

    class ThrusterAllocator(sysModel.SysModel):
        def __init__(self, controlPeriod: float):
            super().__init__()
            self.controlPeriod = controlPeriod
            self.cmdTorqueInMsg = messaging.CmdTorqueBodyMsgReader()
            self.thrOnTimeOutMsg = messaging.THRArrayOnTimeCmdMsg()
            self.thrOnTimePayload = messaging.THRArrayOnTimeCmdMsgPayload()

        def Reset(self, CurrentSimNanos):
            self.thrOnTimePayload.OnTimeRequest = np.zeros(len(C.THRUSTER_POS)).tolist()
            self.thrOnTimeOutMsg.write(self.thrOnTimePayload)

        def UpdateState(self, CurrentSimNanos):
            torque = np.asarray(self.cmdTorqueInMsg().torqueRequestBody, dtype=float)
            throttle = allocation_pinv(np.zeros(3), C.THRUSTER_CONTROL_SCALE * torque)
            self.thrOnTimePayload.OnTimeRequest = (
                np.clip(throttle, 0.0, 1.0) * self.controlPeriod
            ).tolist()
            self.thrOnTimeOutMsg.write(self.thrOnTimePayload, CurrentSimNanos, self.moduleID)

    allocator = ThrusterAllocator(task_step)
    allocator.ModelTag = "thrusterAllocator"
    allocator.cmdTorqueInMsg.subscribeTo(controller.cmdTorqueOutMsg)
    simulation.AddModelToTask(simulationTaskName, allocator)

    scObject.thrOnTimeInMsg.subscribeTo(allocator.thrOnTimeOutMsg)

    torqueLogger = controller.cmdTorqueOutMsg.recorder()
    thrusterOnTimeLogger = allocator.thrOnTimeOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, torqueLogger)
    simulation.AddModelToTask(simulationTaskName, thrusterOnTimeLogger)

    css1 = coarseSunSensor.CoarseSunSensor()
    css1.ModelTag = "css1"
    css1.nHat_B = np.array([1.0, 0.0, 0.0])
    css1.fov = np.deg2rad(80.0)
    css1.scaleFactor = 1.0
    css1.senNoiseStd = 0.01
    css1.senBias = 0.001
    css1.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    css1.sunInMsg.subscribeTo(sunEphemeris.sunOutMsg)
    simulation.AddModelToTask(simulationTaskName, css1)

    css2 = coarseSunSensor.CoarseSunSensor()
    css2.ModelTag = "css2"
    css2.nHat_B = np.array([0.0, 1.0, 0.0])
    css2.fov = np.deg2rad(80.0)
    css2.scaleFactor = 1.0
    css2.senNoiseStd = 0.01
    css2.senBias = 0.001
    css2.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    css2.sunInMsg.subscribeTo(sunEphemeris.sunOutMsg)
    simulation.AddModelToTask(simulationTaskName, css2)

    css3 = coarseSunSensor.CoarseSunSensor()
    css3.ModelTag = "css3"
    css3.nHat_B = np.array([0.0, 0.0, 1.0])
    css3.fov = np.deg2rad(80.0)
    css3.scaleFactor = 1.0
    css3.senNoiseStd = 0.01
    css3.senBias = 0.001
    css3.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    css3.sunInMsg.subscribeTo(sunEphemeris.sunOutMsg)
    simulation.AddModelToTask(simulationTaskName, css3)

    css1Logger = css1.cssDataOutMsg.recorder()
    css2Logger = css2.cssDataOutMsg.recorder()
    css3Logger = css3.cssDataOutMsg.recorder()
    imuLogger = imu.sensorOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, css1Logger)
    simulation.AddModelToTask(simulationTaskName, css2Logger)
    simulation.AddModelToTask(simulationTaskName, css3Logger)
    simulation.AddModelToTask(simulationTaskName, imuLogger)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(duration))
    simulation.ExecuteSimulation()

    if C.ACS_SUN_POINTING:
        final_reference_sigma = np.asarray(reference.sigma, dtype=float)
    else:
        final_reference_sigma = np.asarray(reference.sigma_R0N, dtype=float)

    return {
        "time": css1Logger.times() * macros.NANO2SEC,
        "final_reference_sigma": final_reference_sigma,
        "truth_position": stateLogger.r_CN_N,
        "truth_velocity": stateLogger.v_CN_N,
        "truth_attitude": stateLogger.sigma_BN,
        "truth_gyro": stateLogger.omega_BN_B,
        "css1": css1Logger.OutputData,
        "css2": css2Logger.OutputData,
        "css3": css3Logger.OutputData,
        "gyro": imuLogger.AngVelPlatform,
        "gyro_feedback": navigationLogger.omega_BN_B,
        "torque": torqueLogger.torqueRequestBody,
        "thruster_on_time": thrusterOnTimeLogger.OnTimeRequest,
    }
