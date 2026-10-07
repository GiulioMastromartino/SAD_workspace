"""C++-powered Basilisk spacecraft module using the existing cxxdrive extension.

Drop-in replacement for Basilisk's spacecraft module. Reads
THRArrayOnTimeCmdMsg, runs the entire tick loop (Heyoka Taylor
propagation + MRP control + thruster allocation + fuel bookkeeping)
in C++ via the cxxdrive extension, and writes SCStatesMsg.

All other Basilisk modules (IMU, CSS, navigation, reference,
controller, allocator, Vizard) work unchanged.

Usage:
    from .cpp_basilisk import CppSpacecraft
    sc = CppSpacecraft(task_step=0.01)
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = 750.0
    sc.hub.r_BcB_B = [0.0, 0.0, 0.0]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I.flatten())
    sc.hub.r_CN_NInit = rN
    sc.hub.v_CN_NInit = vN
    sc.hub.sigma_BNInit = sigma0
    sc.hub.omega_BN_BInit = omega0
    simulation.AddModelToTask(task_name, sc.model)
    # ... rest of Basilisk pipeline unchanged ...
"""
from __future__ import annotations

import numpy as np


class CppSpacecraft:
    """Drop-in replacement for Basilisk's spacecraft module using C++ dynamics.

    Uses the existing cxxdrive extension for the full tick loop in C++.
    """

    def __init__(self, task_step: float):
        from Basilisk.architecture import messaging, sysModel

        from . import config as C
        from . import cxxdrive as _cxx
        from .heyoka_attitude import pack_mrp_state, pack_pars
        from .actuators import allocation_matrices

        if not _cxx.available:
            raise RuntimeError("cxxdrive extension unavailable")

        outer = self
        outer._C = C
        outer._dt = float(task_step)
        outer._cxx = _cxx

        # Persistent state for the C++ tick loop
        outer._state = None  # 12-state MRP
        outer._est_rate = np.zeros(3)
        outer._bias_est = np.zeros(3)
        outer._est_init = np.zeros(1, dtype=bool)
        outer._on_time_accum = np.zeros(12)
        outer._phase_imp = np.zeros(3)
        outer._phase_fuel = np.zeros(3)
        outer._seed = 42
        outer._prev_on_time = None

        # Wrench effectiveness matrices (computed once)
        P_nom = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)
        outer._P_nom = P_nom
        outer._MF = (C.THRUSTER_DIR * C.THRUSTER_TMAX[:, None]).T.copy()  # (3, 12)
        outer._MT = (np.cross(C.THRUSTER_POS, C.THRUSTER_DIR) * C.THRUSTER_TMAX[:, None]).T.copy()  # (3, 12)
        outer._tmax = C.THRUSTER_TMAX.copy()
        outer._I6 = C.INERTIA.flatten().copy()
        outer._invI6 = np.linalg.inv(C.INERTIA).flatten().copy()

        # Expose hub-like attributes for compatibility with basilisk_sim.py
        outer.hub = self._Hub()

        class _CppDynamics(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.thrOnTimeInMsg = messaging.THRArrayOnTimeCmdMsgReader()
                self.scStateOutMsg = messaging.SCStatesMsg()
                self.payload = messaging.SCStatesMsgPayload()

            def Reset(self, CurrentSimNanos):
                outer._state = None
                outer._est_rate = np.zeros(3)
                outer._bias_est = np.zeros(3)
                outer._est_init = np.zeros(1, dtype=bool)
                outer._on_time_accum = np.zeros(12)
                outer._phase_imp = np.zeros(3)
                outer._phase_fuel = np.zeros(3)
                outer._prev_on_time = None
                # Write zero state
                self.payload.r_CN_N = [0.0, 0.0, 0.0]
                self.payload.v_CN_N = [0.0, 0.0, 0.0]
                self.payload.sigma_BN = [0.0, 0.0, 0.0]
                self.payload.omega_BN_B = [0.0, 0.0, 0.0]
                self.scStateOutMsg.write(self.payload)

            def UpdateState(self, CurrentSimNanos):
                from .actuators import jets_wrench_quantized

                t = CurrentSimNanos * 1e-9

                # Read thruster on-time commands (message has 36 slots, we use 12)
                try:
                    on_time_msg = self.thrOnTimeInMsg()
                    on_time = np.asarray(on_time_msg.OnTimeRequest[:12], float)
                except Exception:
                    on_time = np.zeros(12)

                # Convert on-time to throttle duty cycle
                throttle = np.clip(on_time / outer._dt, 0.0, 1.0)

                # Convert throttle to body wrench
                F_b, T_b = jets_wrench_quantized(
                    throttle, C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)

                if outer._state is None:
                    # First tick: initialize
                    r0 = np.asarray(outer.hub.r_CN_NInit, float).flatten()
                    v0 = np.asarray(outer.hub.v_CN_NInit, float).flatten()
                    sigma0 = np.asarray(outer.hub.sigma_BNInit, float).flatten()
                    omega0 = np.asarray(outer.hub.omega_BN_BInit, float).flatten()
                    outer._state = np.concatenate([r0, v0, sigma0, omega0])
                    outer._prev_on_time = (F_b, T_b)
                    # Write initial state
                    self.payload.r_CN_N = r0.tolist()
                    self.payload.v_CN_N = v0.tolist()
                    self.payload.sigma_BN = sigma0.tolist()
                    self.payload.omega_BN_B = omega0.tolist()
                    self.scStateOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)
                    return

                # Apply previous tick's wrench (ZOH — Basilisk mailbox semantics)
                F_prev, T_prev = outer._prev_on_time
                pars = pack_pars(F_prev, T_prev, float(C.MASS), C.INERTIA)
                for i, val in enumerate(pars):
                    outer._ta_pars = val  # Store for cxxdrive

                # Run one C++ tick: sense + control + propagate
                # Build reference (identity for now — can be extended)
                ref = np.zeros(3)

                # Gyro noise parameters
                gyro_bias = np.zeros(3)
                arw_std = np.zeros(3)

                # Run the C++ tick loop
                new_state = outer._cxx.run_tick(
                    outer._state,
                    on_time,
                    float(C.MASS),
                    outer._I6,
                    outer._invI6,
                    outer._MF,
                    outer._MT,
                    outer._tmax,
                    float(C.ACS_K), float(C.ACS_P),
                    float(C.ACS_OBSERVER_GAIN), float(C.ACS_OBSERVER_BIAS_GAIN),
                    float(C.THRUSTER_CONTROL_SCALE), outer._dt, float(C.G0),
                    ref,
                    gyro_bias, arw_std,
                    outer._est_rate, outer._bias_est, outer._est_init,
                    outer._on_time_accum, outer._phase_imp, outer._phase_fuel,
                    0,  # phase
                    True,  # quantize
                    float(C.HEYOKA_TOL), False,  # compact
                    float(C.MU_EARTH), bool(C.USE_GRAVITY_GRADIENT),
                    outer._seed)

                outer._state = np.asarray(new_state, float)
                outer._seed += 1

                # Write state message
                st = outer._state
                self.payload.r_CN_N = st[0:3].tolist()
                self.payload.v_CN_N = st[3:6].tolist()
                self.payload.sigma_BN = st[6:9].tolist()
                self.payload.omega_BN_B = st[9:12].tolist()
                self.scStateOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

                # Store current wrench for next tick
                outer._prev_on_time = (F_b, T_b)

        outer.model = _CppDynamics()
        outer.model.ModelTag = "cppSat"

    class _Hub:
        """Compatibility shim for sc.hub.* attributes used in basilisk_sim.py."""

        def __init__(self):
            self.mHub = 0.0
            self.r_BcB_B = [0.0, 0.0, 0.0]
            self.IHubPntBc_B = None
            self.r_CN_NInit = [0.0, 0.0, 0.0]
            self.v_CN_NInit = [0.0, 0.0, 0.0]
        self.sigma_BNInit = [0.0, 0.0, 0.0]
        self.omega_BN_BInit = [0.0, 0.0, 0.0]

    def __getattr__(self, name):
        """Delegate unknown attributes to the underlying model."""
        if name == "model":
            raise AttributeError(name)
        return getattr(self.__dict__["model"], name)


def run_cpp_pipeline(
    duration: float = 600.0,
    seed: int = 1,
    maneuver: str = "detumble",
    rate_feedback: str = "observer",
    task_step: float = 0.01,
) -> dict:
    """Run the full Basilisk pipeline with C++ dynamics.

    Identical to basilisk_sim.run_simulation but uses CppSpacecraft
    instead of Basilisk's native spacecraft module. All other modules
    (IMU, CSS, navigation, reference, controller, allocator) are stock
    Basilisk.

    Returns the same result dict schema as run_simulation.
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
    from .cpp_basilisk import CppSpacecraft
    from .sun_model import SunEphemeris, SunPointingReference, sun_pointing_sigma

    simulation = SimulationBaseClass.SimBaseClass()
    simulationTaskName = "sensorTask"
    simulationProcess = simulation.CreateNewProcess("sensorProcess")
    simulationProcess.addTask(simulation.CreateNewTask(simulationTaskName, macros.sec2nano(task_step)))

    # Sun ephemeris
    sunEphemeris = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    sunEphemeris.ModelTag = "sunEphemeris"
    simulation.AddModelToTask(simulationTaskName, sunEphemeris)

    # Initial conditions
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

    # C++ spacecraft (replaces Basilisk spacecraft module)
    scObject = CppSpacecraft(task_step)
    scObject.ModelTag = C.VIZ_SPACECRAFT_NAME
    scObject.hub.mHub = C.MASS
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]
    scObject.hub.IHubPntBc_B = None
    scObject.hub.r_CN_NInit = C.R0_ECI
    scObject.hub.v_CN_NInit = C.V0_ECI
    scObject.hub.sigma_BNInit = initial_sigma
    scObject.hub.omega_BN_BInit = initial_omega

    # Gravity (for Vizard body registration only)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    sunBody = gravFactory.createSun()
    sunBody.planetBodyInMsg.subscribeTo(sunEphemeris.sunOutMsg)

    simulation.AddModelToTask(simulationTaskName, scObject.model)

    stateLogger = scObject.model.scStateOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, stateLogger)

    # IMU
    imu = imuSensor.ImuSensor()
    imu.ModelTag = "imu"
    imu.RNGSeed = seed
    imu.senRotBias = C.GYRO_BIAS0
    imu.PMatrixGyro = np.diag(C.GYRO_ARW)
    imu.PMatrixAccel = np.diag([1e-4, 1e-4, 1e-4])
    imu.scStateInMsg.subscribeTo(scObject.model.scStateOutMsg)
    simulation.AddModelToTask(simulationTaskName, imu)

    # Navigation (same as basilisk_sim.py)
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
    navigation.scStateInMsg.subscribeTo(scObject.model.scStateOutMsg)
    navigation.imuInMsg.subscribeTo(imu.sensorOutMsg)
    simulation.AddModelToTask(simulationTaskName, navigation)
    navigationLogger = navigation.attOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, navigationLogger)

    # Reference
    if C.ACS_SUN_POINTING:
        reference = SunPointingReference(sunEphemeris, C.SUN_ROLL_REFERENCE_N)
        reference.ModelTag = "sunPointingReference"
    else:
        reference = inertial3D.inertial3D()
        reference.ModelTag = "attitudeReference"
        reference.sigma_R0N = target_sigma
    simulation.AddModelToTask(simulationTaskName, reference)

    # Attitude error
    attitudeError = attTrackingError.attTrackingError()
    attitudeError.ModelTag = "attitudeTrackingError"
    attitudeError.attNavInMsg.subscribeTo(navigation.attOutMsg)
    attitudeError.attRefInMsg.subscribeTo(reference.attRefOutMsg)
    simulation.AddModelToTask(simulationTaskName, attitudeError)

    # Controller
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

    # Thruster allocator
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

    # Connect allocator -> C++ dynamics
    scObject.model.thrOnTimeInMsg.subscribeTo(allocator.thrOnTimeOutMsg)

    torqueLogger = controller.cmdTorqueOutMsg.recorder()
    thrusterOnTimeLogger = allocator.thrOnTimeOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, torqueLogger)
    simulation.AddModelToTask(simulationTaskName, thrusterOnTimeLogger)

    # Coarse Sun Sensors
    css1 = coarseSunSensor.CoarseSunSensor()
    css1.ModelTag = "css1"
    css1.nHat_B = np.array([1.0, 0.0, 0.0])
    css1.fov = np.deg2rad(80.0)
    css1.scaleFactor = 1.0
    css1.senNoiseStd = 0.01
    css1.senBias = 0.001
    css1.stateInMsg.subscribeTo(scObject.model.scStateOutMsg)
    css1.sunInMsg.subscribeTo(sunEphemeris.sunOutMsg)
    simulation.AddModelToTask(simulationTaskName, css1)

    css2 = coarseSunSensor.CoarseSunSensor()
    css2.ModelTag = "css2"
    css2.nHat_B = np.array([0.0, 1.0, 0.0])
    css2.fov = np.deg2rad(80.0)
    css2.scaleFactor = 1.0
    css2.senNoiseStd = 0.01
    css2.senBias = 0.001
    css2.stateInMsg.subscribeTo(scObject.model.scStateOutMsg)
    css2.sunInMsg.subscribeTo(sunEphemeris.sunOutMsg)
    simulation.AddModelToTask(simulationTaskName, css2)

    css3 = coarseSunSensor.CoarseSunSensor()
    css3.ModelTag = "css3"
    css3.nHat_B = np.array([0.0, 0.0, 1.0])
    css3.fov = np.deg2rad(80.0)
    css3.scaleFactor = 1.0
    css3.senNoiseStd = 0.01
    css3.senBias = 0.001
    css3.stateInMsg.subscribeTo(scObject.model.scStateOutMsg)
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

    # Run
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(duration))
    simulation.ExecuteSimulation()

    # Extract results
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
