"""Heyoka-powered spacecraft dynamics module for the Basilisk pipeline.

Replaces Basilisk's native ``spacecraft`` module with a Heyoka Taylor
integrator while keeping the entire rest of the Basilisk pipeline intact:

- Reads the same ``THRArrayOnTimeCmdMsg`` (thruster on-time commands)
- Writes the same ``SCStatesMsg`` (spacecraft state output)
- All downstream modules (IMU, CSS, navigation, reference, controller,
  allocator, Vizard) work unchanged

Usage:
    from .heyoka_dynamics import HeyokaSpacecraft
    sc = HeyokaSpacecraft(task_step=0.01)
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = 750.0
    sc.hub.r_BcB_B = [0.0, 0.0, 0.0]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I.flatten())
    sc.hub.r_CN_NInit = rN
    sc.hub.v_CN_NInit = vN
    sc.hub.sigma_BNInit = sigma0
    sc.hub.omega_BN_BInit = omega0
    simulation.AddModelToTask(task_name, sc)
    # ... rest of Basilisk pipeline unchanged ...
"""
from __future__ import annotations

import numpy as np


class HeyokaSpacecraft:
    """Drop-in replacement for Basilisk's spacecraft module using Heyoka dynamics.

    This is NOT a Basilisk SysModel — it's a factory that creates one.
    Access the actual model via ``.model`` after instantiation.

    The factory pattern matches ``HeyokaShadowPropagator`` so the calling
    code can treat both uniformly.
    """

    def __init__(self, task_step: float):
        from Basilisk.architecture import messaging, sysModel

        from . import config as C
        from .heyoka_attitude import (
            make_mrp_integrator,
            pack_mrp_state,
            pack_pars,
        )
        from .actuators import jets_to_wrench

        outer = self
        outer._C = C
        outer._dt = float(task_step)
        outer._ta = None
        outer._prev_on_time = None
        outer._initialized = False

        # Expose hub-like attributes for compatibility with basilisk_sim.py
        outer.hub = self._Hub()

        class _HeyokaDynamics(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.thrOnTimeInMsg = messaging.THRArrayOnTimeCmdMsgReader()
                self.scStateOutMsg = messaging.SCStatesMsg()
                self.payload = messaging.SCStatesMsgPayload()
                self._first = True

            def Reset(self, CurrentSimNanos):
                outer._ta = None
                outer._prev_on_time = None
                outer._initialized = False
                self._first = True
                # Write zero state so downstream modules don't read garbage
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
                    on_time = np.asarray(on_time_msg.OnTimeRequest[:len(C.THRUSTER_POS)], float)
                except Exception:
                    on_time = np.zeros(len(C.THRUSTER_POS))

                # Convert on-time to throttle duty cycle
                throttle = np.clip(on_time / outer._dt, 0.0, 1.0)

                # Convert throttle to body wrench
                F_b, T_b = jets_wrench_quantized(
                    throttle, C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)

                if outer._ta is None:
                    # First tick: initialize the Heyoka integrator
                    r0 = np.asarray(outer.hub.r_CN_NInit, float).flatten()
                    v0 = np.asarray(outer.hub.v_CN_NInit, float).flatten()
                    sigma0 = np.asarray(outer.hub.sigma_BNInit, float).flatten()
                    omega0 = np.asarray(outer.hub.omega_BN_BInit, float).flatten()
                    outer._ta = make_mrp_integrator(
                        pack_mrp_state(r0, v0, sigma0, omega0))
                    outer._prev_on_time = (F_b, T_b)
                    outer._initialized = True
                    self._first = False
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
                    outer._ta.pars[i] = float(val)

                # Propagate one step
                outer._ta.propagate_for(outer._dt)

                # Shadow-set reset for MRP
                st = np.asarray(outer._ta.state, float)
                sg = st[6:9]
                n2 = float(sg @ sg)
                if n2 > 1.0:
                    sg = -sg / n2
                    st[6:9] = sg
                    outer._ta.state[6:9] = sg

                # Write state message
                self.payload.r_CN_N = st[0:3].tolist()
                self.payload.v_CN_N = st[3:6].tolist()
                self.payload.sigma_BN = st[6:9].tolist()
                self.payload.omega_BN_B = st[9:12].tolist()
                self.scStateOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

                # Store current wrench for next tick
                outer._prev_on_time = (F_b, T_b)
                self._first = False

        outer.model = _HeyokaDynamics()
        outer.model.ModelTag = "heyokaSat"

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


def run_heyoka_pipeline(
    duration: float = 600.0,
    seed: int = 1,
    maneuver: str = "detumble",
    rate_feedback: str = "observer",
    task_step: float = 0.01,
) -> dict:
    """Run the full Basilisk pipeline with Heyoka dynamics.

    Identical to ``basilisk_sim.run_simulation`` but uses HeyokaSpacecraft
    instead of Basilisk's native spacecraft module. All other modules (IMU,
    CSS, navigation, reference, controller, allocator) are stock Basilisk.

    Returns the same result dict schema as ``run_simulation``.
    """
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback
    from Basilisk.simulation import coarseSunSensor, imuSensor, thrusterDynamicEffector
    from Basilisk.utilities import (
        SimulationBaseClass,
        macros,
        simIncludeGravBody,
        simIncludeThruster,
    )

    from . import config as C
    from .actuators import allocation_pinv
    from .heyoka_dynamics import HeyokaSpacecraft
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

    # Heyoka spacecraft (replaces Basilisk spacecraft module)
    scObject = HeyokaSpacecraft(task_step)
    scObject.ModelTag = C.VIZ_SPACECRAFT_NAME
    scObject.hub.mHub = C.MASS
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]
    scObject.hub.IHubPntBc_B = None  # Heyoka uses config inertia directly
    scObject.hub.r_CN_NInit = C.R0_ECI
    scObject.hub.v_CN_NInit = C.V0_ECI
    scObject.hub.sigma_BNInit = initial_sigma
    scObject.hub.omega_BN_BInit = initial_omega

    # Gravity (for Vizard body registration only — Heyoka uses point-mass)
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    sunBody = gravFactory.createSun()
    sunBody.planetBodyInMsg.subscribeTo(sunEphemeris.sunOutMsg)
    # Note: gravFactory.addBodiesTo(scObject) is NOT called — HeyokaSpacecraft
    # is not a Basilisk spacecraft module. Gravity is handled internally by
    # the Heyoka integrator (point-mass Kepler).

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
    # Vehicle config message (required by mrpFeedback)
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

    # Connect allocator -> Heyoka dynamics (replaces thrusterSet.cmdsInMsg)
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

    # Extract results (same schema as basilisk_sim.run_simulation)
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
