"""Optional Basilisk simulation for the 12U spacecraft sensor set."""
from __future__ import annotations

import argparse
import os
import subprocess
import time
from pathlib import Path

import numpy as np

from . import config as C
from . import vizard_model as vizardModel
from .actuators import allocation_pinv

# Maneuvers run by `--maneuver both`; `sunpoint` is a single-maneuver option.
MANEUVERS = ("detumble", "slew")


def run_simulation(
    duration: float = C.T_ORBIT,
    seed: int = 1,
    maneuver: str | None = None,
    rate_feedback: str = "observer",
    visualize: bool = False,
    save_viz: str | None = None,
    live_stream: bool = False,
    auto_launch_vizard: bool = True,
    spacecraft_name: str = C.VIZ_SPACECRAFT_NAME,
    render_model: str = C.VIZ_MODEL_KEY,
    trajectory_line: bool = False,
    sun_constraint: bool = True,
    shadow: bool = False,
) -> dict[str, np.ndarray | None]:
    """Run Basilisk with rate gyros, three coarse Sun sensors, and optional ACS.

    Set ``shadow=True`` (maneuver runs only) to fly the Heyoka MRP sidecar
    alongside truth and get ``result['heyoka_shadow']`` divergence summary.
    """
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback
    from Basilisk.simulation import coarseSunSensor, imuSensor, spacecraft, thrusterDynamicEffector
    from Basilisk.utilities import (
        SimulationBaseClass,
        macros,
        simHelpers,
        simIncludeGravBody,
        simIncludeThruster,
    )

    from .sun_model import SunEphemeris, SunPointingReference, sun_pointing_sigma

    if rate_feedback not in {"truth", "raw", "observer"}:
        raise ValueError("rate_feedback must be 'truth', 'raw', or 'observer'")

    simulation = SimulationBaseClass.SimBaseClass()
    simulationTaskName = "sensorTask"
    simulationProcess = simulation.CreateNewProcess("sensorProcess")
    taskStep = 0.01 if maneuver is not None else 1.0
    simulationProcess.addTask(simulation.CreateNewTask(simulationTaskName, macros.sec2nano(taskStep)))

    # Sun ephemeris first, so its message is already populated when the gravity
    # effector resets. Reading it before anything writes it is what produced the
    # NaN divergence when the Sun was a static default-constructed body.
    sunEphemeris = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    sunEphemeris.ModelTag = "sunEphemeris"
    simulation.AddModelToTask(simulationTaskName, sunEphemeris)

    if maneuver is None:
        initial_sigma = np.array([0.1, -0.2, 0.3])
        target_sigma = initial_sigma
        initial_omega = C.OMEGA0_BODY
    elif maneuver.lower() == "detumble":
        # Tilted start with a residual rate: the loop has to null the rate and
        # repoint onto the Sun at the same time.
        initial_sigma = np.array([0.1, -0.2, 0.3])
        target_sigma = initial_sigma
        initial_omega = C.OMEGA0_BODY
    elif maneuver.lower() == "slew":
        # Identity start, so this is the larger pure rotation onto the Sun.
        initial_sigma = np.zeros(3)
        target_sigma = C.ACS_SLEW_SIGMA
        initial_omega = C.OMEGA0_BODY
    elif maneuver.lower() == "sunpoint":
        # Same target as detumble but started undisturbed, isolating the
        # pointing loop from the rate-nulling duty.
        initial_sigma = np.array([0.1, -0.2, 0.3])
        target_sigma = sun_pointing_sigma(
            np.array([0.0, C.VIZ_SUN_DISTANCE, 0.0]), C.SUN_ROLL_REFERENCE_N
        )
        initial_omega = np.zeros(3)
    else:
        raise ValueError(
            "maneuver must be None, 'detumble', 'slew', or 'sunpoint'"
        )

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = spacecraft_name
    scObject.hub.mHub = C.MASS
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]
    scObject.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(C.INERTIA.flatten())
    scObject.hub.r_CN_NInit = C.R0_ECI
    scObject.hub.v_CN_NInit = C.V0_ECI
    scObject.hub.sigma_BNInit = initial_sigma
    scObject.hub.omega_BN_BInit = initial_omega

    # Earth as the central body. This both closes the orbit (r_CN_NInit and
    # v_CN_NInit describe a 12000 km circular orbit, not a straight line) and
    # registers the planet with Vizard: vizSupport discovers grav bodies through
    # scObject.gravField, so without this there is no parent body, no planet
    # view, and therefore nothing for spacecraftSizeMultiplier to scale in.
    # Point-mass central gravity applies no torque about the CoM and no gravity
    # gradient module is used, so the attitude results are unchanged.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True

    # The Sun is attached so Vizard has a named body for the sun-pointing
    # constraint (keep-in cone, pointing line) to reference. It must carry a
    # written ephemeris: with planetBodyInMsg left at its default the body sits
    # at the origin alongside Earth and the trajectory diverges to NaN. The
    # ephemeris is the rotating SunEphemeris above, which the CSS cluster and the
    # attitude reference also read, so there is a single source of truth.
    sunBody = gravFactory.createSun()
    sunBody.planetBodyInMsg.subscribeTo(sunEphemeris.sunOutMsg)

    gravFactory.addBodiesTo(scObject)

    simulation.AddModelToTask(simulationTaskName, scObject)

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

    torqueLogger = None
    thrusterOnTimeLogger = None
    navigationLogger = None
    reference = None
    shadowProp = None
    if maneuver is not None:
        vehicleConfigMessage = messaging.VehicleConfigMsg().write(
            messaging.VehicleConfigMsgPayload(
                ISCPntB_B=C.INERTIA.flatten().tolist(),
                CoM_B=[0.0, 0.0, 0.0],
            )
        )

        thrusterFactory = simIncludeThruster.thrusterFactory()
        for position, direction in zip(C.THRUSTER_POS, C.THRUSTER_DIR):
            thrusterFactory.create(
                "TEST_Thruster",
                position.tolist(),
                direction.tolist(),
                MaxThrust=float(C.TMAX_JET),
                cutoffFrequency=1000.0,
                useMinPulseTime=False,
            )
        thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
        thrusterFactory.addToSpacecraft("rcsThrusters", thrusterSet, scObject)

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
            # Reference tracks the moving Sun, so the panel-normal boresight
            # satisfies the keep-in cone by the end of the maneuver. target_sigma
            # is then only used for the printed summary.
            reference = SunPointingReference(sunEphemeris, C.SUN_ROLL_REFERENCE_N)
            reference.ModelTag = "sunPointingReference"
            simulation.AddModelToTask(simulationTaskName, reference)
        else:
            # Inertial-fixed attitude. Does not satisfy the sun cone.
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

        allocator = ThrusterAllocator(taskStep)
        allocator.ModelTag = "thrusterAllocator"
        allocator.cmdTorqueInMsg.subscribeTo(controller.cmdTorqueOutMsg)
        thrusterSet.cmdsInMsg.subscribeTo(allocator.thrOnTimeOutMsg)
        simulation.AddModelToTask(simulationTaskName, allocator)
        simulation.AddModelToTask(simulationTaskName, thrusterSet, 2)

        torqueLogger = controller.cmdTorqueOutMsg.recorder()
        thrusterOnTimeLogger = allocator.thrOnTimeOutMsg.recorder()
        simulation.AddModelToTask(simulationTaskName, torqueLogger)
        simulation.AddModelToTask(simulationTaskName, thrusterOnTimeLogger)

        shadowProp = None
        if shadow:
            from .heyoka_sidecar import HeyokaShadowPropagator
            shadowProp = HeyokaShadowPropagator(taskStep)
            shadowProp.model.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
            shadowProp.model.cmdTorqueInMsg.subscribeTo(controller.cmdTorqueOutMsg)
            simulation.AddModelToTask(simulationTaskName, shadowProp.model)

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

    # Third CSS on the +Z boresight, completing a +X/+Y/+Z triad. +X (css1) is
    # the constrained boresight, so css1 is the sensor that actually measures the
    # sun-pointing error; this one views the wing normal.
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

    if visualize:
        from Basilisk.simulation import vizInterface
        from Basilisk.utilities import vizSupport

        if not vizSupport.vizFound:
            raise RuntimeError("Basilisk was built without Vizard vizInterface support")

        imuMarker = vizInterface.GenericSensor()
        imuMarker.r_SB_B = list(C.VIZ_IMU_POSITION_B)
        imuMarker.normalVector = list(C.VIZ_IMU_NORMAL_B)
        imuMarker.fieldOfView.push_back(np.deg2rad(C.VIZ_IMU_FOV_DEG))
        imuMarker.size = C.VIZ_IMU_SIZE
        imuMarker.color = vizInterface.IntVector(vizSupport.toRGBA255("cyan", alpha=0.35))
        imuMarker.label = "IMU"
        imuMarker.genericSensorCmd = 1

        thrusterEffectorList = [thrusterSet] if maneuver is not None else None
        endpoint = f"{C.VIZ_COM_PROTOCOL}://{C.VIZ_COM_ADDRESS}:{C.VIZ_COM_PORT}"
        if live_stream and auto_launch_vizard:
            appPath = Path(C.VIZ_APP_PATH)
            if not appPath.exists():
                raise FileNotFoundError(
                    f"Vizard application not found at {C.VIZ_APP_PATH}. "
                    "Install Vizard, point config.VIZ_APP_PATH at it, or run "
                    "without --live-stream."
                )
            subprocess.run(
                ["open", str(appPath), "--args", "-directComm", endpoint],
                check=True,
            )
            time.sleep(1.0)
        primitiveModel = render_model.upper() in C.VIZ_PRIMITIVE_MODELS
        panelledModel = render_model.upper() == C.VIZ_PANEL_MODEL_KEY.upper()
        if not (primitiveModel or panelledModel) and render_model.lower() not in C.VIZ_BUILTIN_MODELS:
            raise ValueError(
                f"unknown render_model {render_model!r}; expected one of "
                f"{[C.VIZ_PANEL_MODEL_KEY] + list(C.VIZ_PRIMITIVE_MODELS)} "
                f"or {list(C.VIZ_BUILTIN_MODELS.values())}"
            )
        modelKey = (
            None
            if (primitiveModel or panelledModel)
            else C.VIZ_BUILTIN_MODELS[render_model.lower()]
        )
        viz = vizSupport.enableUnityVisualization(
            simulation,
            simulationTaskName,
            [scObject],
            saveFile=save_viz,
            thrEffectorList=thrusterEffectorList,
            cssList=[[css1, css2, css3]],
            genericSensorList=[[imuMarker]],
            modelDictionaryKeyList=modelKey,
            liveStream=live_stream,
        )
        if panelledModel:
            # Bus plus two solar wings, generated in metres at true 12U scale, so
            # the thruster/CSS/IMU overlays stay registered on the bus faces.
            # Materials come from the accompanying .mtl, so no color override.
            objPath = vizardModel.write_panelled_bus(
                Path(C.VIZ_MODEL_DIR) / C.VIZ_PANEL_MODEL_FILENAME,
                lx=C.LX,
                ly=C.LY,
                lz=C.LZ,
                deploy=C.VIZ_PANEL_DEPLOY,
                thickness=C.VIZ_PANEL_THICKNESS,
                normal_B=C.VIZ_PANEL_NORMAL_B,
                deployAxis_B=C.VIZ_PANEL_DEPLOY_AXIS_B,
            )
            vizSupport.createCustomModel(
                viz,
                modelPath=str(objPath),
                simBodiesToModify=[spacecraft_name],
                scale=[1.0, 1.0, 1.0],
            )
        elif primitiveModel:
            # True body dimensions: the local view is not scaled, so the bus must
            # match the metre-valued thruster/CSS/IMU overlays. Oversizing here
            # would swallow them.
            vizSupport.createCustomModel(
                viz,
                modelPath=render_model.upper(),
                simBodiesToModify=[spacecraft_name],
                scale=[C.LX, C.LY, C.LZ],
                color=vizSupport.toRGBA255("silver"),
            )
        # Sun-pointing orientation constraint. SUN_POINTING_AXIS_B (+X) is the
        # constrained boresight, so keeping +X inside this cone is the
        # sun-pointing requirement (css1 on +X measures it; css3 views the
        # +Z wing normal).
        if sun_constraint:
            vizSupport.createConeInOut(
                viz,
                toBodyName=C.VIZ_SUN_BODY_NAME,
                coneColor=C.VIZ_SUN_CONE_COLOR,
                isKeepIn=True,
                normalVector_B=list(C.SUN_POINTING_AXIS_B),
                incidenceAngle=np.deg2rad(C.VIZ_SUN_CONE_ANGLE_DEG),
                coneHeight=C.VIZ_SUN_CONE_HEIGHT,
                fromBodyName=spacecraft_name,
                coneName="sunKeepInCone",
            )
            vizSupport.createPointLine(
                viz,
                toBodyName=C.VIZ_SUN_BODY_NAME,
                fromBodyName=spacecraft_name,
                lineColor=C.VIZ_SUN_LINE_COLOR,
            )
        # Planet / Solar System views only, per the Vizard settings docs. A 0.2 m
        # bus is still sub-pixel from a 12000 km orbit, so fall back to the 2D
        # sprite representation whenever the model gets too small to resolve.
        viz.settings.spacecraftSizeMultiplier = C.VIZ_DISPLAY_SCALE
        viz.settings.showSpacecraftAsSprites = 1
        viz.settings.showCelestialBodiesAsSprites = 1
        viz.settings.showSpacecraftLabels = 1
        # Osculating ellipse (1 = relative to the parent body). Recomputed from
        # the current state each step at a fixed segment count, so it is cheap
        # and shows the orbit shape even on a short run -- one revolution is
        # 24795 s, so a 600 s run only covers 8.7 degrees of arc and would
        # otherwise look like a straight line.
        viz.settings.orbitLinesOn = 1
        viz.settings.orbitLineSegments = C.VIZ_ORBIT_LINE_SEGMENTS
        # The flown trajectory is opt-in: Vizard appends a point per step, so
        # every VizMessage then carries an ever-growing polyline. Measured on
        # this scenario, enabling it pushed a 600 s live-stream run from 35 s
        # to over 20 minutes. Use --trajectory-line for short playback runs.
        viz.settings.trueTrajectoryLinesOn = 1 if trajectory_line else 0
        # Open on the satellite rather than on an arbitrary default view.
        viz.settings.mainCameraTarget = spacecraft_name
        viz.settings.forceStartAtSpacecraftLocalView = 1
        if live_stream:
            viz.reqComProtocol = C.VIZ_COM_PROTOCOL
            viz.reqComAddress = C.VIZ_COM_ADDRESS
            viz.reqPortNumber = C.VIZ_COM_PORT
        vizSupport.setInstrumentGuiSetting(
            viz,
            viewCSSPanel=1,
            viewCSSCoverage=1,
            showCSSLabels=1,
            showGenericSensorLabels=1,
        )
        if maneuver is not None:
            vizSupport.setActuatorGuiSetting(
                viz,
                viewThrusterPanel=1,
                viewThrusterHUD=1,
                showThrusterLabels=1,
            )

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(duration))
    simulation.ExecuteSimulation()

    # The reference moves under sun pointing, so the summary has to be scored
    # against the reference actually held at the end of the run.
    if reference is None:
        final_reference_sigma = None
    elif C.ACS_SUN_POINTING:
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
        "gyro_feedback": None if navigationLogger is None else navigationLogger.omega_BN_B,
        "torque": None if torqueLogger is None else torqueLogger.torqueRequestBody,
        "thruster_forces": (
            None
            if thrusterOnTimeLogger is None
            else (thrusterOnTimeLogger.OnTimeRequest[:, : len(C.THRUSTER_POS)] / taskStep)
            * C.THRUSTER_TMAX
        ),
        "thruster_on_time": None if thrusterOnTimeLogger is None else thrusterOnTimeLogger.OnTimeRequest,
        "heyoka_shadow": None if shadowProp is None else shadowProp.summary(),
    }


def run_maneuver_pair(
    duration: float = 600.0,
    seed: int = 1,
    rate_feedback: str = "observer",
    visualize: bool = False,
    save_viz: str | None = None,
    live_stream: bool = False,
    auto_launch_vizard: bool = True,
    spacecraft_name: str = C.VIZ_SPACECRAFT_NAME,
    render_model: str = C.VIZ_MODEL_KEY,
    trajectory_line: bool = False,
    sun_constraint: bool = True,
) -> dict[str, dict[str, np.ndarray | None]]:
    """Run detumble then slew back to back against a single Vizard instance.

    Vizard is launched once, on the first maneuver only. Each maneuver is an
    independent SimulationBaseClass, so the vizInterface REQ socket is torn
    down and re-bound on the same port between them; the second run therefore
    relies on Vizard reconnecting to that port.
    """
    results: dict[str, dict[str, np.ndarray | None]] = {}
    for index, maneuver in enumerate(MANEUVERS):
        if save_viz is not None:
            stem, extension = os.path.splitext(save_viz)
            maneuverViz = f"{stem}_{maneuver}{extension or '.bin'}"
        else:
            maneuverViz = None
        results[maneuver] = run_simulation(
            duration=duration,
            seed=seed,
            maneuver=maneuver,
            rate_feedback=rate_feedback,
            visualize=visualize,
            save_viz=maneuverViz,
            live_stream=live_stream,
            # Launch once; a second `open` would only re-activate the running
            # instance without re-passing -directComm.
            auto_launch_vizard=auto_launch_vizard and index == 0,
            spacecraft_name=f"{spacecraft_name}_{maneuver}",
            render_model=render_model,
            trajectory_line=trajectory_line,
            sun_constraint=sun_constraint,
        )
        print(f"[Basilisk] finished {maneuver}, Vizard scene: {spacecraft_name}_{maneuver}")
    return results


def run_controller_comparison(
    duration: float = 600.0,
    seed: int = 1,
) -> dict[str, dict[str, dict[str, np.ndarray | None]]]:
    """Run truth, raw-IMU, and observer rate-feedback cases."""
    return {
        maneuver: {
            mode: run_simulation(
                duration=duration,
                seed=seed,
                maneuver=maneuver,
                rate_feedback=mode,
            )
            for mode in ("truth", "raw", "observer")
        }
        for maneuver in ("detumble", "slew")
    }


def plot_controller_comparison(
    results: dict[str, dict[str, dict[str, np.ndarray | None]]],
    output_path: str = "controller_comparison.png",
) -> str:
    """Plot truth, raw-IMU, and observer controller results."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex="col")
    for column, maneuver in enumerate(("detumble", "slew")):
        target = np.array([0.1, -0.2, 0.3]) if maneuver == "detumble" else C.ACS_SLEW_SIGMA
        caseResults = results[maneuver]
        truth = caseResults["truth"]
        time = truth["time"]
        truthRate = np.linalg.norm(truth["truth_gyro"], axis=1)
        truthError = np.linalg.norm(truth["truth_attitude"] - target, axis=1)
        axes[0, column].plot(time, truthRate, "k", label="true state")
        axes[0, column].plot(
            time,
            np.linalg.norm(caseResults["raw"]["gyro_feedback"], axis=1),
            label="no observer",
        )
        axes[0, column].plot(
            time,
            np.linalg.norm(caseResults["observer"]["gyro_feedback"], axis=1),
            label="observer",
        )
        axes[0, column].set_title(f"{maneuver}: angular rate")
        axes[0, column].set_ylabel("rate norm [rad/s]")
        axes[0, column].grid(True)
        axes[0, column].legend()

        axes[1, column].plot(time, truthError, "k", label="true state")
        axes[1, column].plot(
            time,
            np.linalg.norm(caseResults["raw"]["truth_attitude"] - target, axis=1),
            label="no observer",
        )
        axes[1, column].plot(
            time,
            np.linalg.norm(caseResults["observer"]["truth_attitude"] - target, axis=1),
            label="observer",
        )
        axes[1, column].set_ylabel("MRP error norm")
        axes[1, column].set_xlabel("time [s]")
        axes[1, column].grid(True)
        axes[1, column].legend()

    fig.suptitle("Controller comparison: truth, raw IMU, observer")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def _reference_error(result: dict[str, np.ndarray | None]) -> float:
    """Final MRP error against the reference actually held at the end of a run."""
    reference = result.get("final_reference_sigma")
    if reference is None:
        return float("nan")
    return float(np.linalg.norm(np.asarray(result["truth_attitude"][-1]) - reference))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--maneuver",
        choices=("detumble", "slew", "sunpoint", "both"),
        default=None,
        help=(
            "attitude maneuver to run; 'sunpoint' rotates the panel-normal "
            "boresight onto the Sun, 'both' runs detumble then slew in one "
            "Vizard session"
        ),
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--compare", action="store_true")
    parser.add_argument(
        "--rate-feedback",
        choices=("truth", "raw", "observer"),
        default="observer",
    )
    parser.add_argument("--output", default="controller_comparison.png")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--save-viz", default=None)
    parser.add_argument("--live-stream", action="store_true")
    parser.add_argument("--render-model", default=C.VIZ_MODEL_KEY)
    parser.add_argument(
        "--trajectory-line",
        action="store_true",
        help="also draw the flown trajectory; costly on long live-stream runs",
    )
    parser.add_argument("--spacecraft-name", default=C.VIZ_SPACECRAFT_NAME)
    parser.add_argument(
        "--no-sun-constraint",
        action="store_true",
        help="omit the sun keep-in cone and sun pointing line",
    )
    parser.add_argument(
        "--heyoka-shadow",
        action="store_true",
        help="fly the Heyoka MRP sidecar alongside truth (maneuver runs only)",
    )
    args = parser.parse_args()
    duration = args.duration
    if duration is None:
        duration = 600.0 if (args.compare or args.maneuver == "both") else C.T_ORBIT
    if args.compare:
        results = run_controller_comparison(duration=duration)
        outputPath = plot_controller_comparison(results, output_path=args.output)
        print(f"[Basilisk] comparison plot: {outputPath}")
        return
    if args.maneuver == "both":
        results = run_maneuver_pair(
            duration=duration,
            rate_feedback=args.rate_feedback,
            visualize=args.visualize,
            save_viz=args.save_viz,
            live_stream=args.live_stream,
            spacecraft_name=args.spacecraft_name,
            render_model=args.render_model,
            trajectory_line=args.trajectory_line,
            sun_constraint=not args.no_sun_constraint,
        )
        for maneuver, result in results.items():
            print(
                f"[Basilisk] {maneuver:8s} samples={result['gyro'].shape[0]} "
                f"final_gyro={result['gyro'][-1]} "
                f"final_mrp_error={_reference_error(result):.3e}"
            )
        return
    result = run_simulation(
        duration=duration,
        maneuver=args.maneuver,
        rate_feedback=args.rate_feedback,
        visualize=args.visualize,
        save_viz=args.save_viz,
        live_stream=args.live_stream,
        spacecraft_name=args.spacecraft_name,
        render_model=args.render_model,
        trajectory_line=args.trajectory_line,
        sun_constraint=not args.no_sun_constraint,
        shadow=args.heyoka_shadow,
    )
    print(
        f"[Basilisk] samples={result['gyro'].shape[0]} "
        f"duration={result['time'][-1]:.1f}s "
        f"final_gyro={result['gyro'][-1]}"
    )
    if args.maneuver is not None:
        print(f"[Basilisk] final_sigma={result['truth_attitude'][-1]}")
    if result.get("heyoka_shadow") is not None:
        print(f"[Heyoka shadow] {result['heyoka_shadow']}")


if __name__ == "__main__":
    main()
