"""Stock Basilisk runner replicating scenarioAttitudeFeedbackRW baseline.

Same modules and parameters as the demo's ``run(False, False, False)``
configuration (750 kg hub, I=diag(900,800,600), 3x Honeywell HR16 on body
axes at 100/200/300 RPM, inertial identity target, K=3.5/P=30, 600 s at
0.1 s, BalancedWheels, no jitter, no voltage IO) — but returning logged
time series instead of plots, for stock-vs-Heyoka comparison.
"""
from __future__ import annotations

import numpy as np


def run_stock_rw(duration: float = 600.0, dt: float = 0.1,
                 K: float = 3.5, P: float = 30.0,
                 mass: float = 750.0,
                 inertia: np.ndarray | None = None,
                 r0=None, v0=None, sigma0=None, omega0=None,
                 wheel_speeds_rpm=None) -> dict:
    """Stock RW demo. Optional dispersion kwargs (all default to the baseline
    ICs/properties, preserving validated behavior when omitted)."""
    from Basilisk.architecture import messaging
    from Basilisk.fswAlgorithms import (
        attTrackingError,
        inertial3D,
        mrpFeedback,
        rwMotorTorque,
    )
    from Basilisk.simulation import (
        reactionWheelStateEffector,
        simpleNav,
        spacecraft,
    )
    from Basilisk.utilities import (
        SimulationBaseClass,
        fswSetupRW,
        macros,
        orbitalMotion,
        simHelpers,
        simIncludeGravBody,
        simIncludeRW,
    )

    task_name = "simTask"
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("simProcess")
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    I = ([900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
         if inertia is None else list(np.asarray(inertia, float).reshape(9)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = float(mass)
    sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I)
    sim.AddModelToTask(task_name, sc, 1)

    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    grav.addBodiesTo(sc)

    rw_factory = simIncludeRW.rwFactory()
    _ws = [100.0, 200.0, 300.0] if wheel_speeds_rpm is None else \
        [float(v) for v in np.asarray(wheel_speeds_rpm, float).ravel()][:3]
    rw_factory.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=_ws[0])
    rw_factory.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=_ws[1])
    rw_factory.create("Honeywell_HR16", [0, 0, 1], maxMomentum=50.0, Omega=_ws[2],
                      rWB_B=[0.5, 0.5, 0.5])
    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = "RW_cluster"
    rw_factory.addToSpacecraft(rw_effector.ModelTag, rw_effector, sc)
    sim.AddModelToTask(task_name, rw_effector, 2)

    nav = simpleNav.SimpleNav()
    sim.AddModelToTask(task_name, nav)

    ref = inertial3D.inertial3D()
    sim.AddModelToTask(task_name, ref)
    ref.sigma_R0N = [0.0, 0.0, 0.0]

    err = attTrackingError.attTrackingError()
    sim.AddModelToTask(task_name, err)

    ctl = mrpFeedback.mrpFeedback()
    sim.AddModelToTask(task_name, ctl)
    ctl.K = float(K)
    ctl.Ki = -1
    ctl.P = float(P)
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1

    alloc = rwMotorTorque.rwMotorTorque()
    sim.AddModelToTask(task_name, alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    vc_msg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=I))
    fsw_msg = rw_factory.getConfigMessage()

    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    if r0 is None or v0 is None:
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        if r0 is None:
            r0 = rN
        if v0 is None:
            v0 = vN
    sc.hub.r_CN_NInit = r0
    sc.hub.v_CN_NInit = v0
    sc.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]] if sigma0 is None else sigma0
    sc.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]] if omega0 is None else omega0

    nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
    err.attNavInMsg.subscribeTo(nav.attOutMsg)
    err.attRefInMsg.subscribeTo(ref.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
    ctl.vehConfigInMsg.subscribeTo(vc_msg)
    ctl.rwParamsInMsg.subscribeTo(fsw_msg)
    ctl.rwSpeedsInMsg.subscribeTo(rw_effector.rwSpeedOutMsg)
    alloc.rwParamsInMsg.subscribeTo(fsw_msg)
    alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
    rw_effector.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)

    state_log = sc.scStateOutMsg.recorder()
    err_log = err.attGuidOutMsg.recorder()
    torque_log = ctl.cmdTorqueOutMsg.recorder()
    motor_log = alloc.rwMotorTorqueOutMsg.recorder()
    speed_log = rw_effector.rwSpeedOutMsg.recorder()
    for rec in (state_log, err_log, torque_log, motor_log, speed_log):
        sim.AddModelToTask(task_name, rec)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()

    ns = macros.NANO2SEC
    return {
        "t": state_log.times() * ns,
        "sigma_BN": np.asarray(state_log.sigma_BN, float),
        "omega_BN_B": np.asarray(state_log.omega_BN_B, float),
        "r_CN_N": np.asarray(state_log.r_CN_N, float),
        "v_CN_N": np.asarray(state_log.v_CN_N, float),
        "sigma_BR": np.asarray(err_log.sigma_BR, float),
        "omega_BR_B": np.asarray(err_log.omega_BR_B, float),
        "torque_demand": np.asarray(torque_log.torqueRequestBody, float),
        "motor_torque": np.asarray(motor_log.motorTorque, float)[:, :3],
        "wheel_speeds": np.asarray(speed_log.wheelSpeeds, float)[:, :3],
        "mu": float(mu),
    }
