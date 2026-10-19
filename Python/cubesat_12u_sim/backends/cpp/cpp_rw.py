"""C++-powered Basilisk spacecraft module with Heyoka RW dynamics.

Drop-in replacement for Basilisk's spacecraft + reactionWheelStateEffector
modules in the scenarioAttitudeFeedbackRW demo. Reads motor torque commands,
propagates the coupled hub+wheel dynamics via Heyoka Taylor integration in C++,
and writes SCStatesMsg. All other Basilisk modules (navigation, reference,
controller, allocator) work unchanged.

Usage:
    from cubesat_12u_sim.backends.cpp.cpp_rw import CppRW
    sc = CppRW(task_step=0.1)
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = 750.0
    sc.hub.r_BcB_B = [0.0, 0.0, 0.0]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I.flatten())
    sc.hub.r_CN_NInit = rN
    sc.hub.v_CN_NInit = vN
    sc.hub.sigma_BNInit = sigma0
    sc.hub.omega_BN_BInit = omega0
    sc.hub.wheelSpeedsInit = [100.0, 200.0, 300.0]  # RPM
    simulation.AddModelToTask(task_name, sc.model)
    # ... rest of Basilisk pipeline unchanged ...
"""
from __future__ import annotations

import numpy as np


class CppRW:
    """Drop-in replacement for Basilisk spacecraft + RW effector using C++ Heyoka dynamics.

    Uses the rwdrive C++ extension for the 15-state MRP + RW dynamics propagation.
    """

    def __init__(self, task_step: float):
        from Basilisk.architecture import messaging, sysModel

        from cubesat_12u_sim.core import config as C
        from cubesat_12u_sim.native import rwdrive as _rw

        if not getattr(_rw, "available", hasattr(_rw, "run_rw_tick")):
            raise RuntimeError("rwdrive extension unavailable")
        if getattr(_rw, "run_rw_tick", None) is None:
            raise RuntimeError("rwdrive extension unavailable")

        outer = self
        outer._C = C
        outer._dt = float(task_step)
        outer._rw = _rw
        # Hoist hot-loop constants out of UpdateState (one attribute
        # lookup per tick instead of module + config lookups).
        from cubesat_12u_sim.backends.heyoka.heyoka_rw import RW_U_MAX

        outer._tol = float(C.HEYOKA_TOL)
        outer._mu = float(C.MU_EARTH)
        outer._u_max = float(RW_U_MAX)  # HR16 saturation, cf. stock effector
        outer._run_tick = _rw.run_rw_tick

        # Persistent state for the C++ tick loop
        outer._state = None  # 15-state MRP + wheel speeds
        outer._wheel_speeds_log = []  # log wheel speeds at each tick

        # Static parameters (packed 6-vectors [xx,yy,zz,xy,xz,yz] via
        # inertia_pars — NOT .flatten(), which yields 9 row-major elems
        # and silently shifts every channel in the C++ tick).
        from cubesat_12u_sim.backends.heyoka.heyoka_attitude import inertia_pars

        outer._mass = 750.0
        outer._I = np.diag([900.0, 800.0, 600.0])
        _I6, _invI6 = inertia_pars(outer._I)
        outer._I6 = np.ascontiguousarray(_I6, float)
        outer._invI6 = np.ascontiguousarray(_invI6, float)
        outer._Js = np.full(3, 0.07957747154594767)  # HR16 rotor inertia

        # Expose hub-like attributes for compatibility
        outer.hub = self._Hub()

        def _ws36(ws) -> list:
            # RWSpeedMsgPayload.wheelSpeeds is shape (36,): first 3 live.
            out = [0.0] * 36
            out[0], out[1], out[2] = float(ws[0]), float(ws[1]), float(ws[2])
            return out

        class _CppRWDynamics(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.rwMotorCmdInMsg = messaging.ArrayMotorTorqueMsgReader()
                self.scStateOutMsg = messaging.SCStatesMsg()
                self.payload = messaging.SCStatesMsgPayload()
                self.rwSpeedOutMsg = messaging.RWSpeedMsg()
                self.rwPayload = messaging.RWSpeedMsgPayload()

            def Reset(self, CurrentSimNanos):
                outer._state = None
                outer._wheel_speeds_log = []
                # Write zero state
                self.payload.r_CN_N = [0.0, 0.0, 0.0]
                self.payload.v_CN_N = [0.0, 0.0, 0.0]
                self.payload.sigma_BN = [0.0, 0.0, 0.0]
                self.payload.omega_BN_B = [0.0, 0.0, 0.0]
                self.scStateOutMsg.write(self.payload)
                self.rwPayload.wheelSpeeds = [0.0] * 36
                self.rwSpeedOutMsg.write(self.rwPayload)

            def UpdateState(self, CurrentSimNanos):
                # Read motor torque commands (zero on missed read).
                # Explicit 3-vec np array: keeps the pybind forcecast
                # conversion unambiguous (no tuple/list coercion doubt).
                try:
                    motor_msg = self.rwMotorCmdInMsg()
                    mt = motor_msg.motorTorque
                    u_motor = np.array([float(mt[0]), float(mt[1]),
                                        float(mt[2])], float)
                except Exception:
                    u_motor = np.zeros(3)

                if outer._state is None:
                    # First tick: initialize from hub attributes
                    r0 = np.asarray(outer.hub.r_CN_NInit, float).flatten()
                    v0 = np.asarray(outer.hub.v_CN_NInit, float).flatten()
                    sigma0 = np.asarray(outer.hub.sigma_BNInit, float).flatten()
                    omega0 = np.asarray(outer.hub.omega_BN_BInit, float).flatten()
                    # Wheel speeds: convert RPM to rad/s
                    ws0 = np.asarray(outer.hub.wheelSpeedsInit, float).flatten()
                    ws0 = ws0 * 2.0 * np.pi / 60.0
                    outer._state = np.concatenate([r0, v0, sigma0, omega0, ws0])
                    # Write initial state
                    self.payload.r_CN_N = r0.tolist()
                    self.payload.v_CN_N = v0.tolist()
                    self.payload.sigma_BN = sigma0.tolist()
                    self.payload.omega_BN_B = omega0.tolist()
                    self.scStateOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)
                    self.rwPayload.wheelSpeeds = _ws36(ws0)
                    self.rwSpeedOutMsg.write(self.rwPayload, CurrentSimNanos, self.moduleID)
                    return

                # Run one C++ dynamics tick (hoisted refs: no per-tick
                # module/config lookups, no temporary np.asarray copies).
                new_state = outer._run_tick(
                    outer._state,
                    u_motor,
                    outer._mass,
                    outer._I6,
                    outer._invI6,
                    outer._Js,
                    outer._dt,
                    outer._tol,
                    False,  # compact
                    outer._mu,
                    outer._u_max,
                )

                outer._state = np.asarray(new_state, float)
                outer._wheel_speeds_log.append(outer._state[12:15].copy())

                # Write state message
                st = outer._state
                self.payload.r_CN_N = st[0:3].tolist()
                self.payload.v_CN_N = st[3:6].tolist()
                self.payload.sigma_BN = st[6:9].tolist()
                self.payload.omega_BN_B = st[9:12].tolist()
                self.scStateOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)
                self.rwPayload.wheelSpeeds = _ws36(st[12:15])
                self.rwSpeedOutMsg.write(self.rwPayload, CurrentSimNanos, self.moduleID)

        outer.model = _CppRWDynamics()
        outer.model.ModelTag = "cppRWSat"

    class _Hub:
        """Compatibility shim for sc.hub.* attributes."""

        def __init__(self):
            self.mHub = 0.0
            self.r_BcB_B = [0.0, 0.0, 0.0]
            self.IHubPntBc_B = None
            self.r_CN_NInit = [0.0, 0.0, 0.0]
            self.v_CN_NInit = [0.0, 0.0, 0.0]
            self.sigma_BNInit = [0.0, 0.0, 0.0]
            self.omega_BN_BInit = [0.0, 0.0, 0.0]
            self.wheelSpeedsInit = [0.0, 0.0, 0.0]  # RPM

    def __getattr__(self, name):
        """Delegate unknown attributes to the underlying model."""
        if name == "model":
            raise AttributeError(name)
        return getattr(self.__dict__["model"], name)


def run_cpp_rw_pipeline(
    duration: float = 600.0,
    dt: float = 0.1,
    K: float = 3.5,
    P: float = 30.0,
) -> dict:
    """Run the full Basilisk RW pipeline with C++ Heyoka dynamics.

    Identical to bsk_rw_stock.run_stock_rw but uses CppRW instead of
    Basilisk's native spacecraft + reactionWheelStateEffector modules.
    All other modules (navigation, reference, controller, allocator) are stock
    Basilisk.

    Returns the same result dict schema as run_stock_rw.
    """
    from Basilisk.architecture import messaging
    from Basilisk.fswAlgorithms import (
        attTrackingError,
        inertial3D,
        mrpFeedback,
        rwMotorTorque,
    )
    from Basilisk.simulation import simpleNav
    from Basilisk.utilities import (
        SimulationBaseClass,
        macros,
        orbitalMotion,
        simIncludeGravBody,
        simIncludeRW,
    )

    from cubesat_12u_sim.backends.cpp.cpp_rw import CppRW

    task_name = "simTask"
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("simProcess")
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    # C++ spacecraft (replaces Basilisk spacecraft + RW effector)
    scObject = CppRW(dt)
    scObject.ModelTag = "bsk-Sat"
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]
    scObject.hub.IHubPntBc_B = None
    sim.AddModelToTask(task_name, scObject.model, 1)

    # Gravity (mu only — point-mass Kepler is handled internally by the
    # Heyoka integrator, same as heyoka_dynamics.py. Do NOT call
    # grav.addBodiesTo(scObject.model): CppRW.model is a plain SysModel,
    # not a Basilisk Spacecraft, so it has no gravField to attach to.)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    mu = earth.mu

    # RW factory (for config message only — actual RW dynamics are in C++)
    rw_factory = simIncludeRW.rwFactory()
    rw_factory.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=100.0)
    rw_factory.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=200.0)
    rw_factory.create("Honeywell_HR16", [0, 0, 1], maxMomentum=50.0, Omega=300.0,
                      rWB_B=[0.5, 0.5, 0.5])

    # Navigation
    nav = simpleNav.SimpleNav()
    sim.AddModelToTask(task_name, nav)

    # Reference
    ref = inertial3D.inertial3D()
    sim.AddModelToTask(task_name, ref)
    ref.sigma_R0N = [0.0, 0.0, 0.0]

    # Attitude error
    err = attTrackingError.attTrackingError()
    sim.AddModelToTask(task_name, err)

    # Controller
    ctl = mrpFeedback.mrpFeedback()
    sim.AddModelToTask(task_name, ctl)
    ctl.K = float(K)
    ctl.Ki = -1
    ctl.P = float(P)
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1

    # Allocator
    alloc = rwMotorTorque.rwMotorTorque()
    sim.AddModelToTask(task_name, alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # Messages
    vc_msg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=[900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]))
    fsw_msg = rw_factory.getConfigMessage()

    # Initial conditions
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]
    scObject.hub.wheelSpeedsInit = [100.0, 200.0, 300.0]

    # Connect messages
    nav.scStateInMsg.subscribeTo(scObject.model.scStateOutMsg)
    err.attNavInMsg.subscribeTo(nav.attOutMsg)
    err.attRefInMsg.subscribeTo(ref.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
    ctl.vehConfigInMsg.subscribeTo(vc_msg)
    ctl.rwParamsInMsg.subscribeTo(fsw_msg)
    ctl.rwSpeedsInMsg.subscribeTo(scObject.model.rwSpeedOutMsg)
    alloc.rwParamsInMsg.subscribeTo(fsw_msg)
    alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
    scObject.model.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)

    # Loggers
    state_log = scObject.model.scStateOutMsg.recorder()
    err_log = err.attGuidOutMsg.recorder()
    torque_log = ctl.cmdTorqueOutMsg.recorder()
    motor_log = alloc.rwMotorTorqueOutMsg.recorder()
    speed_log = scObject.model.rwSpeedOutMsg.recorder()
    for rec in (state_log, err_log, torque_log, motor_log, speed_log):
        sim.AddModelToTask(task_name, rec)

    # Run
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()

    # Extract results
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


def run_cpp_rw_mission(
    y0: np.ndarray,
    duration: float = 600.0,
    dt: float = 0.1,
    K: float = 3.5,
    P: float = 30.0,
    u_max: float = 0.2,
    mass: float = 750.0,
    inertia: np.ndarray | None = None,
    js: np.ndarray | None = None,
) -> dict:
    """Full closed-loop RW mission in one C++ call (fastest path).

    Same PD + allocation + dynamics as heyoka_rw.run_rw_mirror, but the
    whole tick loop runs in C++ via rwdrive.run_rw_mission: exactly ONE
    Python->C++ call for the full 600 s demo (vs 6000 per-tick calls in
    run_rw_mirror, vs 6000 Basilisk scheduler steps + FSW in stock).
    Returns the same {"t","Y","u_motor","dt"} schema as run_rw_mirror
    (Y is (15, n+1), u_motor is (3, n+1)).
    """
    import numpy as _np

    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.native import rwdrive as _rw
    from cubesat_12u_sim.backends.heyoka.heyoka_rw import RW_JS

    if getattr(_rw, "run_rw_mission", None) is None:
        raise RuntimeError("rwdrive extension unavailable (rebuild needed)")
    if inertia is None:
        inertia = _np.diag([900.0, 800.0, 600.0])
    if js is None:
        js = _np.full(3, RW_JS)
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import inertia_pars

    I6, invI6 = inertia_pars(_np.asarray(inertia, float))
    n_steps = int(round(float(duration) / float(dt)))
    out = _rw.run_rw_mission(
        _np.ascontiguousarray(_np.asarray(y0, float).ravel()),
        float(mass),
        _np.ascontiguousarray(_np.asarray(I6, float)),
        _np.ascontiguousarray(_np.asarray(invI6, float)),
        _np.ascontiguousarray(_np.asarray(js, float).ravel()),
        float(dt), int(n_steps), float(K), float(P), float(u_max),
        float(C.HEYOKA_TOL), False, float(C.MU_EARTH),
    )
    return {"t": _np.asarray(out["t"], float),
            "Y": _np.asarray(out["Y"], float),
            "u_motor": _np.asarray(out["u_motor"], float),
            "dt": float(out["dt"])}
