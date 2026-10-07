"""Basilisk RW pipeline with the NATIVE C++ Heyoka dynamics module.

Identical to cpp_rw.run_cpp_rw_pipeline except the dynamics is a true
compiled Basilisk SysModel (nativerw.HeyokaRWDynamics) instead of a Python
SysModel calling back into C++ per tick. All other modules (navigation,
reference, controller, allocator) are stock Basilisk.

Requires the native extension (see ../native_rw): it is imported lazily so
this module stays importable without it.

ISC License - see native_rw/LICENSE.
"""
from __future__ import annotations

import math
import warnings

import numpy as np

from .heyoka_rw import RW_JS, RW_U_MAX


try:
    from nativerw import HeyokaRWDynamics as _SwigBase

    _NATIVE = True
except Exception:
    _SwigBase = object  # type: ignore[assignment]
    _NATIVE = False


def available() -> bool:
    return bool(_NATIVE)


class NativeSpacecraft(_SwigBase):
    """Drop-in ``spacecraft.Spacecraft`` replacement with Heyoka dynamics.

    A Python subclass of the native SWIG module, so it *is* a Basilisk
    ``SysModel`` to the scheduler: existing scripts swap the class and
    nothing else::

        sc = NativeSpacecraft()          # was: spacecraft.Spacecraft()
        sc.ModelTag = "bsk-Sat"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I)
        sc.hub.r_CN_NInit = rN
        ...
        grav.addBodiesTo(sc)             # accepted: central mu -> module
        sim.AddModelToTask(task_name, sc, 1)

    ``sc.hub.*`` assignments forward immediately into the C++ module (no
    staging / ordering constraints). Reaction wheels are configured via
    ``attachBalancedWheels`` (replacing the factory ``addToSpacecraft`` +
    effector block; keep the factory ``create()`` lines — the FSW config
    message still comes from them).

    Scope notes (by design, matching the RW-demo baseline):
    - rigid hub at CoM (nonzero ``r_BcB_B`` warns: CoM offset unmodeled),
    - 3 body-axis-aligned balanced wheels (no jitter/friction),
    - central-body point-mass Kepler only: extra gravity bodies warn.
    """

    def __init__(self):
        if not _NATIVE:
            raise RuntimeError("nativerw extension unavailable (see native_rw/)")
        super().__init__()
        object.__setattr__(self, "hub", self._Hub(self))
        object.__setattr__(self, "gravField", self._GravField(self))

    def __setattr__(self, name, value):
        if name in ("hub", "gravField"):
            object.__setattr__(self, name, value)
        else:
            super().__setattr__(name, value)  # SWIG property routing

    @property
    def model(self):
        """Back-compat alias: this object already is the SysModel."""
        return self

    def attachBalancedWheels(self, js, u_max, speedsRPM) -> None:
        """Configure the 3 body-axis-aligned wheels (factory-effector equiv).

        Replaces ``rw_factory.addToSpacecraft(...)`` + the effector task for
        the balanced-wheel baseline. Keep the factory ``create()`` lines:
        ``rw_factory.getConfigMessage()`` still feeds the FSW.
        """
        jsv = [float(v) for v in
               np.broadcast_to(np.asarray(RW_JS if js is None else js, float), (3,))]
        if not all(math.isfinite(v) and v > 0.0 for v in jsv):
            raise ValueError("rotor inertias must be finite positive")
        u_max = float(RW_U_MAX) if u_max is None else float(u_max)
        ws = [float(v) for v in np.asarray(speedsRPM, float).ravel()]
        if len(ws) != 3 or not all(math.isfinite(v) for v in ws):
            raise ValueError("attachBalancedWheels needs 3 finite wheel speeds")
        if not (math.isfinite(u_max) and u_max >= 0.0):
            raise ValueError("u_max must be finite non-negative")
        self.setRotorInertia(*jsv)
        self.setUMax(u_max)
        self.setWheelSpeedsInitRPM(*ws)

    class _Hub:
        """``sc.hub.*`` with immediate forwarding into the C++ module."""

        _KNOWN = {"mHub", "r_BcB_B", "IHubPntBc_B", "r_CN_NInit",
                  "v_CN_NInit", "sigma_BNInit", "omega_BN_BInit",
                  "wheelSpeedsInit"}

        def __init__(self, module):
            object.__setattr__(self, "_m", module)
            for k, v in (("mHub", 750.0), ("r_BcB_B", [0.0, 0.0, 0.0]),
                         ("IHubPntBc_B", None),
                         ("r_CN_NInit", [0.0, 0.0, 0.0]),
                         ("v_CN_NInit", [0.0, 0.0, 0.0]),
                         ("sigma_BNInit", [0.0, 0.0, 0.0]),
                         ("omega_BN_BInit", [0.0, 0.0, 0.0]),
                         ("wheelSpeedsInit", [0.0, 0.0, 0.0])):
                object.__setattr__(self, k, v)

        @staticmethod
        def _v3(value) -> list:
            flat = [float(v) for v in np.asarray(value, float).ravel()]
            if len(flat) != 3 or not all(math.isfinite(v) for v in flat):
                raise ValueError("hub 3-vector needs exactly 3 finite values")
            return flat

        def __setattr__(self, name, value):
            object.__setattr__(self, name, value)
            if name not in self._KNOWN:
                warnings.warn(f"NativeSpacecraft.hub.{name} is not modeled "
                              "(assignment stored, dynamics unaffected)")
                return
            m = object.__getattribute__(self, "_m")
            if name == "mHub":
                m.setMass(float(value))
            elif name == "IHubPntBc_B":
                if value is None:
                    return
                I = np.asarray(value, float).reshape(3, 3)
                if not np.allclose(I, I.T, atol=0.0):
                    raise ValueError("inertia matrix must be symmetric")
                m.setInertia(float(I[0, 0]), float(I[1, 1]), float(I[2, 2]),
                             float(I[0, 1]), float(I[0, 2]), float(I[1, 2]))
            elif name == "r_BcB_B":
                off = np.asarray(value, float).ravel()
                if bool((off != 0.0).any()):
                    warnings.warn("NativeSpacecraft: r_BcB_B CoM offset is "
                                  "not modeled (hub assumed at CoM)")
            else:
                setters = {"r_CN_NInit": "setR_CN_NInit",
                           "v_CN_NInit": "setV_CN_NInit",
                           "sigma_BNInit": "setSigmaBNInit",
                           "omega_BN_BInit": "setOmegaBNInit",
                           "wheelSpeedsInit": "setWheelSpeedsInitRPM"}
                meth = getattr(m, setters[name], None)
                if meth is None:
                    warnings.warn(f"NativeSpacecraft.hub.{name} is not "
                                  "supported by this module")
                else:
                    meth(*self._v3(value))

    class _GravField:
        """Minimal gravity-effector protocol shim for gravBodyFactory.

        Accepts ``grav.addBodiesTo(sc)`` and forwards the central body's mu
        to the module (point-mass Kepler inside). Extra bodies warn: only
        central Kepler is modeled.
        """

        def __init__(self, model):
            self._m = model
            self.bodies: list = []

        def setGravBodies(self, gravBodies) -> None:
            try:
                bodies = list(gravBodies)
            except TypeError:
                try:
                    bodies = [gravBodies[i] for i in range(len(gravBodies))]
                except TypeError as e:
                    raise TypeError("setGravBodies needs an iterable or "
                                    "len/getitem vector of gravity bodies") from e
            if not bodies:
                raise ValueError("setGravBodies got zero bodies: mu unknown")
            self.bodies = bodies
            central = [b for b in bodies if bool(getattr(b, "isCentralBody", False))]
            if len(bodies) > 1:
                warnings.warn("NativeSpacecraft: only central-body Kepler is "
                              f"modeled ({len(bodies)} bodies attached)")
            pick = central[0] if central else bodies[0]
            mu = float(pick.mu)
            if not (math.isfinite(mu) and mu > 0.0):
                raise ValueError("central mu must be finite positive")
            self._m.setMu(mu)


def run_native_rw_pipeline(
    duration: float = 600.0,
    dt: float = 0.1,
    K: float = 3.5,
    P: float = 30.0,
    mass: float = 750.0,
    inertia: np.ndarray | None = None,
    r0=None, v0=None, sigma0=None, omega0=None,
    wheel_speeds_rpm=None,
    js=None,
) -> dict:
    """Native RW pipeline. Optional dispersion kwargs (defaults = baseline;
    the FSW vehicle config stays nominal: model error, as in the MC)."""
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

    from Basilisk.utilities import simHelpers

    from . import config as C

    task_name = "simTask"
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("simProcess")
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    # Native dynamics (true C++ SysModel: zero Python per tick), configured
    # with the stock spacecraft scripting API.
    I = ([900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
         if inertia is None else list(np.asarray(inertia, float).reshape(9)))
    sc = NativeSpacecraft()
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = float(mass)
    sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I)
    # wheels come from the RW side (as in stock: factory Omega=100/200/300),
    # not the hub: attachBalancedWheels replaces addToSpacecraft + effector
    _ws = ([100.0, 200.0, 300.0] if wheel_speeds_rpm is None else
           [float(v) for v in np.asarray(wheel_speeds_rpm, float).ravel()][:3])
    sc.attachBalancedWheels(RW_JS if js is None else js, RW_U_MAX, _ws)
    sc.setTol(float(C.HEYOKA_TOL))
    sim.AddModelToTask(task_name, sc, 1)

    # Gravity via the standard factory call (central mu -> module)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    grav.addBodiesTo(sc)

    # RW factory (config message only)
    rw_factory = simIncludeRW.rwFactory()
    rw_factory.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=100.0)
    rw_factory.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=200.0)
    # NOTE: rWB_B / addToSpacecraft have no effect here: the native module
    # has no effector attachment points (wheels come from attachBalancedWheels).
    # The factory create() lines stay: getConfigMessage() feeds the FSW.
    rw_factory.create("Honeywell_HR16", [0, 0, 1], maxMomentum=50.0, Omega=300.0,
                      rWB_B=[0.5, 0.5, 0.5])

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
    ctl.Ki = -1  # stock baseline: negative Ki disables integral feedback
    ctl.P = float(P)
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1  # == -0.2, verbatim from stock

    alloc = rwMotorTorque.rwMotorTorque()
    sim.AddModelToTask(task_name, alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    vc_msg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=[900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]))
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
    # (hub assignments forwarded immediately; nothing to flush)

    nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
    err.attNavInMsg.subscribeTo(nav.attOutMsg)
    err.attRefInMsg.subscribeTo(ref.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
    ctl.vehConfigInMsg.subscribeTo(vc_msg)
    ctl.rwParamsInMsg.subscribeTo(fsw_msg)
    ctl.rwSpeedsInMsg.subscribeTo(sc.rwSpeedOutMsg)
    alloc.rwParamsInMsg.subscribeTo(fsw_msg)
    alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
    sc.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)

    state_log = sc.scStateOutMsg.recorder()
    err_log = err.attGuidOutMsg.recorder()
    torque_log = ctl.cmdTorqueOutMsg.recorder()
    motor_log = alloc.rwMotorTorqueOutMsg.recorder()
    speed_log = sc.rwSpeedOutMsg.recorder()
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
