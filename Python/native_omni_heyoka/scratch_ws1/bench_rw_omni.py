"""RW path via OmniSpacecraft: parity vs stock + per-wheel uMax + N-wheel smoke.

Part A: full closed-loop RW pipeline (stock FSW: nav/ref/err/ctl/alloc)
  against bsk_rw_stock.run_stock_rw, 600 s @ dt=0.1. Gate: settled attitude
  parity ~1e-4 deg (Update-3 envelope 7.19e-05).
Part B: per-wheel saturation — static saturating motor torques, wheel speed
  slopes must equal uMax_i/Js per wheel (validates per-wheel clamp).
Part C: 4-wheel smoke — module accepts/propagates/publishes N != 3.
"""
from __future__ import annotations

import sys

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)

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

from cubesat_12u_sim.backends.heyoka.heyoka_rw import RW_JS, RW_U_MAX
from cubesat_12u_sim.core import config as C

from omniheyoka import omni_heyoka as oh

DUR, DT = 600.0, 0.1
K, P_GAIN = 3.5, 30.0
MASS = 750.0
I9 = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]


def make_omni_sc():
    sc = oh.OmniSpacecraft()
    sc.ModelTag = "omni-Sat"
    sc.setMass(MASS)
    sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
    sc.setMu(float(C.MU_EARTH))
    sc.setUseGravityGradient(False)
    sc.setTol(float(C.HEYOKA_TOL))
    for gs in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
        sc.addWheel(float(gs[0]), float(gs[1]), float(gs[2]), float(RW_JS))
    return sc


def run_omni_rw_pipeline(duration=DUR, dt=DT):
    from Basilisk.utilities import simIncludeGravBody as _g  # noqa: F401 (api parity)
    task_name = "simTask"
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("simProcess")
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    I = list(np.asarray(I9, float).reshape(9))
    sc = make_omni_sc()
    sim.AddModelToTask(task_name, sc, 1)

    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    sc.setMu(float(mu))

    rw_factory = simIncludeRW.rwFactory()
    rw_factory.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=100.0)
    rw_factory.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=200.0)
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
    ctl.Ki = -1
    ctl.P = float(P_GAIN)
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
    r0, v0 = orbitalMotion.elem2rv(mu, oe)
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.1, 0.2, -0.3)
    sc.setOmegaBNInit(0.001, -0.01, 0.03)
    sc.setWheelSpeedsInitRPM([100.0, 200.0, 300.0])

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
    speed_log = sc.rwSpeedOutMsg.recorder()
    for rec in (state_log, speed_log):
        sim.AddModelToTask(task_name, rec)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    ns = macros.NANO2SEC
    return {
        "t": state_log.times() * ns,
        "sigma_BN": np.asarray(state_log.sigma_BN, float),
        "omega_BN_B": np.asarray(state_log.omega_BN_B, float),
        "wheel_speeds": np.asarray(speed_log.wheelSpeeds, float)[:, :3],
    }


def part_B_per_wheel_umax():
    """Per-wheel saturation, omni vs stock: static saturating torques; wheel
    speed histories must agree (both use coupled spin + per-wheel clamp, so
    slopes differ slightly from the naive uMax/Js by the gs.omegaDot term;
    the stock twin is the truth, not the ideal)."""
    from Basilisk.simulation import reactionWheelStateEffector as rwEff
    from Basilisk.utilities import orbitalMotion, simIncludeGravBody, simIncludeRW

    def run(use_omni):
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        oe = orbitalMotion.ClassicElements()
        oe.a, oe.e = 1.0e7, 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
        r0, v0 = orbitalMotion.elem2rv(float(C.MU_EARTH), oe)
        cmd = messaging.ArrayMotorTorqueMsg()
        pay = messaging.ArrayMotorTorqueMsgPayload()
        pay.motorTorque = [1.0, 1.0, 1.0] + [0.0] * 33  # saturating
        cmd.write(pay)
        if use_omni:
            sc = oh.OmniSpacecraft()
            sc.setMass(MASS)
            sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
            sc.setMu(float(C.MU_EARTH))
            sc.setUseGravityGradient(False)
            sc.setTol(1e-12)
            sc.addWheel(1, 0, 0, float(RW_JS), 0.2)
            sc.addWheel(0, 1, 0, float(RW_JS), 0.05)
            sc.addWheel(0, 0, 1, float(RW_JS), 0.2)
            sc.ModelTag = "omniSat"
            sc.setR_CN_NInit(*map(float, r0))
            sc.setV_CN_NInit(*map(float, v0))
            sc.setSigmaBNInit(0.0, 0.0, 0.0)
            sc.setOmegaBNInit(0.0, 0.0, 0.0)
            sc.setWheelSpeedsInitRPM([0.0, 0.0, 0.0])
            sim.AddModelToTask("t", sc, 1)
            sc.rwMotorCmdInMsg.subscribeTo(cmd)
            log = sc.rwSpeedOutMsg.recorder()
        else:
            from Basilisk.simulation import spacecraft
            sc = spacecraft.Spacecraft()
            sc.ModelTag = "bsk-Sat"
            sc.hub.mHub = MASS
            sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                                  [0.0, 0.0, 600.0]]
            sc.hub.r_CN_NInit = r0
            sc.hub.v_CN_NInit = v0
            sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
            sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
            sim.AddModelToTask("t", sc, 1)
            grav = simIncludeGravBody.gravBodyFactory()
            earth = grav.createEarth()
            earth.isCentralBody = True
            grav.addBodiesTo(sc)
            rwf = simIncludeRW.rwFactory()
            rwf.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=0.0,
                       u_max=0.2)
            rwf.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=0.0,
                       u_max=0.05)
            rwf.create("Honeywell_HR16", [0, 0, 1], maxMomentum=50.0, Omega=0.0,
                       u_max=0.2)
            rw = rwEff.ReactionWheelStateEffector()
            rwf.addToSpacecraft("rw", rw, sc)
            sim.AddModelToTask("t", rw, 3)
            rw.rwMotorCmdInMsg.subscribeTo(cmd)
            log = rw.rwSpeedOutMsg.recorder()
        sim.AddModelToTask("t", log)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(2.0))
        sim.ExecuteSimulation()
        return np.asarray(log.wheelSpeeds, float)[:, :3]

    ws, wo = run(False), run(True)
    err = np.abs(ws - wo).max() * 60.0 / (2.0 * np.pi)
    print(f"  max wheel-speed diff vs stock: {err:.3e} RPM")
    assert err < 0.05, err
    # per-wheel clamp ratios still visible: wheel 2 ramps 4x slower
    slopes = (wo[-1] - wo[0])
    assert 3.5 < slopes[0] / slopes[1] < 4.5, slopes
    print("  OK: per-wheel saturation")


def part_C_four_wheels():
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = oh.OmniSpacecraft()
    sc.setMass(MASS)
    sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
    sc.setMu(float(C.MU_EARTH))
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.addWheel(1, 0, 0, float(RW_JS))
    sc.addWheel(0, 1, 0, float(RW_JS))
    sc.addWheel(0, 0, 1, float(RW_JS))
    sc.addWheel(1, 1, 1, float(RW_JS))
    sc.ModelTag = "omniSat"
    sc.setR_CN_NInit(1.0e7, 0.0, 0.0)
    sc.setV_CN_NInit(0.0, 6300.0, 0.0)
    sc.setSigmaBNInit(0.05, -0.05, 0.02)
    sc.setOmegaBNInit(0.01, 0.0, 0.0)
    sc.setWheelSpeedsInitRPM([10.0, 20.0, 30.0, 40.0])
    sim.AddModelToTask("t", sc, 1)
    cmd = messaging.ArrayMotorTorqueMsg()
    pay = messaging.ArrayMotorTorqueMsgPayload()
    pay.motorTorque = [0.05, -0.03, 0.02, 0.01] + [0.0] * 32
    cmd.write(pay)
    sc.rwMotorCmdInMsg.subscribeTo(cmd)
    slog = sc.scStateOutMsg.recorder()
    wlog = sc.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("t", slog)
    sim.AddModelToTask("t", wlog)
    sim.InitializeSimulation()
    assert sc.nStates() == 16, sc.nStates()
    assert sc.nPars() == 4 + 7 + 4, sc.nPars()
    sim.ConfigureStopTime(macros.sec2nano(5.0))
    sim.ExecuteSimulation()
    w4 = np.asarray(wlog.wheelSpeeds, float)
    assert np.all(np.isfinite(np.asarray(slog.sigma_BN, float)))
    assert w4.shape[1] >= 4 and np.all(np.isfinite(w4[:, :4]))
    assert abs(w4[-1, 3] - w4[0, 3]) > 1e-6, "tilted 4th wheel never actuated"
    print(f"  OK: 4 wheels, {sc.nStates()} states, {sc.nPars()} pars, W4 moved")


if __name__ == "__main__":
    import time

    from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff

    print("=== Part A: RW closed-loop parity vs stock ===")
    t0 = time.perf_counter()
    s = run_stock_rw(duration=DUR, dt=DT)
    t_stock = time.perf_counter() - t0
    t0 = time.perf_counter()
    o = run_omni_rw_pipeline(duration=DUR, dt=DT)
    t_omni = time.perf_counter() - t0
    att = float(np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(o["sigma_BN"][-1], s["sigma_BN"][-1])))))
    rate = float(np.degrees(np.linalg.norm(o["omega_BN_B"][-1] - s["omega_BN_B"][-1])))
    dw = float(np.abs(np.asarray(o["wheel_speeds"], float)
                      - np.asarray(s["wheel_speeds"], float)).max()
               * 60.0 / (2.0 * np.pi))
    print(f"  stock {t_stock * 1e3:.1f} ms / omni {t_omni * 1e3:.1f} ms "
          f"({t_stock / t_omni:.2f}x)")
    print(f"  parity: att={att:.3e} deg rate={rate:.3e} deg/s wheels={dw:.3e} RPM")
    assert att < 5e-4 and rate < 1e-5 and dw < 0.5, (att, rate, dw)
    print("  OK: RW parity")

    print("=== Part B: per-wheel uMax ===")
    part_B_per_wheel_umax()

    print("=== Part C: 4-wheel smoke ===")
    part_C_four_wheels()
