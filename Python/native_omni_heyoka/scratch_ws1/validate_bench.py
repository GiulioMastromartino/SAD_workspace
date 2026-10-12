"""Validation bench: stock Basilisk pipeline with OmniSpacecraft as the ONLY
non-stock module.

Everything except the spacecraft is stock Basilisk.  The tilted-wheel
configuration is the generalisation case that native_rw cannot express.
"""
from __future__ import annotations

import sys
import time

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (
    attTrackingError, inertial3D, mrpFeedback, rwMotorTorque)
from Basilisk.simulation import simpleNav
from Basilisk.utilities import (
    SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, simIncludeRW)

from omniheyoka import omni_heyoka as oh
from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff

DURATION, DT = 600.0, 0.1
K, P = 3.5, 30.0
MU, MASS = 3.986004418e14, 750.0
INERTIA = np.diag([900.0, 800.0, 600.0])
JS = 0.07957747154594767
RPM0 = np.array([100.0, 200.0, 300.0])

S3 = 1.0 / np.sqrt(3.0)
ALIGNED = np.eye(3)
TILTED = np.array([[S3, S3, S3], [-S3, S3, S3], [S3, -S3, S3]])


def run_omni_pipeline(axes, duration=DURATION, dt=DT):
    """Stock pipeline; the spacecraft is our native module."""
    r0, v0 = _ics()
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("simProcess")
    proc.addTask(sim.CreateNewTask("simTask", macros.sec2nano(dt)))

    sc = oh.OmniSpacecraft()
    sc.ModelTag = "omniSat"
    sc.setMass(MASS)
    sc.setInertia(float(INERTIA[0, 0]), float(INERTIA[1, 1]), float(INERTIA[2, 2]), 0.0, 0.0, 0.0)
    sc.setMu(MU)
    sc.setUseGravityGradient(False)   # RW demo: Kepler only
    sc.setUMax(0.2)
    sc.setTol(1e-12)
    for i in range(3):                # the generalisation: arbitrary axes
        sc.addWheel(axes[i][0], axes[i][1], axes[i][2], JS)
    sc.setWheelSpeedsInitRPM([float(v) for v in RPM0])
    sc.setR_CN_NInit(*[float(v) for v in r0])
    sc.setV_CN_NInit(*[float(v) for v in v0])
    sc.setSigmaBNInit(0.1, 0.2, -0.3)
    sc.setOmegaBNInit(0.001, -0.01, 0.03)
    sim.AddModelToTask("simTask", sc, 1)

    # RW factory supplies the FSW config message ONLY (effector is ours).
    rw_factory = simIncludeRW.rwFactory()
    for i in range(3):
        rw_factory.create("Honeywell_HR16", [float(v) for v in axes[i]],
                          maxMomentum=50.0, Omega=float(RPM0[i]))
    fsw_msg = rw_factory.getConfigMessage()

    nav = simpleNav.SimpleNav()
    sim.AddModelToTask("simTask", nav)
    ref = inertial3D.inertial3D()
    sim.AddModelToTask("simTask", ref)
    ref.sigma_R0N = [0.0, 0.0, 0.0]
    err = attTrackingError.attTrackingError()
    sim.AddModelToTask("simTask", err)
    ctl = mrpFeedback.mrpFeedback()
    sim.AddModelToTask("simTask", ctl)
    ctl.K, ctl.Ki, ctl.P = float(K), -1, float(P)
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1
    alloc = rwMotorTorque.rwMotorTorque()
    sim.AddModelToTask("simTask", alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    vc_msg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(
            ISCPntB_B=np.asarray(INERTIA, float).ravel()))

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
    sim.AddModelToTask("simTask", state_log)
    sim.AddModelToTask("simTask", speed_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    ns = macros.NANO2SEC
    return {"sigma_BN": np.asarray(state_log.sigma_BN, float),
            "omega_BN_B": np.asarray(state_log.omega_BN_B, float),
            "r_CN_N": np.asarray(state_log.r_CN_N, float),
            "wheel": np.asarray(speed_log.wheelSpeeds, float)[:, :3],
            "build_s": sc.getBuildSeconds()}


def _ics():
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0; oe.e = 0.01
    oe.i = 33.3 * macros.D2R; oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R; oe.f = 85.3 * macros.D2R
    return orbitalMotion.elem2rv(MU, oe)


def compare(stock, omni, tag):
    n = min(len(stock["sigma_BN"]), len(omni["sigma_BN"]))
    att = np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(omni["sigma_BN"][-1], stock["sigma_BN"][n - 1]))))
    rate = np.degrees(np.linalg.norm(omni["omega_BN_B"][-1]
                                     - stock["omega_BN_B"][n - 1]))
    print(f"  {tag}: att={att:.3e} deg  rate={rate:.3e} deg/s  "
          f"|sig| {np.linalg.norm(omni['sigma_BN'][-1]):.6f} vs "
          f"{np.linalg.norm(stock['sigma_BN'][n-1]):.6f}")


if __name__ == "__main__":
    # Warm the JIT for BOTH arms first: the very first symbolic build in a
    # process pays ~200 ms of LLVM warm-up (one-time, process-wide), which is
    # otherwise charged to whichever config runs first and makes the aligned
    # case look 0.78x when it is not.
    from bench_tilted import run_tilted_stock
    for _axes in (ALIGNED, TILTED):
        run_omni_pipeline(_axes, duration=10.0)
    if True:  # stock warm-up
        from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw
        run_stock_rw(duration=10.0, dt=DT)
        run_tilted_stock(duration=10.0, dt=DT)

    for tag, axes in (("ALIGNED", ALIGNED), ("TILTED", TILTED)):
        print(f"\n=== {tag} ===")

        if tag == "ALIGNED":
            from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw
            ts = []
            for _ in range(3):
                t0 = time.perf_counter(); run_stock_rw(duration=DURATION, dt=DT)
                ts.append(time.perf_counter() - t0)
            t_stock = min(ts)
            t0 = time.perf_counter(); stock = run_stock_rw(duration=DURATION, dt=DT)
            t_stock = t_stock
        else:
            ts = []
            for _ in range(3):
                t0 = time.perf_counter(); run_tilted_stock(duration=DURATION, dt=DT)
                ts.append(time.perf_counter() - t0)
            t_stock = min(ts)
            t0 = time.perf_counter(); stock = run_tilted_stock(duration=DURATION, dt=DT)
            t_stock = t_stock

        ts = []
        for _ in range(3):
            t0 = time.perf_counter(); run_omni_pipeline(axes)
            ts.append(time.perf_counter() - t0)
        t_omni = min(ts)
        t0 = time.perf_counter(); omni = run_omni_pipeline(axes)
        t_omni = t_omni

        print(f"  stock Basilisk (full pipeline) : {t_stock*1e3:8.2f} ms")
        print(f"  omni     (full pipeline)       : {t_omni*1e3:8.2f} ms   "
              f"({t_stock/t_omni:.2f}x)")
        compare(stock, omni, "parity")
        print(f"  omni symbolic build (one-time) : {omni['build_s']*1e3:.2f} ms")
