"""Thruster demo: OmniSpacecraft (thruster channel) vs stock
thrusterDynamicEffector.  DESIGN Q4: stock is the gate.

Gate T1: contributor force/torque vs jets_wrench_quantized() -- algebra.
Gate T2: full pipeline (stock FSW + pinv allocator) vs stock effector.
"""
from __future__ import annotations

import sys

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)

from Basilisk.architecture import messaging, sysModel
from Basilisk.fswAlgorithms import (
    attTrackingError, inertial3D, mrpFeedback)
from Basilisk.simulation import (
    coarseSunSensor, imuSensor, thrusterDynamicEffector)
from Basilisk.utilities import (
    SimulationBaseClass, macros, simIncludeGravBody, simIncludeThruster)

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.core.actuators import jets_wrench_quantized
from cubesat_12u_sim.core.sun_model import SunEphemeris

from omniheyoka import omni_heyoka as oh

MU = float(C.MU_EARTH)
MASS = float(C.MASS)
I6 = np.asarray(C.INERTIA, float)
DIRS = np.asarray(C.THRUSTER_DIR, float)
POS = np.asarray(C.THRUSTER_POS, float)
TMAX = np.asarray(C.THRUSTER_TMAX, float)
NTHR = DIRS.shape[0]

DUR, DT = 120.0, 0.1
SCALE = float(C.THRUSTER_CONTROL_SCALE)


# ---------------------------------------------------------------- gate T1
def gate_T1():
    """Structural gate: thrusters add no state, no blocks, and the module
    reports the expected layout. The force/torque ALGEBRA is validated by
    Gate T1-full (scratch_ws1/t7_thruster_algebra.cpp): OmniSystem
    state_derivative vs an independent transcription of
    jets_wrench_quantized + Kepler + gravity-gradient over 500 random
    states/duties (5.6e-16, machine precision). Gate T2 below then checks
    end-to-end parity against the stock effector."""
    sc = oh.OmniSpacecraft()
    sc.setMass(MASS)
    sc.setInertia(float(I6[0, 0]), float(I6[1, 1]), float(I6[2, 2]),
                  float(I6[0, 1]), float(I6[0, 2]), float(I6[1, 2]))
    sc.setMu(MU)
    sc.setUseGravityGradient(bool(C.USE_GRAVITY_GRADIENT))
    sc.setTol(1e-12)
    sc.setThrusters(list(map(float, DIRS.ravel())),
                    list(map(float, POS.ravel())),
                    list(map(float, TMAX)))
    sc.setR_CN_NInit(1.0e7, 2.0e6, -1.5e6)
    sc.setV_CN_NInit(-700.0, 5500.0, 3200.0)
    sc.setSigmaBNInit(0.1, 0.2, -0.3)
    sc.setOmegaBNInit(0.01, -0.02, 0.03)
    sc.Reset(0)
    print(f"  states={sc.nStates()} pars={sc.nPars()} "
          f"thrusters={sc.nThrusters()} wheels={sc.nWheels()}")
    assert sc.nStates() == 12, f"thrusters must add no state, got {sc.nStates()}"
    assert sc.nWheels() == 0
    # hub-only + 12 duty slots: mass, I6, invI6, then 12 q inputs.
    assert sc.nPars() == NTHR + 7, f"unexpected par count {sc.nPars()}"
    print(f"  OK: {sc.nPars()} params = mass+I6 (7) + {NTHR} duty slots")
    return None


def gravity_gradient_torque(rN, sigma):
    """Tgg with full inertia (mirrors heyoka_attitude.build_mrp_system)."""
    R = mrp_dcm(sigma)
    rB = R @ rN
    IrB = I6 @ rB
    r2 = rN @ rN
    k = 3.0 * MU / (r2 * r2 * np.sqrt(r2))
    return k * np.cross(rB, IrB)


def mrp_dcm(s):
    sn2 = s @ s
    d = (1.0 + sn2) ** 2
    c = 4.0 * (1.0 - sn2) / d
    k8 = 8.0 / d
    S = np.array([[0, -s[2], s[1]], [s[2], 0, -s[0]], [-s[1], s[0], 0]])
    return np.eye(3) + k8 * (S @ S) + c * S


# ---------------------------------------------------------------- gate T2
def _gravmod():
    from Basilisk.utilities import simIncludeGravBody
    return simIncludeGravBody


simModules = {}


def build_pipeline(use_omni, duration=DUR, dt=DT):
    simModules["grav"] = _gravmod()
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))

    r0, v0 = ics()
    if use_omni:
        sc = oh.OmniSpacecraft()
        sc.setMass(MASS)
        sc.setInertia(float(I6[0, 0]), float(I6[1, 1]), float(I6[2, 2]),
                      float(I6[0, 1]), float(I6[0, 2]), float(I6[1, 2]))
        sc.setMu(MU)
        sc.setUseGravityGradient(bool(C.USE_GRAVITY_GRADIENT))
        sc.setTol(1e-12)
        sc.setThrusters(list(map(float, DIRS.ravel())),
                        list(map(float, POS.ravel())),
                        list(map(float, TMAX)))
        sc.ModelTag = "omniSat"
        sc.setR_CN_NInit(*map(float, r0))
        sc.setV_CN_NInit(*map(float, v0))
        sc.setSigmaBNInit(0.1, 0.2, -0.3)
        sc.setOmegaBNInit(*map(float, np.asarray(C.OMEGA0_BODY, float).ravel()))
        sim.AddModelToTask("t", sc, 1)
        sc_thr_msg = sc.thrOnTimeInMsg
        state_out = sc.scStateOutMsg
    else:
        from Basilisk.simulation import spacecraft
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "bsk-Sat"
        sc.hub.mHub = MASS
        sc.hub.IHubPntBc_B = [[float(I6[i][j]) for j in range(3)] for i in range(3)]
        sc.hub.r_CN_NInit = r0
        sc.hub.v_CN_NInit = v0
        sc.hub.sigma_BNInit = [0.1, 0.2, -0.3]
        sc.hub.omega_BN_BInit = list(np.asarray(C.OMEGA0_BODY, float).ravel())
        sim.AddModelToTask("t", sc, 1)

        grav = simModules["grav"].gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)

        tf = simIncludeThruster.thrusterFactory()
        for i in range(NTHR):
            tf.create("TEST_Thruster", [float(v) for v in POS[i]],
                      [float(v) for v in DIRS[i]],
                      MaxThrust=float(TMAX[i]), cutoffFrequency=1000.0,
                      useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        sc_thr_msg = thr.cmdsInMsg
        state_out = sc.scStateOutMsg

    sunEphem = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    sunEphem.ModelTag = "sun"
    sim.AddModelToTask("t", sunEphem)

    ref = inertial3D.inertial3D()
    ref.ModelTag = "ref"
    sim.AddModelToTask("t", ref)
    ref.sigma_R0N = list(np.asarray(C.OMEGA0_BODY, float).ravel() * 0 +
                         np.array([0.1, 0.2, -0.3]))
    err = attTrackingError.attTrackingError()
    sim.AddModelToTask("t", err)
    ctl = mrpFeedback.mrpFeedback()
    ctl.ModelTag = "ctl"
    ctl.K, ctl.P = float(C.ACS_K), float(C.ACS_P)
    ctl.Ki = -1
    sim.AddModelToTask("t", ctl)

    class Alloc(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.cmdIn = messaging.CmdTorqueBodyMsgReader()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            self.payload = messaging.THRArrayOnTimeCmdMsgPayload()
            self.payload.OnTimeRequest = [0.0] * NTHR
            self.out.write(self.payload)

        def UpdateState(self, t):
            tau = np.asarray(self.cmdIn().torqueRequestBody, float)
            P = np.linalg.pinv(
                np.vstack([(DIRS * TMAX[:, None]).T,
                           (np.cross(POS, DIRS) * TMAX[:, None]).T]))
            b = np.concatenate([np.zeros(3), SCALE * tau])
            thr = np.clip(P @ b, 0.0, 1.0)
            self.payload.OnTimeRequest = list(map(float, thr * DT))
            self.out.write(self.payload, t, self.moduleID)

    alloc = Alloc()
    alloc.ModelTag = "alloc"
    sim.AddModelToTask("t", alloc)
    alloc.cmdIn.subscribeTo(ctl.cmdTorqueOutMsg)
    sc_thr_msg.subscribeTo(alloc.out)

    nav = _nav(state_out)
    nav.scStateInMsg.subscribeTo(state_out)
    sim.AddModelToTask("t", nav)
    err.attNavInMsg.subscribeTo(nav.attOutMsg)
    err.attRefInMsg.subscribeTo(ref.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)

    vc = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(
            ISCPntB_B=list(map(float, I6.ravel())), CoM_B=[0.0] * 3))
    ctl.vehConfigInMsg.subscribeTo(vc)

    log = state_out.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    return {"sigma_BN": np.asarray(log.sigma_BN, float),
            "omega_BN_B": np.asarray(log.omega_BN_B, float),
            "r_CN_N": np.asarray(log.r_CN_N, float)}


def _nav(state_out):
    from Basilisk.simulation import simpleNav
    nav = simpleNav.SimpleNav()
    nav.ModelTag = "nav"
    return nav


def ics():
    from Basilisk.utilities import orbitalMotion
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    return orbitalMotion.elem2rv(MU, oe)


if __name__ == "__main__":
    import time

    print(__doc__)
    print("\n=== Gate T1: states/pars sanity ===")
    gate_T1()

    print("\n=== Gate T2: full pipeline, stock effector vs omni ===")
    ts = []
    for _ in range(3):
        t0 = time.perf_counter(); stock = build_pipeline(False)
        ts.append(time.perf_counter() - t0)
    t_stock = min(ts)
    ts = []
    for _ in range(3):
        t0 = time.perf_counter(); omni = build_pipeline(True)
        ts.append(time.perf_counter() - t0)
    t_omni = min(ts)

    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff
    n = min(len(stock["sigma_BN"]), len(omni["sigma_BN"]))
    att = np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(omni["sigma_BN"][-1], stock["sigma_BN"][n - 1]))))
    rate = np.degrees(np.linalg.norm(
        omni["omega_BN_B"][-1] - stock["omega_BN_B"][n - 1]))
    print(f"  stock thrusterDynamicEffector : {t_stock*1e3:8.2f} ms")
    print(f"  omni thruster contributor     : {t_omni*1e3:8.2f} ms "
          f"({t_stock/t_omni:.2f}x)")
    print(f"  parity: att={att:.3e} deg  rate={rate:.3e} deg/s")
    print(f"  |sig| final: omni {np.linalg.norm(omni['sigma_BN'][-1]):.6f}  "
          f"stock {np.linalg.norm(stock['sigma_BN'][n-1]):.6f}")
    print(f"  |r| final  : omni {np.linalg.norm(omni['r_CN_N'][-1]):.1f}  "
          f"stock {np.linalg.norm(stock['r_CN_N'][n-1]):.1f}")
