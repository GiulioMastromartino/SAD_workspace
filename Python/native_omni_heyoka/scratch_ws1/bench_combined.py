"""Combined gate (item 5): hub + 3 RW + 12 thrusters vs stock, open-loop gentle.

Both arms: static small motor torques (RW has no staleness rule) + fresh
small on-time commands rewritten every tick (stock fires on fresh messages
only). 2 s @ dt=0.1 — gentle enough that pulsed-vs-averaged second-order
effects stay small; any plumbing/layout error shows up immediately.
"""
from __future__ import annotations

import sys

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)

from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simIncludeGravBody,
    simIncludeRW,
    simIncludeThruster,
)

from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff
from cubesat_12u_sim.backends.heyoka.heyoka_rw import RW_JS, RW_U_MAX
from cubesat_12u_sim.core import config as C

from omniheyoka import omni_heyoka as oh
from omniheyoka.basilisk_cfg import (
    configure_omni_thrusters,
    create_stock_thrusters,
)

MU = float(C.MU_EARTH)
MASS = float(C.MASS)
I6 = np.asarray(C.INERTIA, float)
SPECS = [(list(p), list(d), float(t)) for p, d, t in
         zip(np.asarray(C.THRUSTER_POS, float), np.asarray(C.THRUSTER_DIR, float),
             np.asarray(C.THRUSTER_TMAX, float))]
NTHR = len(SPECS)
DUR, DT = 1.0, 0.1
# Gentle open-loop inputs: small motor torques + a single thruster at 1/6
# duty + SLOW wheels (20/40/60 RPM). Three deliberate choices:
#  - pulsed (not averaged) valve emulation makes trajectories depend on the
#    PWM *frequency* (duty-identical 10 Hz vs 20 Hz pulse trains differ at
#    O(0.1) deg), so stock and omni must be compared at MATCHED dt;
#  - slow wheels keep momentum precession (~1.5 rad/s) resolvable by stock's
#    fixed-step integrator at dt <= 0.1 (fast 100+ RPM wheels need dt << 0.1
#    for stock itself to converge, independent of omni);
#  - gentle torques keep rotation-within-step second-order effects small.
# See BUILD_STATUS Update 5 for the ladders behind these choices.
U_MOTOR = [0.002, -0.001, 0.0015]
Q = np.zeros(NTHR)
Q[0] = 1.0 / 6.0
WHEEL_RPM = [20.0, 40.0, 60.0]


class FreshOnTime(sysModel.SysModel):
    def __init__(self):
        super().__init__()
        self.out = messaging.THRArrayOnTimeCmdMsg()

    def Reset(self, t):
        p = messaging.THRArrayOnTimeCmdMsgPayload()
        p.OnTimeRequest = [0.0] * 36
        self.out.write(p)

    def UpdateState(self, t):
        p = messaging.THRArrayOnTimeCmdMsgPayload()
        ot = [0.0] * 36
        for i in range(NTHR):
            ot[i] = float(Q[i]) * DT
        p.OnTimeRequest = ot
        self.out.write(p, t, self.moduleID)


def build(use_omni, dt=None):
    global DT
    if dt is None:
        dt = DT
    else:
        DT = dt  # FreshOnTime sizes on-times from the run dt (duty = ot/dt)
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)

    motor = messaging.ArrayMotorTorqueMsg()
    mpay = messaging.ArrayMotorTorqueMsgPayload()
    mpay.motorTorque = list(U_MOTOR) + [0.0] * 33
    motor.write(mpay)
    drv = FreshOnTime()

    if use_omni:
        sc = oh.OmniSpacecraft()
        sc.setMass(MASS)
        sc.setInertia(float(I6[0, 0]), float(I6[1, 1]), float(I6[2, 2]),
                      float(I6[0, 1]), float(I6[0, 2]), float(I6[1, 2]))
        sc.setMu(MU)
        sc.setUseGravityGradient(bool(C.USE_GRAVITY_GRADIENT))
        sc.setTol(1e-12)
        for gs in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            sc.addWheel(float(gs[0]), float(gs[1]), float(gs[2]), float(RW_JS))
        configure_omni_thrusters(sc, SPECS)
        sc.ModelTag = "omniSat"
        sc.setR_CN_NInit(*map(float, r0))
        sc.setV_CN_NInit(*map(float, v0))
        sc.setSigmaBNInit(0.1, 0.2, -0.3)
        sc.setOmegaBNInit(0.01, -0.02, 0.03)
        sc.setWheelSpeedsInitRPM(list(WHEEL_RPM))
        sim.AddModelToTask("t", sc, 1)
        assert sc.nThrusters() == NTHR
        sc.rwMotorCmdInMsg.subscribeTo(motor)
        sc.thrOnTimeInMsg.subscribeTo(drv.out)
        out = sc.scStateOutMsg
    else:
        from Basilisk.simulation import spacecraft
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "bsk-Sat"
        sc.hub.mHub = MASS
        sc.hub.IHubPntBc_B = [[float(I6[i][j]) for j in range(3)] for i in range(3)]
        sc.hub.r_CN_NInit = r0
        sc.hub.v_CN_NInit = v0
        sc.hub.sigma_BNInit = [0.1, 0.2, -0.3]
        sc.hub.omega_BN_BInit = [0.01, -0.02, 0.03]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        rwf = simIncludeRW.rwFactory()
        rwf.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=WHEEL_RPM[0],
                   u_max=float(RW_U_MAX))
        rwf.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=WHEEL_RPM[1],
                   u_max=float(RW_U_MAX))
        rwf.create("Honeywell_HR16", [0, 0, 1], maxMomentum=50.0, Omega=WHEEL_RPM[2],
                   u_max=float(RW_U_MAX))
        rw = __import__('Basilisk.simulation.reactionWheelStateEffector',
                        fromlist=['x']).ReactionWheelStateEffector()
        rwf.addToSpacecraft("rw", rw, sc)
        sim.AddModelToTask("t", rw, 3)
        rw.rwMotorCmdInMsg.subscribeTo(motor)
        tf = simIncludeThruster.thrusterFactory()
        create_stock_thrusters(tf, SPECS, cutoffFrequency=1000.0,
                               useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        thr.cmdsInMsg.subscribeTo(drv.out)
        out = sc.scStateOutMsg

    sim.AddModelToTask("t", drv)
    log = out.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(DUR))
    sim.ExecuteSimulation()
    return {"sig": np.asarray(log.sigma_BN, float),
            "om": np.asarray(log.omega_BN_B, float),
            "r": np.asarray(log.r_CN_N, float),
            "v": np.asarray(log.v_CN_N, float)}


if __name__ == "__main__":
    # Matched-dt rows: same PWM frequency both arms. Residual is stock-side
    # switching-integration error, linear in dt (0.80/0.40/0.16 at
    # dt=0.1/0.05/0.02); the dt=0.02 row is the gate, dt=0.1 characterizes.
    for dt, limits in ((0.1, (1.0, 2.0)), (0.02, (0.25, 0.5))):
        s = build(False, dt)
        o = build(True, dt)
        n = min(len(s["sig"]), len(o["sig"]))
        att = np.degrees(4 * np.arctan(np.linalg.norm(mrp_diff(o["sig"][-1], s["sig"][n - 1]))))
        rate = np.degrees(np.linalg.norm(o["om"][-1] - s["om"][n - 1]))
        dr = np.linalg.norm(o["r"][-1] - s["r"][n - 1])
        dv = np.linalg.norm(o["v"][-1] - s["v"][n - 1])
        print(f"combined RW+thruster open-loop {DUR}s @ dt={dt}: att={att:.3e} deg "
              f"rate={rate:.3e} deg/s dr={dr:.3e} m dv={dv:.3e} m/s")
        assert att < limits[0] and rate < limits[1] and dr < 0.05 and dv < 0.02, \
            (dt, att, rate, dr, dv)
    print("OK: combined gate")
