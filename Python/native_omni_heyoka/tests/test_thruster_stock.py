"""Stock ground-truth lock-ins for the omni thruster channel.

These tests pin the three measured stock behaviors the C++ channel
reproduces (see BUILD_STATUS Update 5 for the probe record):

* staircase edges: achieved duty vs commanded duty, including exact 0.5/1.0
  boundaries, over/negative commands and infinitesimal commands;
* no valve lag: full-step firing achieves exactly duty 1 (deficit 0);
* stale-idle: a command message written once fires a single step; later
  steps with no fresh command go idle (both arms).

Plus a round-trip test for the single-source geometry helper
(omniheyoka.basilisk_cfg).

Each stock probe is a 2-tick sim (~0.5 s wall time); the whole file runs in
seconds. Run from Python/: ./env/bin/python -m pytest native_omni_heyoka/tests -q
"""
import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    simIncludeGravBody,
    simIncludeThruster,
)

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.core.actuators import quantize_duty_cycle

MU = float(C.MU_EARTH)
MASS = float(C.MASS)
I6 = np.asarray(C.INERTIA, float)
DT = 0.1


def _one_thruster_sim(duty_cmd, dt=DT, writer="fresh"):
    """Stock arm, single thruster (idx 0), sigma=0/omega=0/GG-off.

    Returns (dv, dom) of the SECOND propagation step. With writer="fresh"
    the driver rewrites the same command every tick; with "once" it writes
    once before the sim (stale afterwards).
    """
    from Basilisk.architecture import sysModel

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import orbitalMotion
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[float(I6[i][j]) for j in range(3)] for i in range(3)]
    sc.hub.r_CN_NInit = list(r0)
    sc.hub.v_CN_NInit = list(v0)
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc, 1)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    tf = simIncludeThruster.thrusterFactory()
    tf.create("TEST_Thruster", [0.1, 0.1, 0.0], [-1, 0, 0],
              MaxThrust=1.0, cutoffFrequency=1000.0, useMinPulseTime=False)
    thr = thrusterDynamicEffector.ThrusterDynamicEffector()
    tf.addToSpacecraft("thr", thr, sc)
    sim.AddModelToTask("t", thr, 2)

    if writer == "fresh":
        class Driver(sysModel.SysModel):
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
                ot[0] = float(duty_cmd) * dt
                p.OnTimeRequest = ot
                self.out.write(p, t, self.moduleID)

        drv = Driver()
        sim.AddModelToTask("t", drv)
        thr.cmdsInMsg.subscribeTo(drv.out)
    else:
        drv = messaging.THRArrayOnTimeCmdMsg()
        pay = messaging.THRArrayOnTimeCmdMsgPayload()
        ot = [0.0] * 36
        ot[0] = float(duty_cmd) * dt
        pay.OnTimeRequest = ot
        drv.write(pay)
        thr.cmdsInMsg.subscribeTo(drv)

    log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(3 * dt))
    sim.ExecuteSimulation()
    v = np.asarray(log.v_CN_N, float)
    om = np.asarray(log.omega_BN_B, float)
    return v, om


def _achieved_torque(duty_cmd):
    """Achieved duty from the z-torque step (thruster 0 gives T=[0,0,+0.1];
    returns the negative staircase branch to match the measured sign)."""
    v, om = _one_thruster_sim(duty_cmd, writer="fresh")
    dom = om[2] - om[1]
    return -dom[2] * I6[2, 2] / (0.1 * DT)


@pytest.mark.parametrize("duty", [0.0, 1e-12, 0.02, 0.49, 0.5, 0.9, 0.9999999, 1.0, 1.5, -0.1])
def test_staircase_edges_match_stock(duty):
    expect = -float(quantize_duty_cycle(np.clip(np.array([duty]), 0.0, 1.0))[0])
    got = _achieved_torque(duty)
    assert got == pytest.approx(expect, abs=1e-4), (duty, got, expect)


def test_no_valve_lag():
    assert _achieved_torque(1.0) == pytest.approx(-1.0, abs=1e-6)


def test_stale_message_goes_idle():
    # Edge-triggered consumption, verified matrix (stock == omni):
    # fresh/advancing commands idle the fill step then fire every step;
    # a write-once command fires the first propagation then idles.
    # Sample[0]=t0 ICs; deltas[0]=fill step, deltas[1:]=propagations.
    _, om_fresh = _one_thruster_sim(5.0 / 6.0, writer="fresh")
    d = [om_fresh[k] - om_fresh[k - 1] for k in range(1, 4)]
    assert np.linalg.norm(d[0]) < 0.05 * np.linalg.norm(d[1])
    np.testing.assert_allclose(d[2], d[1], rtol=0.05)
    _, om_stale = _one_thruster_sim(5.0 / 6.0, writer="once")
    s = [om_stale[k] - om_stale[k - 1] for k in range(1, 4)]
    np.testing.assert_allclose(s[0], d[1], rtol=0.05)  # write-once fires once
    assert np.linalg.norm(s[1]) < 0.05 * np.linalg.norm(s[0])  # then idles
    assert np.linalg.norm(s[2]) < 0.05 * np.linalg.norm(s[0])


def test_omni_matches_stock_stale_path():
    """Omni applies the same edge-triggered rule (needs the built wheel)."""
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    from omniheyoka.basilisk_cfg import configure_omni_thrusters

    DIRS = np.asarray(C.THRUSTER_DIR, float)
    POS = np.asarray(C.THRUSTER_POS, float)
    TMAX = np.asarray(C.THRUSTER_TMAX, float)
    specs = [(list(p), list(d), float(t)) for p, d, t in zip(POS, DIRS, TMAX)]

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    from Basilisk.utilities import orbitalMotion
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc = oh.OmniSpacecraft()
    sc.setMass(MASS)
    sc.setInertia(float(I6[0, 0]), float(I6[1, 1]), float(I6[2, 2]),
                  float(I6[0, 1]), float(I6[0, 2]), float(I6[1, 2]))
    sc.setMu(MU)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    configure_omni_thrusters(sc, specs[:1])
    sc.ModelTag = "omniSat"
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sim.AddModelToTask("t", sc, 1)
    drv = messaging.THRArrayOnTimeCmdMsg()
    pay = messaging.THRArrayOnTimeCmdMsgPayload()
    ot = [0.0] * 36
    ot[0] = (5.0 / 6.0) * DT
    pay.OnTimeRequest = ot
    drv.write(pay)
    sc.thrOnTimeInMsg.subscribeTo(drv)
    log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(3 * DT))
    sim.ExecuteSimulation()
    om = np.asarray(log.omega_BN_B, float)
    o1 = om[1] - om[0]
    o2 = om[2] - om[1]
    # single thruster 0 at 5/6 duty: T=[0,0,+0.1], fire-once then idle
    expect_fire = np.array([0.0, 0.0, (5.0 / 6.0) * 0.1 / I6[2, 2] * DT])
    np.testing.assert_allclose(o1, expect_fire, rtol=0.02)
    assert np.linalg.norm(o2) < 0.05 * np.linalg.norm(o1)


def test_geometry_helper_round_trip():
    from omniheyoka.basilisk_cfg import (
        configure_omni_thrusters,
        create_stock_thrusters,
        omni_thruster_arrays,
        thruster_specs_from_factory,
    )

    specs = [([0.1, 0.1, 0.0], [-1, 0, 0], 1.0),
             ([0.0, -0.1, 0.15], [0, 1, 0], 2.0)]
    tf = simIncludeThruster.thrusterFactory()
    create_stock_thrusters(tf, specs, cutoffFrequency=1000.0, useMinPulseTime=False)
    back = thruster_specs_from_factory(tf)
    assert len(back) == 2
    for (p0, d0, t0), (p1, d1, t1) in zip(specs, back):
        np.testing.assert_allclose(p1, p0, atol=0.0)
        np.testing.assert_allclose(d1, d0, atol=1e-12)
        assert t1 == pytest.approx(t0)
    dirs, pos, tmax = omni_thruster_arrays(specs)
    assert len(dirs) == len(pos) == 6 and len(tmax) == 2

    class FakeModule:
        def __init__(self):
            self.args = None

        def setThrusters(self, dirs, pos, tmax):
            self.args = (dirs, pos, tmax)

    m = FakeModule()
    configure_omni_thrusters(m, specs)
    assert m.args == (dirs, pos, tmax)
