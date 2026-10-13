"""WP25-B2 SysModel + vec guards (B2.1 hijack, B2.2 sign, B2.4 stochastic/freeze).

Host-only sims (Basilisk + built ``omni_heyoka``) via importorskip: skipped
(no failure) on containers without them. Gated tests are the evidence; the
C++ paths are cited per test (``src/omni_spacecraft.cpp`` /
``src/omni_vec_integrator.cpp``).

Covers:
* B2.1: linked ``RWArrayConfigMsg`` raises loudly (``checkB2Guards``);
  unlinked (disarmed) configs run.
* B2.2: ``setFrictionHubSign(-1)`` raises (graph bakes +1); default runs.
* B2.4: diffusion-enabled stock state raises vec-side at setup
  (``OmniVecIntegrator::buildOnce`` ``getNumNoiseSources`` gate -- the
  state-name check is insufficient by construction); normal stop/restart
  with an unchanged census runs (freeze without false positives).
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

MU = 3.986004418e14
MASS = 750.0
IXX, IYY, IZZ = 900.0, 800.0, 600.0
DT = 0.1


def _fresh_hub_wheel():
    """Minimal hub + 1-wheel omni sim. Returns (sim, sc, macros)."""
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))

    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)

    sc = oh.OmniSpacecraft()
    sc.setMass(MASS)
    sc.setInertia(IXX, IYY, IZZ)
    sc.setMu(MU)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.addWheel(1.0, 0.0, 0.0, 0.07957747154594767, 0.2)
    sc.ModelTag = "wp25b2_guard"
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sc.setWheelSpeedsInitRPM([0.0])
    sim.AddModelToTask("t", sc)
    return sim, sc, macros


def _run_two_ticks(sim, macros):
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(DT * 2))
    sim.ExecuteSimulation()


def _need_api(sc, *names):
    missing = [n for n in names if not callable(getattr(sc, n, None))
               and not hasattr(sc, n)]
    if missing:
        pytest.skip(f"WP25-B2 C++ spec not yet applied: {missing} missing")


# --- B2.1: RWArrayConfigMsg hijack ------------------------------------------

def test_b2_rwarrayconfig_linked_raises_and_unlinked_runs():
    """B2.1: linked RWArrayConfigMsg raises pre-propagation (checkB2Guards).

    Geometry comes from addWheel/addRatedWheel only (Omega_max setter-only);
    a linked FSW config message would silently disagree, so it is rejected.
    """
    from Basilisk.architecture import messaging
    sim, sc, _ = _fresh_hub_wheel()
    _need_api(sc, "rwArrayConfigInMsg")
    cfg_msg = messaging.RWArrayConfigMsg()
    cfg_msg.write(messaging.RWArrayConfigMsgPayload())
    sc.rwArrayConfigInMsg.subscribeTo(cfg_msg)
    with pytest.raises(Exception, match="RWArrayConfigMsg"):
        sim.InitializeSimulation()
    # Disarmed: unlinked config message runs.
    sim2, _, macros2 = _fresh_hub_wheel()
    _run_two_ticks(sim2, macros2)


# --- B2.2: friction hub-sign pin ---------------------------------------------

def test_b2_friction_hub_sign_minus_one_raises():
    """B2.2 (H8): s = -1 raises at the setter (graph bakes +1, unthreaded).

    The old setter silently computed +1 physics for a -1 request; now the
    request is loud. Default (+1) configs run (see the disarmed guard suite).
    """
    _, sc, _ = _fresh_hub_wheel()
    _need_api(sc, "setFrictionHubSign")
    with pytest.raises(Exception, match="[Hh]ub sign"):
        sc.setFrictionHubSign(-1.0)
    sc.setFrictionHubSign(1.0)  # pinned value accepted
    sim2, _, macros2 = _fresh_hub_wheel()
    _run_two_ticks(sim2, macros2)


# --- B2.4: stochastic/diffusion + freeze (vec path) --------------------------

def _hub_only_stock_sim(tag, omega0=(0.0, 0.0, 0.0)):
    """Hub-only stock sim (central earth); caller mirrors + integrates."""
    pytest.importorskip("Basilisk.architecture.messaging")
    pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                    simIncludeGravBody)

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[IXX, 0.0, 0.0], [0.0, IYY, 0.0], [0.0, 0.0, IZZ]]
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R, 347.8 * macros.D2R,
                                85.3 * macros.D2R)
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = list(omega0)
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    return sim, sc, macros


def test_b24_vec_diffusion_enabled_raises():
    """B2.4: diffusion-enabled stock state raises vec-side before propagation.

    Code path: ``OmniVecIntegrator::buildOnce`` inspects
    ``StateData::getNumNoiseSources()`` on every claimed state (the
    state-name claim check passes -- diffusion attaches to the SAME names).
    """
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config

    sim, sc, macros = _hub_only_stock_sim("b24_diffusion")
    oi = OmniVecIntegrator(sc)
    mirror_stock_config(sim, sc, oi)
    sc.setIntegrator(oi)
    try:
        hub_omega = sc.dynManager.getStateObject("hubOmega")
        hub_omega.setNumNoiseSources(1)
    except AttributeError as exc:
        pytest.skip(f"stock diffusion API not visible from Python: {exc}")
    if not callable(getattr(oi, "getMRPSwitchCount", None)):
        pytest.skip("WP25-B2.5 C++ spec not yet applied")
    # buildOnce (with the diffusion gate) runs at first integrate -- during
    # init or the first step depending on the stock version; cover both.
    with pytest.raises(Exception, match="[Dd]iffusion"):
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * 2))
        sim.ExecuteSimulation()


def test_b24_vec_freeze_stop_restart_clean():
    """B2.4: stop/restart with an unchanged census runs (no false positive).

    Records freeze at build (first integrate); the per-tick census compare
    in ``integrate`` must pass silently when nothing changed. Stock-side
    edits (new/removed states, second object) raise -- code path cited,
    live double-object attach is a host-manual cell (leftover).
    """
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config

    sim, sc, macros = _hub_only_stock_sim("b24_freeze")
    oi = OmniVecIntegrator(sc)
    mirror_stock_config(sim, sc, oi)
    sc.setIntegrator(oi)
    if not callable(getattr(oi, "getMRPSwitchCount", None)):
        pytest.skip("WP25-B2.5 C++ spec not yet applied")
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(DT * 5))
    sim.ExecuteSimulation()
    # Unchanged census across a stop/restart: must NOT raise.
    sim.ConfigureStopTime(macros.sec2nano(DT * 10))
    sim.ExecuteSimulation()
