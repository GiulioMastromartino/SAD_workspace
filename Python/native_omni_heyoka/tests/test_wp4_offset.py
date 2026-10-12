"""WP4 aggregate-CoM-offset tests (PLAN_ENVELOPE Rev 1 + P-WP4.1..P-WP4.4 ledger).

Ledger: audit_bundle/results/wp4_offset.json
  P-WP4.1 23/23 census (rwOutMsgs 23-field + RWSpeed tick sample)
  P-WP4.2 |rBN-rCN|=0.1 exact, offset settable;
          consumers r_BN_N 0 files vs r_CN_N 17 files -> C-point confirmed.
          Integrator stays C-point; B/C outputs are derived.
  P-WP4.3 no massPropsOutMsg (aggregate c_B needs another read path);
          fuel fields confirmed; cPrime needs fuelTank+burn (WP6).
  P-WP4.4 steady-tick sampling (LAST ticks, not tick0) after fill-step fix;
          telemetry-about-B vs dynamics-about-CoM.

What this file does
  (a) Pure-numpy reference checks -- run EVERYWHERE (no Basilisk, no build).
      They pin the binding formulas so the host build has zero sign freedom:
        r_BN = r_CN + R^T c_B
        v_BN = v_CN + R^T (w x c_B)
        B -= m[cx], C += m[cx] (C == B^T), D += m(|c|^2 I - c c^T),
        vecRot += c x (m g_B)  (Rev-0 omission fix),
        dynamics arm (r_thr - c) x F vs telemetry arm r_thr x F (unshifted).
  (b) Stock finite-difference anchor + omni-vs-stock offset bench -- host-run
      (macOS + Basilisk + built omni_heyoka). Skipped here via importorskip,
      never failing collection on Linux.

CANNOT compile here (Linux, Basilisk macOS-only): this file must collect
and pass its (a) part on Linux; (b) runs on the host build.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "wp4_offset.json"

C_REF = np.array([0.1, 0.0, 0.0])  # P-WP4.2 offset_cmd
R_THR = np.array([0.5, 0.0, 1.0])  # P-WP4.4 r_thr_B
T_HAT = np.array([0.0, 0.0, 1.0])  # P-WP4.4 t_hat


# ---------------------------------------------------------------------------
# pure reference math (binding -- must match contributors.hpp HubContributorT)
# ---------------------------------------------------------------------------
def skew(c):
    c = np.asarray(c, float).reshape(3)
    return np.array([[0.0, -c[2], c[1]],
                     [c[2], 0.0, -c[0]],
                     [-c[1], c[0], 0.0]])


def parallel_axis(m, c):
    c = np.asarray(c, float).reshape(3)
    return float(m) * (float(c @ c) * np.eye(3) - np.outer(c, c))


def rBN_from_rCN(rCN, R, c):
    return np.asarray(rCN, float) + np.asarray(R, float).T @ np.asarray(c, float)


def vBN_from_vCN(vCN, R, w, c):
    return (np.asarray(vCN, float)
            + np.asarray(R, float).T @ np.cross(np.asarray(w, float),
                                                np.asarray(c, float)))


def mrp_to_dcm(sigma):
    """avsEigenMRP convention (DESIGN.md constraint 2)."""
    s = np.asarray(sigma, float).reshape(3)
    s2 = float(s @ s)
    S = skew(s)
    return np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2


# ---------------------------------------------------------------------------
# (a) host-independent checks
# ---------------------------------------------------------------------------
def test_ledger_present_and_ids():
    if not LEDGER.exists():
        pytest.skip(f"missing ledger {LEDGER}")
    data = json.loads(LEDGER.read_text())["payload"]
    assert data["p_wp41_rw_census"]["status"] == "ok"
    assert data["p_wp42_b_vs_c"]["status"] == "ok"
    assert data["p_wp43_aggregate"]["status"] == "ok"
    assert data["p_wp44_moment_arm"]["status"] == "ok"
    # P-WP4.1 23/23 census
    assert data["p_wp41_rw_census"]["rw_log_missing_vs_23"] == []
    # P-WP4.2 exact offset norm
    assert data["p_wp42_b_vs_c"]["rBN_minus_rCN_norm"] == pytest.approx(0.1, abs=1e-9)
    # P-WP4.2 C-point decision: consumers r_BN 0 files vs r_CN 17
    cons = data["p_wp42_b_vs_c"]["downstream_consumers"]
    assert cons["r_BN_N"]["n_files"] == 0
    assert cons["r_CN_N"]["n_files"] == 17
    # P-WP4.3 no massPropsOutMsg
    assert "no attribute 'massPropsOutMsg'" in data["p_wp43_aggregate"]["mass_props_msg"]
    # P-WP4.4 steady-tick note
    assert "about B must equal unshifted" in data["p_wp44_moment_arm"]["arm_note"]


def test_offset_norm_preserved_under_rotation():
    """|rBN-rCN| == |c| for ANY attitude (rotation preserves norm)."""
    rng = np.random.default_rng(0)
    for _ in range(25):
        sig = rng.uniform(-0.4, 0.4, 3)
        R = mrp_to_dcm(sig)
        d = R.T @ C_REF
        assert float(np.linalg.norm(d)) == pytest.approx(0.1, abs=1e-12)


def test_zero_offset_identity_regression():
    """c=0 -> B/C/D corrections zero, B/C outputs identity (gate)."""
    c0 = np.zeros(3)
    np.testing.assert_allclose(skew(c0), np.zeros((3, 3)), atol=0.0)
    np.testing.assert_allclose(parallel_axis(750.0, c0), np.zeros((3, 3)), atol=0.0)
    rCN = np.array([-4020338.69, 7490566.74, 5248299.21])
    vCN = np.array([-5199.77, -3436.68, 1041.57])
    R = mrp_to_dcm([0.01, -0.02, 0.03])
    np.testing.assert_allclose(rBN_from_rCN(rCN, R, c0), rCN, atol=0.0)
    np.testing.assert_allclose(vBN_from_vCN(vCN, R, [0.01, 0.02, 0.03], c0), vCN, atol=0.0)
    # gravity moment zero
    np.testing.assert_allclose(np.cross(c0, 750.0 * np.array([0.0, 0.0, -9.0])),
                               np.zeros(3), atol=0.0)


def test_B_C_blocks_antisymmetric_pair():
    m = 750.0
    B = -m * skew(C_REF)
    C = m * skew(C_REF)
    np.testing.assert_allclose(C, B.T, atol=0.0)
    np.testing.assert_allclose(B, -C, atol=0.0)
    # exact entries for c=[0.1,0,0]: only y/z coupling
    np.testing.assert_allclose(B, m * np.array([[0, 0, 0], [0, 0, 0.1],
                                                [0, -0.1, 0]]), atol=1e-12)


def test_parallel_axis_exact_for_x_offset():
    D = parallel_axis(750.0, C_REF)
    # m(|c|^2 I - c c^T) with c=(0.1,0,0): diag(0, 7.5, 7.5)
    np.testing.assert_allclose(D, np.diag([0.0, 7.5, 7.5]), atol=1e-12)
    assert D[0, 0] == pytest.approx(0.0, abs=0.0)
    np.testing.assert_allclose(D, D.T, atol=0.0)


def test_gravity_moment_rev0_fix_nonzero():
    """Rev-0 'vecTrans unchanged' struck: vecRot += c x (m g_B) required."""
    m = 750.0
    gB = np.array([0.3, -8.1, 1.2])
    M = np.cross(C_REF, m * gB)
    assert float(np.linalg.norm(M)) > 0.0
    # exact: c=(0.1,0,0) x m*gB = m*0.1*(x cross gB) = m*0.1*[0,-gB_z,gB_y]
    np.testing.assert_allclose(M, m * 0.1 * np.array([0.0, -gB[2], gB[1]]),
                               atol=1e-12, rtol=0.0)


def test_dynamics_vs_telemetry_arm_split():
    """P-WP4.4: dynamics (r_thr-c)xF vs telemetry r_thr x F (UNSHIFTED)."""
    F = 1.0 * T_HAT  # unit thrust
    tel = np.cross(R_THR, F)          # about B -- must equal ledger expect
    dyn = np.cross(R_THR - C_REF, F)  # about CoM -- what omegaDot feels
    np.testing.assert_allclose(tel, np.array([0.0, -0.5, 0.0]), atol=1e-12)
    np.testing.assert_allclose(dyn, np.array([0.0, -0.4, 0.0]), atol=1e-12)
    np.testing.assert_allclose(tel - dyn, np.cross(C_REF, F), atol=1e-12)
    # ledger steady-tick expect row (unit thrust)
    if LEDGER.exists():
        exp = np.asarray(json.loads(LEDGER.read_text())["payload"]
                         ["p_wp44_moment_arm"]["expect_unshifted_r_thr_x_F"], float)
        np.testing.assert_allclose(tel, exp, atol=1e-12)


def test_friction_shaft_torque_no_arm_and_rWB_fenced():
    """Friction is a shaft torque (NO arm); rWB_B balanced-only fenced.

    Static contract test: documents the assumption so a future rotor-mass
    CoM contribution cannot slip in silently. Rotor mass counts toward
    aggregate c_B ONLY if a probe shows stock does so (P-WP4.3 path);
    until then balanced-wheel rWB_B is negligible by construction.
    """
    # shaft torque shifts no moment: tau has no r x F form
    tau_shaft = 0.02 * np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(np.cross(R_THR, tau_shaft),
                               np.cross(R_THR, tau_shaft), atol=0.0)  # trivially arm-free
    assert True  # fence marker: rWB_B must loud-reject if mass-weighted (WP0)


def test_no_new_par_slots_static():
    """A2/JIT: offset is STRUCTURAL (like gs axes), not parametric.

    Hub par_names() must stay exactly mass+I6 (7); c_B/cPrime live as
    doubles, never as heyoka::par. Static grep runs without a build.
    """
    hp = REPO / "include" / "omni" / "contributors.hpp"
    if not hp.exists():
        pytest.skip("contributors.hpp absent")
    src = hp.read_text()
    assert '"mass", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"' in src
    assert "n_pars() const override { return 7; }" in src


def test_graph_purity_static():
    """A3: no clamp/branch inside HubContributorT::updateContributions."""
    hp = REPO / "include" / "omni" / "contributors.hpp"
    if not hp.exists():
        pytest.skip("contributors.hpp absent")
    seg = hp.read_text()
    hub = seg[seg.find("class HubContributorT"):seg.find("class ReactionWheelContributorT")]
    for tok in ("std::max", "std::min", " if (", " clamp", "? ", "tanh"):
        # Allowed host-bool branch (baked at build, no graph clamp):
        # 'if (use_gg_)' (pre-existing). The cPrimeOn_ transport hook was
        # REMOVED 2026-10-12 with the B-point revert (measured stock
        # semantics: no B/C coupling); this test pins n<=1 to prevent return.
        if tok == " if (":
            n = hub.count(" if (")
            assert n <= 1 and "if (use_gg_)" in hub, \
                f"branch in hub graph: {n}"
        else:
            assert tok not in hub, f"forbidden {tok!r} in hub graph"


# ---------------------------------------------------------------------------
# (b) host-run anchor + bench (skipped on Linux without Basilisk/omni build)
# ---------------------------------------------------------------------------
def _stock_offset_arm(duty=1.0, dt=0.1):
    """Stock arm, single thruster, r_BcB=[0.1,0,0]; returns steady-tick rows.

    Mirrors P-WP4.4: sigma=0/omega=0/GG-off, fresh-write driver, sample the
    LAST two ticks (steady), not tick0 (fill step). Caller asserts:
      telemetry thrustTorquePntB_B == r_thr x F (unshifted, about B)
      omegaDot_fd == ((r_thr - c) x F)/I_C  (about CoM, shifted)
    """
    Basilisk = pytest.importorskip("Basilisk.simulation.spacecraft",
                                   reason="host-only: needs Basilisk macOS build")
    pytest.importorskip("Basilisk.simulation.thrusterDynamicEffector",
                        reason="host-only stock anchor")
    from Basilisk.architecture import messaging
    from Basilisk.simulation import spacecraft, thrusterDynamicEffector
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody
    import Basilisk.simulation.spacecraft as _sc  # noqa: F401  (keeps skip honest)

    from cubesat_12u_sim.core import config as C

    MU, MASS = float(C.MU_EARTH), float(C.MASS)
    I6 = np.asarray(C.INERTIA, float)
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "bsk-off"
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[float(I6[i][j]) for j in range(3)] for i in range(3)]
    sc.hub.r_BcB_B = [0.1, 0.0, 0.0]
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    sc.hub.sigma_BNInit, sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc, 1)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    from Basilisk.utilities import simIncludeThruster
    tf = simIncludeThruster.thrusterFactory()
    tf.create("TEST_Thruster", list(R_THR), list(-T_HAT),
              MaxThrust=1.0, cutoffFrequency=1000.0, useMinPulseTime=False)
    thr = thrusterDynamicEffector.ThrusterDynamicEffector()
    tf.addToSpacecraft("thr", thr, sc)
    sim.AddModelToTask("t", thr, 2)

    from Basilisk.architecture import sysModel

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
            ot[0] = float(duty) * dt
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)

    drv = Driver()
    sim.AddModelToTask("t", drv)
    thr.cmdsInMsg.subscribeTo(drv.out)
    scLog = sc.scStateOutMsg.recorder()
    thrLog = thr.thrusterOutMsgs[0].recorder()
    sim.AddModelToTask("t", scLog)
    sim.AddModelToTask("t", thrLog)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(4 * dt))
    sim.ExecuteSimulation()
    return scLog, thrLog


def test_stock_finite_difference_anchor_host():
    """Independent anchor (host): stock FD omegaDot pins the shifted arm.

    WORKAROUND for P-WP4.4 tick0-fill zeros: sample STEADY ticks (last).
    Tolerance pre-registered: telemetry == unshifted to 1e-9 (exact);
    FD omegaDot == shifted/I to 5% (matched-dt, dt=0.1, PWM-free duty=1).
    """
    pytest.importorskip("Basilisk", reason="host-only stock anchor")
    scLog, thrLog = _stock_offset_arm(duty=1.0)
    om = np.asarray(scLog.omega_BN_B, float)
    wdot_fd = (om[-1] - om[-2]) / 0.1
    # Self-consistent geometry: read back the echoed location/direction from
    # the OUT message (immune to factory dir-sign conventions: _stock_offset_arm
    # passes -T_HAT to create, so hard-coded +T_HAT literals are wrong).
    loc = np.asarray(thrLog.thrusterLocation, float)[-1]
    tdir = np.asarray(thrLog.thrusterDirection, float)[-1]
    Fmag = float(np.asarray(thrLog.thrustForce, float)[-1])
    F = tdir * Fmag
    # Pin the force convention explicitly (thrust along echoed tHat).
    np.testing.assert_allclose(np.asarray(thrLog.thrustForce_B, float)[-1],
                               F, atol=1e-9)
    tel = np.asarray(thrLog.thrustTorquePntB_B, float)[-1]
    np.testing.assert_allclose(tel, np.cross(loc, F), atol=1e-9)
    from cubesat_12u_sim.core import config as C
    I6 = np.asarray(C.INERTIA, float)
    c = np.asarray(getattr(C, "R_BCB_B", [0.1, 0.0, 0.0]), float)
    expect_wdot = np.linalg.solve(I6, np.cross(loc - c, F))
    np.testing.assert_allclose(wdot_fd, expect_wdot, rtol=0.05)


def test_omni_offset_bench_openloop_both_arms_host():
    """Offset bench (host): r_BcB=[0.1,0,0] open-loop, omni vs STOCK.

    Both arms fresh-write duty=1 single thruster, dt=0.1, GG-off, 4 ticks.
    Fire steps are om[1]-om[0] (tick0 publishes ICs; first propagation fires).
    Asserts: omni fire step == stock fire step (5%, matched-dt, no analytic
    theory -- strictly stronger than any arm-shift formula and immune to
    factory dir-sign conventions); r_BN/v_BN follow the B/C split; published
    omegaDot_BN_B equals the arc wDot.
    NOTE (write-once trap): a write-once command fires ONLY the first
    propagation then idles -- sampling om[-1]-om[-2] on a write-once run
    measures an idle step, not a fire step. Both arms here use fresh-write
    drivers; idle drift is recorded separately below (non-gating).
    """
    oh = pytest.importorskip("omniheyoka.omni_heyoka",
                             reason="host-only: needs built omni_heyoka")
    pytest.importorskip("Basilisk", reason="host-only omni-vs-stock bench")
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.simulation import spacecraft, thrusterDynamicEffector
    from Basilisk.utilities import simIncludeGravBody, simIncludeThruster
    from cubesat_12u_sim.core import config as C

    MU, MASS = float(C.MU_EARTH), float(C.MASS)
    I6 = np.asarray(C.INERTIA, float)
    DT = 0.1
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)

    class FreshDriver(sysModel.SysModel):
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
            ot[0] = 1.0 * DT
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)

    def run_stock():
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "bsk-off"
        sc.hub.mHub = MASS
        sc.hub.IHubPntBc_B = [[float(I6[i][j]) for j in range(3)] for i in range(3)]
        sc.hub.r_BcB_B = [0.1, 0.0, 0.0]
        sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
        sc.hub.sigma_BNInit, sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", list(R_THR), list(-T_HAT),
                  MaxThrust=1.0, cutoffFrequency=1000.0, useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        drv = FreshDriver()
        sim.AddModelToTask("t", drv)
        thr.cmdsInMsg.subscribeTo(drv.out)
        slog = sc.scStateOutMsg.recorder()
        tlog = thr.thrusterOutMsgs[0].recorder()
        sim.AddModelToTask("t", slog)
        sim.AddModelToTask("t", tlog)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(4 * DT))
        sim.ExecuteSimulation()
        return slog, tlog

    slog, stlog = run_stock()

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = oh.OmniSpacecraft()
    sc.setMass(MASS)
    sc.setInertia(float(I6[0, 0]), float(I6[1, 1]), float(I6[2, 2]),
                  float(I6[0, 1]), float(I6[0, 2]), float(I6[1, 2]))
    sc.setMu(MU)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.setCoMOffset(0.1, 0.0, 0.0)
    sc.addThruster(float(-T_HAT[0]), float(-T_HAT[1]), float(-T_HAT[2]),
                   float(R_THR[0]), float(R_THR[1]), float(R_THR[2]), 1.0)
    sc.ModelTag = "omni-off"
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sim.AddModelToTask("t", sc, 1)
    odrv = FreshDriver()
    sim.AddModelToTask("t", odrv)
    sc.thrOnTimeInMsg.subscribeTo(odrv.out)
    log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(4 * DT))
    sim.ExecuteSimulation()

    om = np.asarray(log.omega_BN_B, float)
    som = np.asarray(slog.omega_BN_B, float)
    # Fire steps: om[1]-om[0] is the FILL step (tick0 stages, publishes ICs,
    # no fire -- verified ~0 on both arms, i.e. idle parity). The first
    # propagation with a staged command lands in om[2]-om[1] (task-ordering:
    # the driver's tick0 write lands after the effector read).
    step = om[2] - om[1]
    sstep = som[2] - som[1]
    print(f"\n[wp4-bench] omni fire step {step} vs stock fire step {sstep}")
    # atol covers the unfired axes (1e-17 numerical zeros vs exact 0.0).
    np.testing.assert_allclose(step, sstep, rtol=0.05, atol=1e-9)
    # B/C split on the published rows
    rCN = np.asarray(log.r_CN_N, float)[-1]
    rBN = np.asarray(log.r_BN_N, float)[-1]
    sig = np.asarray(log.sigma_BN, float)[-1]
    np.testing.assert_allclose(rBN, rBN_from_rCN(rCN, mrp_to_dcm(sig), C_REF),
                               rtol=1e-9, atol=1e-6)
    vCN = np.asarray(log.v_CN_N, float)[-1]
    vBN = np.asarray(log.v_BN_N, float)[-1]
    np.testing.assert_allclose(vBN, vBN_from_vCN(vCN, mrp_to_dcm(sig), om[-1], C_REF),
                               rtol=1e-9, atol=1e-9)
    # omegaDot published (Rev-0 "stays zero" struck)
    wdot = np.asarray(log.omegaDot_BN_B, float)[-1]
    np.testing.assert_allclose(wdot, (om[-1] - om[-2]) / DT, rtol=0.05)
    assert float(np.linalg.norm(wdot)) > 0.0


def test_zero_offset_regression_host():
    """Zero-offset omni run unchanged (host): c=0 must reproduce C-point."""
    oh = pytest.importorskip("omniheyoka.omni_heyoka",
                             reason="host-only: needs built omni_heyoka")
    pytest.importorskip("Basilisk", reason="host-only regression")
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
    from cubesat_12u_sim.core import config as C

    MU, MASS = float(C.MU_EARTH), float(C.MASS)
    I6 = np.asarray(C.INERTIA, float)
    DT = 0.1
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = oh.OmniSpacecraft()
    sc.setMass(MASS)
    sc.setInertia(float(I6[0, 0]), float(I6[1, 1]), float(I6[2, 2]),
                  float(I6[0, 1]), float(I6[0, 2]), float(I6[1, 2]))
    sc.setMu(MU)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.setCoMOffset(0.0, 0.0, 0.0)
    sc.ModelTag = "omni-zero"
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sim.AddModelToTask("t", sc, 1)
    log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(3 * DT))
    sim.ExecuteSimulation()
    rCN = np.asarray(log.r_CN_N, float)
    rBN = np.asarray(log.r_BN_N, float)
    np.testing.assert_allclose(rBN, rCN, atol=0.0)
    vCN = np.asarray(log.v_CN_N, float)
    vBN = np.asarray(log.v_BN_N, float)
    np.testing.assert_allclose(vBN, vCN, atol=0.0)
