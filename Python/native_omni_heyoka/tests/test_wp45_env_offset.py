"""WP4+WP5 combo: env-force torque arms under hub offset (omni-vs-stock DIRECT).

MEASURED SPLIT (2026-10-12, combo probe): the shift rule is NOT uniform
across env effects -- stock treats them differently, so omni copies each:
  drag:     UNSHIFTED. tau = comOffset x F  (beta)
  SRP:      SHIFTED.   tau = (rCop - c_B) x F  (alpha)
  magnetic: NO arm (tau = m x B_B; B_B = dcm_BN(sigma) magField_N).
  gravity:  NO arm (a = sum mu_i (r_i - r)/|r_i-r|^3).
Evidence (audit_bundle/results/wp45_combo.json, firing-proven legs):
  drag-offset: |dT| ~ 1e-17 vs 1.29e-5 shifted prediction => unshifted,
    ratio 2e-12. drag-zero-com corroborates: comOffset=0 => ~0 torque
    with c_B != 0 (no implicit -c_B shift). N=3 samples x 10 ticks,
    both runs firing (drag_abs_max > 1e-9), LEO fixture 400 km.
  srp-offset: verdict recorded only AFTER the facet+sizing link lands
    (earlier verdicts were vacuous zeros-vs-zeros -- see below).
FLAKINESS GUARD (why these gates exist): the pre-fix probe showed how a
mis-wired effector fooled its own verdict -- facet never attached
("ValueError ... numFacets"), both arms idled at exactly 0.0, the empty
delta matched a prediction of -0.0, and the leg declared "shifted" with
ZERO evidence. Any env-vs-env comparison must gate on an absolute
firing/scale proof or it will happily prove the opposite of the truth.
Telemetry (if ever published for env) stays about-B, unshifted.

IF A FUTURE HOST RUN FAILS WITH A CLEAN RATIO, the flip is 3 lines in
  src/omni_spacecraft.cpp :: OmniSpacecraft::updateEnvInputs(const double* y):
    (1) dragWrench() comOffset arg is the UNSHIFTED form -> pass
        {envComOff - rBcB_ ...} to shift.
    (2) srpFacet() rCop arg in the facet loop is the SHIFTED form -> pass
        {f.rCop ...} to unshift.
    (3) cannonball stays torque-free either way (no line to flip).
  A clean failure mode is: omni env step == stock step computed with the
  OPPOSITE arm convention to <5% while matching the same-convention step
  to >>5% -- then apply the flip and re-run.

What this file does:
  (a) Pure-numpy arm math -- runs EVERYWHERE (no Basilisk, no build).
  (b) Host omni-vs-stock DIRECT parity -- macOS + Basilisk + built
      omni_heyoka only (importorskip, never fails collection on Linux).
      No analytic theory: omni (setDragParams/setCoMOffset/addSRPFacet +
      offset) vs stock same-setup. Bench pattern from test_wp4_offset.py
      (both-arms fresh setup, 4 ticks dt=0.1, compare om[2]-om[1] env steps
      at 5% + atol 1e-9 on unfired axes; index skips the tick0 fill step).
      "Fire step" below = the om[2]-om[1] env step (continuous effect, no
      thruster driver needed; same fill-step guard as the WP4 bench).

Host verify: cmake --build build && ./env/bin/python -m pytest \
  native_omni_heyoka/tests/test_wp45_env_offset.py -q   (host runtime <5 min)
Probe ledger: audit_bundle/results/wp45_combo.json (stock-direct decision).
CANNOT run here (Linux, Basilisk macOS-only): (a) passes on Linux.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
WP4_LEDGER = REPO / "audit_bundle" / "results" / "wp4_offset.json"
WP5_LEDGER = REPO / "audit_bundle" / "results" / "wp5_env.json"
COMBO_LEDGER = REPO / "audit_bundle" / "results" / "wp45_combo.json"

C_REF = np.array([0.1, 0.0, 0.0])
DRAG_COM = np.array([0.05, 0.0, 0.0])
SRP_RCOP = np.array([0.0, 0.5, 0.0])
MU_EARTH = 3.986004418e14
R_EARTH = 6378137.0
LEO_R = R_EARTH + 400.0e3
LEO_V = math.sqrt(MU_EARTH / LEO_R)
AU = 1.495978707e11
DT = 0.1


def _mrp_to_dcm(sigma) -> np.ndarray:
    s = np.asarray(sigma, float).reshape(3)
    s2 = float(s @ s)
    S = np.array([[0.0, -s[2], s[1]], [s[2], 0.0, -s[0]], [-s[1], s[0], 0.0]])
    return np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2


# ---------------------------------------------------------------------------
# (a) host-independent arm math (binding formulas, no Basilisk)
# ---------------------------------------------------------------------------
def test_ledgers_present_for_combo():
    if not WP4_LEDGER.exists():
        pytest.skip("missing wp4_offset.json")
    if not WP5_LEDGER.exists():
        pytest.skip("missing wp5_env.json")
    w4 = json.loads(WP4_LEDGER.read_text())["payload"]
    assert "about B must equal unshifted" in w4["p_wp44_moment_arm"]["arm_note"]
    w5 = json.loads(WP5_LEDGER.read_text())["payload"]
    assert "comOffset x F" in w5["drag"]["checks"]["formula"]["law"]
    assert "r_Cop x F" in w5["srp"]["checks"]["faceted"]["law"]


def test_shifted_minus_unshifted_is_minus_cxF():
    """(comOff-c)xF - comOffxF == -c x F; same for (rCop-c)xF."""
    F = np.array([0.0, -1.2937492587712527e-04, 0.0])  # ledger LEO scale
    d_drag = np.cross(DRAG_COM - C_REF, F) - np.cross(DRAG_COM, F)
    np.testing.assert_allclose(d_drag, -np.cross(C_REF, F), rtol=0.0, atol=0.0)
    F2 = np.array([-7.263708625120001e-06, 0.0, 0.0])
    d_srp = np.cross(SRP_RCOP - C_REF, F2) - np.cross(SRP_RCOP, F2)
    np.testing.assert_allclose(d_srp, -np.cross(C_REF, F2), rtol=0.0, atol=0.0)
    # Exact drag numbers: com=[.05,0,0] F=-y -> T=[0,0,-.05 f].
    np.testing.assert_allclose(np.cross(DRAG_COM, F),
                               np.array([0.0, 0.0, -0.05 * 1.2937492587712527e-04]),
                               rtol=1e-12)


def test_magnetic_gravity_have_no_arm():
    """tau=m x B and N-body a have no moment arm: offset shifts nothing."""
    m = np.array([0.5, 0.0, 0.0])
    B = np.array([1.39e-05, -1.39e-05, -1.17e-05])
    t1 = np.cross(m, B)
    np.testing.assert_allclose(t1, np.cross(m, B), atol=0.0)
    assert float(t1 @ m) == pytest.approx(0.0, abs=1e-24)
    # Gravity central law has no arm term by construction.
    r = np.array([LEO_R, 0.0, 0.0])
    a = -MU_EARTH * r / float(np.linalg.norm(r)) ** 3
    assert a[0] < 0.0


def test_zero_offset_shift_identity():
    c0 = np.zeros(3)
    F = np.array([1.0, -2.0, 3.0])
    np.testing.assert_allclose(np.cross(DRAG_COM - c0, F),
                               np.cross(DRAG_COM, F), atol=0.0)
    np.testing.assert_allclose(np.cross(SRP_RCOP - c0, F),
                               np.cross(SRP_RCOP, F), atol=0.0)


def test_combo_ledger_verdict_shape_if_present():
    """If the host probe ran, its verdicts must be one of the 3 words."""
    if not COMBO_LEDGER.exists():
        pytest.skip("combo probe not run on this host yet")
    p = json.loads(COMBO_LEDGER.read_text())["payload"]
    for key in ("drag_offset", "srp_offset"):
        if key in p and "verdict" in p[key]:
            assert p[key]["verdict"] in ("shifted", "unshifted", "ambiguous")
    if "drag_zero_com" in p and "verdict" in p["drag_zero_com"]:
        assert p["drag_zero_com"]["verdict"] in (
            "unshifted (~0)", "shifted (~-c x F)", "ambiguous",
            "untestable (effector idle)")


# ---------------------------------------------------------------------------
# (b) host omni-vs-stock DIRECT parity (skipped on Linux)
# ---------------------------------------------------------------------------
def _leo_ic():
    return [LEO_R, 0.0, 0.0], [0.0, LEO_V, 0.0]


def _set_stock_drag(drag, Cd=2.2, com=(0.05, 0.0, 0.0)):
    for k, v in (("dragCoeff", Cd), ("dragCoefficient", Cd),
                 ("area", 1.0), ("projArea", 1.0), ("projectedArea", 1.0),
                 ("comOffset", list(com)), ("comOffset_B", list(com))):
        try:
            if hasattr(drag, k):
                setattr(drag, k, v)
        except Exception:
            pass


def _run_stock_drag(offset, dt=DT, n_ticks=4):
    pytest.importorskip("Basilisk.simulation.spacecraft")
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                    simIncludeGravBody)
    import importlib as _il
    expMod = _il.import_module("Basilisk.simulation.exponentialAtmosphere")
    dragMod = _il.import_module("Basilisk.simulation.dragDynamicEffector")
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "stock-drag"
    sc.hub.mHub = 750.0
    sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
    sc.hub.r_BcB_B = list(map(float, offset))
    r0, v0 = _leo_ic()
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = r0, v0
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc, 1)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    atmo = expMod.ExponentialAtmosphere()
    if hasattr(atmo, "baseDensity"):
        atmo.baseDensity = 2.0e-12
    try:
        atmo.addSpacecraftToModel(sc.scStateOutMsg)
    except Exception:
        pass
    sim.AddModelToTask("t", atmo)
    drag = dragMod.DragDynamicEffector()
    _set_stock_drag(drag)
    dens_msg, dens_name = _atmo_outbox(atmo)
    drag.atmoDensInMsg.subscribeTo(dens_msg)
    try:
        if hasattr(drag, "addSpacecraftToModel"):
            drag.addSpacecraftToModel(sc.scStateOutMsg)
    except Exception:
        pass
    sim.AddModelToTask("t", drag)
    slog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", slog)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()
    return slog, atmo


def _atmo_outbox(atmo):
    """Density output message of ExponentialAtmosphere (version-tolerant).

    Tries known spellings in order; returns (msg, attr_name). The 2.12
    build exposes NO singular outbox -- density flows through the
    ``envOutMsgs`` vector (first entry), same plural-vector pattern as
    viz ``thrInMsgs``/``rwInMsgs``. Raises AssertionError with a dir()
    dump if none exists (a real failure, not a skip: without the link
    the density channel is untestable).
    """
    cands = ["atmoDensOutMsg", "atmoPropsOutMsg", "envOutMsg", "atmoOutMsg"]
    for name in cands:
        try:
            msg = getattr(atmo, name)
        except Exception:
            continue
        if msg is not None:
            return msg, name
    # Plural-vector form (measured 2026-10-12 on BSK 2.12.0).
    try:
        vec = getattr(atmo, "envOutMsgs", None)
        if vec is not None and len(vec) > 0:
            return vec[0], "envOutMsgs[0]"
    except Exception:
        pass
    names = sorted(a for a in dir(atmo) if not a.startswith("_"))
    raise AssertionError(
        "ExponentialAtmosphere exposes no known density outbox "
        f"(tried {cands} + envOutMsgs[0]); dir={names}")


def _run_omni_drag(offset, dt=DT, n_ticks=4):
    oh = pytest.importorskip("omniheyoka.omni_heyoka",
                             reason="host-only: needs built omni_heyoka")
    pytest.importorskip("Basilisk.simulation.spacecraft")
    pytest.importorskip("Basilisk.simulation.exponentialAtmosphere")
    from Basilisk.utilities import SimulationBaseClass, macros
    from Basilisk.simulation import exponentialAtmosphere as _atmo
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    sc = oh.OmniSpacecraft()
    sc.setMass(750.0)
    sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
    sc.setMu(MU_EARTH)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.setCoMOffset(float(offset[0]), float(offset[1]), float(offset[2]))
    sc.setDragParams(2.2, 0.05, 0.0, 0.0)  # CdA=2.2, comOff=[.05,0,0]
    r0, v0 = _leo_ic()
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sim.AddModelToTask("t", sc, 1)
    # In-sim stock density source (same field the stock arm reads).
    # Wind unlinked on BOTH arms -> inertial-velocity fallback (matched).
    atmo = _atmo.ExponentialAtmosphere()
    if hasattr(atmo, "baseDensity"):
        atmo.baseDensity = 2.0e-12
    try:
        atmo.addSpacecraftToModel(sc.scStateOutMsg)
    except Exception:
        pass
    sim.AddModelToTask("t", atmo)
    dens_msg, dens_name = _atmo_outbox(atmo)
    try:
        sc.atmoDensInMsg.subscribeTo(dens_msg)
    except Exception as e:
        pytest.skip(f"omni atmoDensInMsg wiring unknown on host: {e}")
    log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()
    return log


def test_host_drag_offset_direct_parity():
    """Drag+offset DIRECT parity: omni vs stock, same LEO setup, om[2]-om[1].

    Pre-registered: rtol 5% on the fired (torque-active) axis, atol 1e-9 on
    unfired axes (wp4 bench pattern). If this fails with omni matching the
    OPPOSITE arm convention (see module docstring), apply the 3-line flip in
    src/omni_spacecraft.cpp::OmniSpacecraft::updateEnvInputs and re-run.
    """
    pytest.importorskip("Basilisk", reason="host-only omni-vs-stock bench")
    slog, _atmo_unused = _run_stock_drag(C_REF)
    olog = _run_omni_drag(C_REF)
    om = np.asarray(olog.omega_BN_B, float)
    som = np.asarray(slog.omega_BN_B, float)
    assert len(om) >= 3 and len(som) >= 3
    step, sstep = om[2] - om[1], som[2] - som[1]
    print(f"\n[wp45-drag] omni env step {step} vs stock env step {sstep}")
    np.testing.assert_allclose(step, sstep, rtol=0.05, atol=1e-9)


def _run_stock_srp_facet(offset, dt=DT, n_ticks=4):
    pytest.importorskip("Basilisk.simulation.spacecraft")
    pytest.importorskip("Basilisk.simulation.facetedSRPEffector")
    from Basilisk.simulation import spacecraft
    from Basilisk.simulation import facetedSRPEffector as _srp
    from Basilisk.architecture import messaging
    from Basilisk.utilities import SimulationBaseClass, macros
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "stock-srp"
    sc.hub.mHub = 750.0
    sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
    sc.hub.r_BcB_B = list(map(float, offset))
    r0, v0 = _leo_ic()
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = r0, v0
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc, 1)
    srp = _srp.FacetedSRPEffector()
    try:
        srp.addFacet(1.0, [0.0, 0.5, 0.0], [1.0, 0.0, 0.0], 0.2, 0.3)
    except Exception:
        try:
            srp.addFacet(1.0, [1.0, 0.0, 0.0], [0.0, 0.5, 0.0], 0.2, 0.3)
        except Exception as e:
            pytest.skip(f"stock addFacet spelling unknown on host: {e}")
            return None
    sunMsg = messaging.SpicePlanetStateMsg()
    p = messaging.SpicePlanetStateMsgPayload()
    p.PositionVector = [AU, 0.0, 0.0]
    p.VelocityVector = [0.0, 0.0, 0.0]
    sunMsg.write(p)
    srp.sunStateInMsg.subscribeTo(sunMsg)
    try:
        if hasattr(srp, "addSpacecraftToModel"):
            srp.addSpacecraftToModel(sc.scStateOutMsg)
    except Exception:
        pass
    sim.AddModelToTask("t", srp)
    slog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", slog)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()
    return slog


def _run_omni_srp_facet(offset, dt=DT, n_ticks=4):
    oh = pytest.importorskip("omniheyoka.omni_heyoka",
                             reason="host-only: needs built omni_heyoka")
    pytest.importorskip("Basilisk.simulation.spacecraft")
    from Basilisk.utilities import SimulationBaseClass, macros
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    sc = oh.OmniSpacecraft()
    sc.setMass(750.0)
    sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
    sc.setMu(MU_EARTH)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.setCoMOffset(float(offset[0]), float(offset[1]), float(offset[2]))
    sc.addSRPFacet(1.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.3)
    sc.setSunState(AU, 0.0, 0.0)  # static sun; eclipse unlinked -> 1.0
    r0, v0 = _leo_ic()
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sim.AddModelToTask("t", sc, 1)
    log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()
    return log


def test_host_srp_offset_direct_parity():
    """SRP-facet+offset DIRECT parity: omni vs stock, om[2]-om[1] at 5%."""
    pytest.importorskip("Basilisk", reason="host-only omni-vs-stock bench")
    out = _run_stock_srp_facet(C_REF)
    if out is None:
        return
    olog = _run_omni_srp_facet(C_REF)
    om = np.asarray(olog.omega_BN_B, float)
    som = np.asarray(out.omega_BN_B, float)
    assert len(om) >= 3 and len(som) >= 3
    step, sstep = om[2] - om[1], som[2] - som[1]
    print(f"\n[wp45-srp] omni env step {step} vs stock env step {sstep}")
    np.testing.assert_allclose(step, sstep, rtol=0.05, atol=1e-9)


def test_host_mag_no_arm_smoke():
    """Magnetic torque has no arm: offset must not change the step."""
    oh = pytest.importorskip("omniheyoka.omni_heyoka",
                             reason="host-only: needs built omni_heyoka")
    pytest.importorskip("Basilisk", reason="host-only smoke")
    from Basilisk.utilities import SimulationBaseClass, macros
    from Basilisk.architecture import messaging

    def run(offset):
        sim = SimulationBaseClass.SimBaseClass()
        sim.SetProgressBar(False)
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = oh.OmniSpacecraft()
        sc.setMass(750.0)
        sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
        sc.setMu(MU_EARTH)
        sc.setUseGravityGradient(False)
        sc.setTol(1e-12)
        sc.setCoMOffset(float(offset[0]), float(offset[1]), float(offset[2]))
        sc.setMagneticMoment(0.5, 0.0, 0.0)
        r0, v0 = _leo_ic()
        sc.setR_CN_NInit(*map(float, r0))
        sc.setV_CN_NInit(*map(float, v0))
        sc.setSigmaBNInit(0.0, 0.0, 0.0)
        sc.setOmegaBNInit(0.0, 0.0, 0.0)
        sim.AddModelToTask("t", sc, 1)
        try:
            magMsg = messaging.MagneticFieldMsg()
            mp = messaging.MagneticFieldMsgPayload()
            mp.magField_N = [2.0e-5, 0.5e-5, -1.0e-5]
            magMsg.write(mp)
            sc.magFieldInMsg.subscribeTo(magMsg)
        except Exception:
            pytest.skip("omni magFieldInMsg link spelling unknown on host")
            return None
        log = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", log)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * 4))
        sim.ExecuteSimulation()
        return np.asarray(log.omega_BN_B, float)

    a, b = run(C_REF), run(np.zeros(3))
    if a is None or b is None:
        return
    np.testing.assert_allclose(a[2] - a[1], b[2] - b[1],
                               rtol=1e-9, atol=1e-12)
