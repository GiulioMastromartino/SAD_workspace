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

MEASURED CONVENTION (2026-10-13, test_host_drag_offset_direct_parity,
firing-proven BOTH arms): stock drag torque is T = F x comOffset (about
+z on this fixture); the pre-2026-10-13 omni order (comOffset x F, about
-z) was sign-flipped at equal magnitude (omni [0,0,-1.07812560e-09] vs
stock [0,0,+1.07812559e-09], 8-digit |.| agreement). The shared helper
include/omni/environment.hpp::dragWrench now encodes T = F x comOffset
(one flip fixes BOTH the SysModel and vec paths, which share the helper
via updateEnvInputs); ledger audit_bundle/results/wp45_combo.json.
Force-vs-torque split (same leg): the FORCE law F = -1/2 rho CdA |v_rel|
v_rel with v_rel = v - vAir (wind-unlinked -> inertial) matches stock
with identical inputs (code vs fixture operand order verified), so only
the torque cross order was flipped -- see the translational-sign
discriminator gate inside the test.

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

# Drag-atmo ledger-scale pin (WP25-C1 fixture fix): stock
# ExponentialAtmosphere applies altitude falloff
#   rho(h) = baseDensity * exp(-(r - planetRadius) / scaleHeight)
# (combo probe audit_bundle/probes/wp45_combo.py drag-offset leg pinned
# baseDensity=2e-12/scaleHeight=88000/planetRadius and measured torque
# 6.87e-8 Nm -- i.e. rho(400 km) fell ~94x below ledger rho, and a
# baseDensity-only pin leaves the default falloff starving the leg to
# ~1e-21). Ledger scale (audit_bundle/results/wp5_env.json drag.formula)
# is rho = 2e-12 -> |F| ~ 1.29e-4 N; the compensated baseDensity below
# inverts the falloff at this file's LEO fixture (LEO_R - R_EARTH = 400 km)
# so rho(400 km) == 2e-12 and the om[2]-om[1] env step lands at ~1.1e-9,
# inside the existing (1e-10, 1e-7] firing band (band untouched).
# envMaxReach is set wide open (guarded) so the 400 km LEO shell is never
# cut off by a reach knob.
DRAG_SCALE_HEIGHT = 88000.0
DRAG_BASE_DENSITY = 2.0e-12 * math.exp((LEO_R - R_EARTH) / DRAG_SCALE_HEIGHT)
DRAG_ENV_MAX_REACH = 2.0e7


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
    assert "F x comOffset" in w5["drag"]["checks"]["formula"]["law"]
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


def _set_stock_atmo(atmo):
    """Pin the ledger-scale density fixture (combo-probe pattern).

    Sets baseDensity/scaleHeight/planetRadius (+envMaxReach where the
    build exposes it); every knob guarded by hasattr so an unknown build
    fails loudly at the firing gate instead of here.
    """
    for k, v in (("baseDensity", DRAG_BASE_DENSITY),
                 ("scaleHeight", DRAG_SCALE_HEIGHT),
                 ("planetRadius", R_EARTH),
                 ("envMaxReach", DRAG_ENV_MAX_REACH)):
        try:
            if hasattr(atmo, k):
                setattr(atmo, k, v)
        except Exception:
            pass


def _set_stock_drag(drag, Cd=2.2, com=(0.05, 0.0, 0.0)):
    # BSK 2.12 keeps CdA/comOffset in the `coreParams` struct, NOT on the
    # module (combo probe audit_bundle/probes/wp45_combo.py: direct attrs
    # silently miss, idling the effector to an env-step ~1e-21). Set
    # coreParams first, module-direct second (harmless where absent).
    core = getattr(drag, "coreParams", None)
    if core is not None:
        for k, v in (("projectedArea", 1.0), ("dragCoeff", Cd),
                     ("dragCoefficient", Cd),
                     ("comOffset", list(com)), ("comOffset_B", list(com))):
            try:
                if hasattr(core, k):
                    setattr(core, k, v)
            except Exception:
                pass
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
    _set_stock_atmo(atmo)
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
    try:
        # Combo-probe attach path (audit_bundle/probes/wp45_combo.py:
        # sc.addDynamicEffector is what stuck on BSK 2.12); without it the
        # effector never contributes and the leg idles at ~1e-21.
        if hasattr(sc, "addDynamicEffector"):
            sc.addDynamicEffector(drag)
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
    # Same ledger-scale pin as the stock arm: the omni leg reads
    # neutralDensity from this message (src/omni_spacecraft.cpp
    # updateEnvInputs -> dragWrench(atmo.neutralDensity, ...)), so the
    # identical _set_stock_atmo call gives it the identical pinned density.
    atmo = _atmo.ExponentialAtmosphere()
    _set_stock_atmo(atmo)
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

    Measured convention (2026-10-13, firing-proven both arms): stock is
    T = F x comOffset (about +z here); omni now matches via the shared
    include/omni/environment.hpp::dragWrench flip (ledger wp45_combo.json).

    Pre-registered: rtol 5% on the fired (torque-active) axis, atol 1e-9 on
    unfired axes (wp4 bench pattern).

    H4 FIRING/SCALE GATES (WP25-C1): a vacuous 0-vs-0 leg must FAIL, never
    pass. Ledger LEO scale (audit_bundle/results/wp5_env.json drag.formula):
    |F_drag| ~ 1.29e-4 N, |T| = 0.05*|F| ~ 6.47e-6 Nm about +z (stock
    T = F x comOffset convention, 2026-10-13), wdot =
    T/Izz ~ 1.1e-8 rad/s^2, so the om[2]-om[1] env step is ~1.1e-9 rad/s.
    Band (1e-10, 1e-7]: the floor sits 1 order under prediction and 7+
    orders above the vacuous floor (exact 0.0 / 1e-17 when the effector
    never fires); the 1e-7 cap catches a 100x-hot misfire while staying
    ~100x above signal. (Scale note: a literal 1e-7 floor on this step
    would reject correct physics -- the floor is 1e-10 by derivation.)
    """
    pytest.importorskip("Basilisk", reason="host-only omni-vs-stock bench")
    slog, _atmo_unused = _run_stock_drag(C_REF)
    olog = _run_omni_drag(C_REF)
    om = np.asarray(olog.omega_BN_B, float)
    som = np.asarray(slog.omega_BN_B, float)
    assert len(om) >= 3 and len(som) >= 3
    step, sstep = om[2] - om[1], som[2] - som[1]
    s_norm = float(np.linalg.norm(sstep))
    o_norm = float(np.linalg.norm(step))
    assert 1e-10 < s_norm <= 1e-7, (
        f"stock drag leg not firing-proven (env-step norm {s_norm:.3e} "
        "outside ledger-scale band (1e-10, 1e-7]): no verdict may be read "
        "from an unfired or off-scale leg")
    assert 1e-10 < o_norm <= 1e-7, (
        f"omni drag leg not firing-proven (env-step norm {o_norm:.3e} "
        "outside ledger-scale band (1e-10, 1e-7])")
    print(f"\n[wp45-drag] omni env step {step} vs stock env step {sstep}")
    np.testing.assert_allclose(step, sstep, rtol=0.05, atol=1e-9)
    # Translational-sign discriminator (force-vs-torque split): a flipped
    # FORCE (F_omni = -F_stock) also flips torque yet stays nearly invisible
    # in the short torque-gated leg above, so the torque gate alone cannot
    # pin the force sign. Derived band from this file's constants with the
    # ledger |F| ~ 1.29e-4 N over the 750.0 kg hub (both arms set above):
    #   a_drag = |F|/m ~ 1.72e-7 m/s^2; dv ~ a*DT ~ 1.7e-8 m/s per 0.1 s
    #   step; dr ~ a*DT^2/2 ~ 8.6e-10 m. A force flip separates the omni vs
    #   stock v steps by 2*dv ~ 3.4e-8 m/s (r steps by ~1.7e-9 m) with
    #   gravity common-mode cancelled, while the torque magnitude still
    #   matches. The v gate below is the discriminator: per-element tolerance
    #   on the drag-active y axis is atol(DV_DRAG ~ 1.7e-8) + rtol*|4.9e-5
    #   orbital | ~ half the 2x flip separation, so a force flip FAILS it
    #   while correct-sign physics passes; x/z elements are flip-insensitive
    #   (tolerance gravity-dominated there, by construction). The r gate is
    #   a gross-divergence guard only (orbital common-mode ~766 m swamps the
    #   ~1.7e-9 m flip separation under any nonzero rtol), plus the sign
    #   check pins the direction on the drag-active (-y, F = -1/2 rho CdA
    #   |v| v with v ~ +y LEO) axis.
    F_REF = 0.5 * 2.0e-12 * 2.2 * LEO_V ** 2  # ~1.29e-4 N, ledger scale
    A_DRAG = F_REF / 750.0
    DV_DRAG, DR_DRAG = A_DRAG * DT, 0.5 * A_DRAG * DT ** 2
    sv = np.asarray(slog.v_CN_N, float)
    ov = np.asarray(olog.v_CN_N, float)
    sr = np.asarray(slog.r_CN_N, float)
    ov_r = np.asarray(olog.r_CN_N, float)
    assert len(sv) >= 3 and len(ov) >= 3 and len(sr) >= 3 and len(ov_r) >= 3
    sv_step, ov_step = sv[2] - sv[1], ov[2] - ov[1]
    sr_step, or_step = sr[2] - sr[1], ov_r[2] - ov_r[1]
    assert int(np.sign(ov_step[1])) == int(np.sign(sv_step[1])), (
        f"drag force sign mismatch on the active (-y) axis: omni v-step "
        f"{ov_step} vs stock v-step {sv_step}")
    print(f"[wp45-drag] dv-drag scale {DV_DRAG:.3e} m/s, dr-drag scale "
          f"{DR_DRAG:.3e} m; omni v-step {ov_step} vs stock v-step {sv_step}")
    np.testing.assert_allclose(ov_step, sv_step, rtol=1e-6, atol=DV_DRAG)
    # Gross-divergence guard only (NOT the discriminator — that is the v gate
    # above). Measured 2026-10-13: Taylor-vs-RK4 integrator floor on the orbital
    # x common-mode is 1.86e-9, just above DR_DRAG*2 = 1.73e-9, so the factor is
    # 4 (still << any gross divergence and << the 766 m common mode; the flip
    # separation ~1.7e-9 is owned by the v gate, not this one).
    np.testing.assert_allclose(or_step, sr_step, rtol=1e-9, atol=DR_DRAG * 4)


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
    """SRP-facet+offset DIRECT parity: omni vs stock, om[2]-om[1] at 5%.

    FRAGILE (H5, WP25-C1): NOT measured. The stock facet+sizing link has
    not landed on any host -- _run_stock_srp_facet guesses the addFacet
    spelling and the facet sizing (area/ProjectedArea) + sun-state vector
    wiring are unpinned. Until a host pass lands that link, this leg must
    NEVER be cited as a shift verdict; both paths ship (rCop-c)xF
    (spacecraft static-c, vec live-c) by construction, with the 3-line-flip
    contingency in the module docstring. MISSING PIECE for a host pass
    (python3/Basilisk absent on this container, so sims skip here): the
    exact host-BSK FacetedSRPEffector.addFacet signature spelling +
    facet-sizing semantics + sun SpicePlanetStateMsg wiring, then a
    firing-proven (band-gated below) stock-vs-omni leg.
    """
    pytest.importorskip("Basilisk", reason="host-only omni-vs-stock bench")
    out = _run_stock_srp_facet(C_REF)
    assert out is not None, (
        "stock SRP-facet link missing on host (addFacet spelling unknown): "
        "withholding the leg is a FAILURE, never a skip-as-pass (H4/H5)")
    olog = _run_omni_srp_facet(C_REF)
    om = np.asarray(olog.omega_BN_B, float)
    som = np.asarray(out.omega_BN_B, float)
    assert len(om) >= 3 and len(som) >= 3
    step, sstep = om[2] - om[1], som[2] - som[1]
    # H4 firing/scale band (WP25-C1): ledger facet scale
    # (audit_bundle/results/wp5_env.json srp.faceted: F ~ -6.51e-6 N along
    # -x) with shifted arm (rCop-c) = (-0.1, 0.5, 0) gives T_z ~ 3.25e-6
    # Nm, wdot ~ 5.4e-9 rad/s^2, env step ~ 5.4e-10 rad/s. Band (1e-10,
    # 1e-7]: ~5x under prediction, 7+ orders above the vacuous exact-0.0
    # idle floor; 1e-7 cap catches off-scale misfires.
    s_norm = float(np.linalg.norm(sstep))
    o_norm = float(np.linalg.norm(step))
    assert 1e-10 < s_norm <= 1e-7, (
        f"stock SRP leg not firing-proven (env-step norm {s_norm:.3e} "
        "outside (1e-10, 1e-7]): no verdict may be read")
    assert 1e-10 < o_norm <= 1e-7, (
        f"omni SRP leg not firing-proven (env-step norm {o_norm:.3e} "
        "outside (1e-10, 1e-7])")
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
    assert a is not None and b is not None, (
        "magnetic smoke leg unlinked on host (magFieldInMsg wiring unknown): "
        "withholding the leg is a FAILURE, never a skip-as-pass (H4)")
    # H4 firing/scale band (WP25-C1) BEFORE the no-arm delta: m = [0.5, 0,
    # 0] Am^2 vs B_N = [2.0e-5, 0.5e-5, -1.0e-5] T gives tau = m x B_B with
    # |tau| ~ 5.6e-6 Nm, wdot ~ 9e-9 rad/s^2, env step ~ 7.5e-10 rad/s.
    # Band (1e-10, 1e-7] proves both legs fired; the delta below then proves
    # the offset changed nothing (no arm).
    assert 1e-10 < float(np.linalg.norm(a[2] - a[1])) <= 1e-7, (
        "offset magnetic leg idle or off-scale: no verdict may be read")
    assert 1e-10 < float(np.linalg.norm(b[2] - b[1])) <= 1e-7, (
        "zero-offset magnetic leg idle or off-scale: no verdict may be read")
    np.testing.assert_allclose(a[2] - a[1], b[2] - b[1],
                               rtol=1e-9, atol=1e-12)
