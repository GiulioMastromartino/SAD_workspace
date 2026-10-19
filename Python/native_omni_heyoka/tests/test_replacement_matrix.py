"""Replacement matrix: 100%-visibility registry for "replace the stock integrator".

Operational definition (PLAN_INTEGRATOR_SWAP.md Phase 3): three target scripts
run unmodified except the two integrator lines. Every stock feature in that
scope is exactly one of MIRRORED / REJECTS-LOUDLY / GAP -- never silent.

How to read a failure here:
  - Section A (works pins): a capability REGRESSED (ledger verdict/cell broke).
  - Section B (freshness): a proof ledger is STALE (src newer than the numbers).
  - Section C (reject surface): a loud reject was removed/weakened, or a GAP
    CLOSED without updating this registry (absence-assertions go red on fix --
    that red is the signal to move the cell to Section A, not to delete it).
  - Section D (acceptance scripts): a target script appeared/changed shape.
  - Section E (docs): docs disagree (red today: B/C sign) or a reconciled
    marker was lost.
  - Section F (Phase-3 gaps): speedup table / FSW loop landed (red = update).

Ledger-backed tests SKIP (never pass vacuously) when the probe ledger is
absent on this host. Everything else runs on bare pytest: stdlib only, no
numpy, no Basilisk, no wheel. Depth lives in the per-area suites
(test_swap_p1.py, test_wp*.py); this file pins the INTEGRATION state.
"""
from __future__ import annotations

import importlib.util
import json
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "audit_bundle" / "results"
MATRIX = RESULTS / "replacement_matrix.json"
SWAP_SRC = REPO / "omniheyoka" / "swap.py"
SIM_DIR = REPO.parents[0] / "cubesat_12u_sim"

FRESH_SET = ("swap_p1_parity", "swap_multi", "closed_loop_fuel",
             "vec_mrp_switch_trace", "wp2_friction_jitter_voltage",
             "wp3_timing", "wp45_combo")


def _matrix():
    if not MATRIX.exists():
        pytest.skip("replacement_matrix probe not run on this host yet")
    return json.loads(MATRIX.read_text())["payload"]


def _ledger(name):
    p = RESULTS / f"{name}.json"
    if not p.exists():
        pytest.skip(f"{name} probe not run on this host yet")
    doc = json.loads(p.read_text())
    return doc.get("payload", doc)


def _swap_text():
    return SWAP_SRC.read_text(encoding="utf-8")


# --------------------------------------------------------------------------
# A. Works pins (integration keys; depth in per-area suites)
# --------------------------------------------------------------------------

def test_matrix_probe_sections():
    m = _matrix()
    for section in ("mirror_surface", "reject_cells", "proof_ledgers",
                    "acceptance_scripts", "doc_pins", "phase3_gaps"):
        assert section in m, f"matrix missing section {section}"


def test_swap_p1_integration_keys():
    d = _ledger("swap_p1_parity")
    assert d["status"] == "ok"
    assert d["verdict"] == "parity green"
    cells = d["cells"]
    # C-point scale (B/C sign, WP25-A1): stock-pinned 2.24e-08 band, not exact
    # (robust to ledger refresh; depth in test_swap_p1.py).
    assert 1e-9 < cells["thr_offset"]["r_BN_N"] < 1e-7
    assert 0.0 == cells["fuel_centered"]["fuel_rel_err"]  # bit-exact law
    assert cells["fuel_centered"]["fuel_burned"] > 0.1  # firing-proven
    assert cells["fuel_drag"]["arm"] == "live-c-tracking"  # H10 fixed
    assert cells["fuel_drag"]["shift_sig_dcm"] < cells["fuel_drag"]["dcm_max"]
    assert d["rejects"]["verdict"] == "all loud"


def test_swap_multi_integration_keys():
    d = _ledger("swap_multi")
    assert d["status"] == "ok"
    assert d["verdict"] == "parity green"


def test_closed_loop_integration_keys():
    d = _ledger("closed_loop_fuel")
    assert d["status"] == "ok"
    assert d["verdict"] == "parity green"
    leg = d["cells"]["leg"]
    assert leg["fuel_rel_err"] == 0.0 and leg["fuel_burned"] > 0.1
    assert isinstance(leg["loadavg"], list) and len(leg["loadavg"]) == 3  # L9
    rej = d["cells"]["rejected"]  # M17 scope boundary in-ledger
    assert rej["status"] == "rejected-non-parity" and len(rej["exceeds"]) >= 1


def test_arm_artifact_live_gates_resolve():
    """H9 remediation: the discrimination numbers must point at tests that
    EXIST (host-free text check; the host run itself is test_wp45's job)."""
    p = RESULTS / "arm_rule_discrimination.json"
    if not p.exists():
        pytest.skip("arm_rule_discrimination artifact not recorded yet")
    doc = json.loads(p.read_text())
    assert "CITATION ARTIFACT ONLY" in doc.get("role", "")
    t = (REPO / "tests" / "test_wp45_env_offset.py").read_text(encoding="utf-8")
    for gate in doc.get("live_gates", []):
        name = gate.split("::")[-1]
        assert f"def {name}(" in t, f"live gate {name} missing from test_wp45"
    pts = doc["stock_drag_arm_sweep"]["points_h_com__to__step_z"]
    assert pts["(0.05, 0.05)"] == 0.0  # zero at h==c, arm-rule discriminator
    assert pts["(0.1, 0.05)"] > 0 and pts["(0.0, 0.05)"] < 0  # sign flip


# --------------------------------------------------------------------------
# B. Proof-ledger freshness (stale proof is NEVER cited as green)
# --------------------------------------------------------------------------

def test_proof_ledgers_fresh():
    m = _matrix()
    stale = {n: c for n, c in m["proof_ledgers"].items()
             if n in FRESH_SET and c.get("exists") and not c.get("fresh")}
    assert not stale, ("stale proof ledgers (regen on host before citing): "
                       + str({n: c.get("margin_s") for n, c in stale.items()}))


# --------------------------------------------------------------------------
# C. Reject surface + env-mirror gap (always runs; swap.py text scan)
# --------------------------------------------------------------------------

def test_mirrored_set_exact():
    """Item-1 tripwire, EXTENDED by the wiring batch (PLAN_ENV_MIRROR
    §8, list.md §A1): _MIRRORED stays exactly hub+RW+thruster (the probe
    parses that literal, and a staged suite pins it via AST); the
    newly-mirrorable env effectors live in the adjacent _MIRRORED_ENV.
    The UNION is the effective mirrored set -- any reader landing
    changes one of the two sets, and red means move cells to
    Section A. The ledger still reports the _MIRRORED literal (stable
    by design, asserted below)."""
    swap = _load_swap()
    assert swap._MIRRORED == {"Spacecraft", "ReactionWheelStateEffector",
                              "ThrusterDynamicEffector"}
    assert (swap._MIRRORED | swap._MIRRORED_ENV) == {
        "Spacecraft", "ReactionWheelStateEffector",
        "ThrusterDynamicEffector", "DragDynamicEffector",
        "FacetDragDynamicEffector", "RadiationPressure",
        "FacetSRPDynamicEffector", "FacetedSRPEffector",
        "FacetedSpacecraftProjectedArea", "MtbEffector",
        "GravityEffector", "ExtForceTorque", "ThrusterStateEffector"}
    m = _matrix()["mirror_surface"]
    assert m["mirrored_set"] == ["ReactionWheelStateEffector", "Spacecraft",
                                 "ThrusterDynamicEffector"]
    assert m["entry_points"] == ["mirror_stock_config", "mirror_sync_configs",
                                 "task_effectors"]


def test_reject_site_count_and_triggers():
    src = _swap_text()
    m = _matrix()["mirror_surface"]
    assert m["n_reject_sites"] >= 20, "reject sites removed without registry update"
    for trig in ("exactly one central body", "use omni env setters instead",
                 "registration order is ambiguous", "fewer rwOutMsgs",
                 "only BalancedWheels is mirrored", "needs explicit",
                 "need >= 1 object spec", "thr_factory given",
                 "geometry join needs", "order mismatch", "MinOnTime=",
                 "degenerate", "out of the staircase envelope (M6)",
                 "swirl is out", "no thr_cmd_msg", "mirrors exactly one",
                 "no mirrored thrusters"):
        assert trig in src, f"reject trigger lost: {trig!r}"
    missing = [c["cell"] for c in _matrix()["reject_cells"]
               if not c["trigger_present"]]
    assert not missing, f"reject cells without trigger: {missing}"


def test_env_mirror_gap_open():
    """Phase-3 item 1 GAP-CLOSED (wiring batch, PLAN_ENV_MIRROR §8,
    list.md §A1): the per-family _mirror_* wiring landed in swap.py, so
    this pin now ADOPTS the wiring (cf. test_fsw_loop_gap_open style)
    instead of asserting its absence. Red here = wiring removed or
    de-specced -- restore it or move the cell back to the open gap.

    Ledger note (no ledger assert here by design): the probe's
    env_setters_unmirrored counts only literal oi.* calls in swap.py
    text, so delegation keeps listing the staged-applied setters; that
    static gap definition is superseded by the delegation asserts
    below (registry coverage + ledger-field records on fakes)."""
    src = _swap_text()
    for fn in ("_mirror_drag", "_mirror_srp", "_mirror_mag",
               "_mirror_eclipse_link", "_mirror_env_bodies", "_mirror_j2"):
        assert f"def {fn}(" in src, f"env wiring missing ({fn})"
    swap = _load_swap()
    # Registry covers the newly-mirrorable classes (union pin; the
    # ledger still reports the _MIRRORED literal, pinned above).
    assert (swap._MIRRORED | swap._MIRRORED_ENV) == {
        "Spacecraft", "ReactionWheelStateEffector",
        "ThrusterDynamicEffector", "DragDynamicEffector",
        "FacetDragDynamicEffector", "RadiationPressure",
        "FacetSRPDynamicEffector", "FacetedSRPEffector",
        "FacetedSpacecraftProjectedArea", "MtbEffector",
        "GravityEffector", "ExtForceTorque", "ThrusterStateEffector"}
    # Records carry the ledger fields: cannonball drag leg on fakes
    # (swap_p1_parity.py:456-467 fixture: A=5.0, Cd=2.2, com=[0.05,0,0]).
    atmo_owner = object()
    drag = _cannonball_drag(atmo_owner)
    oi = _EnvOiStub()
    rec = swap._mirror_drag([drag], [], [], oi, None)
    assert rec["n_drag"] == 1 and rec["class"] == "DragDynamicEffector"
    assert rec["CdA"] == pytest.approx(2.2 * 5.0)
    assert rec["comOffset"] == [0.05, 0.0, 0.0]
    assert rec["atmo"] == "linked" and rec["wind"] == "inertial-fallback"
    assert ("atmoDensInMsg.subscribeTo", atmo_owner) in oi.calls
    assert ("setDragParams", 2.2 * 5.0, 0.05, 0.0, 0.0) in oi.calls


def _load_swap():
    """Load omniheyoka/swap.py by path (module-level imports are
    stdlib-only, so this never needs the host env -- the gaps-test
    pattern)."""
    spec = importlib.util.spec_from_file_location(
        "swap_matrix_adoption", SWAP_SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _EnvOiStub:
    """Oi stub: slot-0 env message members subscribe; every setter
    records its call."""

    def __init__(self):
        self.calls: list = []
        for member in ("atmoDensInMsg", "windVelInMsg", "sunStateInMsg",
                       "moonStateInMsg", "eclipseInMsg", "magFieldInMsg"):
            def _sub(src, _member=member):
                self.calls.append((_member + ".subscribeTo", src))
            self.__dict__[member] = SimpleNamespace(subscribeTo=_sub)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)

        def _rec(*args):
            self.calls.append((name,) + tuple(args))
        return _rec


def _cannonball_drag(atmo_owner):
    """Fake DragDynamicEffector (wrapper dragDynamicEffector.py:599-650
    surface: coreParams, atmo/wind readers, modelType, correction
    state-name linkage)."""
    cls = type("DragDynamicEffector", (), {})
    m = cls()
    m.coreParams = SimpleNamespace(projectedArea=5.0, dragCoeff=2.2,
                                   comOffset=[0.05, 0.0, 0.0])
    m.atmoDensInMsg = SimpleNamespace(
        isLinked=lambda: True, _get_source_owner=lambda: atmo_owner)
    m.windVelInMsg = SimpleNamespace(isLinked=lambda: False,
                                     _get_source_owner=lambda: None)
    m.modelType = "cannonball"
    m.densityCorrectionStateName = ""
    return m


# --------------------------------------------------------------------------
# D. Acceptance scripts (Phase-3 §script->envelope map)
# --------------------------------------------------------------------------

def test_demo_script_envelope_pins():
    p = SIM_DIR / "backends" / "basilisk" / "basilisk_sim.py"
    assert p.exists(), "Script 1 (repo demo) missing"
    t = p.read_text(encoding="utf-8", errors="replace")
    assert "def run_simulation(" in t  # swap target surface
    assert "vizSupport" in t  # D6 OUT-BY-LABEL pin (direct-vizInterface only)
    assert "SunEphemeris" in t  # D7 env-body-or-reject pin


def test_leo_drag_gap_open():
    """Script 2 LANDED as a stock sim (host run pending): LEO-drag sim exists
    with the L1/L2/L3 envelope surface. Red here = script removed or
    de-specced -- update registry."""
    p = SIM_DIR / "backends" / "basilisk" / "leo_drag_sim.py"
    assert p.exists(), "LEO-drag sim missing again -- re-add per L1/L2/L3"
    t = p.read_text(encoding="utf-8", errors="replace")
    assert "def run_simulation(" in t  # swap target surface
    assert "dragDynamicEffector" in t  # L1 env effector present
    assert "exponentialAtmosphere" in t  # L1 density model present
    assert not any(l.strip().startswith(("import omniheyoka",
                                         "from omniheyoka"))
                   for l in t.splitlines()), "must stay a stock sim"


def test_stationkeeping_gap_open():
    """Script 3: analytic loops unchanged AND stock sim LANDED (host run
    pending). Red here = stock sim removed or re-analyticized."""
    for name in ("run_stationkeeping.py", "run_3year.py"):
        p = SIM_DIR / "missions" / name
        assert p.exists(), f"{name} missing"
        t = p.read_text(encoding="utf-8", errors="replace")
        assert "setIntegrator" not in t, f"{name} gained a swap surface?"
    st = (SIM_DIR / "missions" / "run_stationkeeping.py").read_text(encoding="utf-8")
    assert "heyoka_propagate" in st  # analytic-loop pin (S1 basis)
    # Stock sim landed: swap surface + staircase RCS + single tank, stock-only.
    s = SIM_DIR / "missions" / "run_stationkeeping_stock.py"
    assert s.exists(), "stock stationkeeping sim missing again -- re-add per S2"
    u = s.read_text(encoding="utf-8", errors="replace")
    assert "setIntegrator" in u  # two-line exchange surface (marker counts)
    assert "THRArrayOnTimeCmdMsg" in u  # staircase RCS surface
    assert "FuelTank" in u  # single-tank surface
    assert "heyoka_propagate" not in u  # not another analytic loop
    assert not any(l.strip().startswith(("import omniheyoka",
                                         "from omniheyoka"))
                   for l in u.splitlines()), "must stay a stock sim"


# --------------------------------------------------------------------------
# E. Doc pins (contradictions are red; reconciled markers are guarded)
# --------------------------------------------------------------------------

def test_bc_sign_docs_agree():
    """KNOWN-DIVERGENT (must-fix): PLAN_ENVELOPE still pins B/C `+` while
    DESIGN pins stock-measured `−`. Fails until the PLAN line is struck."""
    plan = (REPO / "PLAN_ENVELOPE.md").read_text(encoding="utf-8")
    design = (REPO / "DESIGN.md").read_text(encoding="utf-8")

    def signs(text):
        return {m.group(1) for m in
                re.finditer(r"r_BN_N\s*=\s*r_CN_N\s*(\S)", text)}

    ps, ds = signs(plan), signs(design)
    assert ps and ds, "B/C sign statements lost from docs"
    assert ps == ds, (f"DIVERGENT: PLAN_ENVELOPE B/C signs {sorted(ps)} vs "
                      f"DESIGN {sorted(ds)} (stock-pinned `−`, thr_offset 2.24e-08)")


def test_docs_reconciled_markers():
    """Guards the already-reconciled doc claims against silent un-fixing."""
    design = (REPO / "DESIGN.md").read_text(encoding="utf-8")
    plan = (REPO / "PLAN_ENVELOPE.md").read_text(encoding="utf-8")
    build = (REPO / "BUILD_STATUS.md").read_text(encoding="utf-8")
    for marker, text in (
            ("superseded-by-measurement", design),   # B/C + readings struck
            ("Reconciliation note:", design),        # guard-scope reconciliation
            ("Decision 3 locked", plan),             # amend-then-enforce
            ("is DELETED", build),                   # cutoff filter deletion
            ("INERT", build),                        # cutoff-inert truth
            ("~~Top-level", build)):                 # A7 exports struck
        assert marker in text, f"reconciled marker lost: {marker!r}"


# --------------------------------------------------------------------------
# F. Phase-3 gaps (speedup table, FSW loop)
# --------------------------------------------------------------------------

def test_speedup_gap_open():
    """Item 4: swap-path cold-vs-warm/per-step table. Ledger absent ->
    GAP-open (documents the missing table). Ledger present -> adopted as
    proof via shape pins (600-tick WP-H-class fixture, loadavg provenance)."""
    p = RESULTS / "swap_speed.json"
    if not p.exists():
        assert _matrix()["phase3_gaps"]["speedup_table"]["ledger_hits"] == []
        return
    d = json.loads(p.read_text())
    payload = d.get("payload", d)
    if payload.get("status") == "skipped-no-basilisk":
        pytest.skip("swap_speed probe skipped (no Basilisk on this host)")
        return
    assert payload["status"] == "ok"
    assert payload["n_runs"] == 3  # means of 3 per phase, cold incl. subprocess
    assert payload["ticks"] == 600 and payload["steps"] == 600
    for key in ("cold_build_s", "warm_build_s", "stock_wall_s",
                "omni_wall_s", "per_step_ms", "speedup_warm", "loadavg",
                "cold_build_wall_s", "warm_build_wall_s",
                "warm_run_wall_s", "warm_per_step_ms",
                "stock_wall_min_s", "stock_wall_max_s",
                "first_run_min_s", "first_run_max_s",
                "warm_run_min_s", "warm_run_max_s",
                "cold_build_wall_min_s", "cold_build_wall_max_s"):
        assert key in payload, f"speed table missing {key}"
    assert payload["warm_steps"] == 600  # warm leg stepped fully too
    assert payload["sim_end_nanos_ok"] is True  # both arms simulated full 60 s
    assert payload["sim_end_nanos_expected"] == 60000000000
    for key in ("stock_wall_samples_s", "first_run_samples_s",
                "warm_run_samples_s", "cold_build_wall_samples_s"):
        assert key in payload, f"speed table missing {key}"
        assert len(payload[key]) == 3, f"{key} must carry all 3 runs"
    assert payload["per_step_ms"] > 0
    assert isinstance(payload["loadavg"], list) and len(payload["loadavg"]) == 3
    assert payload["duty"] == 0.5
    assert payload["fixture_class"] == "firing"


def test_fsw_loop_gap_open():
    """Item 3: FSW-in-the-loop tick-by-tick comparison. Ledger absent ->
    GAP-open (documents the missing loop comparison). Ledger present ->
    adopted as proof via shape pins (6000-tick PD closed-loop leg; depth
    lives in tests/test_fsw_loop.py -- this is the registry-side adoption)."""
    g = _matrix()["phase3_gaps"]["fsw_loop"]
    assert g["pieces"]["test_vec_mrp"] and g["pieces"]["closed_loop_ledger"] \
        and g["pieces"]["test_swap_multi"], "FSW pieces lost"
    hits = g["ledger_hits"]
    if not hits:
        return  # GAP still open: nothing to adopt yet
    assert "fsw_loop.json" in hits, f"unexpected FSW ledger hits {hits}"
    p = RESULTS / "fsw_loop.json"
    assert p.exists(), "registry cites fsw_loop.json but the ledger is gone"
    d = json.loads(p.read_text())
    payload = d.get("payload", d)
    assert payload["status"] == "ok"
    assert payload["verdict"] == "parity green"
    assert payload["ticks"] == 6000 and payload["dt"] == pytest.approx(0.1)
    c = payload["cells"]["leg"]
    # Step-count audit (L14): one integrate step and one call per tick.
    assert c["ticks"] == 6000 and c["steps"] == 6000 and c["calls"] == 6000
    # MRP trace live on both sides, totals matched within the stated window.
    assert c["mrp_trace_len"] >= 6000
    assert len(c["stock_mrp_trace"]) == c["mrp_trace_len"]
    assert abs(c["mrp_omni"] - c["mrp_stock"]) <= c["sat_tol_ticks"]
    # Allocator firing proof (not zeros-vs-zeros) + chatter agreement.
    assert c["thr_fire_stock"] > 0 and c["thr_fire_omni"] > 0
    assert c["thr_chatter_stock"] == c["thr_chatter_omni"]
    assert c["fuel_burned"] > 0.1 and c["fuel_burned_omni"] > 0.1
    assert c["fuel_rel_err"] < 1e-9  # bit-exact law
    assert c["disp_stock"] > 1e5 and c["disp_omni"] > 1e5
    for f, tol in (("r_BN_N", 2e-2), ("v_BN_N", 1e-4), ("sigma_BN", 2e-4),
                   ("omega_BN_B", 1e-4), ("dcm_max", 2e-3), ("wheels", 1e-4)):
        assert c[f] < tol, f"leg.{f}={c[f]:.3e}"
    for edge in ("sat_u", "sat_w"):
        assert len(c[f"{edge}_stock"]) == 3 and len(c[f"{edge}_omni"]) == 3
    assert isinstance(payload["loadavg"], list) and len(payload["loadavg"]) == 3
