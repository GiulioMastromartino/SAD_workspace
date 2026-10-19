"""Benchmark-suite registry + ledger gates (bench_stock_vs_omni).

The probe is host-only (Basilisk + built wheel), but THIS file runs on bare
pytest (stdlib only): the registry half loads the probe module by file path
(its top level is stdlib-only by construction) and pins the "bench every
surface" contract -- tier nesting, smoke bounds, tolerance sanity, run_all
wiring. The ledger half SKIPs (never fails vacuously) without the host ledger.
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PROBE = REPO / "audit_bundle" / "probes" / "bench_stock_vs_omni.py"
HIJACK_PROBE = REPO / "audit_bundle" / "probes" / "hijack_demo.py"
LEDGER = REPO / "audit_bundle" / "results" / "bench_stock_vs_omni.json"

# Every inline surface the suite promises, by family. Adding a surface to the
# probe without listing it here fails test_registry_lists_every_surface --
# that red is the signal to extend the pin, not to delete it.
EXPECTED_SURFACES = {
    "hub": ["hub"],
    "rw": ["rw1", "rw2", "rw3_open_pos", "rw3_open_neg", "rw3_pd",
           "rw1tilt", "rw_diag3", "rw3_fric"],
    "thruster": ["thr_fresh_05", "thr_fresh_10", "thr_writeonce",
                 "thr_frozen", "thr_lag10", "thr_offset"],
    "env": ["env_drag", "env_drag_still", "env_srp"],
    "fuel": ["fuel_centered", "fuel_offset", "fuel_drag", "fuel_srp"],
    "fsw": ["fsw"],
    "demo": ["demo_basicOrbit_LEO", "demo_basicOrbit_GTO",
             "demo_basicOrbit_GEO", "demo_attFeedbackRW",
             "demo_momentumDumping", "demo_dragDeorbit", "demo_attitudeGG"],
}
EXTENDED_LEDGERS = ["swap_p1_parity", "fsw_loop", "swap_multi", "swap_speed",
                    "swap_speed_legs", "loop_split", "env_parity_drag",
                    "env_parity_srp", "env_parity_mag", "env_parity_eclipse",
                    "env_parity_envbodies", "hijack_scenarioAttitudeFeedbackRW",
                    "hijack_scenarioBasicOrbit"]


def _mod():
    if not PROBE.exists():
        pytest.skip("bench probe not present")
    spec = importlib.util.spec_from_file_location("bench_stock_vs_omni",
                                                  PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _ledger():
    if not LEDGER.exists():
        pytest.skip("bench_stock_vs_omni probe not run on this host yet")
    doc = json.loads(LEDGER.read_text())
    return doc.get("payload", doc)


# ---------------------------------------------------------------------------
# Registry half (host-free)
# ---------------------------------------------------------------------------
def test_registry_lists_every_surface():
    m = _mod()
    assert set(m.SURFACES) == {k for fam in EXPECTED_SURFACES.values()
                               for k in fam}, (
        "probe SURFACES drifted from the test pin -- extend both")
    for fam, keys in EXPECTED_SURFACES.items():
        got = [k for k, s in m.SURFACES.items() if s["family"] == fam]
        assert sorted(got) == sorted(keys), "family %s: %s" % (fam, got)


def test_registry_spec_shape():
    m = _mod()
    for key, s in m.SURFACES.items():
        for field in ("family", "kind", "kw", "T", "T_quick", "tol", "tiers"):
            assert field in s, "%s missing %s" % (key, field)
        assert s["T"] > 0 and s["T_quick"] > 0, key
        assert s["T_quick"] <= s["T"], "%s smoke longer than standard" % key
        assert set(s["tiers"]) <= {"quick", "standard", "full", "mc_smoke",
                                   "demos"}, key
        assert all(v > 0 for v in s["tol"].values()), "%s tol" % key


def test_tier_nesting():
    """quick/smoke-tier sets nest inside the fuller tiers (no silent drop)."""
    m = _mod()
    quick = {k for k, s in m.SURFACES.items() if "quick" in s["tiers"]}
    standard = {k for k, s in m.SURFACES.items() if "standard" in s["tiers"]}
    full = {k for k, s in m.SURFACES.items() if "full" in s["tiers"]}
    demos = {k for k, s in m.SURFACES.items() if "demos" in s["tiers"]}
    allcells = set(m.SURFACES)
    assert quick <= standard <= full <= allcells, (
        sorted(quick - standard), sorted(standard - full))
    assert demos <= full <= allcells
    assert set(m.MC_TIER_INLINE) <= set(m.SURFACES)
    assert set(m.TIERS) == {"quick", "standard", "full", "mc", "demos",
                            "all"}
    for tier, par in m.TIERS.items():
        assert par["mc_n"] >= 0 and par["mc_kind"] in ("pd", "open"), tier
    assert m.TIERS["demos"]["mc_n"] == 0  # demos tier is setups, not MC
    assert m.TIERS["mc"]["mc_n"] >= 1000
    assert m.TIERS["all"]["mc_n"] == 100  # uniform per-campaign default
    assert m.TIERS["all"]["extended"] and m.TIERS["all"]["ladder"]


def test_hijack_tier_wiring():
    """demos/all drive the whole hijack registry; others leave it off."""
    m = _mod()
    assert m.TIERS["demos"]["hijack"] and m.TIERS["all"]["hijack"]
    for tier in ("quick", "standard", "full", "mc"):
        assert not m.TIERS[tier]["hijack"], tier
    # the full tier still covers its two canonical hijack cells via EXTENDED
    assert {"hijack_scenarioAttitudeFeedbackRW",
            "hijack_scenarioBasicOrbit"} <= {e[1] for e in m.EXTENDED}
    assert callable(getattr(m, "_run_hijack_sweep", None))
    assert callable(getattr(m, "_hijack_registry", None))
    assert callable(getattr(m, "_hijack_excluded", None))


def test_hijack_registry_matches_sibling_probe():
    """The sweep enumerates exactly hijack_demo.CONFIG's runnable set."""
    m = _mod()
    if not HIJACK_PROBE.exists():
        pytest.skip("hijack probe not present")
    spec = importlib.util.spec_from_file_location("hijack_demo_x",
                                                  HIJACK_PROBE)
    hm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hm)
    want = {k: c["mode"] for k, c in hm.CONFIG.items()
            if c["mode"] != "excluded"}
    got = m._hijack_registry()
    assert set(got) == set(want), "bench sweep drifted from hijack CONFIG"
    for name, cfg in got.items():
        assert cfg["mode"] in ("parity", "reject"), name
        assert cfg["file"] == name + ".py", name


def test_bench_hijack_all_recorded():
    """demos/all ledger ACCOUNTS for every hijack CONFIG entry.

    Runnable legs must PASS (the owning probe's verdict); the documented
    ``excluded`` entries must appear too, marked skipped with their reason.
    A demo missing from the block would read as a silent gap."""
    d = _ledger()
    if "hijack" not in d:
        pytest.skip("hijack registry sweep runs in demos/all tiers only")
    m = _mod()
    want = {"hijack_%s" % Path(c["file"]).stem
            for c in m._hijack_registry().values()}
    excluded = m._hijack_excluded()
    got = set(d["hijack"])
    assert want <= got, "missing hijack cells: %s" % sorted(want - got)
    assert set(excluded) <= got, ("missing excluded hijack cells: %s"
                                  % sorted(set(excluded) - got))
    for name, rec in d["hijack"].items():
        assert rec["pass"], "%s exit=%s status=%s verdict=%s" % (
            name, rec["exit"], rec["ledger_status"], rec["ledger_verdict"])
    for name, reason in excluded.items():
        rec = d["hijack"][name]
        assert rec["ledger_status"] == "skipped-excluded", name
        assert rec.get("reason") == reason, name


def test_quick_stays_quick():
    """Smoke bound: short legs, bounded total tick count."""
    m = _mod()
    dt = 0.1
    total = 0
    for key, s in m.SURFACES.items():
        if "quick" not in s["tiers"]:
            continue
        assert s["T_quick"] <= 60.0, "%s T_quick=%s" % (key, s["T_quick"])
        total += int(round(s["T_quick"] / dt))
    # ~21 surfaces x 2 arms; 20k ticks keeps the smoke at minutes, not hours.
    assert total <= 20000, total
    assert m.TIERS["quick"]["mc_n"] <= 4
    assert m.TIERS["mc"]["mc_n"] >= 1000


def test_tolerances_sane():
    """Switching-class bounds sit above the quiet class (ordering, not noise)."""
    m = _mod()
    q = m.TOL_QUIET["r_BN_N"]
    assert m.TOL_THR["r_BN_N"] >= 100 * q
    assert m.TOL_WO["r_BN_N"] >= q
    assert m.TOL_FUEL["r_BN_N"] >= m.TOL_THR["r_BN_N"]
    assert m.TOL_FSW["r_BN_N"] >= m.TOL_THR["r_BN_N"]
    assert m.TOL_DRAG_STILL["r_BN_N"] <= m.TOL_DRAG["r_BN_N"]
    for key, s in m.SURFACES.items():
        if s["family"] in ("rw", "hub"):
            assert s["tol"]["r_BN_N"] <= 1e-9, key


def test_mc_bounds_honest():
    """MC bounds are tighter than the Part-A code assert (5e-4 deg) with the
    wp7 record (2.7e-13 deg) far below -- margin on both sides."""
    m = _mod()
    assert 0 < m.MC_ATT_TOL_DEG <= 1e-6
    assert m.MC_ATT_TOL_DEG <= 5e-4
    assert m.MC_TRANSIENT_ATT_DEG == pytest.approx(5e-4)
    assert 0 < m.MC_WHEEL_TOL_RPM <= 1e-3
    assert 0 < m.MC_RATE_TOL_DEGS <= 1e-6


def test_extended_covers_remaining_surfaces():
    """Mag / eclipse / envbodies / multi / speed / loop-split all aggregate
    through the extended sweep -- no surface benches nowhere."""
    m = _mod()
    ledgers = [entry[1] for entry in m.EXTENDED]
    assert sorted(ledgers) == sorted(EXTENDED_LEDGERS)
    assert "env_parity_envbodies" in ledgers  # honest NO-GO, recorded


def test_demo_cells_reference_registry():
    """Every demo SURFACES entry names a DEMOS metadata record (upstream
    scenario + deviations); horizons sane (smoke <= canonical)."""
    m = _mod()
    demo_keys = [k for k, s in m.SURFACES.items() if s["family"] == "demo"]
    assert demo_keys, "no demo cells registered"
    for key in demo_keys:
        name = m.SURFACES[key]["kw"]["demo"]
        assert name in m.DEMOS, "%s missing DEMOS[%s]" % (key, name)
        rec = m.DEMOS[name]
        assert rec["scenario"].endswith(".py"), name
        assert rec["url"].startswith("https://"), name
        assert isinstance(rec["deviations"], list), name
        assert m.SURFACES[key]["T"] >= m.SURFACES[key]["T_quick"] > 0, key


def test_demo_coverage_resolves():
    """DEMO_COVERAGE: live cells exist, reject cells exist, every other
    entry carries a reason. No upstream dynamics demo is unaccounted for."""
    m = _mod()
    assert m.DEMO_COVERAGE, "empty coverage registry"
    for demo, disp in m.DEMO_COVERAGE.items():
        status, _, detail = disp.partition(":")
        assert status in ("live", "covered-by", "reject", "out"), demo
        assert detail.strip(), "%s has no detail" % demo
        if status == "live":
            for cell in detail.split(","):
                assert cell in m.SURFACES, "%s -> unknown %s" % (demo, cell)
        if status == "reject":
            for cell in detail.split(","):
                assert cell in m.DEMO_REJECTS, \
                    "%s -> unknown %s" % (demo, cell)
    # Spot pins: the flagship demo is live, the wrench demo is a loud reject.
    assert "live:demo_attFeedbackRW" in m.DEMO_COVERAGE[
        "scenarioAttitudeFeedbackRW"]
    assert "covered-by:" in m.DEMO_COVERAGE[
        "scenarioAttitudeFeedback (extForceTorque PD)"]


def test_run_all_wires_bench():
    text = (REPO / "audit_bundle" / "run_all.sh").read_text()
    assert "bench_" in text, "run_all.sh does not run the bench suite"


def test_probe_lists_surfaces_without_basilisk():
    """--list-surfaces is stdlib-only: works on bare hosts (incl. this one)."""
    p = subprocess.run([sys.executable, str(PROBE), "--list-surfaces"],
                       capture_output=True, text=True, timeout=60)
    assert p.returncode == 0, p.stderr[-1000:]
    for key in ("hub", "rw3_pd", "thr_fresh_05", "env_srp",
                "fuel_centered", "fsw"):
        assert key in p.stdout


def test_probe_rejects_unknown_surface_without_basilisk():
    p = subprocess.run(
        [sys.executable, str(PROBE), "--surfaces", "nope_nope"],
        capture_output=True, text=True, timeout=60)
    assert p.returncode == 2
    assert "unknown surfaces" in p.stdout + p.stderr


def test_probe_rejects_unknown_mc_kind():
    p = subprocess.run(
        [sys.executable, str(PROBE), "--mc-kinds", "pd,nope_nope"],
        capture_output=True, text=True, timeout=60)
    assert p.returncode == 2
    assert "unknown mc-kinds" in p.stdout + p.stderr


# ---------------------------------------------------------------------------
# Ledger half (host-only; skip without the ledger)
# ---------------------------------------------------------------------------
def test_bench_ledger_green():
    d = _ledger()
    assert d["status"] == "ok", d.get("verdict")
    assert d["verdict"] == "bench green", d.get("verdict")
    assert d["tier"] in ("quick", "standard", "full", "mc", "demos", "all")


def test_bench_cells_pass():
    d = _ledger()
    assert d["cells"], "ledger has no cells"
    for key, cell in d["cells"].items():
        assert cell["pass"], "%s fails=%s vacuity=%s" % (
            key, cell["fails"], cell["vacuity"])
        assert cell["nanos_ok"] is True, "%s nanos audit" % key
        assert cell["stock_wall_s"] > 0 and cell["omni_wall_s"] > 0, key


def test_bench_rejects_all_loud():
    d = _ledger()
    if "rejects" not in d:
        pytest.skip("tier ran with --surfaces (rejects not forced)")
    assert d["rejects"].get("verdict") == "all loud"
    for k, rec in d.get("demo_rejects", {}).items():
        assert rec.get("pass"), "%s not loud: %s" % (k, rec.get("raised"))


def test_bench_ladder_linear():
    d = _ledger()
    if "ladder" not in d:
        pytest.skip("ladder runs in the full tier only")
    lad = d["ladder"]
    assert lad["pass"] is True
    rows = lad["rows"]
    dv1, dv2, dv3 = rows["dt0.1"]["dv"], rows["dt0.05"]["dv"], rows["dt0.02"]["dv"]
    assert 0.3 < dv2 / dv1 < 0.7, (dv1, dv2)
    assert 0.2 < dv3 / dv2 < 0.6, (dv2, dv3)


def test_bench_mc_campaign():
    d = _ledger()
    if "mc" not in d:
        pytest.skip("no MC block in this ledger")
    mc = d["mc"]
    assert mc["n"] >= 4
    assert mc["pass"] is True, mc["fails"][:3]
    assert mc["pass_rate"] == 1.0
    assert len(mc["samples"]) == mc["n"]
    if mc.get("demo"):
        # Demo campaign: margin against the cell tol (pass < 1.0), worst
        # diffs recorded against the same tol.
        m = _mod()
        assert mc["demo"] in m.DEMOS
        assert mc["bounds"]["margin_pass_below"] == 1.0
        assert mc["p95_margin"] is not None and mc["p95_margin"] < 1.0
        for f, v in mc["worst"].items():
            assert v < mc["bounds"]["tol"][f], (f, v)
    else:
        assert mc["worst"]["att"] <= mc["bounds"]["att_deg"]
        assert mc["p95_att"] <= mc["bounds"]["att_deg"]


def test_bench_mc_campaigns_all():
    """All-tier ledger: every campaign present and green (compat 'mc' = pd)."""
    d = _ledger()
    if "mc_campaigns" not in d:
        pytest.skip("no all-tier campaign block in this ledger")
    m = _mod()
    expect = {"pd", "open"} | set(m.DEMOS)
    assert set(d["mc_campaigns"]) == expect, (
        sorted(set(d["mc_campaigns"]) ^ expect))
    for kind, rec in d["mc_campaigns"].items():
        assert rec["pass"], "%s fails=%s" % (kind, rec["fails"][:2])
        assert rec["pass_rate"] == 1.0, kind
        assert rec["n"] == d["mc_campaigns"]["pd"]["n"], kind
        if not rec.get("demo"):
            continue
        # Demo campaign: the dispersed bound is the canonical cell tol except
        # for two documented derivations (the r_BN_N float-resolution floor,
        # and the momentum-dumping wheels switch allowance). Pin that shape so
        # the bound cannot be silently relaxed, and keep headroom visible.
        import math as _math
        b = rec["bounds"]
        assert "cell_tol" in b, kind
        assert set(b["tol"]) == set(b["cell_tol"]), kind
        for f, t in b["tol"].items():
            assert t >= b["cell_tol"][f], (kind, f)
            widened_ok = (f == "r_BN_N"
                          or (kind == "momentumDumping" and f == "wheels"))
            if not widened_ok:
                assert t == b["cell_tol"][f], (kind, f)
            if f in rec["worst"]:
                assert rec["worst"][f] < t, (kind, f, rec["worst"][f], t)
        oe = m._DEMO_MC_OE.get(kind)
        if oe and b["tol"].get("r_BN_N", 0) > b["cell_tol"].get("r_BN_N", 0):
            # A few ULPs of the orbit scale, not an ad-hoc number.
            assert b["tol"]["r_BN_N"] <= 16 * _math.ulp(float(oe[0])), kind
    assert d["mc"]["pass"] is True  # compat primary (pd) stays green


def test_bench_drag_liveness_selfcalibrating():
    """Drag cells prove liveness by arm-to-arm density agreement, not a flat
    absolute floor (which would misread the 240-260 km campaign window as
    vacuous air at rho down to ~6e-14 kg/m^3)."""
    d = _ledger()
    cells = d.get("cells", {})
    dragged = [k for k, c in cells.items()
               if "density_ratio" in c.get("diffs", {})]
    assert dragged, "no drag cell records density_ratio"
    for k in dragged:
        r = cells[k]["diffs"]["density_ratio"]
        assert 0.5 < r < 2.0, (k, r)



def test_mc_kind_accepts_demo_names():
    """--mc-kind accepts every DEMOS name (dispersed-setup campaigns)."""
    m = _mod()
    for name in m.DEMOS:
        args = m.parse_args(["--mc-kind", name])
        assert args.mc_kind == name
    for name in ("pd", "open"):
        assert m.parse_args(["--mc-kind", name]).mc_kind == name


def test_bench_extended_all_recorded():
    d = _ledger()
    if "extended" not in d:
        pytest.skip("extended sweep runs in the full tier only")
    for name in EXTENDED_LEDGERS:
        assert name in d["extended"], "missing ext %s" % name
        rec = d["extended"][name]
        assert rec["pass"], "%s exit=%s status=%s" % (
            name, rec["exit"], rec["ledger_status"])
