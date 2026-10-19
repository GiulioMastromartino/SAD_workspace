"""Upstream-demo hijack gates (hijack_demo.py).

The probe fetches byte-pinned upstream scenarios, generates the ported file
(upstream + marked swap block only), diff-gates it against upstream, runs
stock vs omni arms, and compares. THIS file runs on bare pytest (stdlib
only): config-shape pins + subprocess smoke + ledger gates (skip without
host ledgers).
"""
from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PROBE = REPO / "audit_bundle" / "probes" / "hijack_demo.py"
RESULTS = REPO / "audit_bundle" / "results"


def _mod():
    if not PROBE.exists():
        pytest.skip("hijack probe not present")
    spec = importlib.util.spec_from_file_location("hijack_demo", PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _ledger(name):
    p = RESULTS / (name + ".json")
    if not p.exists():
        pytest.skip("%s probe not run on this host yet" % name)
    doc = json.loads(p.read_text())
    return doc.get("payload", doc)


# ---------------------------------------------------------------------------
# Config half (host-free)
# ---------------------------------------------------------------------------
def test_hijack_config_shape():
    m = _mod()
    assert m.TAG.startswith("v") and len(m.TAG) > 1
    assert m.BASE_URL.startswith("https://") and m.TAG in m.BASE_URL
    for key, c in m.CONFIG.items():
        assert c["file"] == key + ".py", key
        assert len(c["sha256"]) == 64, key
        assert c["mode"] in ("parity", "reject", "excluded"), key
        if c["mode"] == "excluded":
            assert c["note"], key
            continue
        for field in ("sim", "sc", "task", "note"):
            assert c[field], "%s missing %s" % (key, field)
        assert isinstance(c["tol"], dict), key


def test_hijack_covers_bench_demos_or_says_why():
    """Every live bench demo has a hijack disposition (no silent gaps)."""
    m = _mod()
    expect = {
        "demo_attFeedbackRW": ("scenarioAttitudeFeedbackRW", "parity"),
        "demo_momentumDumping": ("scenarioMomentumDumping", "excluded"),
        "demo_basicOrbit_LEO": ("scenarioBasicOrbit", "parity"),
        "demo_basicOrbit_GTO": ("scenarioBasicOrbit", "parity"),
        "demo_basicOrbit_GEO": ("scenarioBasicOrbit", "parity"),
        "demo_dragDeorbit": ("scenarioDragDeorbit", "parity"),
        "demo_attitudeGG": ("scenarioAttitudeGG", "parity"),
    }
    for bench_cell, (demo, mode) in expect.items():
        assert demo in m.CONFIG, bench_cell
        assert m.CONFIG[demo]["mode"] == mode, (bench_cell, demo)


def test_hijack_markers_paired():
    m = _mod()
    assert m.SWAP_BEGIN != m.SWAP_END and m.DUMP_BEGIN != m.DUMP_END
    assert "OMNI-HIJACK" in m.SWAP_BEGIN and "OMNI-HIJACK" in m.DUMP_BEGIN


def test_hijack_swap_block_uses_repo_swap_by_path():
    """The port must mirror through the CURRENT repo swap.py (file path),
    not the installed wheel's lagging copy -- the wheel has no env readers
    and rejects drag/SRP/mag outright. Guarded by the OMNI_REPO_SWAP env
    handoff so the generated port stays free of host paths."""
    m = _mod()
    cfg = {"sim": "sim", "sc": "sc", "task": "task", "rw": None, "dump": None,
           "thr_factory": None, "thr_cmd": None}
    joined = "\n".join(m._swap_block(cfg))
    assert "OMNI_REPO_SWAP" in joined
    assert "from omniheyoka.swap import mirror_stock_config" not in joined
    assert "from omniheyoka.omni_heyoka import OmniVecIntegrator" in joined
    assert "spec_from_file_location" in joined
    # a path import, never a sys.path mutation (conftest.py rationale)
    assert "sys.path.insert" not in joined
    assert "sys.path.append" not in joined


def test_hijack_drag_deorbit_capped_leg():
    """scenarioDragDeorbit is a capped-horizon parity leg (it used to be
    ``excluded``): the two injected deviations, the measured coarse-dt tol
    class and the generator emission are pinned so a later edit cannot
    silently drop the leg or relax the bound without saying so."""
    m = _mod()
    c = m.CONFIG["scenarioDragDeorbit"]
    assert c["mode"] == "parity"
    assert c["cap_s"] == 1200.0
    assert c["set_central"] == "planet"
    assert c["tol"]["r_BN_N"] > 0 and c["tol"]["v_BN_N"] > 0
    note = c["note"].lower()
    assert "clamp" in note or "cap" in note
    assert "dt=15" in note or "dt = 15" in note
    assert m.UPSTREAM_ALL["scenarioDragDeorbit.py"][1].startswith("parity")
    # the generator emits both deviations inside the marked block ...
    ccfg = {"sim": "sim", "sc": "sc", "task": "task", "rw": None,
            "dump": None, "thr_factory": None, "thr_cmd": None,
            "cap_s": 1200.0, "set_central": "planet"}
    joined = "\n".join(m._swap_block(ccfg))
    assert "_hjk_cap_ns" in joined and "ConfigureStopTime" in joined
    assert "planet.isCentralBody = True" in joined
    # ... and only when a CONFIG entry asks for them
    plain = "\n".join(m._swap_block(
        {"sim": "sim", "sc": "sc", "task": "task", "rw": None, "dump": None,
         "thr_factory": None, "thr_cmd": None}))
    assert "ConfigureStopTime" not in plain
    assert "isCentralBody" not in plain


def test_probe_lists_demos_without_basilisk():
    p = subprocess.run([sys.executable, str(PROBE), "--list-demos"],
                       capture_output=True, text=True, timeout=60)
    assert p.returncode == 0, p.stderr[-1000:]
    for key in ("scenarioAttitudeFeedbackRW", "scenarioMomentumDumping",
                "scenarioBasicOrbit", "scenarioAttitudeGG"):
        assert key in p.stdout


def test_probe_rejects_unknown_demo():
    p = subprocess.run([sys.executable, str(PROBE), "--demo", "nope_nope"],
                       capture_output=True, text=True, timeout=60)
    assert p.returncode != 0


def test_hijack_second_batch_config():
    """The six verified follow-ups: JupiterArrival parity + five extTorque
    rejects (2T family, OrbitManeuverTH, NoEarth). Var names verified
    against the v2.12.0 tag sources before registration."""
    m = _mod()
    assert m.CONFIG["scenarioJupiterArrival"]["mode"] == "parity"
    assert m.CONFIG["scenarioJupiterArrival"]["task"] == "simTaskName"
    for name in ("scenarioAttitudeFeedback2T_TH",
                 "scenarioAttitudeFeedback2T_stateEffTH",
                 "scenarioAttitudeFeedbackNoEarth"):
        assert m.CONFIG[name]["mode"] == "reject", name
    assert m.CONFIG["scenarioOrbitManeuverTH"]["mode"] == "parity"
    assert m.CONFIG["scenarioAttitudeFeedback2T"]["mode"] == "parity"
    assert m.CONFIG["scenarioAttitudeFeedback2T"]["task"] == "dynTaskName"
    assert m.CONFIG["scenarioAttitudeFeedbackNoEarth"]["task"] == "simTaskName"


def test_hijack_dynamics_promotion_batch():
    """The dynamics-promotion batch: the additional upstream demos with a
    dynamics task promoted to live coverage -- three light-demand parity
    cells (RWPower, centered dipole, TAM) and the extTorque/state-effector
    rejects. Pins mode + task var so a later edit can't silently drop a leg."""
    m = _mod()
    for name in ("scenarioAttitudeFeedbackRWPower",
                 "scenarioMagneticFieldCenteredDipole",
                 "scenarioTAM"):
        assert m.CONFIG[name]["mode"] == "parity", name
        assert m.CONFIG[name]["task"] == "simTaskName", name
        assert m.CONFIG[name]["tol"], name
    for name in ("scenarioAttitudeGuidance",
                 "scenarioAttitudePointing",
                 "scenarioAttitudePointingPy",
                 "scenarioAttLocPoint",
                 "scenarioAttGuideHyperbolic",
                 "scenarioSweepingSpacecraft",
                 "scenarioInertialSpiral",
                 "scenarioGroundLocationImaging",
                 "scenarioGroundMapping"):
        assert m.CONFIG[name]["mode"] == "reject", name
    for name in ("scenarioAttitudeFeedback", "scenarioAttitudeSteering"):
        assert m.CONFIG[name]["mode"] == "parity", name
        assert m.CONFIG[name]["task"] == "simTaskName", name
        assert m.CONFIG[name]["tol"], name
    # numba demos stay out of live CONFIG (numba absent on the host); they are
    # recorded as na with a reason instead.
    for name in ("scenarioAttitudeFeedbackNumba",
                 "scenarioAttitudePointingNumba"):
        assert name not in m.CONFIG, name
        path = next(f for f in m.UPSTREAM_ALL if f.endswith(name + ".py"))
        assert m.UPSTREAM_ALL[path][1].startswith("na:"), name


def _bench_mod():
    repo = REPO / "audit_bundle" / "probes" / "bench_stock_vs_omni.py"
    if not repo.exists():
        pytest.skip("bench probe not present")
    spec = importlib.util.spec_from_file_location("bench_for_hijack_test",
                                                  repo)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_upstream_inventory_complete():
    """The registry pins exactly the 187 upstream .py files (no gaps, no
    phantoms): 180 scenario scripts + SunLineKF helper + 6 in-directory
    support scripts. Adding an upstream file without a row fails here."""
    m = _mod()
    assert len(m.UPSTREAM_ALL) == 187, len(m.UPSTREAM_ALL)
    keys = set(m.UPSTREAM_ALL)
    assert len(keys) == 187  # no duplicate rows
    for f in keys:
        assert f.endswith(".py"), f
        assert not f.startswith("/") and ".." not in f, f
    # Spot-checks across every directory, incl. obscure files a hand-written
    # registry is most likely to drop or mistype.
    must_have = [
        "scenarioBasicOrbit.py",
        "scenarioDragDeorbit.py",
        "scenarioGaussMarkovRandomWalk.py",
        "scenarioBskLog.py",
        "scenarioDataDemo.py",
        "scenarioVizPoint.py",
        "scenarioTempMeasurementAttitude.py",
        "MultiSatBskSim/scenariosMultiSat/scenario_constellationFromTle.py",
        "BskSim/scenarios/scenario_AddRWFault.py",
        "BskSim/scenarios/scenario_AttEclipse.py",
        "BskSim/scenarios/scenario_FeedbackRW.py",
        "MultiSatBskSim/scenariosMultiSat/scenario_StationKeepingMultiSat.py",
        "OpNavScenarios/scenariosOpNav/scenario_CNNAttOD.py",
        "OpNavScenarios/scenariosOpNav/OpNavMC/MonteCarlo.py",
        "OpNavScenarios/scenariosOpNav/CNN_ImageGen/scenario_CNNImages.py",
        "MonteCarloExamples/scenarioRerunMonteCarlo.py",
        "mujoco/scenarioUnbalancedThrusters.py",
        "mujoco/BSK_mujocoMasters.py",
        "dynamicsComparison/sweepVariableMassPendulumInertia.py",
        "dynamicsComparison/runAllComparisons.py",
        "dynamicsComparison/_comparePlots.py",
        "SunLineKF_test_utilities.py",
    ]
    for f in must_have:
        assert f in keys, "missing row: %s" % f


def test_upstream_inventory_resolves():
    """Every row's disposition points at something real: live cells exist
    in the bench registry, reject cells exist, hijack modes match the
    hijack CONFIG, everything else carries a reason."""
    m = _mod()
    b = _bench_mod()
    for path, (bench, hijack) in m.UPSTREAM_ALL.items():
        status, _, detail = bench.partition(":")
        assert status in ("live", "reject", "covered", "out", "audit"), path
        assert detail.strip(), "%s has no detail" % path
        if status == "live":
            for cell in detail.split(","):
                assert cell in b.SURFACES, "%s -> unknown cell %s" % (
                    path, cell)
        if status == "reject":
            # Cell name is the first token; a parenthetical note may follow.
            assert detail.split()[0] in b.DEMO_REJECTS, "%s -> unknown %s" % (
                path, detail)
        hstatus, _, hdetail = hijack.partition(":")
        assert hstatus in ("parity", "reject", "excluded", "na"), path
        if hstatus in ("parity", "reject"):
            name = path.rsplit("/", 1)[-1].replace(".py", "")
            assert name in m.CONFIG, "%s hijack missing CONFIG" % path
            assert m.CONFIG[name]["mode"] == hstatus, (path, hstatus)
        else:
            assert hdetail.strip(), "%s hijack has no reason" % path


# ---------------------------------------------------------------------------
# Ledger half (host-only; skip without the ledger)
# ---------------------------------------------------------------------------
def test_hijack_attitude_feedback_rw_green():
    d = _ledger("hijack_scenarioAttitudeFeedbackRW")
    assert d["status"] == "ok", d.get("verdict")
    assert d["verdict"] == "parity green", d.get("verdict")
    assert d["mode"] == "parity"
    assert d["marked_blocks"] == 2  # swap + dump, nothing else
    assert d["stock_exit"] == 0 and d["omni_exit"] == 0
    assert isinstance(d["speedup"], (int, float)) and d["speedup"] > 0
    assert d["tag"] == _mod().TAG


def test_hijack_basic_orbit_green():
    d = _ledger("hijack_scenarioBasicOrbit")
    assert d["status"] == "ok", d.get("verdict")
    assert d["verdict"] == "parity green", d.get("verdict")
    assert d["marked_blocks"] == 2
    assert isinstance(d["speedup"], (int, float)) and d["speedup"] > 0


def test_hijack_drag_deorbit_green():
    """The capped-horizon drag leg: parity green, live, and inside its
    measured coarse-dt class (the bound is pinned, not just recorded)."""
    d = _ledger("hijack_scenarioDragDeorbit")
    assert d["status"] == "ok", d.get("verdict")
    assert d["verdict"] == "parity green", d.get("verdict")
    assert d["mode"] == "parity"
    assert d["marked_blocks"] == 2
    assert d["stock_exit"] == 0 and d["omni_exit"] == 0
    assert d["vacuity"] == []
    for f in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
        assert d["diffs"][f] < d["tol"][f], (f, d["diffs"][f], d["tol"][f])
    assert d["tag"] == _mod().TAG
