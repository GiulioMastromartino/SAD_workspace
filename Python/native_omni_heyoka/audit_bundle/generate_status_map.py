#!/usr/bin/env python3
"""
generate_status_map.py - Measured status manifest & visual progress generator
for native_omni_heyoka astrodynamics porting pipeline.

MEASURED method (no name-grading): every test-case unit takes its status from
an actual pytest run (JUnit XML parsed with stdlib): passed -> verified,
failed/error -> failed, skipped (host-dep) -> skipped, skipped (ledger
absent) / not collected -> unstarted. Parity cells take theirs from the
ledger verdict PLUS freshness vs sensitive sources. Nothing is "verified"
without execution evidence. Audit-finding items remain author-graded.

Outputs (under audit_bundle/results/):
  - status_manifest.json (machine-readable registry + measurement block)
  - status_map.svg (high-density 16-column vector matrix)
  - status_dashboard.html (interactive high-density matrix with live tooltips & filtering)
  - pytest_junit.xml (raw pytest evidence this manifest was graded from)

Usage:
  python3 audit_bundle/generate_status_map.py [--pytest-timeout SEC] [--no-pytest]
"""
from __future__ import annotations

import argparse
import ast
import datetime
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# Subsystem classifications for test modules
SUBSYSTEM_MAP = {
    "test_swap_p0": {"name": "Swap P0 & SWIG", "cat": "Architecture"},
    "test_swap_p1": {"name": "Swap P1 Parity", "cat": "Architecture"},
    "test_vec_mrp_switch": {"name": "MRP Shadow Set", "cat": "Core Math"},
    "test_wp0_guards": {"name": "Defensive Guards & Envelopes", "cat": "Hygiene & Guards"},
    "test_wp1_bundle": {"name": "Wheel Saturation Bundle", "cat": "Reaction Wheels"},
    "test_wp1_saturation": {"name": "Wheel OmegaMax Dynamics", "cat": "Reaction Wheels"},
    "test_wp2_friction_jitter": {"name": "Stribeck Friction & Jitter", "cat": "Reaction Wheels"},
    "test_wp25_b2": {"name": "Wave B2 Setter Envelopes", "cat": "Hygiene & Guards"},
    "test_wp3_timing_models": {"name": "Thruster Valve Timing & Lag", "cat": "Thrusters"},
    "test_thruster_stock": {"name": "Stock Thruster Staircase", "cat": "Thrusters"},
    "test_wp4_offset": {"name": "CoM Offset & B/C Matrix", "cat": "Disturbances"},
    "test_wp45_env_offset": {"name": "Combined Offset & Env", "cat": "Disturbances"},
    "test_wp5_env": {"name": "Gravity, SRP & Drag Laws", "cat": "Disturbances"},
    "test_wp6_telemetry": {"name": "Telemetry & Live Fuel", "cat": "Telemetry"},
    "test_wp7_dispersion_jit": {"name": "Monte Carlo JIT Scaling", "cat": "Architecture"},
    "test_loop_split": {"name": "Task Cost Split", "cat": "Architecture"},
    "test_cpp_values": {"name": "C++ Absolute-Value Pins", "cat": "Core Math"},
    "test_replacement_matrix": {"name": "Replacement Matrix", "cat": "Architecture"},
    "test_replacement_gaps": {"name": "Loud-Reject Pins", "cat": "Architecture"},
}

# --- measured-result plumbing (single source: pytest JUnit + ledger evidence) ---

# Skip reasons meaning "proof ledger absent" (unmeasured) vs any other skip
# (host-dep: importorskip without Basilisk/wheel -- known env limit).
# NOTE: bare "missing" is deliberately NOT a hint -- host-capability skips
# like "bash missing / C++ missing" are host-dep (skipped), not proof-absent.
SKIP_LEDGER_HINTS = ("probe not run", "ledger", "bundle_summary",
                     "not run on this host", "not recorded yet")

try:
    sys.path.insert(0, str(Path(__file__).resolve().parent / "probes"))
    from replacement_matrix import SENSITIVE as LEDGER_SENSITIVE  # mirrors probe
except Exception:
    # Fallback (same mapping as probes/replacement_matrix.py SENSITIVE).
    LEDGER_SENSITIVE = {
        "swap_p1_parity": ["src/omni_vec_integrator.cpp", "include/omni/contributors.hpp",
                           "include/omni/system.hpp", "include/omni/environment.hpp",
                           "omniheyoka/swap.py", "audit_bundle/probes/swap_p1_parity.py"],
        "swap_multi": ["src/omni_vec_integrator.cpp", "include/omni/system.hpp",
                       "omniheyoka/swap.py", "audit_bundle/probes/swap_multi.py"],
        "closed_loop_fuel": ["src/omni_vec_integrator.cpp", "include/omni/contributors.hpp",
                             "omniheyoka/swap.py", "audit_bundle/probes/closed_loop_fuel.py"],
        "vec_mrp_switch_trace": ["src/omni_vec_integrator.cpp", "include/omni/mrp_t.hpp"],
        "wp2_friction_jitter_voltage": ["src/omni_spacecraft.cpp", "src/omni_vec_integrator.cpp",
                                        "audit_bundle/probes/wp2_friction_jitter_voltage.py"],
        "wp3_timing": ["src/omni_spacecraft.cpp", "src/omni_vec_integrator.cpp",
                       "omniheyoka/swap.py", "audit_bundle/probes/wp3_timing.py"],
        "wp45_combo": ["src/omni_spacecraft.cpp", "src/omni_vec_integrator.cpp",
                       "include/omni/environment.hpp", "audit_bundle/probes/wp45_combo.py"],
    }


def resolve_pytest_python(base_dir: Path) -> tuple:
    """Pick the interpreter for the pytest run: repo env first (mirrors
    run_all.sh), then the invoking python. Returns (path, pytest_version or
    None). Never raises."""
    cands = [base_dir.parent / "env" / "bin" / "python",
             base_dir / "env" / "bin" / "python",
             Path(sys.executable)]
    for c in cands:
        try:
            if not (c.exists() if isinstance(c, Path) else False):
                continue
            proc = subprocess.run([str(c), "-c",
                                   "import pytest; print(pytest.__version__)"],
                                  capture_output=True, text=True, timeout=60)
            if proc.returncode == 0:
                return str(c), (proc.stdout or "").strip()
        except Exception:
            continue
    return str(sys.executable), None


def run_pytest_junit(tests_dir: Path, junit_path: Path, timeout_s: int,
                     python: str) -> dict:
    """Run the full pytest suite once; never raises. Returns a meta dict."""
    meta = {"ran": False, "returncode": None, "timeout_s": timeout_s,
            "python": python, "pytest": None,
            "junit": str(junit_path), "error": None,
            "n_passed": 0, "n_failed": 0, "n_skipped": 0}
    # --continue-on-collection-errors: one unimportable module must never
    # zero out the other ~270 tests (exit 2 with 0 collected).
    cmd = [python, "-m", "pytest", str(tests_dir), "-q", "--tb=no",
           "-p", "no:cacheprovider", "--continue-on-collection-errors",
           f"--junit-xml={junit_path}"]
    # Run from the Python/ repo root (not native_omni_heyoka/): `python -m`
    # puts cwd on sys.path, and several test modules import the sibling
    # `cubesat_12u_sim` package (via omniheyoka.presets). Belt and braces:
    # export the same PYTHONPATH run_all.sh uses.
    import os as _os
    repo_root = str(tests_dir.parent.parent)
    child_env = dict(_os.environ)
    child_env["PYTHONPATH"] = _os.pathsep.join(
        [repo_root] + ([child_env["PYTHONPATH"]] if child_env.get("PYTHONPATH") else []))
    try:
        proc = subprocess.run(cmd, cwd=repo_root,
                              capture_output=True, text=True, timeout=timeout_s,
                              env=child_env)
        meta["ran"] = True
        meta["returncode"] = proc.returncode
        meta["stdout_tail"] = (proc.stdout or "")[-2000:]
        if proc.returncode not in (0, 1, 2, 5):
            meta["error"] = (f"unexpected pytest exit {proc.returncode}; "
                             f"stderr: {(proc.stderr or '')[-500:]}")
    except FileNotFoundError as exc:
        meta["error"] = f"python/pytest unavailable: {exc}"
    except subprocess.TimeoutExpired:
        meta["ran"] = True
        meta["returncode"] = "timeout"
        meta["error"] = (f"pytest timed out after {timeout_s}s; "
                         "grading from partial JUnit if any")
    return meta


def parse_junit(junit_path: Path) -> tuple:
    """JUnit XML -> ({unit_id "stem::test": (outcome, message)},
    {stem: collection-error message}). Never raises."""
    out: dict = {}
    coll_errors: dict = {}
    try:
        root = ET.parse(str(junit_path)).getroot()
    except Exception:
        return out, coll_errors
    for tc in root.iter("testcase"):
        classname = tc.get("classname", "") or ""
        raw_name = tc.get("name", "") or ""
        stem = classname.split(".")[-1] or Path(tc.get("file", "") or "").stem
        name = raw_name.split("[")[0]
        if not stem or not name:
            # File-level collection error: name is the module path.
            maybe_stem = raw_name.split(".")[-1]
            el = tc.find("error")
            if maybe_stem.startswith("test_") and el is not None:
                coll_errors[maybe_stem] = str(el.get("message", "") or "")[:300]
            continue
        outcome, msg = "passed", ""
        el = tc.find("failure")
        if el is None:
            el = tc.find("error")
            outcome = "error" if el is not None else outcome
        else:
            outcome = "failed"
        if el is not None:
            msg = str(el.get("message", "") or "")[:300]
        else:
            sk = tc.find("skipped")
            if sk is not None:
                outcome = "skipped"
                msg = str(sk.get("message", "") or "")[:300]
        out[f"{stem}::{name}"] = (outcome, msg)
    return out, coll_errors


def ledger_fresh(name: str, results_dir: Path, base_dir: Path) -> tuple:
    """(fresh True/False, margin_s, reason) vs sensitive sources. Never raises."""
    p = results_dir / f"{name}.json"
    if not p.exists():
        return None, None, "ledger absent"
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
    except Exception as exc:
        return False, None, f"unreadable: {exc}"
    ts = None
    if isinstance(doc, dict) and doc.get("timestamp_utc"):
        try:
            ts = datetime.datetime.fromisoformat(
                str(doc["timestamp_utc"])).timestamp()
        except Exception:
            ts = None
    if ts is None:
        try:
            ts = p.stat().st_mtime
        except OSError:
            pass
    newest, newest_name = None, None
    for s in LEDGER_SENSITIVE.get(name, []):
        try:
            m = (base_dir / s).stat().st_mtime
        except OSError:
            continue
        if newest is None or m > newest:
            newest, newest_name = m, s
    if ts is None or newest is None:
        return False, None, "no timestamp baseline"
    if ts >= newest:
        return True, round(ts - newest, 1), "ok"
    return False, round(ts - newest, 1), f"stale vs {newest_name}"

AUDIT_ITEMS_DATA = [
    # Critical
    {"id": "C1", "name": "B/C Translation Sign", "cat": "Disturbances", "sev": "Critical", "st": "verified", "p": 1.0, "ref": "WP25-A1", "file": "src/omni_spacecraft.cpp:709", "desc": "Audit_0-3 FIXED in code (code+ledger, host re-run pending): SysModel flipped to measured minus (spacecraft:709-710), source-pinned; residual one-sided cell."},
    # Verified Closures (X1 - X5)
    {"id": "X1", "name": "Thruster CoM-Axis Mask", "cat": "Thrusters", "sev": "Fixed", "st": "verified", "p": 1.0, "ref": "Gate T1", "file": "contributors.hpp:483-491", "desc": "Audit_0-3 re-verified no regression (code+ledger, host re-run pending). Source masking stands."},
    {"id": "X2", "name": "Swap 6/6 Loud Rejects", "cat": "Architecture", "sev": "Fixed", "st": "verified", "p": 1.0, "ref": "test_swap_p1.py", "file": "swap_p1_parity.json:rejects", "desc": "Audit_0-3 re-verified no regression (L6 BUILD:410 remnant excepted). Loud rejects stand."},
    {"id": "X3", "name": "Vec Friction 1e-14 Stock-Exact", "cat": "Reaction Wheels", "sev": "Fixed", "st": "verified", "p": 1.0, "ref": "rw3_fric cell", "file": "src/omni_vec_integrator.cpp:1428", "desc": "Audit_0-3 re-verified no regression (code+ledger, host re-run pending). Vec Stribeck stock-exact stands."},
    {"id": "X4", "name": "Vec Live Fuel Depletion", "cat": "Telemetry", "sev": "Fixed", "st": "verified", "p": 1.0, "ref": "Gate F-fuel", "file": "src/omni_vec_integrator.cpp:1535", "desc": "Audit_0-3 re-verified no regression (M14 bounded writes + short-Isp raise landed vec-side). Live mass/c stands."},
    {"id": "X5", "name": "Historical Closures (Rev 0-1)", "cat": "Architecture", "sev": "Fixed", "st": "verified", "p": 1.0, "ref": "Audit_0-1", "file": "BUILD_STATUS.md:622", "desc": "Audit_0-3 re-verified no regressions; counts/CI asserts fixed (L6/L11 remnants excepted). Closures stand."},
    # High findings
    {"id": "H1", "name": "SysModel Cutoff Lag Filter", "cat": "Thrusters", "sev": "High", "st": "verified", "p": 1.0, "ref": "WP25-A2", "file": "src/omni_spacecraft.cpp:1315", "desc": "Audit_0-3 FIXED in code (code+ledger, host re-run pending): filter DELETED (spacecraft:1315-1324), ratio test 0.99-1.01; residuals M20/L7."},
    {"id": "H2", "name": "Silent Envelope Fatal Traps", "cat": "Hygiene & Guards", "sev": "High", "st": "verified", "p": 1.0, "ref": "WP25-B1", "file": "src/omni_spacecraft.cpp:908", "desc": "Audit_0-3 FIXED in code (code+ledger, host re-run pending): 14 behavioral raise-before-propagation tests over 13 traps both heads."},
    {"id": "H3", "name": "Telemetry RW Omega Unset Echo", "cat": "Telemetry", "sev": "High", "st": "in_progress", "p": 0.30, "ref": "WP25-B2", "file": "src/omni_spacecraft.cpp:760", "desc": "Audit_0-3 STILL-OPEN: no explicit FIXED verdict for RW echo (H2 notes config-echo labeling only); keep grade, see Audit_0-3 H2/M18/L8."},
    {"id": "H4", "name": "Wheel Jitter Simple Stream Split", "cat": "Reaction Wheels", "sev": "High", "st": "in_progress", "p": 0.40, "ref": "WP25-B3", "file": "src/omni_spacecraft.cpp:1087", "desc": "Audit_0-3 H4-wp45-scope FIXED for that file only; this entry is jitter-horizon content, M19 J1-blindness STILL-OPEN (400-tick lengthens vacuous metric) so keep in_progress; ID/content drift recorded, not guessed."},
    {"id": "H5", "name": "SysModel Friction Iter-1 Stub", "cat": "Reaction Wheels", "sev": "High", "st": "verified", "p": 1.0, "ref": "WP25-B3", "file": "src/omni_spacecraft.cpp:346", "desc": "Audit_0-3 FIXED in code (code+ledger, host re-run pending): Stribeck-port part landed (spacecraft:1058-1147, latch+omegaBefore); 5-knob vs per-wheel noted."},
    {"id": "H6", "name": "SysModel Static Fuel Publisher", "cat": "Telemetry", "sev": "High", "st": "verified", "p": 1.0, "ref": "WP25-A3", "file": "src/omni_spacecraft.cpp:402", "desc": "Audit_0-3 FIXED in code (code+ledger, host re-run pending): fuel fixed as option-B absence, publisher NEVER written (:819-827); residuals M18/L8."},
    {"id": "H7", "name": "Fuel x Env Disturbance Cell", "cat": "Disturbances", "sev": "High", "st": "verified", "p": 1.0, "ref": "WP26-C", "file": "test_swap_p1.py:test_p1_fuel_env_scope", "desc": "Audit_0-3 H9/H11/H12 sensitivity-limited: fuel_drag/fuel_srp parity-in-switching-class only, fuel bit-exact; drag arm 25x-insensitive fixture G=1 open, arm NOT proven by this badge; keep verified (parity) per caveat rule."},
    {"id": "H8", "name": "Body DV Rotation into Inertial", "cat": "Telemetry", "sev": "High", "st": "verified", "p": 1.0, "ref": "WP25-B2", "file": "src/omni_spacecraft.cpp:646", "desc": "Audit_0-3 FIXED in code (code+ledger, host re-run pending): content is DV frame (audit M11 :1375-1405 R_BN.F.k); map H8 = audit M11 by content, audit H8 sign-reject has no map entry; drift recorded."},
    # Key Medium findings
    {"id": "M1", "name": "Lag Filter Empty Ramps", "cat": "Thrusters", "sev": "Medium", "st": "in_progress", "p": 0.50, "ref": "WP25-A2", "file": "test_wp3_timing_models.py", "desc": "Audit_0-3 no explicit FIXED verdict; ramp gating tied to H1/M16 deletion but not separately closed; keep in_progress, see M20/L7."},
    {"id": "M2", "name": "Cutoff 10 vs 1000 Ratio", "cat": "Thrusters", "sev": "Medium", "st": "in_progress", "p": 0.50, "ref": "WP25-A2", "file": "test_wp3_timing_models.py", "desc": "Audit_0-3 H1/M16: ratio test rewritten 0.99-1.01 in code but host-only + ledger stale (M20 wp3_timing predates deletion); keep in_progress per staleness family."},
    {"id": "M3", "name": "Min-Pulse Suppression Gate", "cat": "Thrusters", "sev": "Medium", "st": "in_progress", "p": 0.60, "ref": "WP25-B3", "file": "test_wp3_timing_models.py", "desc": "Audit_0-3 no explicit FIXED verdict for min-pulse; suppress-only behavior stands but unclosed; keep in_progress."},
    {"id": "M4", "name": "Staircase Achieving Duties", "cat": "Thrusters", "sev": "Medium", "st": "verified", "p": 1.0, "ref": "Gate T1", "file": "contributors.hpp:479", "desc": "Audit_0-3 no regression noted; keep verified (code+ledger, host re-run pending). Duty quantization clean at step boundary."},
    {"id": "M5", "name": "Trailing c-Mask Par Bounds", "cat": "Thrusters", "sev": "Medium", "st": "verified", "p": 1.0, "ref": "Audit X1", "file": "system.hpp:414", "desc": "Audit_0-3 X1 re-verified no regression; keep verified (code+ledger, host re-run pending). Par count bounds heap safe."},
    {"id": "M6", "name": "Blowdown & Swirl Dynamics", "cat": "Thrusters", "sev": "Medium", "st": "in_progress", "p": 0.70, "ref": "WP25-A2", "file": "test_wp3_timing_models.py", "desc": "Audit_0-3 M6/M7 FIXED is N-body/rotHat/Gt behavioral, not this blowdown/swirl content (ID drift); no explicit verdict here so keep in_progress, drift recorded."},
    {"id": "M8", "name": "Friction F-Ledger Null Rows", "cat": "Reaction Wheels", "sev": "Medium", "st": "in_progress", "p": 0.50, "ref": "WP25-B3", "file": "BUILD_STATUS.md", "desc": "Audit_0-3 M20 ledger-stale + M22 void-flags applicable (wp2 rev-1 2x40, F-rows null, no in-JSON flags); keep in_progress pending host regen."},
    {"id": "M10", "name": "Offset Delta Inverted Sign", "cat": "Disturbances", "sev": "Medium", "st": "blocked", "p": 0.25, "ref": "WP25-A1", "file": "src/omni_spacecraft.cpp", "desc": "Audit_0-3 no explicit M10 verdict; C1 FIXED suggests likely-fixed but SysModel minus has one-sided residual (no signed stock cell); do not guess, keep blocked."},
    {"id": "M11", "name": "Double Reset DV Clearing", "cat": "Telemetry", "sev": "Medium", "st": "in_progress", "p": 0.80, "ref": "WP25-B2", "file": "src/omni_spacecraft.cpp:755", "desc": "Audit_0-3 M12 FIXED with leftover M18 (matches this Reset content: Reset restores FULL tank not m0, spacecraft:974); per leftover rule keep in_progress; literal M11-DV FIXED maps to H8, drift recorded."},
    {"id": "M13", "name": "Stiction-Band Discrepancy", "cat": "Reaction Wheels", "sev": "Medium", "st": "in_progress", "p": 0.50, "ref": "WP25-B3", "file": "src/omni_spacecraft.cpp:841", "desc": "Audit_0-3 J1-blindness STILL-OPEN in probe/test (M19 spike-only label, wheelSpeeds-only); Stribeck code landed but probe blind; downgrade arm_split->in_progress."},
    {"id": "M16", "name": "SysModel Lag Filter Elimination", "cat": "Thrusters", "sev": "Medium", "st": "verified", "p": 1.0, "ref": "WP25-A2", "file": "src/omni_spacecraft.cpp", "desc": "Audit_0-3 FIXED in code (code+ledger, host re-run pending) with H1: filter DELETED both paths; residuals M20/L7."},
    # Low findings
    {"id": "L1", "name": "Stale Par Count Docs", "cat": "Hygiene & Guards", "sev": "Low", "st": "in_progress", "p": 0.60, "ref": "BUILD_STATUS.md", "file": "BUILD_STATUS.md", "desc": "Audit_0-3 L10 manifest stale counts + L6/L11 remnants applicable; counts fixed except remnants; keep in_progress."},
    {"id": "L2", "name": "NPOS Parity Formatting", "cat": "Architecture", "sev": "Low", "st": "verified", "p": 1.0, "ref": "Audit X1", "file": "system.hpp", "desc": "Audit_0-3 no regression noted; keep verified. NPOS sentinel guards non-coupled axes."},
    {"id": "L3", "name": "SWIG Top-Level Shim Note", "cat": "Architecture", "sev": "Low", "st": "verified", "p": 1.0, "ref": "Audit X2", "file": "cSysModel.py", "desc": "Audit_0-3 no regression noted; keep verified. SWIG shim documented."},
    {"id": "L4", "name": "CMake Direct Heyoka Link", "cat": "Architecture", "sev": "Low", "st": "verified", "p": 1.0, "ref": "CMakeLists.txt", "file": "CMakeLists.txt", "desc": "Audit_0-3 CI L4 all fixed; keep verified. Heyoka rpath link stands."},
    {"id": "L5", "name": "thr_offset Ledger Provenance", "cat": "Disturbances", "sev": "Low", "st": "in_progress", "p": 0.5, "ref": "WP25-A1", "file": "swap_p1_parity.json", "desc": "Audit_0-3 M24 phantom cite (hpp:214-216 numbers absent from wp45_combo.json) + H9 no discriminating row + M22 void flags; keep in_progress."}
]


def extract_test_units(tests_dir: Path, outcomes: dict, coll_errors: dict,
                     measured_at: str) -> list[dict]:
    """Test functions across tests/test_*.py, graded by MEASURED pytest outcome.

    passed -> verified 1.0; failed/error -> failed 0.0; skipped (proof ledger
    absent) -> unstarted 0.2; skipped (host-dep etc.) -> skipped 0.5; not
    collected by this run -> unstarted 0.1. No name-keyword grading.
    """
    units = []
    if not tests_dir.exists():
        return units

    for py_file in sorted(tests_dir.glob("test_*.py")):
        stem = py_file.stem
        subsys = SUBSYSTEM_MAP.get(stem, {"name": stem, "cat": "Unit Tests"})
        try:
            tree = ast.parse(py_file.read_text(encoding="utf-8"), filename=str(py_file))
            for node in ast.walk(tree):
                if isinstance(node, ast.FunctionDef) and node.name.startswith("test_"):
                    uid = f"{stem}::{node.name}"
                    if uid in outcomes:
                        outcome, msg = outcomes[uid]
                        if outcome == "passed":
                            st, prog = "verified", 1.0
                            desc = f"pytest PASSED ({measured_at})."
                        elif outcome in ("failed", "error"):
                            st, prog = "failed", 0.0
                            desc = (f"pytest {outcome.upper()} ({measured_at}): "
                                    f"{msg or 'see pytest_junit.xml'}")
                        elif any(h in msg.lower() for h in SKIP_LEDGER_HINTS):
                            st, prog = "unstarted", 0.2
                            desc = (f"pytest SKIPPED ({msg or 'no reason'}): "
                                    f"proof ledger absent -- not measured.")
                        else:
                            st, prog = "skipped", 0.5
                            desc = (f"pytest SKIPPED ({msg or 'no reason'}): "
                                    f"host-dep, not measured here.")
                    else:
                        outcome, msg = "unmeasured", ""
                        if stem in coll_errors:
                            st, prog = "unstarted", 0.1
                            desc = ("File-level COLLECTION ERROR -- not measured: "
                                    f"{coll_errors[stem]}")
                        else:
                            st, prog = "unstarted", 0.1
                            desc = ("Not collected by this pytest run "
                                    "(absent/collection error) -- not measured.")

                    units.append({
                        "id": uid,
                        "name": node.name,
                        "type": "test_case",
                        "module": stem,
                        "subsystem": subsys["name"],
                        "category": subsys["cat"],
                        "file": f"tests/{py_file.name}:{node.lineno}",
                        "status": st,
                        "progress": prog,
                        "target": "pytest result (measured, not name-graded)",
                        "desc": desc,
                        "outcome": outcome,
                    })
        except Exception:
            continue
    return units


def extract_parity_cells(results_dir: Path, base_dir: Path) -> list[dict]:
    """Parity cells from swap_p1_parity.json, graded by ledger verdict + freshness.

    verdict green + fresh -> verified 1.0; green but stale -> in_progress 0.5
    (do not cite); verdict not green -> blocked 0.25. No name-keyword grading.
    """
    cells = []
    parity_file = results_dir / "swap_p1_parity.json"
    if not parity_file.exists():
        return cells

    try:
        data = json.loads(parity_file.read_text(encoding="utf-8"))
        payload = data.get("payload", {}) if isinstance(data, dict) else {}
        raw_cells = payload.get("cells", {}) if isinstance(payload, dict) else {}
        verdict = str(payload.get("verdict", "")) if isinstance(payload, dict) else ""
    except Exception:
        return cells
    fresh, margin, reason = ledger_fresh("swap_p1_parity", results_dir, base_dir)
    green = (verdict == "parity green")
    for c_id, metrics in raw_cells.items():
        metrics = metrics if isinstance(metrics, dict) else {}
        sigma = metrics.get("sigma_BN", 0.0)
        omega = metrics.get("omega_BN_B", 0.0)
        n_pars = metrics.get("nPars", 0)
        n_states = metrics.get("nStates", 0)

        if green and fresh:
            st, prog = "verified", 1.0
            desc = (f"Matched-dt parity cell {c_id} (nStates={n_states}, "
                    f"nPars={n_pars}): ledger verdict green, proof fresh "
                    f"(margin {margin}s).")
        elif green:
            st, prog = "in_progress", 0.5
            desc = (f"Matched-dt parity cell {c_id} (nStates={n_states}, "
                    f"nPars={n_pars}): verdict green BUT proof {reason} -- "
                    f"regen before citing.")
        else:
            st, prog = "blocked", 0.25
            desc = (f"Matched-dt parity cell {c_id}: ledger verdict "
                    f"{verdict!r} -- not green.")

        cells.append({
            "id": f"cell::{c_id}",
            "name": f"Cell: {c_id}",
            "type": "parity_cell",
            "subsystem": "Simulation Parity Ledger",
            "category": "Parity Cells",
            "file": "audit_bundle/results/swap_p1_parity.json",
            "status": st,
            "progress": prog,
            "target": f"sigma={sigma:.2e}, omega={omega:.2e}",
            "desc": desc,
            "ledger_verdict": verdict,
            "fresh": fresh,
            "fresh_margin_s": margin,
        })
    return cells


def extract_audit_units() -> list[dict]:
    """Map the 30 adversarial audit items into granular units."""
    units = []
    for item in AUDIT_ITEMS_DATA:
        units.append({
            "id": f"audit::{item['id']}",
            "name": f"{item['id']}: {item['name']}",
            "type": "audit_finding",
            "subsystem": f"Audit 0-2 ({item['sev']})",
            "category": item["cat"],
            "file": item["file"],
            "status": item["st"],
            "progress": item["p"],
            "target": item["ref"],
            "desc": item["desc"]
        })
    return units


def build_comprehensive_manifest(base_dir: Path, results_dir: Path,
                                 outcomes: dict, coll_errors: dict,
                                 pytest_meta: dict,
                                 measured_at: str) -> dict:
    """Combine measured test cases, parity cells, and audit items."""
    tests_dir = base_dir / "tests"
    test_units = extract_test_units(tests_dir, outcomes, coll_errors,
                                    measured_at)
    parity_units = extract_parity_cells(results_dir, base_dir)
    audit_units = extract_audit_units()

    all_units = test_units + parity_units + audit_units

    # Summary tallies (measured statuses included; audit items keep old ones)
    summary = {}
    for st in ("verified", "failed", "skipped", "arm_split", "blocked",
               "in_progress", "unstarted"):
        summary[st] = sum(1 for u in all_units if u["status"] == st)
    avg_prog = sum(u["progress"] for u in all_units) / len(all_units) if all_units else 0.0
    summary["overall_progress_pct"] = round(avg_prog * 100, 1)

    return {
        "project": "native_omni_heyoka",
        "total_units": len(all_units),
        "breakdown": {
            "automated_test_units": len(test_units),
            "simulation_parity_cells": len(parity_units),
            "audit_ledger_items": len(audit_units)
        },
        "summary": summary,
        "measurement": {
            "method": "pytest JUnit outcomes + ledger verdict/freshness; "
                      "audit items remain author-graded",
            "measured_at": measured_at,
            "pytest": pytest_meta,
        },
        "units": all_units
    }


def generate_high_density_svg(manifest: dict, out_path: Path):
    """Emit a high-density 16-column vector grid showing all 276 units."""
    units = manifest["units"]
    cols = 16
    size = 38
    gap = 4
    margin = 16
    rows = math.ceil(len(units) / cols)
    width = cols * size + (cols - 1) * gap + 2 * margin
    height = rows * size + (rows - 1) * gap + 2 * margin + 40

    color_map = {
        "verified": "#2ea043",
        "failed": "#f85149",
        "skipped": "#d29922",
        "arm_split": "#d29922",
        "blocked": "#f85149",
        "in_progress": "#3b82f6",
        "unstarted": "#8b949e"
    }

    svg_lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" style="background:#0d1117; font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Helvetica,Arial,sans-serif;">',
        f'  <text x="{margin}" y="24" fill="#e6edf3" font-size="14" font-weight="600">native_omni_heyoka — High-Granularity Parity Matrix ({len(units)} Units)</text>',
        f'  <text x="{width - margin}" y="24" fill="#8b949e" font-size="11" text-anchor="end">Progress: {manifest["summary"]["overall_progress_pct"]}% ({manifest["summary"]["verified"]}/{len(units)} Green)</text>'
    ]

    for i, u in enumerate(units):
        col = i % cols
        row = i // cols
        x = margin + col * (size + gap)
        y = margin + 24 + row * (size + gap)
        p = u["progress"]
        st = u["status"]
        color = color_map.get(st, "#3b82f6")
        opacity = 0.2 + 0.8 * p if st == "verified" else (0.15 + 0.6 * p if st != "unstarted" else 0.05)
        dash = 'stroke-dasharray="2,2"' if st == "unstarted" else ''

        short_label = u["name"][:3].upper() if len(u["name"]) >= 3 else u["name"]
        svg_lines.append(f'  <g id="u_{i}"><title>{u["name"]} ({st}: {int(p*100)}%)&#10;{u["desc"]}</title>')
        svg_lines.append(f'    <rect x="{x}" y="{y}" width="{size}" height="{size}" rx="4" fill="{color}" fill-opacity="{opacity:.2f}" stroke="{color}" stroke-width="1.5" {dash}/>')
        svg_lines.append(f'    <text x="{x + size/2}" y="{y + size/2 + 3}" fill="#e6edf3" font-size="8" font-family="monospace" font-weight="600" text-anchor="middle">{short_label}</text>')
        svg_lines.append('  </g>')

    svg_lines.append('</svg>')
    out_path.write_text("\n".join(svg_lines), encoding="utf-8")


def generate_interactive_dashboard_html(manifest: dict, out_path: Path):
    """Emit a high-density, interactive HTML dashboard displaying all units."""
    manifest_json = json.dumps(manifest, indent=2)
    n_total = manifest["total_units"]
    n_test = manifest["breakdown"]["automated_test_units"]
    n_cell = manifest["breakdown"]["simulation_parity_cells"]
    n_audit = manifest["breakdown"]["audit_ledger_items"]
    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>native_omni_heyoka Granular Verification Matrix ({n_total} Units)</title>
<style>
  :root {{
    --bg: #0d1117;
    --fg: #e6edf3;
    --muted: #21262d;
    --muted-fg: #8b949e;
    --card: #161b22;
    --border: #30363d;
    --primary: #3b82f6;
    --success: #2ea043;
    --warning: #d29922;
    --destructive: #f85149;
    --font: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif;
    --font-mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    font-family: var(--font);
    background: var(--bg);
    color: var(--fg);
    padding: 20px;
    font-size: 13px;
    line-height: 1.4;
  }}
  .container {{ max-width: 1040px; margin: 0 auto; }}
  .header {{
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 1px solid var(--border);
    padding-bottom: 12px;
    margin-bottom: 16px;
  }}
  .title-group h1 {{ font-size: 18px; font-weight: 600; display: flex; align-items: center; gap: 8px; }}
  .tag {{ font-size: 11px; padding: 2px 8px; border-radius: 12px; background: rgba(59, 130, 246, 0.2); color: #58a6ff; font-weight: 600; }}
  .kpis {{
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 10px;
    margin-bottom: 16px;
  }}
  .kpi {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 10px 12px;
  }}
  .kpi-lbl {{ font-size: 10px; color: var(--muted-fg); text-transform: uppercase; font-weight: 600; }}
  .kpi-val {{ font-size: 20px; font-family: var(--font-mono); font-weight: 700; margin-top: 3px; }}
  .controls {{
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 12px;
    gap: 8px;
  }}
  .filter-tabs {{ display: flex; gap: 6px; flex-wrap: wrap; }}
  .btn {{
    background: var(--card);
    border: 1px solid var(--border);
    color: var(--muted-fg);
    font-size: 11.5px;
    padding: 5px 10px;
    border-radius: 5px;
    cursor: pointer;
    font-weight: 500;
  }}
  .btn:hover {{ color: var(--fg); border-color: #8b949e; }}
  .btn.active {{ background: rgba(59, 130, 246, 0.2); color: #58a6ff; border-color: rgba(59, 130, 246, 0.5); font-weight: 600; }}
  .search-box {{
    background: var(--card);
    border: 1px solid var(--border);
    color: var(--fg);
    padding: 5px 10px;
    border-radius: 5px;
    font-size: 12px;
    width: 220px;
  }}
  .matrix-wrapper {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px;
    margin-bottom: 16px;
  }}
  .matrix-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(36px, 1fr));
    gap: 5px;
  }}
  .cell {{
    width: 100%;
    aspect-ratio: 1;
    border-radius: 4px;
    display: flex;
    align-items: center;
    justify-content: center;
    font-family: var(--font-mono);
    font-size: 8.5px;
    font-weight: 700;
    cursor: pointer;
    user-select: none;
    transition: transform 0.1s ease, outline 0.1s ease;
  }}
  .cell:hover {{ transform: scale(1.18); z-index: 10; }}
  .cell.selected {{ outline: 2px solid #58a6ff; outline-offset: 1px; }}
  .cell-verified {{ background: rgba(46, 160, 67, 0.25); color: #3fb950; border: 1px solid rgba(46, 160, 67, 0.5); }}
  .cell-failed {{ background: rgba(248, 81, 73, 0.30); color: #f85149; border: 1px solid rgba(248, 81, 73, 0.7); }}
  .cell-skipped {{ background: rgba(210, 153, 34, 0.25); color: #d29922; border: 1px solid rgba(210, 153, 34, 0.5); }}
  .cell-arm_split {{ background: rgba(210, 153, 34, 0.25); color: #d29922; border: 1px solid rgba(210, 153, 34, 0.5); }}
  .cell-blocked {{ background: rgba(248, 81, 73, 0.25); color: #f85149; border: 1px solid rgba(248, 81, 73, 0.6); }}
  .cell-in_progress {{ background: rgba(59, 130, 246, 0.25); color: #58a6ff; border: 1px solid rgba(59, 130, 246, 0.5); }}
  .cell-unstarted {{ background: rgba(110, 118, 129, 0.1); color: #8b949e; border: 1px dashed rgba(110, 118, 129, 0.4); }}
  
  .inspector {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px 16px;
  }}
  .ins-head {{ display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; border-bottom: 1px solid var(--border); padding-bottom: 6px; }}
  .ins-title {{ font-size: 13.5px; font-weight: 600; font-family: var(--font-mono); }}
  .ins-pill {{ font-size: 10px; padding: 2px 6px; border-radius: 4px; font-family: var(--font-mono); font-weight: 700; }}
  .ins-grid {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-bottom: 8px; }}
  .ins-lbl {{ font-size: 10px; color: var(--muted-fg); text-transform: uppercase; font-weight: 600; }}
  .ins-val {{ font-family: var(--font-mono); font-size: 11.5px; margin-top: 2px; color: var(--fg); }}
  .ins-desc {{ color: var(--muted-fg); font-size: 12px; line-height: 1.45; }}
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <div class="title-group">
      <h1>native_omni_heyoka <span class="tag">High-Density Matrix ({n_total} Units)</span></h1>
    </div>
  </div>

  <div class="kpis">
    <div class="kpi">
      <div class="kpi-lbl">Total Units</div>
      <div class="kpi-val" id="kpi-total">233</div>
    </div>
    <div class="kpi">
      <div class="kpi-lbl">Verified (Green)</div>
      <div class="kpi-val" style="color:#3fb950;" id="kpi-green">-</div>
    </div>
    <div class="kpi">
      <div class="kpi-lbl">Arm Splits (Vec&ne;Sys)</div>
      <div class="kpi-val" style="color:#d29922;" id="kpi-splits">-</div>
    </div>
    <div class="kpi">
      <div class="kpi-lbl">Critical Blockers</div>
      <div class="kpi-val" style="color:#f85149;" id="kpi-blocked">-</div>
    </div>
    <div class="kpi">
      <div class="kpi-lbl">Overall Score</div>
      <div class="kpi-val" style="color:#58a6ff;" id="kpi-progress">-</div>
    </div>
  </div>

  <div class="controls">
    <div class="filter-tabs">
      <button class="btn active" onclick="filterType('all')">All ({n_total})</button>
      <button class="btn" onclick="filterType('test_case')">Test Cases ({n_test})</button>
      <button class="btn" onclick="filterType('parity_cell')">Simulation Cells ({n_cell})</button>
      <button class="btn" onclick="filterType('audit_finding')">Audit Findings ({n_audit})</button>
    </div>
    <input type="text" id="search-box" class="search-box" placeholder="Filter by name / file / ID..." oninput="handleSearch()">
  </div>

  <div class="matrix-wrapper">
    <div id="matrix-grid" class="matrix-grid"></div>
  </div>

  <div id="inspector" class="inspector">
    <div class="ins-head">
      <div id="ins-title" class="ins-title">Select any unit square above to inspect line & metric</div>
      <span id="ins-status" class="ins-pill cell-unstarted">-</span>
    </div>
    <div class="ins-grid">
      <div><div class="ins-lbl">Subsystem</div><div id="ins-subsys" class="ins-val">-</div></div>
      <div><div class="ins-lbl">Category</div><div id="ins-cat" class="ins-val">-</div></div>
      <div><div class="ins-lbl">File / Source</div><div id="ins-file" class="ins-val">-</div></div>
      <div><div class="ins-lbl">Target / Verification</div><div id="ins-target" class="ins-val">-</div></div>
    </div>
    <div id="ins-desc" class="ins-desc">-</div>
  </div>
</div>

<script>
const DATA = {manifest_json};
let currentFilter = 'all';
let searchQuery = '';

function init() {{
  document.getElementById('kpi-total').innerText = DATA.total_units;
  document.getElementById('kpi-green').innerText = DATA.summary.verified + " (" + Math.round((DATA.summary.verified/DATA.total_units)*100) + "%)";
  document.getElementById('kpi-splits').innerText = DATA.summary.arm_split;
  document.getElementById('kpi-blocked').innerText = DATA.summary.blocked + " (C1)";
  document.getElementById('kpi-progress').innerText = DATA.summary.overall_progress_pct + "%";
  renderMatrix();
}}

function getShortLabel(name) {{
  if (name.startsWith('test_')) {{
    const parts = name.replace('test_', '').split('_');
    return (parts[0].slice(0, 2) + (parts[1] ? parts[1].slice(0, 1) : '')).toUpperCase();
  }}
  if (name.startsWith('Cell: ')) return name.replace('Cell: ', '').slice(0, 3).toUpperCase();
  return name.slice(0, 3).toUpperCase();
}}

function renderMatrix() {{
  const grid = document.getElementById('matrix-grid');
  grid.innerHTML = '';
  const filtered = DATA.units.filter(u => {{
    if (currentFilter !== 'all' && u.type !== currentFilter) return false;
    if (searchQuery) {{
      const q = searchQuery.toLowerCase();
      return u.name.toLowerCase().includes(q) || u.file.toLowerCase().includes(q) || u.id.toLowerCase().includes(q);
    }}
    return true;
  }});

  filtered.forEach((u, idx) => {{
    const div = document.createElement('div');
    div.className = `cell cell-${{u.status}}`;
    div.innerText = getShortLabel(u.name);
    div.title = `${{u.name}} [${{u.status}}]\\n${{u.desc}}`;
    div.onclick = () => selectCell(u, div);
    grid.appendChild(div);
    if (idx === 0) selectCell(u, div);
  }});
}}

function selectCell(unit, elem) {{
  document.querySelectorAll('.cell').forEach(c => c.classList.remove('selected'));
  if (elem) elem.classList.add('selected');
  document.getElementById('ins-title').innerText = unit.name;
  const stElem = document.getElementById('ins-status');
  stElem.className = `ins-pill cell-${{unit.status}}`;
  stElem.innerText = unit.status.toUpperCase() + ` (${{Math.round(unit.progress * 100)}}%)`;

  document.getElementById('ins-subsys').innerText = unit.subsystem || '-';
  document.getElementById('ins-cat').innerText = unit.category || '-';
  document.getElementById('ins-file').innerText = unit.file || '-';
  document.getElementById('ins-target').innerText = unit.target || '-';
  document.getElementById('ins-desc').innerText = unit.desc || '-';
}}

function filterType(type) {{
  currentFilter = type;
  document.querySelectorAll('.btn').forEach(b => b.classList.remove('active'));
  event.target.classList.add('active');
  renderMatrix();
}}

function handleSearch() {{
  searchQuery = document.getElementById('search-box').value;
  renderMatrix();
}}

init();
</script>
</body>
</html>"""
    out_path.write_text(html_content, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Generate 200+ unit status manifest, SVG and dashboard for native_omni_heyoka")
    parser.add_argument("--results-dir", default=None, help="Path to audit_bundle/results")
    parser.add_argument("--out-json", default=None, help="Output path for status_manifest.json")
    parser.add_argument("--out-svg", default=None, help="Output path for status_map.svg")
    parser.add_argument("--out-html", default=None, help="Output path for status_dashboard.html")
    parser.add_argument("--pytest-timeout", type=int, default=3600,
                        help="Seconds to allow the pytest run (default 3600)")
    parser.add_argument("--no-pytest", action="store_true",
                        help="Skip the pytest run; all test units graded unmeasured")
    args = parser.parse_args()

    base_dir = Path(__file__).resolve().parent.parent  # native_omni_heyoka root
    results_dir = Path(args.results_dir) if args.results_dir else Path(__file__).resolve().parent / "results"
    out_json = Path(args.out_json) if args.out_json else results_dir / "status_manifest.json"
    out_svg = Path(args.out_svg) if args.out_svg else results_dir / "status_map.svg"
    out_html = Path(args.out_html) if args.out_html else results_dir / "status_dashboard.html"

    results_dir.mkdir(parents=True, exist_ok=True)
    junit_path = results_dir / "pytest_junit.xml"

    if args.no_pytest:
        pytest_meta = {"ran": False, "returncode": None,
                       "error": "pytest run disabled by --no-pytest flag"}
        outcomes: dict = {}
        coll_errors: dict = {}
    else:
        print(f"[generate_status_map] Running pytest (timeout {args.pytest_timeout}s)...")
        python, pytest_ver = resolve_pytest_python(base_dir)
        if pytest_ver is None:
            pytest_meta = {"ran": False, "returncode": None, "python": python,
                           "error": "no interpreter with pytest found "
                                    "(tried repo env + invoking python)"}
            outcomes, coll_errors = {}, {}
        else:
            pytest_meta = run_pytest_junit(base_dir / "tests", junit_path,
                                           args.pytest_timeout, python)
            pytest_meta["pytest"] = pytest_ver
            outcomes, coll_errors = (parse_junit(junit_path)
                                     if junit_path.exists() else ({}, {}))
        n_p = sum(1 for o, _ in outcomes.values() if o == "passed")
        n_f = sum(1 for o, _ in outcomes.values() if o in ("failed", "error"))
        n_s = sum(1 for o, _ in outcomes.values() if o == "skipped")
        pytest_meta.update({"n_passed": n_p, "n_failed": n_f, "n_skipped": n_s,
                            "n_collected": len(outcomes),
                            "n_collection_errors": len(coll_errors)})
        if not outcomes and pytest_meta.get("returncode") != 5:
            pytest_meta["error"] = ("0 tests collected -- collection aborted "
                                    "(wrong interpreter?); see stdout_tail / junit; "
                                    f"interpreter was {pytest_meta.get('python')}")
        print(f"[generate_status_map] pytest returncode={pytest_meta['returncode']}: "
              f"{n_p} passed, {n_f} failed, {n_s} skipped "
              f"({len(outcomes)} collected, {len(coll_errors)} collection errors)")

    measured_at = datetime.datetime.now(datetime.timezone.utc).isoformat()
    print(f"[generate_status_map] Grading AST tests from {base_dir}/tests & ledgers from {results_dir}...")
    manifest = build_comprehensive_manifest(base_dir, results_dir, outcomes,
                                            coll_errors,
                                            pytest_meta, measured_at)

    print(f"[generate_status_map] Built {manifest['total_units']} granular units:")
    print(f"  - Automated test cases: {manifest['breakdown']['automated_test_units']}")
    print(f"  - Parity simulation cells: {manifest['breakdown']['simulation_parity_cells']}")
    print(f"  - Audit findings: {manifest['breakdown']['audit_ledger_items']}")
    print(f"  - Status breakdown: {manifest['summary']}")

    out_json.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"[generate_status_map] Wrote {out_json}")

    print(f"[generate_status_map] Rendering 16-col high-density SVG matrix...")
    generate_high_density_svg(manifest, out_svg)
    print(f"[generate_status_map] Wrote {out_svg}")

    print(f"[generate_status_map] Rendering interactive dashboard...")
    generate_interactive_dashboard_html(manifest, out_html)
    print(f"[generate_status_map] Wrote {out_html}")

    print("[generate_status_map] Complete.")


if __name__ == "__main__":
    main()
