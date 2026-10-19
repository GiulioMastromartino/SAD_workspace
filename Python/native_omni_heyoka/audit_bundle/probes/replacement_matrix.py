"""Replacement-matrix probe: 100%-visibility state vector for the swap.

Operational definition (Phase 3): three target scripts run unmodified except
the two integrator lines. "100% = complete replacement" means every stock
feature in that scope is either MIRRORED (parity-gated) or LOUDLY REJECTED,
and every gap is a tracked GAP cell -- never silent.

This probe is HOST-FREE (stdlib only; never imports Basilisk/the wheel). It
records the CURRENT state; tests/test_replacement_matrix.py enforces it:
  - proof ledgers fresh + green  -> red on stale proof or regression;
  - reject surface intact        -> red if a raise is removed/weakened;
  - GAP cells still open         -> red when a gap closes (forces registry update);
  - doc contradictions           -> recorded agree/divergent per snippet.

Ledger: audit_bundle/results/replacement_matrix.json (via common.save_result).
Run:  PYTHONPATH=audit_bundle:probes python3 audit_bundle/probes/replacement_matrix.py
      (also covered by run_all.sh replacement_*.py glob).
"""

from __future__ import annotations

import ast
import glob
import json
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import RESULTS_DIR, save_result  # noqa: E402

REPO = Path(__file__).resolve().parents[2]          # native_omni_heyoka/
PY_REPO = REPO.parent                                # Python/ (cubesat_12u_sim lives here)
SWAP = REPO / "omniheyoka" / "swap.py"
VEC_HPP = REPO / "include" / "omni" / "vec_integrator.hpp"

# Ledgers that PROVE Phase-2.6 claims -> sensitive sources whose mtime must
# predate the ledger ts. Archaeology ledgers are recorded, never freshness-gated.
SENSITIVE = {
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

# Permanent-reject registry: (cell, swap.py trigger substring, gaps-test name).
REJECT_CELLS = [
    ("nbody", "exactly one central body", "test_reject_nbody_two_bodies"),
    ("unmirrorable-dynamics", "use omni env setters instead", "test_reject_unmirrorable_env_effector"),
    ("multi-rw", "registration order is ambiguous", "test_reject_two_rw_effectors"),
    ("rw-logs", "fewer rwOutMsgs", "test_reject_rw_logs_fewer_than_wheels"),
    ("jitter", "only BalancedWheels is mirrored", "test_reject_jitter_wheel"),
    ("per-object-explicit", "needs explicit", "test_reject_per_object_needs_explicit_lists"),
    ("sync-empty", "need >= 1 object spec", "test_reject_sync_empty_specs"),
    ("thr-no-factory", "thr_factory given", "test_reject_thruster_no_factory"),
    ("thr-join", "geometry join needs", "test_reject_thruster_join_mismatch"),
    ("thr-order", "order mismatch", "test_reject_thruster_order_mismatch"),
    ("min-pulse", "MinOnTime=", "test_reject_min_pulse"),
    ("cutoff-degenerate", "degenerate", "test_reject_cutoff_zero"),
    ("blowdown", "out of the staircase envelope (M6)", "test_reject_blowdown"),
    ("swirl", "swirl is out", "test_reject_swirl"),
    ("thr-cmd-msg", "no thr_cmd_msg", "test_reject_thruster_no_cmd_msg"),
    ("multi-tank", "mirrors exactly one", "test_reject_multi_tank"),
    ("tank-thrusterless", "no mirrored thrusters", "test_reject_tank_without_thrusters"),
]

# Per-family env-mirror functions in swap.py (list.md §A1 wiring batch,
# PLAN_ENV_MIRROR §8: the §8 wiring diff added _mirror_drag / _mirror_srp
# / _mirror_mag / _mirror_eclipse_link / _mirror_env_bodies (+ _mirror_j2
# inside env-bodies), all delegating to the staged env_mirror_* readers --
# so this list names the §8 surface, NOT the pre-wiring placeholders.
# Absent entries mean wiring removed (tripwire); an empty list is the
# wired state. NOTE: env_setters_unmirrored below counts only LITERAL
# oi.* calls in swap.py text, so delegation-covered setters still list --
# that static definition is superseded by the delegation asserts in
# tests/test_replacement_matrix.py::test_env_mirror_gap_open (documented
# there, not redefined here).
ENV_READER_GAP = ["_mirror_drag", "_mirror_srp", "_mirror_mag",
                  "_mirror_eclipse_link", "_mirror_env_bodies",
                  "_mirror_j2"]


def _mtime(path: Path) -> float | None:
    try:
        return path.stat().st_mtime
    except OSError:
        return None


def _mirror_surface() -> dict:
    src = SWAP.read_text(encoding="utf-8")
    tree = ast.parse(src)
    mirrored: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == "_MIRRORED":
                    mirrored = set(ast.literal_eval(node.value))
    entry_points: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == "__all__":
                    entry_points = list(ast.literal_eval(node.value))
    raises: list[dict] = []
    for i, line in enumerate(src.splitlines(), 1):
        if re.search(r"^\s*raise ValueError\(", line):
            ctx = " ".join(src.splitlines()[i:i + 3])[:160]
            raises.append({"line": i, "context": ctx})
    called = set(re.findall(r"\boi\.(\w+)", src))
    hpp = VEC_HPP.read_text(encoding="utf-8")
    decls: dict[str, int] = {}
    for i, line in enumerate(hpp.splitlines(), 1):
        m = re.search(r"void\s+(\w+)\s*\(", line)
        if m:
            decls.setdefault(m.group(1), i)
    env_roots = ("J2", "Drag", "SRP", "Sun", "Magnet", "MtbDipole",
                 "EnvironmentBody", "Density", "Atmo", "Wind", "Eclipse")
    # NOTE: substring (not startswith) -- every setter is prefixed set/add.
    gap = [{"setter": s, "hpp_line": decls[s]}
           for s in sorted(decls)
           if any(r in s for r in env_roots)
           and s not in called
           and s.replace("ForObject", "") not in called]
    return {"mirrored_set": sorted(mirrored),
            "entry_points": entry_points,
            "n_reject_sites": len(raises),
            "reject_sites": raises,
            "oi_calls": sorted(called),
            "env_setters_unmirrored": gap,
            "env_reader_functions_absent": [f for f in ENV_READER_GAP
                                            if f"def {f}(" not in src]}


def _proof_ledgers() -> dict:
    out: dict = {}
    for name, sensitive in SENSITIVE.items():
        p = RESULTS_DIR / f"{name}.json"
        if not p.exists():
            out[name] = {"exists": False}
            continue
        try:
            doc = json.loads(p.read_text(encoding="utf-8"))
        except Exception as exc:
            out[name] = {"exists": True, "unreadable": f"{type(exc).__name__}"}
            continue
        payload = doc.get("payload", doc) if isinstance(doc, dict) else {}
        cells = payload.get("cells", {}) if isinstance(payload, dict) else {}
        ts_raw = doc.get("timestamp_utc") if isinstance(doc, dict) else None
        try:
            import datetime
            ts = (datetime.datetime.fromisoformat(str(ts_raw)).timestamp()
                  if ts_raw else None)
        except Exception:
            ts = None
        if ts is None:
            ts = _mtime(p)
        src_mtimes = {s: _mtime(REPO / s) for s in sensitive}
        newest = max((m for m in src_mtimes.values() if m), default=None)
        fresh = bool(ts and newest and ts >= newest)
        out[name] = {
            "exists": True,
            "timestamp_utc": ts_raw,
            "status": payload.get("status"),
            "verdict": payload.get("verdict"),
            "top_keys": sorted(payload.keys())[:24] if isinstance(payload, dict) else [],
            "cell_keys": sorted(cells.keys()) if isinstance(cells, dict) else [],
            "sensitive_newest_mtime": newest,
            "fresh": fresh,
            "margin_s": round(ts - newest, 1) if ts and newest else None,
        }
    # Citation artifact (hand-written, no common.py envelope): record + mtime only.
    arm = RESULTS_DIR / "arm_rule_discrimination.json"
    if arm.exists():
        try:
            adoc = json.loads(arm.read_text(encoding="utf-8"))
            out["arm_rule_discrimination"] = {
                "exists": True, "envelope": "hand-written (no timestamp_utc)",
                "role": str(adoc.get("role", ""))[:120],
                "live_gates": adoc.get("live_gates", []),
                "sweep_points": sorted((adoc.get("stock_drag_arm_sweep", {})
                                        .get("points_h_com__to__step_z", {}).keys())),
            }
        except Exception as exc:
            out["arm_rule_discrimination"] = {"exists": True, "unreadable": str(exc)}
    else:
        out["arm_rule_discrimination"] = {"exists": False}
    return out


def _acceptance_scripts() -> dict:
    demo = PY_REPO / "cubesat_12u_sim" / "backends" / "basilisk" / "basilisk_sim.py"
    d: dict = {"path": str(demo), "exists": demo.exists()}
    if demo.exists():
        t = demo.read_text(encoding="utf-8", errors="replace")
        d.update({"has_run_simulation": "def run_simulation(" in t,
                  "imports_vizSupport": "vizSupport" in t,
                  "has_sun_ephemeris": "SunEphemeris" in t,
                  "has_shadow_sidecar": "HeyokaShadowPropagator" in t})
    bb = PY_REPO / "cubesat_12u_sim" / "backends" / "basilisk"
    leo_hits: list[str] = []
    if bb.is_dir():
        for f in sorted(bb.glob("*.py")):
            low = f.name.lower()
            if "drag" in low or "leo" in low:
                leo_hits.append(f.name)
        for f in sorted(bb.glob("*.py")):
            try:
                if "dragDynamicEffector" in f.read_text(encoding="utf-8", errors="replace"):
                    leo_hits.append(f"{f.name}:dragDynamicEffector")
            except OSError:
                pass
    st, yr = (PY_REPO / "cubesat_12u_sim" / "missions" / n
              for n in ("run_stationkeeping.py", "run_3year.py"))
    s: dict = {"stationkeeping_exists": st.exists(), "year3_exists": yr.exists()}
    for key, p in (("stationkeeping", st), ("year3", yr)):
        if p.exists():
            t = p.read_text(encoding="utf-8", errors="replace")
            s[key] = {"has_setIntegrator": "setIntegrator" in t,
                      "imports_heyoka_propagate": "heyoka_propagate" in t}
    return {"demo_script_1": d,
            "leo_drag_script_2": {"exists": bool(leo_hits), "hits": leo_hits},
            "stationkeeping_script_3": s}


def _doc_pins() -> dict:
    plan = (REPO / "PLAN_ENVELOPE.md").read_text(encoding="utf-8", errors="replace")
    design = (REPO / "DESIGN.md").read_text(encoding="utf-8", errors="replace")
    build = (REPO / "BUILD_STATUS.md").read_text(encoding="utf-8", errors="replace")

    def bc_signs(text: str) -> list:
        return [(m.group(1), i + 1)
                for i, line in enumerate(text.splitlines())
                for m in [re.search(r"r_BN_N\s*=\s*r_CN_N\s*(\S)", line)] if m]

    plan_signs, design_signs = bc_signs(plan), bc_signs(design)
    return {
        "bc_sign": {"plan": plan_signs, "design": design_signs,
                    "agree": bool(plan_signs) and bool(design_signs)
                    and {s for s, _ in plan_signs} == {s for s, _ in design_signs}},
        "reconciled_markers": {
            "design_superseded_by_measurement": "superseded-by-measurement" in design,
            "design_reconciliation_note": "Reconciliation note:" in design,
            "plan_decision3_locked": "Decision 3 locked" in plan,
            "build_filter_deleted": "is DELETED" in build,
            "build_cutoff_inert": "INERT" in build,
            "build_a7_struck": "~~Top-level" in build,
        },
    }


def _phase3_gaps() -> dict:
    # NOTE: status_manifest.json + replacement_matrix.json describe the gaps
    # themselves -- they are never evidence that a gap closed. Exclude them.
    SELF_DESCRIPTIVE = {"status_manifest.json", "replacement_matrix.json"}
    texts = {}
    for p in sorted(RESULTS_DIR.glob("*.json")):
        if p.name in SELF_DESCRIPTIVE:
            continue
        try:
            texts[p.name] = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            pass
    speedup_hits = [n for n, t in texts.items()
                    if re.search(r'"cold"|"warm"|per_step|speedup', t)]
    build_s_files = [n for n, t in texts.items() if '"build_s"' in t]
    fsw_hits = [n for n in texts if "fsw" in n.lower()] + \
               [n for n, t in texts.items() if re.search(r"fsw_loop|fsw-in-the-loop", t, re.I)]
    return {"speedup_table": {"ledger_hits": speedup_hits, "open": not speedup_hits,
                              "build_s_recorded_in": sorted(build_s_files)},
            "fsw_loop": {"ledger_hits": sorted(set(fsw_hits)), "open": not fsw_hits,
                         "pieces": {
                             "test_vec_mrp": (REPO / "tests" / "test_vec_mrp_switch.py").exists(),
                             "closed_loop_ledger": (RESULTS_DIR / "closed_loop_fuel.json").exists(),
                             "test_swap_multi": (REPO / "tests" / "test_swap_multi.py").exists()}}}


def main() -> int:
    src = SWAP.read_text(encoding="utf-8")
    payload = {
        "scope": ("Phase-3 two-line swap on 3 scripts; every stock feature is "
                  "MIRRORED / REJECTS-LOUDLY / GAP -- never silent."),
        "mirror_surface": _mirror_surface(),
        "reject_cells": [
            {"cell": cell,
             "trigger_present": trig in src,
             "trigger_line": next((i for i, ln in enumerate(src.splitlines(), 1)
                                   if trig in ln), None),
             "gaps_test": test}
            for cell, trig, test in REJECT_CELLS],
        "proof_ledgers": _proof_ledgers(),
        "acceptance_scripts": _acceptance_scripts(),
        "doc_pins": _doc_pins(),
        "phase3_gaps": _phase3_gaps(),
        "how_to_refresh": ("run_all.sh (replacement_*.py glob) or "
                           "PYTHONPATH=audit_bundle:probes python3 "
                           "audit_bundle/probes/replacement_matrix.py; host-gated "
                           "ledgers need Basilisk 2.12.0 + built wheel."),
    }
    out = save_result("replacement_matrix", payload)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
