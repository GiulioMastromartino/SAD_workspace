"""WP0 guards + layout hygiene (static only -- runs in Linux CI, no sim).

Covers PLAN_ENVELOPE.md Rev 1 WP0/WP7 section, the Risks trap table and
Ground rules 7-9. Nothing here imports Basilisk: the layout module is loaded
from its file path (bypassing ``omniheyoka/__init__``, which needs heyoka),
and the CI script is exercised as a subprocess (skipped when bash is absent).
"""
from __future__ import annotations

import importlib.util
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
CI_SCRIPT = REPO / "scripts" / "ci_envelope_checks.sh"
LAYOUT_PY = REPO / "omniheyoka" / "layout.py"
PLAN = REPO / "PLAN_ENVELOPE.md"

# Every trap row of the Rev 1 Risks loud-reject table, plus Ground rules 7-9
# headers. Each keyword must appear verbatim in PLAN_ENVELOPE.md.
TRAP_KEYWORDS = [
    "useRWfriction",      # Stribeck on
    "jitter",             # Jitter on (numRWJitter / U_s,U_d / theta states)
    "voltage",            # Voltage wired
    "u_min",              # u_min / P_max non-default
    "P_max",
    "useMinPulseTime",    # Min-pulse on
    "ThrusterOnRamp",     # Ramps set
    "cutoffFrequency",    # Default cutoff
    "blowdown",           # Blowdown / fuel / swirl / force-msg
    "swirl",
    "force-msg",
    "USE_J2",             # Harmonics / J2 on
    "r_BcB_B",            # Offset nonzero
    "bskError",           # Required behavior until modeled
    "Loud rejection",     # Ground rule 7
    "JIT budget",         # Ground rule 8
    "MC dispersion",      # Ground rule 9
]


def _load_layout():
    """Load omniheyoka/layout.py by path (numpy only, no Basilisk/heyoka)."""
    pytest.importorskip("numpy")
    assert LAYOUT_PY.exists(), f"missing {LAYOUT_PY}"
    spec = importlib.util.spec_from_file_location(
        "omniheyoka_layout_wp0", str(LAYOUT_PY))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_ci_envelope_checks_pass_on_current_tree():
    assert CI_SCRIPT.exists(), f"missing {CI_SCRIPT}"
    if shutil.which("bash") is None:
        pytest.skip("bash missing -- cannot run ci_envelope_checks.sh here")
    proc = subprocess.run(
        ["bash", str(CI_SCRIPT)], cwd=str(REPO),
        capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, (
        f"ci_envelope_checks.sh failed (rc={proc.returncode}):\n"
        f"--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}")


def test_layout_name_api_present():
    layout = _load_layout()
    for name in ("par_names", "par_index", "sigma_slice"):
        assert callable(getattr(layout, name, None)), (
            f"omniheyoka/layout.py lacks required name-based API {name!r}")
    # Functional smoke: resolution by name, never by hard-coded count.
    names = ["u1", "u2", "u3", "mass", "Ixx"]
    assert layout.Layout(names).index("Ixx") == 4
    assert layout.par_index("Ixx", names=names) == 4
    hub = ["rx", "ry", "rz", "vx", "vy", "vz",
           "s1", "s2", "s3", "omx", "omy", "omz", "W1"]
    assert layout.sigma_slice(names=hub) == slice(6, 9)


def test_plan_trap_list_documented():
    assert PLAN.exists(), f"missing {PLAN}"
    text = PLAN.read_text(encoding="utf-8")
    missing = [k for k in TRAP_KEYWORDS if k not in text]
    assert not missing, f"PLAN_ENVELOPE.md omits trap keywords: {missing}"
