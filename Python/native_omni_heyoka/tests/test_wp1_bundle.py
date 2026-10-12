"""Static bundle tests for WP1 saturation probe (no sim, fast)."""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
WP1 = REPO / "audit_bundle" / "probes" / "wp1_sat.py"
COMMON = REPO / "audit_bundle" / "probes" / "common.py"
PLAN = REPO / "PLAN_ENVELOPE.md"


def test_wp1_sat_exists():
    assert WP1.exists(), f"missing {WP1}"
    src = WP1.read_text()
    assert "_one_wheel_sim" in src
    assert "save_result('wp1_sat'" in src or 'save_result("wp1_sat"' in src


def test_common_save_result_api_present():
    if COMMON.exists():
        src = COMMON.read_text()
        assert "def save_result" in src, "common.py lacks save_result"
    else:
        # common.py assumed on macOS host; locally assert the import contract.
        src = WP1.read_text()
        assert "from probes.common import" in src
        assert "save_result" in src


def test_plan_omega_max_by_message_struck():
    assert PLAN.exists(), f"missing {PLAN}"
    text = PLAN.read_text()
    assert "setter-only" in text, "PLAN must record Omega_max setter-only"
    assert "no `Omega_max`" in text or "no Omega_max" in text or "absent from" in text
