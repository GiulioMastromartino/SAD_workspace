"""Eclipse-link firing parity gates: stock-vs-omni at matched dt (§A1).

Ledger: audit_bundle/results/env_parity_eclipse.json
(probe env_parity_eclipse.py -- SRP-cannonball carrier + Eclipse field,
LEO 400 km still starting at 100° from the sun line so the 600 s leg at
matched dt=0.1 contains the penumbra/umbra ENTRY plus both regimes; the
shadow series is computed stock-side in both arms, omni only consumes
shadowFactor, so edge timing is common-mode).

Tolerances (measured basis, never invented): the pre-registered
FSW-loop switching class (tests/test_fsw_loop.py docstring bounds:
r<2e-2, v<1e-4, sigma<2e-4, omega<1e-4, dcm<2e-3 -- the shadow-edge
switching envelope). Ledger-measured eclipse norms (r 1.3e-8) clear
those bounds by >=6 orders of magnitude.
Shadow vacuity (link must modulate IN the leg -- a missed entry fails
loudly, never records vacuous parity): umbra reached (shadow_min < 0.5)
AND full sun present (shadow_max > 0.99). Carrier vacuity:
single-tick SRP-only dv within 10% of analytic (swap_p1 pattern).
Mirror spot-check: eclipse head forwarded + SRP sun link.
Step audit (L14): steps == calls == ticks.

Pure-JSON proof pins (host-free shape): skip (fail open, never fail)
when the probe ledger is absent on this host.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "env_parity_eclipse.json"

TICKS = 6000
TOL = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
       "omega_BN_B": 1e-4, "dcm_max": 2e-3}


def _load():
    if not LEDGER.exists():
        pytest.skip("env_parity_eclipse probe not run on this host yet")
    return json.loads(LEDGER.read_text())["payload"]


def test_leg_ledger_green():
    d = _load()
    assert d["status"] == "ok"
    assert d["verdict"] == "parity green"
    assert d["dt"] == pytest.approx(0.1)
    assert d["ticks"] == TICKS


def test_eclipse_state_parity():
    """State norms within the FSW-loop switching class."""
    c = _load()["cells"]["leg"]
    for f, tol in TOL.items():
        assert c[f] < tol, f"leg.{f}={c[f]:.3e}"


def test_eclipse_shadow_vacuity():
    """Link modulated in-leg: umbra reached AND full sun present."""
    c = _load()["cells"]["leg"]
    assert c["shadow_min"] < 0.5, (
        f"leg vacuous no umbra shadow_min={c['shadow_min']}")
    assert c["shadow_max"] > 0.99, (
        f"leg vacuous no full sun shadow_max={c['shadow_max']}")
    assert c["first_shadowed_tick"] is not None


def test_eclipse_carrier_live():
    """Carrier live: single-tick dv matches analytic within 10%."""
    c = _load()["cells"]["leg"]
    assert 0.9 < c["srp_single_tick_ratio"] < 1.1, (
        f"leg vacuous single-tick ratio={c['srp_single_tick_ratio']:.4f}")


def test_eclipse_mirror_record():
    """Wired-path spot-check: eclipse head + SRP sun link forwarded."""
    m = _load()["cells"]["leg"]["mirror"]["env"]
    assert m["eclipse"]["n_eclipse"] == 1
    assert m["eclipse"]["eclipse_link"] == "eclipseInMsg.subscribeTo"
    assert m["srp"]["sun_link"] == "sunStateInMsg.subscribeTo"


def test_step_count_audit():
    """L14: exactly one integrate step and one call per tick."""
    c = _load()["cells"]["leg"]
    assert c["steps"] == TICKS, f"leg steps={c['steps']}"
    assert c["calls"] == TICKS, f"leg calls={c['calls']}"


def test_loadavg_provenance():
    """L9: host load recorded (contention noted, never gated)."""
    c = _load()["cells"]["leg"]
    assert isinstance(c["loadavg"], list) and len(c["loadavg"]) == 3
