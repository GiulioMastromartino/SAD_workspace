"""SRP-cannonball firing parity gates: stock-vs-omni at matched dt (§A1).

Ledger: audit_bundle/results/env_parity_srp.json
(probe env_parity_srp.py -- swap_p1 _env_srp_cell verbatim through the
WIRED mirror_stock_config path: A=10, C_R=1.5 -> cr=0.5, static sun,
full sun, dt=0.1, 600 s).

Tolerances (measured basis, never invented):
  r<1e-6, v<1e-8, sigma<1e-9, omega<1e-9 -- the swap_p1 env_srp class
  (tests/test_swap_p1.py::test_p1_env_parity; ledger-measured r 4.4e-8,
  so the adopted bounds carry >=20x margin).
Vacuity: single-tick SRP-only dv within 10% of analytic P*A*C_R*dt/m
(swap_p1 pattern -- effect live, not zeros-vs-zeros).
Mirror spot-check: cannonball record (args [10, 0.5]) + REQUIRED sun
link on the slot-0 member.
Step audit (L14): steps == calls == ticks.

Pure-JSON proof pins (host-free shape): skip (fail open, never fail)
when the probe ledger is absent on this host.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "env_parity_srp.json"

TICKS = 6000
TOL = {"r_BN_N": 1e-6, "v_BN_N": 1e-8, "sigma_BN": 1e-9,
       "omega_BN_B": 1e-9}


def _load():
    if not LEDGER.exists():
        pytest.skip("env_parity_srp probe not run on this host yet")
    return json.loads(LEDGER.read_text())["payload"]


def test_leg_ledger_green():
    d = _load()
    assert d["status"] == "ok"
    assert d["verdict"] == "parity green"
    assert d["dt"] == pytest.approx(0.1)
    assert d["ticks"] == TICKS


def test_srp_state_parity():
    """State norms within the measured SRP-cannonball class."""
    c = _load()["cells"]["leg"]
    for f, tol in TOL.items():
        assert c[f] < tol, f"leg.{f}={c[f]:.3e}"


def test_srp_live_vs_analytic():
    """Carrier live: single-tick dv matches analytic within 10%."""
    c = _load()["cells"]["leg"]
    assert 0.9 < c["srp_single_tick_ratio"] < 1.1, (
        f"leg vacuous single-tick ratio={c['srp_single_tick_ratio']:.4f}")


def test_srp_mirror_record():
    """Wired-path spot-check: cannonball args + sun link forwarded."""
    m = _load()["cells"]["leg"]["mirror"]["env"]["srp"]
    entry = m["srp"][0]
    assert entry["kind"] == "cannonball"
    assert entry["args"] == [10.0, pytest.approx(0.5)]
    assert m["sun_link"] == "sunStateInMsg.subscribeTo"
    assert m["n_sun_sources"] == 1


def test_step_count_audit():
    """L14: exactly one integrate step and one call per tick."""
    c = _load()["cells"]["leg"]
    assert c["steps"] == TICKS, f"leg steps={c['steps']}"
    assert c["calls"] == TICKS, f"leg calls={c['calls']}"


def test_loadavg_provenance():
    """L9: host load recorded (contention noted, never gated)."""
    c = _load()["cells"]["leg"]
    assert isinstance(c["loadavg"], list) and len(c["loadavg"]) == 3
