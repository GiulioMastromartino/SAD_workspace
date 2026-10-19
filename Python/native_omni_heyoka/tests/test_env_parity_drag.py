"""Drag-family firing parity gates: stock-vs-omni at matched dt (list.md §A1).

Ledger: audit_bundle/results/env_parity_drag.json
(probe env_parity_drag.py -- LEO 350 km still, com=[0,0,0], wind-linked
corotation, dt=0.1, 600 s through the WIRED mirror_stock_config path).

Tolerances (measured basis, never invented):
  r<2e-5, v<1e-7, sigma<1e-7, omega<1e-9 -- the swap_p1 env_drag_still
  class (tests/test_swap_p1.py::test_p1_env_parity; ledger-measured
  still norms r 4.2e-6 / v 1.4e-8, so the adopted bounds carry >=5x
  margin over the measured floor).
Firing proof: density_max above leo_drag_sim.DENSITY_FIRING_GATE (1e-13)
and macroscopic decay (zeros-vs-zeros vacuity ban, DESIGN A13).
Mirror spot-check: the wired record carries the parity fixture
(swap_p1_parity.py:456-467 shape: CdA=11, atmo linked, wind forwarded).
Step audit (L14): steps == calls == ticks on the single-spacecraft path.

Pure-JSON proof pins (host-free shape): skip (fail open, never fail)
when the probe ledger is absent on this host.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "env_parity_drag.json"

TICKS = 6000
TOL = {"r_BN_N": 2e-5, "v_BN_N": 1e-7, "sigma_BN": 1e-7,
       "omega_BN_B": 1e-9}
FIRING_GATE = 1e-13


def _load():
    if not LEDGER.exists():
        pytest.skip("env_parity_drag probe not run on this host yet")
    return json.loads(LEDGER.read_text())["payload"]


def test_leg_ledger_green():
    d = _load()
    assert d["status"] == "ok"
    assert d["verdict"] == "parity green"
    assert d["dt"] == pytest.approx(0.1)
    assert d["ticks"] == TICKS


def test_drag_state_parity():
    """State norms within the measured still-leg class."""
    c = _load()["cells"]["leg"]
    for f, tol in TOL.items():
        assert c[f] < tol, f"leg.{f}={c[f]:.3e}"


def test_drag_firing_proof():
    """Effect live: density above the firing gate, orbit decayed."""
    c = _load()["cells"]["leg"]
    assert c["density_max"] > FIRING_GATE, (
        f"leg vacuous density_max={c['density_max']:.3e}")
    assert c["decay_m"] > 0.0, f"leg vacuous decay={c['decay_m']:.3e}"


def test_drag_mirror_record():
    """Wired-path spot-check: the mirror record carries the fixture."""
    m = _load()["cells"]["leg"]["mirror"]["env"]["drag"]
    assert m["n_drag"] == 1 and m["class"] == "DragDynamicEffector"
    assert m["CdA"] == pytest.approx(2.2 * 5.0)
    assert m["comOffset"] == [0.0, 0.0, 0.0]
    assert m["atmo"] == "linked" and m["wind"] == "forwarded"


def test_step_count_audit():
    """L14: exactly one integrate step and one call per tick."""
    c = _load()["cells"]["leg"]
    assert c["steps"] == TICKS, f"leg steps={c['steps']}"
    assert c["calls"] == TICKS, f"leg calls={c['calls']}"


def test_loadavg_provenance():
    """L9: host load recorded (contention noted, never gated)."""
    c = _load()["cells"]["leg"]
    assert isinstance(c["loadavg"], list) and len(c["loadavg"]) == 3
