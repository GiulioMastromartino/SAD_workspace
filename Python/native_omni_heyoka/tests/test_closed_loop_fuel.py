"""WP-H closed-loop fuel gates (single-tank scope): PD + burn + depletion in the loop.

Ledger: audit_bundle/results/closed_loop_fuel.json (probe closed_loop_fuel.py).
Bounds (pre-registered from the swap_p1 classes at matched dt=0.1, 600 s leg):
  r_BN_N < 2e-2, v_BN_N < 1e-4 (fuel-cell switching class, 5 N burn);
  sigma_BN < 2e-4 (thruster switching class TOL_THR_SIG -- the PD law
  stabilizes the leg); omega_BN_B < 1e-4 (fuel-cell 1e-4 scale);
  dcm_max < 2e-3 (switch-invariant rotation, fuel-cell class);
  wheels < 1e-4 (coupled-state switching bound, test_swap_multi class);
  fuel_rel_err < 1e-9 (depletion law near-bit-exact, fuel-cell class).
Step/MRP audit: ticks == 6000 (600 s @ dt=0.1); MRP switch totals within
+-2 of stock (test_swap_multi mechanism). Vacuity: burned > 0.1 kg and
both-arm displacement > 1e5 m (effects live, not zeros-vs-zeros).

Host-only sims: skipped (no failure) without Basilisk/the built wheel
(importorskip, sibling-test pattern) or without the probe ledger.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "closed_loop_fuel.json"

TICKS = 6000
THR = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
       "omega_BN_B": 1e-4, "dcm_max": 2e-3, "wheels": 1e-4}


def _load():
    pytest.importorskip("Basilisk.simulation.spacecraft")
    pytest.importorskip("omniheyoka.omni_heyoka")
    if not LEDGER.exists():
        pytest.skip("closed_loop_fuel probe not run on this host yet")
    return json.loads(LEDGER.read_text())


def test_leg_ledger_green():
    d = _load()
    assert d["payload"]["status"] == "ok"
    assert d["payload"]["verdict"] == "parity green"
    assert d["payload"]["dt"] == pytest.approx(0.1)
    assert d["payload"]["ticks"] == TICKS
    assert d["payload"]["cells"]["leg"]["ticks"] == TICKS
    assert d["env"]["basilisk"] == "2.12.0"


def test_closed_loop_att_rate_parity():
    """Attitude/rate parity in the pre-registered switching classes."""
    c = _load()["payload"]["cells"]["leg"]
    for f, tol in THR.items():
        assert c[f] < tol, f"leg.{f}={c[f]:.3e}"


def test_closed_loop_wheels_coupled_bound():
    """Wheel-speed parity under the coupled-state switching bound."""
    c = _load()["payload"]["cells"]["leg"]
    assert c["wheels"] < 1e-4, f"leg.wheels={c['wheels']:.3e}"


def test_closed_loop_fuel_bit_exact():
    """Depletion law near-bit-exact with a live burn."""
    c = _load()["payload"]["cells"]["leg"]
    assert c["fuel_burned"] > 0.1, f"leg vacuous burned={c['fuel_burned']:.3e}"
    assert c["fuel_rel_err"] < 1e-9, (
        f"leg.fuel_rel_err={c['fuel_rel_err']:.3e}")


def test_closed_loop_mrp_and_vacuity():
    """MRP switch totals within +-2 of stock; both arms displaced."""
    c = _load()["payload"]["cells"]["leg"]
    assert abs(c["mrp_omni"] - c["mrp_stock"]) <= 2, (
        f"leg mrp omni={c['mrp_omni']} stock={c['mrp_stock']}")
    assert c["disp_stock"] > 1e5, f"leg stock vacuous disp={c['disp_stock']:.3e}"
    assert c["disp_omni"] > 1e5, f"leg omni vacuous disp={c['disp_omni']:.3e}"
