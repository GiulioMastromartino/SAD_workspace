"""WP-C4 FSW-in-the-loop gates: tick-by-tick saturation-edge / allocator /
MRP-count loop comparison (list.md section C4).

Ledger: audit_bundle/results/fsw_loop.json (probe fsw_loop.py -- the
accepted closed_loop_fuel.py leg + loop recorders).
Bounds (pre-registered from the swap_p1 classes at matched dt=0.1, 600 s leg):
  r_BN_N < 2e-2, v_BN_N < 1e-4 (fuel-cell switching class, 5 N burn);
  sigma_BN < 2e-4 (thruster switching class TOL_THR_SIG -- the PD law
  stabilizes the leg); omega_BN_B < 1e-4 (fuel-cell 1e-4 scale);
  dcm_max < 2e-3 (switch-invariant rotation, fuel-cell class);
  wheels < 1e-4 (coupled-state switching bound, test_swap_multi class).
Loop audit: MRP switch totals within +-2 of stock (test_swap_multi
mechanism; B2.5: tick-exact equality NOT required); saturation-edge first
ticks match within the ledger-stated SAT_TOL_TICKS window (5, B2.5 jitter
provenance); allocator firing-tick counts nonzero on both arms (firing
proof) with chatter recorded per arm. Step-count audit (L14): per-leg
steps == ticks AND calls == ticks (no double-step on the
single-spacecraft vec path). Vacuity: burned > 0.1 kg and both-arm
displacement > 1e5 m (effects live, not zeros-vs-zeros).
Build (L9): this config is recorded with host loadavg
(contention-inflated) and explicitly NOT gated.

Host-only sims: skipped (no failure) without Basilisk/the built wheel
(importorskip, sibling-test pattern) or without the probe ledger.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "fsw_loop.json"

TICKS = 6000
THR = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
       "omega_BN_B": 1e-4, "dcm_max": 2e-3, "wheels": 1e-4}
SAT_TOL = 5


def _load():
    pytest.importorskip("Basilisk.simulation.spacecraft")
    pytest.importorskip("omniheyoka.omni_heyoka")
    if not LEDGER.exists():
        pytest.skip("fsw_loop probe not run on this host yet")
    return json.loads(LEDGER.read_text())


def test_leg_ledger_green():
    d = _load()
    assert d["payload"]["status"] == "ok"
    assert d["payload"]["verdict"] == "parity green"
    assert d["payload"]["dt"] == pytest.approx(0.1)
    assert d["payload"]["ticks"] == TICKS
    assert d["payload"]["cells"]["leg"]["ticks"] == TICKS
    assert d["payload"]["cells"]["leg"]["sat_tol_ticks"] == SAT_TOL
    assert d["env"]["basilisk"] == "2.12.0"


def test_fsw_att_rate_parity():
    """Attitude/rate parity in the pre-registered switching classes."""
    c = _load()["payload"]["cells"]["leg"]
    for f, tol in THR.items():
        assert c[f] < tol, f"leg.{f}={c[f]:.3e}"


def test_fsw_wheels_coupled_bound():
    """Wheel-speed parity under the coupled-state switching bound."""
    c = _load()["payload"]["cells"]["leg"]
    assert c["wheels"] < 1e-4, f"leg.wheels={c['wheels']:.3e}"


def test_fsw_mrp_totals_and_vacuity():
    """MRP switch totals within +-2 of stock; both arms displaced; trace live."""
    c = _load()["payload"]["cells"]["leg"]
    assert abs(c["mrp_omni"] - c["mrp_stock"]) <= 2, (
        f"leg mrp omni={c['mrp_omni']} stock={c['mrp_stock']}")
    assert c["mrp_trace_len"] >= TICKS, (
        f"leg vacuous mrp trace len={c['mrp_trace_len']}")
    assert len(c["stock_mrp_trace"]) == c["mrp_trace_len"]
    assert c["disp_stock"] > 1e5, f"leg stock vacuous disp={c['disp_stock']:.3e}"
    assert c["disp_omni"] > 1e5, f"leg omni vacuous disp={c['disp_omni']:.3e}"


def _assert_edges_match(es, eo, label):
    assert len(es) == 3 and len(eo) == 3, f"leg.{label} not per-wheel triples"
    for i, (a, b) in enumerate(zip(es, eo)):
        assert (a is None) == (b is None), (
            f"leg.{label}[{i}] one-sided saturation stock={a} omni={b}")
        if a is not None:
            assert abs(a - b) <= SAT_TOL, (
                f"leg.{label}[{i}] stock={a} omni={b} tol={SAT_TOL}")


def test_fsw_saturation_edge_tick_match():
    """First-saturation ticks match within the stated window, both edge
    classes (u_max command edges + stock-relative wheel-speed edges)."""
    c = _load()["payload"]["cells"]["leg"]
    _assert_edges_match(c["sat_u_stock"], c["sat_u_omni"], "sat_u")
    _assert_edges_match(c["sat_w_stock"], c["sat_w_omni"], "sat_w")


def test_fsw_allocator_firing_proof():
    """Allocator chatter recorded per arm; firing-tick counts nonzero on
    both arms (burn live) with fuel burned as the effect-side proof."""
    c = _load()["payload"]["cells"]["leg"]
    assert c["thr_fire_stock"] > 0, "leg stock allocator never fired"
    assert c["thr_fire_omni"] > 0, "leg omni allocator never fired"
    assert isinstance(c["thr_chatter_stock"], int)
    assert isinstance(c["thr_chatter_omni"], int)
    assert c["thr_chatter_stock"] == c["thr_chatter_omni"], (
        f"leg chatter stock={c['thr_chatter_stock']} "
        f"omni={c['thr_chatter_omni']}")
    assert c["fuel_burned"] > 0.1, f"leg vacuous burned={c['fuel_burned']:.3e}"
    assert c["fuel_burned_omni"] > 0.1, (
        f"leg omni vacuous burned={c['fuel_burned_omni']:.3e}")


def test_step_count_audit():
    """L14: exactly one integrate step and one call per tick (no double-step)."""
    c = _load()["payload"]["cells"]["leg"]
    assert c["steps"] == TICKS, f"leg steps={c['steps']}"
    assert c["calls"] == TICKS, f"leg calls={c['calls']}"
