"""WP25-A4 swap gates (PLAN_PHASE25.md): two-spacecraft synced sync path.

Ledger: audit_bundle/results/swap_multi.json (probe swap_multi.py).
Bounds (pre-registered, same switching class as the swap_p1 THR cells at
matched dt=0.1; rotation by switch-invariant DCM trajectory-max, fuel-cell
class; wheels by the coupled-state switching bound -- a double-step
diverges O(1), so every margin here is honest):
  r_BN_N < 2e-3, v_BN_N < 2e-5, omega_BN_B < 1e-5, dcm_max < 2e-3,
  wheels < 1e-4, per object, both legs.
Step-count audit (no double-step on synced objects): per-object steps ==
ticks AND integrate calls == ticks (600 s @ dt=0.1 -> 6000).
Build/share rule: per-object build_s < 5.0 s (Ground-8 JIT tier);
shared leg has shared_graph + equal fingerprints, split leg the opposite.
Vacuity: distinct ICs (ic_separation > 1e5 m), every object displaced on
both arms (> 1e5 m), MRP totals within +-2 of stock per object.
Negative gate: the unmirrored-slot cell raised loudly naming object 1.

Host-only sims: skipped (no failure) without the probe ledger. The
single-object regression (swap_p1_parity.json ledger, test_swap_p1.py) is
untouched by construction -- this WP re-greens it byte-identical on host.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "swap_multi.json"

TICKS = 6000
THR = {"r_BN_N": 2e-3, "v_BN_N": 2e-5, "omega_BN_B": 1e-5,
       "dcm_max": 2e-3, "wheels": 1e-4}


def _load():
    if not LEDGER.exists():
        pytest.skip("swap_multi probe not run on this host yet")
    return json.loads(LEDGER.read_text())


def test_multi_ledger_green():
    d = _load()
    assert d["payload"]["status"] == "ok"
    assert d["payload"]["verdict"] == "parity green"
    assert d["payload"]["dt"] == pytest.approx(0.1)
    assert d["payload"]["ticks"] == TICKS
    assert d["env"]["basilisk"] == "2.12.0"


def _leg(name):
    d = _load()["payload"]["cells"]
    assert name in d, f"missing cell {name}"
    return d[name]


def test_multi_shared_leg_parity():
    c = _leg("leg_shared")
    for tag in ("objA", "objB"):
        assert tag in c, f"missing {tag}"
        for f, tol in THR.items():
            assert c[tag][f] < tol, f"leg_shared.{tag}.{f}={c[tag][f]:.3e}"


def test_multi_split_leg_parity():
    c = _leg("leg_split")
    for tag in ("objA", "objB"):
        assert tag in c, f"missing {tag}"
        for f, tol in THR.items():
            assert c[tag][f] < tol, f"leg_split.{tag}.{f}={c[tag][f]:.3e}"


def test_multi_step_audit_no_double_step():
    """Synced objects step exactly once per tick: steps[i] == calls == ticks.
    A double-step would integrate every object twice per tick (O(1)
    divergence); equality here is the direct proof it never happened."""
    d = _load()["payload"]["cells"]
    for leg in ("leg_shared", "leg_split"):
        c = d[leg]
        assert c["calls"] == TICKS, f"{leg}.calls={c['calls']}"
        for tag in ("objA", "objB"):
            assert c[tag]["steps"] == TICKS, (
                f"{leg}.{tag}.steps={c[tag]['steps']}")


def test_multi_build_and_share_rule():
    """JIT tiers bind every build; the shared-graph rule is exact: identical
    configs share one graph (equal fingerprints), any difference splits."""
    d = _load()["payload"]["cells"]
    for leg in ("leg_shared", "leg_split"):
        for tag in ("objA", "objB"):
            assert d[leg][tag]["build_s"] < 5.0, (
                f"{leg}.{tag}.build_s={d[leg][tag]['build_s']}")
    sh = d["leg_shared"]
    assert sh["shared_graph"] is True
    assert sh["fp_equal"] is True
    assert sh["fp0"] == sh["fp1"]
    sp = d["leg_split"]
    assert sp["shared_graph"] is False
    assert sp["fp_equal"] is False
    assert sp["fp0"] != sp["fp1"]


def test_multi_vacuity_and_mrp():
    """Distinct ICs, live (displaced) trajectories on both arms, and MRP
    switch totals within +-2 of stock per object (B2.5 mechanism, honest-loose
    on this unmeasured config until a host tightens it)."""
    d = _load()["payload"]["cells"]
    for leg in ("leg_shared", "leg_split"):
        c = d[leg]
        assert c["ic_separation"] > 1e5, c["ic_separation"]
        for tag in ("objA", "objB"):
            assert c[tag]["disp_stock"] > 1e5, f"{leg}.{tag} stock vacuous"
            assert c[tag]["disp_omni"] > 1e5, f"{leg}.{tag} omni vacuous"
            assert abs(c[tag]["mrp_omni"] - c[tag]["mrp_stock"]) <= 2, (
                f"{leg}.{tag} mrp omni={c[tag]['mrp_omni']} "
                f"stock={c[tag]['mrp_stock']}")


def test_multi_unmirrored_slot_loud():
    """Object B must never silently inherit object A's graph: mirroring slot
    0 only, then syncing, raises loudly naming object 1."""
    d = _load()["payload"]["cells"]
    assert "reject_unmirrored" in d
    raised = d["reject_unmirrored"].get("raised")
    assert isinstance(raised, str) and "object 1" in raised, raised
