"""Task cost split gates (stock-only characterization, no wheel needed).

Ledger: audit_bundle/results/loop_split.json (probe loop_split.py).
Gates (medians: outlier-proof at n=3; means recorded alongside):
  - ledger ok, 3 runs/leg, 600 ticks, nanos audit true (full legs only);
  - every leg median wall > 0;
  - load-bearing orderings only: L2>L1+1ms (thr/tank wired) and L4>L3+1ms
    (driver firing). RW-idle and recorders both measure ~0 (twice) -- below
    gateable resolution, so ordering there would gate noise; their coverage
    is presence-only, stated not implied;
  - additivity: |sum(parts) - total| <= 50% of total (ablation consistency;
    generous because subtraction amplifies noise -- a breach means a leg
    is miswired, e.g. an idle driver that never fires).
Attribution signs are NOT gated (noise can print small negatives on
sub-ms deltas); magnitudes are the evidence.
Host-free test (pure JSON asserts); the probe needs Basilisk only.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "loop_split.json"
ORDER = ("L0_hub", "L1_rw", "L2_thr_tank_idle", "L3_fire",
         "L4_python_driver", "L5_recorders")
PARTS = ("rw_ms", "thr_tank_idle_ms", "fire_ms", "python_driver_ms",
         "recorders_ms")


def _load():
    if not LEDGER.exists():
        pytest.skip("loop_split probe not run on this host yet")
    doc = json.loads(LEDGER.read_text())
    return doc.get("payload", doc)


def test_loop_split_ledger_green():
    d = _load()
    assert d["status"] == "ok"
    assert d["n_runs"] == 3
    assert d["ticks"] == 600
    assert d["nanos_ok"] is True
    assert d["throughput_ticks_per_s"] > 0


def test_loop_split_legs_shape():
    legs = _load()["legs"]
    for name in ORDER:
        assert name in legs, f"missing leg {name}"
        c = legs[name]
        assert len(c["wall_samples_s"]) == 3
        assert c["wall_median_s"] > 0
        assert c["per_tick_median_ms"] > 0


def test_loop_split_load_bearing_orderings():
    """Only deltas big enough to clear the noise floor are gated: thr+tank
    wiring (L2 vs L1, measured +6.5 ms) and the Python driver (L4 vs L3,
    measured +10 ms). A broken thruster/tank/driver leg would read back at
    its base and trip these. RW-idle and recorders both measure ~0 (twice)
    -- below gateable resolution; asserting order there would gate noise,
    so their wiring is covered by presence (600 ticks, nanos ok) only."""
    med = {n: _load()["legs"][n]["wall_median_s"] for n in ORDER}
    assert med["L2_thr_tank_idle"] >= med["L1_rw"] + 1e-3, (
        "thr/tank leg at/below RW leg: thruster or tank not wired?")
    assert med["L4_python_driver"] >= med["L3_fire"] + 1e-3, (
        "driver leg at/below firing leg: PD driver not firing?")


def test_loop_split_additivity():
    a = _load()["attribution_ms"]
    parts = sum(a[p] for p in PARTS)
    total = a["total_ms"]
    assert total > 0
    assert abs(parts - total) <= 0.5 * total, (
        f"parts sum {parts:.4f}ms vs total {total:.4f}ms: ablation inconsistent")
