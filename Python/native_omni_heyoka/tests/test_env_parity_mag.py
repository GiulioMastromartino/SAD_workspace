"""Mag/MTB firing parity gates: stock-vs-omni at matched dt (§A1).

Ledger: audit_bundle/results/env_parity_mag.json
(probe env_parity_mag.py -- latched 0.5 x-Am² single-bar identity-Gt MTB
+ centered-dipole Tesla field, LEO 400 km still, dt=0.1, 600 s through
the WIRED mirror_stock_config path).

Tolerances (measured basis, never invented): this family has no swap_p1
quiet-leg precedent, so the pre-registered FSW-loop switching class
applies (tests/test_fsw_loop.py docstring bounds: r<2e-2, v<1e-4,
sigma<2e-4, omega<1e-4, dcm<2e-3 -- the firing/torque leg envelope).
Ledger-measured mag norms (r 9.3e-10 / sigma 6.7e-12) clear those bounds
by >=7 orders of magnitude.
Vacuity (in-leg measured effect, >=10x margin): latched dipole x live
Tesla field torques the hub (ledger tau_max ~1.2e-5 N.m, attitude drift
~7e-4 rad). The tight parity itself pins the shared Tesla convention
(an nT-scale input spins the hub to chaos on both arms alike).
Mirror spot-check: latched dipole record + REQUIRED mag link wired.
Step audit (L14): steps == calls == ticks.

Pure-JSON proof pins (host-free shape): skip (fail open, never fail)
when the probe ledger is absent on this host.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "env_parity_mag.json"

TICKS = 6000
TOL = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
       "omega_BN_B": 1e-4, "dcm_max": 2e-3}


def _load():
    if not LEDGER.exists():
        pytest.skip("env_parity_mag probe not run on this host yet")
    return json.loads(LEDGER.read_text())["payload"]


def test_leg_ledger_green():
    d = _load()
    assert d["status"] == "ok"
    assert d["verdict"] == "parity green"
    assert d["dt"] == pytest.approx(0.1)
    assert d["ticks"] == TICKS


def test_mag_state_parity():
    """State norms within the FSW-loop switching class."""
    c = _load()["cells"]["leg"]
    for f, tol in TOL.items():
        assert c[f] < tol, f"leg.{f}={c[f]:.3e}"


def test_mag_firing_proof():
    """Effect live: dipole torque nonzero, hub attitude drifted."""
    c = _load()["cells"]["leg"]
    assert c["tau_max_stock"] > 1e-6, (
        f"leg vacuous tau_max={c['tau_max_stock']:.3e}")
    assert c["sigF_norm_stock"] > 1e-6, (
        f"leg vacuous sigF={c['sigF_norm_stock']:.3e}")
    assert c["B_max_stock_T"] > 0.0


def test_mag_mirror_record():
    """Wired-path spot-check: latched dipole + mag link forwarded."""
    m = _load()["cells"]["leg"]["mirror"]["env"]["mag"]
    assert m["n_mtb"] == 1 and m["n_magfield"] == 1
    assert m["magnetic_moment"] == [0.5, 0.0, 0.0]
    assert m["max_mtb_dipole"] == 0.8
    assert m["mag_msg_wired"] is True
    assert m["latched_dipole"] is True


def test_step_count_audit():
    """L14: exactly one integrate step and one call per tick."""
    c = _load()["cells"]["leg"]
    assert c["steps"] == TICKS, f"leg steps={c['steps']}"
    assert c["calls"] == TICKS, f"leg calls={c['calls']}"


def test_loadavg_provenance():
    """L9: host load recorded (contention noted, never gated)."""
    c = _load()["cells"]["leg"]
    assert isinstance(c["loadavg"], list) and len(c["loadavg"]) == 3
