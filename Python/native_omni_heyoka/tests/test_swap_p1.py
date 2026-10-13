"""Phase-1 swap gates (PLAN_INTEGRATOR_SWAP.md): stock-RK4 vs OmniVecIntegrator.

Ledger: audit_bundle/results/swap_p1_parity.json (probe swap_p1_parity.py).
Bounds (pre-registered, margins >= 5x):
  hub/RW cells: 1e-9 states, 1e-9 wheels (measured ~1e-13).
  thruster fresh cells: dt-aware switching residual (linear in dt, Update-4
    class): r < 2e-3, v < 2e-5, sigma < 2e-4, omega < 1e-5 at dt=0.1.
  write-once/frozen (single firing step): r < 1e-5, v < 1e-7,
    sigma < 1e-6, omega < 1e-7.
Rejects: fuel / jitter / friction / multi-RW / missing-factory /
  min-pulse / low-cutoff must ALL raise (ValueError, pre-sim).
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "swap_p1_parity.json"

TIGHT = 1e-9
THR = {"r_BN_N": 2e-3, "v_BN_N": 2e-5, "sigma_BN": 2e-4, "omega_BN_B": 1e-5}
WO = {"r_BN_N": 1e-5, "v_BN_N": 1e-7, "sigma_BN": 1e-6, "omega_BN_B": 1e-7}


def _load():
    if not LEDGER.exists():
        pytest.skip("swap_p1 probe not run on this host yet")
    return json.loads(LEDGER.read_text())


def test_p1_ledger_green():
    d = _load()
    assert d["payload"]["status"] == "ok"
    assert d["payload"]["verdict"] == "parity green"
    assert d["env"]["basilisk"] == "2.12.0"


def test_p1_hub_rw_parity():
    d = _load()["payload"]["cells"]
    for cell in ("hub_60s", "rw3_open_pos", "rw3_open_neg", "rw3_pd",
                 "rw3_fric", "rw1", "rw2", "rw1tilt"):
        assert cell in d, f"missing cell {cell}"
        for f in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
            assert d[cell][f] < TIGHT, f"{cell}.{f}={d[cell][f]:.3e}"
        if "wheels" in d[cell]:
            assert d[cell]["wheels"] < TIGHT
    # Build-shape sanity: hub+3RW = 15 states; pars = inputs + hub7 + Js.
    assert d["rw3_open_pos"]["nStates"] == 15
    assert d["hub_60s"]["nStates"] == 12
    assert d["hub_60s"]["build_s"] < 5.0  # one-time JIT budget sanity


def test_p1_thruster_parity():
    d = _load()["payload"]["cells"]
    for cell, bounds in (("thr_fresh_05", THR), ("thr_fresh_10", THR),
                         ("thr_lag10", THR),
                         ("thr_writeonce", WO), ("thr_frozen", WO),
                         ("thr_offset", THR)):
        assert cell in d, f"missing cell {cell}"
        for f, tol in bounds.items():
            assert d[cell][f] < tol, f"{cell}.{f}={d[cell][f]:.3e}"
    assert d["thr_fresh_05"]["nStates"] == 12  # thrusters add no states
    # Offset cell also checks the C-point outputs stock derives.
    assert d["thr_offset"]["nStates"] == 12
    for f in ("r_CN_N", "v_CN_N"):
        assert f in d["thr_offset"], f"missing {f} in thr_offset"
    assert d["thr_offset"]["r_CN_N"] < THR["r_BN_N"]
    assert d["thr_offset"]["v_CN_N"] < THR["v_BN_N"]


def test_p1_env_parity():
    d = _load()["payload"]["cells"]
    drag = d["env_drag"]
    for f, tol in (("r_BN_N", 5e-4), ("v_BN_N", 2e-6), ("sigma_BN", 5e-7),
                   ("omega_BN_B", 1e-8)):
        assert drag[f] < tol, f"env_drag.{f}={drag[f]:.3e}"
    assert drag["density_max"] > 1e-13  # effect live, not vacuous
    still = d["env_drag_still"]
    for f, tol in (("r_BN_N", 2e-5), ("v_BN_N", 1e-7), ("sigma_BN", 1e-7),
                   ("omega_BN_B", 1e-9)):
        assert still[f] < tol, f"env_drag_still.{f}={still[f]:.3e}"
    srp = d["env_srp"]
    for f, tol in (("r_BN_N", 1e-6), ("v_BN_N", 1e-8), ("sigma_BN", 1e-9),
                   ("omega_BN_B", 1e-9)):
        assert srp[f] < tol, f"env_srp.{f}={srp[f]:.3e}"
    assert 0.9 < srp["srp_single_tick_ratio"] < 1.1  # live vs analytic


def test_p1_switching_ladder_linear():
    """Residual halves with dt (stock-side kink error, Update-4 class)."""
    lad = _load()["payload"]["thr_ladder_30s"]
    dv1, dv2, dv3 = lad["dt0.1"]["dv"], lad["dt0.05"]["dv"], lad["dt0.02"]["dv"]
    assert 0.3 < dv2 / dv1 < 0.7, (dv1, dv2)
    assert 0.2 < dv3 / dv2 < 0.6, (dv2, dv3)


def test_p1_rejects_all_loud():
    r = _load()["payload"]["rejects"]
    for name in ("jitter_wheel",
                 "two_rw_effectors", "thruster_no_factory", "min_pulse",
                 "cutoff_zero"):
        # Helper guards raise ValueError("mirror_stock_config: ..."); any
        # other exception type/message fails the gate (no vacuous pass).
        assert str(r.get(name, "")).startswith("mirror_stock_config"), name
    # C++ defense-in-depth path names the unclaimed state.
    assert "fuelMass" in str(r.get("fuel_unmirrored_cxx", ""))
    assert r.get("verdict") == "all loud"


def test_p1_fuel_parity():
    d = _load()["payload"]["cells"]
    for cell in ("fuel_centered", "fuel_offset"):
        assert cell in d, f"missing cell {cell}"
        c = d[cell]
        assert c["r_BN_N"] < 2e-2, f"{cell}.r={c['r_BN_N']:.3e}"
        assert c["v_BN_N"] < 1e-4, f"{cell}.v={c['v_BN_N']:.3e}"
        assert c["omega_BN_B"] < 1e-4, f"{cell}.om={c['omega_BN_B']:.3e}"
        assert c["dcm_max"] < 2e-3, f"{cell}.dcm={c['dcm_max']:.3e}"
        assert c["fuel_burned"] > 0.1  # effect live, not vacuous
        assert c["fuel_rel_err"] < 1e-9  # depletion law near-bit-exact
    assert d["fuel_centered"]["nStates"] == 13  # hub12 + fuel (no wheels)
