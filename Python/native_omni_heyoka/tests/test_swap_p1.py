"""Phase-1 swap gates (PLAN_INTEGRATOR_SWAP.md): stock-RK4 vs OmniVecIntegrator.

Ledger: audit_bundle/results/swap_p1_parity.json (probe swap_p1_parity.py).
Bounds (pre-registered, margins >= 5x):
  hub/RW cells: 1e-9 states, 1e-9 wheels (measured ~1e-13).
  thruster fresh cells: dt-aware switching residual (linear in dt, Update-4
    class): r < 2e-3, v < 2e-5, sigma < 2e-4, omega < 1e-5 at dt=0.1.
  write-once/frozen (single firing step): r < 1e-5, v < 1e-7,
    sigma < 1e-6, omega < 1e-7.
Rejects: jitter / multi-RW-effectors / missing-factory / min-pulse /
  low-cutoff must ALL raise (ValueError, pre-sim) + the structural
  fuel_unmirrored_cxx cell names fuelMass (ledger verdict "all loud").
  Friction is PARITY since Update 8 (rw3_fric cell, 1e-9 wheels), not a
  reject. (WP25-C3 X2: was "fuel / jitter / friction / multi-RW /
  missing-factory / min-pulse / low-cutoff ... 7/7" — friction entry struck.)
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
    # Build-shape sanity: hub+3RW = 15 states / 16 pars (6 u+tauF inputs + 7 hub
    # + 3 Js); hub-only = 12 states / 7 pars. Ledger: swap_p1_parity.json
    # rw3_open_pos / hub_60s cells, vec arm (pinned WP25-C3 against the in-tree file).
    assert d["rw3_open_pos"]["nStates"] == 15
    assert d["rw3_open_pos"]["nPars"] == 16
    assert d["hub_60s"]["nStates"] == 12
    assert d["hub_60s"]["nPars"] == 7
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
    assert d["thr_fresh_05"]["nPars"] == 11  # 3q + swirl + 7 hub (vec arm, ledger-pinned WP25-C3)
    # Offset cell also checks the C-point outputs stock derives.
    assert d["thr_offset"]["nStates"] == 12
    assert d["thr_offset"]["nPars"] == 11  # same thruster config as thr_fresh (ledger-pinned WP25-C3)
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
    assert d["fuel_centered"]["nPars"] == 12  # 1 mdot + 3q + swirl + 7 hub (vec arm, ledger-pinned WP25-C3)
    assert d["fuel_offset"]["nPars"] == 12


def test_p1_sysmodel_fuel_publisher_absent():
    """WP25-A3 option B (Decision 4, closes H6): the SysModel path must not
    publish fuel telemetry -- the static-full-tank publish (fuelMass_=m0
    verbatim, fuelMassDot=0) was wrong telemetry under any burn. Regression
    gate: fails if a `fuelTankOutMsg.write` reappears in
    src/omni_spacecraft.cpp. The setFuelParams record surface + B1
    single-tank guard stay (see test_wp0_guards.py)."""
    src = (REPO / "src" / "omni_spacecraft.cpp").read_text()
    assert "fuelTankOutMsg.write" not in src
    assert "WP25-A3 option B" in src


def test_p1_fuel_env_scope():
    """WP-C MEASURED (H7-LIFTED 2026-10-14): fuel+drag and fuel+SRP combined
    cells are measured at matched dt (probe _env_fuel_drag_cell /
    _env_fuel_srp_cell) and gated in the switching class (pre-registered in
    the _env_fuel_cell docstring: r < 2e-2, v < 1e-4, omega < 1e-4,
    dcm_max < 2e-3, fuel_rel_err < 1e-9, burned > 0.1). Arm verdict: static
    drag arm stands (ledger fuel_drag arm == "static"); the ledger carries
    the sensitivity bound explicitly -- stock-measured |dc| (dc_norm) with
    the 1:1-shift signature estimate (shift_sig_dcm) BELOW the dcm residual
    (sensitivity-limited consistency: a tracking gain >= ~25 would have
    tripped the dcm gate; G = 1 is not excluded -- recorded, not guessed).
    fuel_srp is force-level parity only (cannonball torque identically zero
    both arms) and cannot discriminate by construction."""
    d = _load()["payload"]["cells"]
    for cell in ("fuel_drag", "fuel_srp"):
        assert cell in d, f"missing cell {cell}"
        c = d[cell]
        assert c.get("status", "ok") != "H7-OUT", f"{cell} unmeasured"
        assert c["arm"] == "static", f"{cell}.arm={c.get('arm')}"
        assert c["r_BN_N"] < 2e-2, f"{cell}.r={c['r_BN_N']:.3e}"
        assert c["v_BN_N"] < 1e-4, f"{cell}.v={c['v_BN_N']:.3e}"
        assert c["omega_BN_B"] < 1e-4, f"{cell}.om={c['omega_BN_B']:.3e}"
        assert c["dcm_max"] < 2e-3, f"{cell}.dcm={c['dcm_max']:.3e}"
        assert c["fuel_burned"] > 0.1  # effect live, not vacuous
        assert c["fuel_rel_err"] < 1e-9
        assert c["dc_norm"] > 1e-4, f"{cell}.dc={c['dc_norm']:.3e}"
        assert c["shift_sig_dcm"] < c["dcm_max"], (
            f"{cell} sensitivity sig={c['shift_sig_dcm']:.3e} "
            f"dcm={c['dcm_max']:.3e}")
        assert c["nStates"] == 13  # hub12 + fuel (no wheels)
        assert c["nPars"] == 27  # 1 mdot + 3q + swirl + 7 hub + env (ledger-pinned)
    assert d["fuel_drag"]["density_max"] > 1e-13  # drag live, not vacuous
