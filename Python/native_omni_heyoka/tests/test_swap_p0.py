"""Phase-0 swap gates (PLAN_INTEGRATOR_SWAP.md): ledger shape + verdict pins.

Four probes, all host-run on the macOS build (BSK 2.12.0 / heyoka 7.12.1):
  swap_p0_callback — no numeric-callback RHS in heyoka (mirror+compose mandatory)
  swap_p0_census   — registered-state names/sizes + bit-exact scatter round-trip
  swap_p0_swig     — setIntegrator surface (C++-only, pointer-checked, header present)
  swap_p0_ordering — stock tick-ordering + ownership + RW-source facts record
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "audit_bundle" / "results"


def _load(name: str):
    p = RESULTS / f"{name}.json"
    if not p.exists():
        pytest.skip(f"{name} probe not run on this host yet")
    return json.loads(p.read_text())


def test_p0_callback_absent():
    d = _load("swap_p0_callback")
    assert d["payload"]["status"] == "ok"
    assert d["payload"]["ctors_symbolic_only"] is True
    assert d["payload"]["numeric_rhs_overload_found"] is False
    assert d["payload"]["func_requires_taylor_decompose"] is True
    assert d["payload"]["verdict"].startswith("absent")


def test_p0_census_layout_and_roundtrip():
    d = _load("swap_p0_census")
    assert d["payload"]["status"] == "ok"
    cfg = d["payload"]["configs"]
    hub = cfg["hub_only"]
    assert hub["n_states_total"] == 12
    assert [s["name"] for s in hub["states"]] == [
        "hubPosition", "hubVelocity", "hubSigma", "hubOmega"]
    assert all(s["rows"] == 3 and s["cols"] == 1 for s in hub["states"])
    rw = cfg["hub_3rw"]
    assert rw["n_states_total"] == 15
    omegas = [s for s in rw["states"] if s["name"] == "reactionWheelOmegas"]
    assert len(omegas) == 1 and omegas[0]["rows"] == 3
    fuel = cfg["hub_fuel"]
    assert fuel["n_states_total"] == 13
    assert any(s["name"] == "fuelMass" for s in fuel["states"])
    for tag, c in cfg.items():
        assert c["roundtrip_bitexact"], tag
        assert c["roundtrip_max_abs_delta"] == 0.0, tag


def test_p0_swig_surface():
    d = _load("swap_p0_swig")
    assert d["payload"]["status"] == "ok"
    p = d["payload"]
    assert p["setIntegrator_stock_ok"] is True
    assert p["wrong_type_rejected"] is True
    assert p["python_subclass_blocked"] is True  # C++-only: no directors
    assert p["syncDynamicsIntegration_present"] is True
    assert p["bsk_sdk_stateVecIntegrator_h"] is True


def test_p0_ordering_record():
    d = _load("swap_p0_ordering")
    assert d["payload"]["status"] == "ok"
    p = d["payload"]
    assert p["upstream_tag"] == "v2.12.0"
    assert "svIntegratorRK4" in p["ctor"]
    assert "integrateState" in p["UpdateState_order"][1]
    assert "dynPtrs[0]" in p["setIntegrator_rules"][1]
    assert "timeBefore" in p["integrate_args"]
    assert "reactionWheelOmegas" in p["rw_registered_states"]
    assert d["env"]["basilisk"] == "2.12.0"
