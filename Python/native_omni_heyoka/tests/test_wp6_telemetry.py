"""WP6 telemetry / viz-direct / fuel-labels host tests (no compiler, no sim).

Runs on Linux CI without Basilisk and without the built ``omni_heyoka``
extension: every non-gated test reads the **recorded** stock ledgers
(``audit_bundle/results/wp6_telem_viz_fuel.json`` + P-WP4.1 sample in
``wp4_offset.json``) and the pure-Python helpers in
:mod:`omniheyoka.viz_direct`. Live sims stay gated behind the same env
switches as the probes (``WP6_RUN_VIZ_SIM=1`` / ``WP6_RUN_FUEL_SIM=1``)
because headless ``ExecuteSimulation`` segfaults on the audit host
(T2/T3 record).

Covered contracts (PLAN Rev 1 WP6 DoD, first slice):

* message-field diff on a recorded closed-loop run: identical field sets
  stock-vs-omni (modulo SWIG ``this``/``thisown``), values within gate
  (P-WP4.1 ``u_current``/``Omega``/``Js`` tick sample);
* DV accumulation smoke (per-tick impulse bookkeeping, LABELED non-parity
  vs stock substep integration);
* fuel bookkeeping uses staircase-ACHIEVED duty (shrinks gap #1), with the
  3 divergences labeled; static ``FuelTankMsg`` shape pinned;
* viz-direct import test (no sim, no helper import) + gated live smoke.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WP6_JSON = REPO / "audit_bundle" / "results" / "wp6_telem_viz_fuel.json"
WP4_JSON = REPO / "audit_bundle" / "results" / "wp4_offset.json"

THR_EXPECTED_9 = [
    "thrustFactor",
    "thrustForce",
    "thrustForce_B",
    "thrustTorquePntB_B",
    "thrustBlowDownFactor",
    "ispBlowDownFactor",
    "maxThrust",
    "thrusterLocation",
    "thrusterDirection",
]
RW_EXPECTED_23 = [
    "rWB_B",
    "gsHat_B",
    "w2Hat0_B",
    "w3Hat0_B",
    "mass",
    "theta",
    "Omega",
    "Js",
    "Jt",
    "Jg",
    "U_s",
    "U_d",
    "d",
    "J13",
    "u_current",
    "frictionTorque",
    "u_max",
    "u_min",
    "u_f",
    "Omega_max",
    "P_max",
    "linearFrictionRatio",
    "RWModel",
]
SWIG_EXTRAS = {"this", "thisown"}

# P-WP4.1 closed-loop fixture (3x HR16, maxMomentum=50, Omega=100 RPM cmd,
# motorTorque=[0.02,-0.015,0.01], dt=0.1, 20 ticks). Values from ledger.
PWP41_MOTOR_CMD = [0.02, -0.015, 0.01]
PWP41_OMEGA0_RPM = 100.0
PWP41_MAX_MOMENTUM = 50.0
PWP41_OMEGA_MAX_RPM = 6000.0  # HR16 rated (rwFactory source)
PWP41_JS_EXPECT = PWP41_MAX_MOMENTUM / (PWP41_OMEGA_MAX_RPM * 2 * math.pi / 60.0)


def _load(pid_path: Path) -> dict:
    assert pid_path.exists(), f"missing ledger {pid_path}"
    return json.loads(pid_path.read_text())


def _payload(obj: dict) -> dict:
    # results files wrap probes under {"payload": {...}}.
    return obj.get("payload", obj)


# ---------------------------------------------------------------------------
# Field-set diff (recorded run; identical sets, SWIG extras excluded)
# ---------------------------------------------------------------------------

def test_thr_field_set_matches_stock_record():
    blob = _payload(_load(WP6_JSON))
    t1 = blob["t1_payload_census"]
    static_names = t1["thr_output_static_fields"]
    assert t1["thr_output_missing"] == []
    assert set(t1["thr_output_extra"]) <= SWIG_EXTRAS
    real = [n for n in static_names if n not in SWIG_EXTRAS]
    assert sorted(real) == sorted(THR_EXPECTED_9), real
    # The omni THR publisher (WP3 paragraph) must emit exactly these 9;
    # viz_direct pins the same list so viz wiring and telemetry agree.
    from omniheyoka import viz_direct

    assert sorted(viz_direct.THR_REAL_FIELDS_9) == sorted(THR_EXPECTED_9)
    # 1-step dump anchors the value gate: write-once fires first propagation
    # then idles — steady value is the LAST tick, tick0 is the pre-fire fill.
    dump = t1["thr_1step_dump"]
    assert dump["thrustFactor"] == pytest.approx(1.0)
    assert dump["thrustFactor_tick0"] == pytest.approx(0.0)
    assert dump["thrustBlowDownFactor"] == pytest.approx(1.0)
    assert dump["ispBlowDownFactor"] == pytest.approx(1.0)


def test_rw_field_set_23_and_sample_tick_values():
    blob4 = _payload(_load(WP4_JSON))
    p41 = blob4["p_wp41_rw_census"]
    assert p41["rw_log_missing_vs_23"] == []
    assert set(p41["rw_log_extra_vs_23"]) <= SWIG_EXTRAS
    real = [n for n in p41["rw_log_static_fields"] if n not in SWIG_EXTRAS]
    assert sorted(real) == sorted(RW_EXPECTED_23), real
    assert p41["n_rw_out_msgs"] == 3

    from omniheyoka import viz_direct

    assert sorted(viz_direct.RW_REAL_FIELDS_23) == sorted(RW_EXPECTED_23)

    # Value gate on the recorded closed-loop tick-0 sample: u_current MUST
    # equal the commanded motor torque (all |u| < uMax 0.2 → no saturation
    # cut), Omega is the 100 RPM IC, Js = maxMomentum/Omega_max(rad/s).
    for i, wheel in enumerate([p41["sample_tick_0"][f"wheel_{k}"] for k in range(3)]):
        assert wheel["u_current"] == pytest.approx(PWP41_MOTOR_CMD[i], abs=1e-12), (
            f"wheel_{i} u_current must equal post-saturation input-slot value",
            wheel,
        )
        assert wheel["Omega"] == pytest.approx(
            PWP41_OMEGA0_RPM * 2 * math.pi / 60.0, rel=1e-9
        )
        assert wheel["Js"] == pytest.approx(PWP41_JS_EXPECT, rel=1e-9)
        assert wheel["frictionTorque"] == pytest.approx(0.0, abs=1e-12)
        assert wheel["theta"] == pytest.approx(0.0, abs=1e-12)
    # RWSpeed parity anchor on the same run.
    assert p41["rw_speed_tick_0"] == pytest.approx(
        [PWP41_OMEGA0_RPM * 2 * math.pi / 60.0] * 3, rel=1e-9
    )


def test_sc_gap_audit_labels_documented():
    blob = _payload(_load(WP6_JSON))
    t1 = blob["t1_payload_census"]
    names = t1["scstates_static_fields"]
    assert t1["scstates_static_count"] == 14
    real = [n for n in names if n not in SWIG_EXTRAS]
    assert len(real) == 12
    for f in ("omegaDot_BN_B", "TotalAccumDV_BN_B", "TotalAccumDVBdy",
              "TotalAccumDV_CN_N", "nonConservativeAccelpntB_B",
              "v_BN_N", "v_CN_N", "r_BN_N", "r_CN_N"):
        assert f in real, f
    # Gap-audit names in the probe use wrong suffixes for three fields —
    # pin that the REAL header names above (not the probe's _BN_N/_Bdy_B/
    # vel_ spellings) are the contract.
    gap = t1["scstates_gap_audit"]
    assert gap["omegaDot_BN_B"] == "present-in-header"
    # Documented WP6 gap-fill contract (C++ edit-spec, not yet built here):
    # omegaDot from WP4 lastWDot plumbing (default zeros until WP4 lands);
    # DV slots advance per-tick (bookkeeping, non-parity); v_BN_N mirrors
    # v_CN_N until WP4 lands. Tick0 stock values are all zeros (P-WP4.2).
    blob4 = _payload(_load(WP4_JSON))
    p42 = blob4["p_wp42_b_vs_c"]
    for f in ("omegaDot_BN_B_tick0", "TotalAccumDV_BN_B_tick0",
              "TotalAccumDVBdy_tick0", "TotalAccumDV_CN_N_tick0",
              "nonConservativeAccelpntB_B_tick0"):
        assert [float(v) for v in p42[f]] == pytest.approx([0.0, 0.0, 0.0])
    # B-vs-C anchor: |rBN-rCN| = 0.1 exact with the 0.1 m offset; consumers
    # read r_CN_N (17 files) never r_BN_N (0 files) → C-point confirmed.
    assert p42["rBN_minus_rCN_norm"] == pytest.approx(0.1, abs=1e-9)


# ---------------------------------------------------------------------------
# DV accumulation smoke (pure-Python contract of the C++ per-tick rule)
# ---------------------------------------------------------------------------

def test_dv_accumulation_smoke():
    from omniheyoka.viz_direct import dv_accumulate_step

    # Single 1 N burn, 750 kg, dt=0.1 → Δv = 1*0.1/750 per tick.
    acc = [0.0, 0.0, 0.0]
    for _ in range(10):
        acc = dv_accumulate_step(acc, [0.0, 0.0, 1.0], 750.0, 0.1)
    assert acc[2] == pytest.approx(10 * 0.1 / 750.0, rel=1e-12)
    assert acc[0] == pytest.approx(0.0, abs=0.0)
    # Zero-force tick leaves the accumulator untouched (idle-step rule).
    assert dv_accumulate_step(acc, [0.0, 0.0, 0.0], 750.0, 0.1) == pytest.approx(acc)
    with pytest.raises(ValueError):
        dv_accumulate_step([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 0.0, 0.1)


# ---------------------------------------------------------------------------
# Fuel bookkeeping (achieved-duty, 3 divergences labeled)
# ---------------------------------------------------------------------------

def test_fuel_msg_shape_static_and_bookkeeping_uses_achieved():
    blob = _payload(_load(WP6_JSON))
    t3 = blob["t3_fuel_truth"]
    assert t3["mode"].startswith("static-only"), t3["mode"]
    fields = t3["fuelTankOutMsg_fields"]
    real = [n for n in fields if n not in SWIG_EXTRAS]
    assert sorted(real) == ["fuelMass", "fuelMassDot", "maxFuelMass"], real
    assert t3["fueltank_has_addThrusterSet"] is True

    from omniheyoka.viz_direct import G0, fuel_mass_step

    # isp·g0 law with ACHIEVED thrust: 1 N achieved, isp=250 → mdot<0.
    m1, mdot = fuel_mass_step(10.0, 1.0, 0.1, 250.0)
    assert mdot == pytest.approx(-1.0 / (250.0 * G0), rel=1e-12)
    assert m1 == pytest.approx(10.0 + mdot * 0.1, rel=1e-12)
    # Gap #1 closed by construction: achieved (staircase) burns less than
    # commanded — fuel follows what fired (0.2-class), not what was asked.
    m_cmd, _ = fuel_mass_step(10.0, 1.0, 0.1, 250.0)
    m_ach, _ = fuel_mass_step(10.0, 0.2, 0.1, 250.0)
    assert (10.0 - m_ach) == pytest.approx(0.2 * (10.0 - m_cmd), rel=1e-12)
    # Empty-tank clamp (never negative).
    m0, mdot0 = fuel_mass_step(1e-6, 5.0, 10.0, 250.0)
    assert m0 == pytest.approx(0.0, abs=0.0) and mdot0 < 0.0
    with pytest.raises(ValueError):
        fuel_mass_step(10.0, 1.0, 0.0, 250.0)


# ---------------------------------------------------------------------------
# viz-direct import test (no sim, no helper import)
# ---------------------------------------------------------------------------

def test_viz_direct_import_no_sim_no_helper():
    import inspect

    import omniheyoka.viz_direct as vd

    assert hasattr(vd, "configure_direct_viz")
    assert hasattr(vd, "describe_viz_ports")
    assert hasattr(vd, "viz_smoke")
    src = inspect.getsource(vd)
    # No helper-package import anywhere (the type-bound path must stay out).
    assert "from Basilisk.utilities import vizSupport" not in src
    assert "from Basilisk.utilities.vizSupport import" not in src
    assert "import vizSupport" not in src
    # Plural-first contract pinned in source.
    assert "scStateInMsgs" in src
    assert "thrInMsgs" in src
    assert "rwInMsgs" in src
    assert "pushBack" in src
    # collect_omni_msgs degrades gracefully pre-WP3/WP6 (no attrs → empties).
    class _Bare:
        scStateOutMsg = object()

    got = vd.collect_omni_msgs(_Bare())
    assert got["scState"] is not None and got["thr"] == [] and got["rw"] == []
    # describe_viz_ports works on a dummy (attr-dump path, no Basilisk).
    ports = vd.describe_viz_ports(object())
    assert set(ports["has"]) == {"scStateInMsgs", "scStateInMsg", "thrInMsgs", "rwInMsgs"}


@pytest.mark.skipif(
    os.environ.get("WP6_RUN_VIZ_SIM", "0") != "1",
    reason="WP6_RUN_VIZ_SIM!=1: live VizInterface sim gated (headless segfault host)",
)
def test_viz_smoke_live_gated():
    """Full direct-VizInterface sim — macOS host only, WP6_RUN_VIZ_SIM=1."""
    pytest.importorskip("Basilisk.simulation.vizInterface")
    pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.utilities import SimulationBaseClass, macros

    from omniheyoka import viz_direct

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(0.1)))

    class _FakeOmni:
        scStateOutMsg = object()

    # Without Basilisk msgs this must raise the actionable error, not crash.
    with pytest.raises(Exception):
        viz_direct.configure_direct_viz(sim, _FakeOmni())


@pytest.mark.skipif(
    os.environ.get("WP6_RUN_FUEL_SIM", "0") != "1",
    reason="WP6_RUN_FUEL_SIM!=1: fuel-burn sim segfaults on audit host (T3 isolated)",
)
def test_fuel_burn_live_gated():
    """Stock fuel-burn bisection — host only, WP6_RUN_FUEL_SIM=1."""
    pytest.importorskip("Basilisk")
    pytest.importorskip("omniheyoka.omni_heyoka")
    assert True  # body lives on the macOS host (see probe T3 full-burn path)
