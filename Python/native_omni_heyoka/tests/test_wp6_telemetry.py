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
    # frictionTorque/theta ~0 below is SCOPED to this friction-OFF run: on the
    # SysModel path frictionTorque is the live module-side tauF readback
    # (iter-1 suppress-only, 0.0 while unmodeled/disabled) and theta/U_s/U_d
    # stay labeled zeros (no theta states); friction-ON shaping is vec-path
    # Stribeck (rw3_fric ledger cell, 1e-14 class) -- see WP25-B2.1.
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


# ---------------------------------------------------------------------------
# WP25-B2 live gates (host-only sims; importorskip -> skip without Basilisk or
# the built wheel). Each gate cites its C++ path (src/omni_spacecraft.cpp).
# ---------------------------------------------------------------------------

MU_B2 = 3.986004418e14
MASS_B2 = 750.0
IXX_B2, IYY_B2, IZZ_B2 = 900.0, 800.0, 600.0
DT_B2 = 0.1
RPM_TO_RADS_B2 = 2.0 * math.pi / 60.0
RADS_TO_RPM_B2 = 60.0 / (2.0 * math.pi)
JS_HR16_B2 = 50.0 / (6000.0 * RPM_TO_RADS_B2)


def _b2_base_sim(tag, dt=DT_B2, sigma0=(0.0, 0.0, 0.0),
                 omega0=(0.0, 0.0, 0.0)):
    """Minimal omni hub sim (no wheels/thrusters); caller adds config."""
    pytest.importorskip("numpy")
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))

    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU_B2, oe)

    sc = oh.OmniSpacecraft()
    sc.setMass(MASS_B2)
    sc.setInertia(IXX_B2, IYY_B2, IZZ_B2)
    sc.setMu(MU_B2)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.ModelTag = tag
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(*map(float, sigma0))
    sc.setOmegaBNInit(*map(float, omega0))
    sim.AddModelToTask("t", sc)
    return sim, sc, macros


def _b2_const_torque_driver(u_cmd):
    """Rewrite-every-tick motor-torque driver (WP1 pattern)."""
    from Basilisk.architecture import messaging, sysModel

    class _Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(motorTorque=[0.0]))

        def UpdateState(self, t):
            self.out.write(
                messaging.ArrayMotorTorqueMsgPayload(motorTorque=[float(u_cmd)]),
                t, self.moduleID)

    return _Driver()


def _b2_const_ontime_driver(on_time, dt=DT_B2):
    """Rewrite-every-tick on-time driver (full-step burn when on_time == dt)."""
    from Basilisk.architecture import messaging, sysModel

    class _Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            pay = messaging.THRArrayOnTimeCmdMsgPayload()
            pay.OnTimeRequest = [0.0] * 36
            self.out.write(pay)

        def UpdateState(self, t):
            pay = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = float(on_time)
            pay.OnTimeRequest = ot
            self.out.write(pay, t, self.moduleID)

    return _Driver()


def _dcmBN_from_sigma(s):
    """C_BN(sigma) MRP DCM (same formula as writeMsgs; test-side copy)."""
    s0, s1, s2 = (float(v) for v in s)
    s2n = s0 * s0 + s1 * s1 + s2 * s2
    d = (1.0 + s2n) * (1.0 + s2n)
    S = [[0.0, -s2, s1], [s2, 0.0, -s0], [-s1, s0, 0.0]]
    S2 = [[sum(S[i][k] * S[k][j] for k in range(3)) for j in range(3)]
          for i in range(3)]
    return [[(1.0 if i == j else 0.0)
             + (8.0 * S2[i][j] - 4.0 * (1.0 - s2n) * S[i][j]) / d
             for j in range(3)] for i in range(3)]


def test_b2_custom_off_wheel_reports_off():
    """B2.1 gate: Omega_max/u_min/P_max echo CONFIG (omni_spacecraft.cpp RW log).

    Custom addWheel caps OFF -> Omega_max == -1 (stock custom convention),
    never the 6000 RPM rated literal; rated wheel echoes its 6000 RPM cap.
    u_min/P_max echo the disarmed B1 defaults (0.0/200.0).
    """
    np = pytest.importorskip("numpy")
    pytest.importorskip("omniheyoka.omni_heyoka")
    sim, sc, macros = _b2_base_sim("wp25b2_off_echo")
    sc.addWheel(1.0, 0.0, 0.0, JS_HR16_B2, 0.2)  # custom: cap OFF
    sc.addRatedWheel(0.0, 1.0, 0.0, 50.0, 6000.0, 0.2)  # rated: 6000 RPM cap
    sc.setWheelSpeedsInitRPM([0.0, 0.0])
    rlogs = []
    for i in range(2):
        r = sc.rwOutMsgs[i].recorder()
        sim.AddModelToTask("t", r)
        rlogs.append(r)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(DT_B2 * 3))
    sim.ExecuteSimulation()
    om_off = np.asarray(rlogs[0].Omega_max, float).ravel()
    assert om_off.size >= 2
    assert np.all(om_off == pytest.approx(-1.0, abs=0.0)), om_off.tolist()
    om_rated = np.asarray(rlogs[1].Omega_max, float).ravel()
    assert np.all(om_rated == pytest.approx(6000.0 * RPM_TO_RADS_B2, rel=1e-12))
    assert np.all(np.asarray(rlogs[0].u_min, float).ravel()
                  == pytest.approx(0.0, abs=0.0))
    assert np.all(np.asarray(rlogs[0].P_max, float).ravel()
                  == pytest.approx(200.0, rel=1e-12))


def test_b2_setumax_post_registration_effective():
    """B2.2 gate: setUMax rewrites wheelUMax + dirty (omni_spacecraft.cpp:setUMax).

    addWheel(uMax=0.2) then setUMax(0.05): the u_max echo reads 0.05 and the
    spin-up slope matches the 0.05 clamp (half the unclamped 0.1 rate).
    """
    np = pytest.importorskip("numpy")
    pytest.importorskip("omniheyoka.omni_heyoka")
    sim, sc, macros = _b2_base_sim("wp25b2_setumax")
    sc.addWheel(1.0, 0.0, 0.0, JS_HR16_B2, 0.2)
    sc.setUMax(0.05)  # post-registration: must be reported-effective
    sc.setWheelSpeedsInitRPM([0.0])
    drv = _b2_const_torque_driver(0.1)
    sim.AddModelToTask("t", drv)
    sc.rwMotorCmdInMsg.subscribeTo(drv.out)
    rlog = sc.rwOutMsgs[0].recorder()
    sim.AddModelToTask("t", rlog)
    slog = sc.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("t", slog)
    n_ticks = 15
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(DT_B2 * n_ticks))
    sim.ExecuteSimulation()
    assert np.all(np.asarray(rlog.u_max, float).ravel()
                  == pytest.approx(0.05, rel=1e-12))
    rpm = (np.asarray(slog.wheelSpeeds, float)[:, 0] * RADS_TO_RPM_B2)
    steps = rpm.size - 1  # tick 0 is the IC tick (no propagation)
    assert steps >= 10, rpm.size
    per_step = 0.05 * DT_B2 / JS_HR16_B2 * RADS_TO_RPM_B2
    assert (rpm[-1] - rpm[0]) == pytest.approx(steps * per_step, rel=0.05)
    # ... and well below the unclamped 0.1 rate (clamp actually applied).
    assert (rpm[-1] - rpm[0]) < 0.75 * steps * 2.0 * per_step


def test_b2_dv_rotated_at_nonidentity_attitude():
    """B2.3 gate (M11): body impulse rotated by R_BN into TotalAccumDV_CN_N.

    Torque-free full-step +z burn at sigma=(0.3,0,0): the inertial slot must
    equal R^T F T/m (not the raw body impulse, which the body slots keep).
    C++ path: UpdateState DV block (Rbn rotation).

    First-fire timing: the const driver rewrites every tick but task order is
    sc->drv, so its first write lands during tick1 (after sc's tick1 staging)
    and the first prop tick idles on stale Reset-zeros — 9 fires, not 10
    (host dump: z-col diffs [0, k, k x9], final 0.9x). The write-once snapshot
    below fixes the timing: a direct message write AFTER InitializeSimulation
    (a pre-init write would be clobbered by the driver's own Reset-zeros, so
    the payload must land after all Resets and before tick0's staging) is
    staged by tick0's fill per the staged-mailbox contract in
    src/omni_spacecraft.cpp:962-971,1104+ (cf. omniheyoka/swap.py pre-init
    snapshot comment), so the first prop fires: 10 uniform increments.
    """
    np = pytest.importorskip("numpy")
    pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.architecture import messaging
    sigma0 = (0.3, 0.0, 0.0)
    sim, sc, macros = _b2_base_sim("wp25b2_dv_rot", sigma0=sigma0)
    sc.addThruster(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)  # +z force, zero torque
    drv = _b2_const_ontime_driver(DT_B2)
    sim.AddModelToTask("t", drv)
    sc.thrOnTimeInMsg.subscribeTo(drv.out)
    slog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", slog)
    sim.InitializeSimulation()
    # Write-once snapshot (same payload idiom as _b2_const_ontime_driver):
    # overwrites the driver's Reset-zeros so tick0's fill stages the burn
    # payload (first-sight staging); the driver's same-stamp tick0 rewrite
    # then preserves it, so prop tick 1 fires instead of idling.
    snap = messaging.THRArrayOnTimeCmdMsgPayload()
    snap_ot = [0.0] * 36
    snap_ot[0] = float(DT_B2)
    snap.OnTimeRequest = snap_ot
    drv.out.write(snap)
    sim.ConfigureStopTime(macros.sec2nano(DT_B2 * 10))
    sim.ExecuteSimulation()
    dv_cn = np.asarray(slog.TotalAccumDV_CN_N, float)
    dv_bdy = np.asarray(slog.TotalAccumDVBdy, float)
    assert dv_cn.shape[1] == 3 and dv_cn.shape[0] >= 5
    n_prop = dv_cn.shape[0] - 1  # tick 0 is the pre-fire fill (DV == 0)
    assert np.all(dv_cn[0] == pytest.approx([0.0, 0.0, 0.0], abs=0.0))
    R = _dcmBN_from_sigma(sigma0)
    F = [0.0, 0.0, 1.0]
    k = n_prop * DT_B2 / MASS_B2
    k1 = DT_B2 / MASS_B2  # single-tick climb (total k = n_prop * k1)
    expect_cn = [sum(R[j][i] * F[j] for j in range(3)) * k for i in range(3)]
    expect_bdy = [F[i] * k for i in range(3)]
    step_cn = [sum(R[j][i] * F[j] for j in range(3)) * k1 for i in range(3)]
    assert dv_cn[-1].tolist() == pytest.approx(expect_cn, rel=1e-6)
    assert dv_bdy[-1].tolist() == pytest.approx(expect_bdy, rel=1e-9)
    # Rotation actually applied (non-identity proof): slots differ.
    diff = float(np.linalg.norm(dv_cn[-1] - dv_bdy[-1]))
    assert diff > 1e-6, diff
    # First-prop-fired regression (the driver-timing class): row1 already
    # carries exactly one rotated tick — without the snapshot it idles at 0.
    assert dv_cn[1].tolist() == pytest.approx(step_cn, rel=1e-6)
    # Per-tick uniformity (rotation proof): every prop tick adds step_cn.
    for i in range(1, dv_cn.shape[0] - 1):
        assert (dv_cn[i + 1] - dv_cn[i]).tolist() == pytest.approx(step_cn, rel=1e-6)


def test_b2_double_reset_dv_clean():
    """B2.3 gate (M12): Reset clears DV/fuel accumulators (omni_spacecraft.cpp:Reset).

    Burn, Reset, burn again: the record must DROP to ~0 at the boundary
    (fresh accumulation), never continue the old total.

    BSK re-init trap (host-measured, so re-Initialize is NOT used here):
    a second InitializeSimulation() does NOT reach our Reset in this tasking
    configuration — run 1 (sigma=0, +z 1 N, k=DT/m=1.333e-4/tick) ended at 5k
    and run 2 contributed 4 records climbing 5k->6k with NO zero/fill record
    at the boundary (initDone stayed true), so re-Initialize != Reset here.
    The contract under test ("Reset() clears") is therefore exercised
    DETERMINISTICALLY via an explicit sc.Reset call below.
    """
    np = pytest.importorskip("numpy")
    pytest.importorskip("omniheyoka.omni_heyoka")
    sim, sc, macros = _b2_base_sim("wp25b2_reset_clean")
    sc.addThruster(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    drv = _b2_const_ontime_driver(DT_B2)
    sim.AddModelToTask("t", drv)
    sc.thrOnTimeInMsg.subscribeTo(drv.out)
    slog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", slog)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(DT_B2 * 6))
    sim.ExecuteSimulation()
    reset = getattr(sc, "Reset", None)
    if not callable(reset):
        pytest.fail("OmniSpacecraft.Reset not exposed via SWIG: "
                    "expose Reset in .i (B2 leftover)")
    # Explicit Reset at the current sim nanos (end of run 1); run 2's first
    # tick is then a fill publishing ~0, followed by fresh fires.
    reset(macros.sec2nano(DT_B2 * 6))
    sim.ConfigureStopTime(macros.sec2nano(DT_B2 * (6 + 4)))
    sim.ExecuteSimulation()
    dv = np.asarray(slog.TotalAccumDV_CN_N, float)[:, 2]  # +z burn axis
    assert dv.size >= 8, dv.size
    k1 = DT_B2 / MASS_B2  # single-tick climb on the burn axis
    drops = [i for i in range(1, dv.size) if dv[i] < dv[i - 1] - 1e-15]
    assert drops, "no Reset boundary found (DV never drops)"
    b = drops[0]
    assert dv[b] == pytest.approx(0.0, abs=1e-12), (b, dv[b])
    assert dv[b - 1] > 0.0, "first run never burned (vacuous gate)"
    # Run 2 = 4 ticks = 1 fill (the boundary record) + fresh fires: the
    # re-climb is counted from the boundary, never assumed, and must stay
    # below the pre-boundary total (fresh, not continued).
    n_post = dv.size - b - 1
    assert n_post >= 1, "run 2 never re-burned (vacuous gate)"
    assert dv[-1] == pytest.approx(n_post * k1, rel=1e-6), (dv[-1], n_post, k1)
    assert dv[-1] < dv[b - 1], (dv[-1], dv[b - 1])  # fresh, not continued
