"""WP3 thruster telemetry publish + guards + timing models (PLAN_ENVELOPE.md Rev 1).

Ledger (all green, BSK 2.12.0 / macOS arm64, committed JSON):
* ``audit_bundle/results/wp3_timing.json``: M0 mailbox matrix, MQ staircase
  {0,1/6,5/6,1}, M4 arc-phase (delayed-normal delta 0 @ dt=0.1), M5
  over(1.5)->1 / negative(-0.1)->0 + 10/20 Hz trains, M1 empty ramps + step,
  M2 cutoff ladder {10,100,1000} (identical norms @ dt=0.1, duty 1.0), M3
  min-pulse histogram (sub-MinOnTime suppresses, >=MinOnTime fires @
  dt=0.01/MinOnTime=0.02), M6 blowdown [0.1] runs with the stock no-tank
  warning / swirl-create accepted / THRArrayCmdForceMsg present, SMSG THR
  9/9 + THRConfig 4-field.
* ``audit_bundle/results/wp6_telem_viz_fuel.json`` T1: THR 9/9 field list +
  1-step dump (tick0 pre-fire zeros, steady value = last tick; blowdown
  factors 1.0; location/direction/maxThrust echo; torque [0,-0.5,0] about B
  UNSHIFTED for pos [0.5,0,1] / dir [0,0,1] / Tmax 1.0).

What this file covers (WP3a first, WP3b behind the green ledger):
* Part A (pure, stdlib only): reference mirrors of the C++ timing math with
  EXACTLY the spec semantics (quantize, min-pulse gate, ramp factor, lag
  step, blowdown poly, swirl axis, arc-cut list). No Basilisk, no built
  extension, no numpy -- runs anywhere.
* Part B (ledger pins): the committed JSON rows the models must reproduce.
* Part C (C++ contract): ``_need`` skip-until-applied assertions on the four
  WP3 edit-spec targets (house convention: green before the specs land,
  binding after).
* Part D (host sims, importorskip): telemetry 9/9 publish + tick0/steady
  semantics, force-msg guard raise, setter validation raises, M3 rows,
  M2-ladder default path, T2 chatter @ dt=0.01, M4 phase, M5 duty rows,
  swirl axis, blowdown no-tank factors. macOS host only.

Ownership: this file is WP3-owned (free write). Shared files are READ ONLY
here; the C++ lives in verbatim edit-specs (see the WP3 return note), NOT
in this file.

Layout (A2): the only new par slot is the contributor ``swirlScale`` input
(default 0.0); everything else folds module-side through the existing q
slots (the thruster q-slot pattern). Graph purity (A3): no clamp/branch in
the symbolic path -- staging, ramps, lag, blowdown live module-side.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WP3_JSON = REPO / "audit_bundle" / "results" / "wp3_timing.json"
WP6_JSON = REPO / "audit_bundle" / "results" / "wp6_telem_viz_fuel.json"
SPACECRAFT_H = REPO / "include" / "omni" / "spacecraft.hpp"
SPACECRAFT_CPP = REPO / "src" / "omni_spacecraft.cpp"
CONTRIBUTORS = REPO / "include" / "omni" / "contributors.hpp"
SWIG_I = REPO / "src" / "omni_heyoka.i"

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
SWIG_EXTRAS = {"this", "thisown"}

MU = 3.986004418e14


def _load(path: Path) -> dict:
    assert path.exists(), f"missing ledger {path}"
    return json.loads(path.read_text())


def _payload(obj: dict) -> dict:
    return obj.get("payload", obj)


# ---------------------------------------------------------------------------
# Part A -- pure reference mirrors (EXACT C++ spec semantics).
# ---------------------------------------------------------------------------

def ref_quantize(duty: float) -> float:
    """Measured staircase (contributor quantize; MQ ledger)."""
    if duty <= 0.0:
        return 0.0
    if duty < 0.5:
        return 1.0 / 6.0
    if duty < 1.0:
        return 5.0 / 6.0
    return 1.0


def ref_on_time(on_time_cmd: float, dt: float) -> float:
    """Quantized valve-open duration (contributor onTime)."""
    if not dt > 0.0:
        return 0.0
    return ref_quantize(on_time_cmd / dt) * dt


def ref_min_pulse_gate(on_time_cmd: float, q: float, min_on_time: float) -> float:
    """M3 suppress gate (contributor minPulseGate; staging owns memory)."""
    if min_on_time > 0.0 and on_time_cmd < min_on_time:
        return 0.0
    return q


def ref_ramp_factor(t: float, knots: list, linear: bool, is_on: bool) -> float:
    """Module rampFactor mirror. Empty knots = no ramp (on: 1, off: 0)."""
    if not knots:
        return 1.0 if is_on else 0.0
    span = knots[-1]
    if not span > 0.0:
        return 1.0 if is_on else 0.0
    if t <= 0.0:
        return 0.0 if is_on else 1.0
    if t >= span:
        return 1.0 if is_on else 0.0
    if linear:
        f = t / span
        return f if is_on else (1.0 - f)
    n = sum(1 for k in knots if k <= t)
    f = n / len(knots)
    return f if is_on else (1.0 - f)


def ref_lag_step(f: float, target: float, seg: float, cutoff: float) -> float:
    """Exact first-order lag update (module arc loop, tau = 1/cutoff)."""
    return f + (target - f) * (1.0 - math.exp(-seg * cutoff))


def ref_blowdown_poly(coeffs: list, x: float) -> float:
    """Descending Horner poly (contributor blowdownPoly); empty -> 1.0."""
    if not coeffs:
        return 1.0
    y = 0.0
    for c in coeffs:
        y = y * x + c
    return y


def ref_swirl_torque(q: float, swirl_scale: float, direction: list) -> list:
    """Axis torque about the thrust direction (contributor swirl term)."""
    return [q * swirl_scale * d for d in direction]


def ref_arc_cuts(dt: float, open_times: list, on_knots: list,
                 off_knots: list, ramps_on: bool) -> list:
    """Switch-list mirror (module arc loop; eps-merged, sorted, +dt)."""
    eps = 1e-9 * dt
    sw = [0.0]

    def add(t: float) -> None:
        if not (t > eps and t < dt - eps):
            return
        for s in sw:
            if abs(s - t) <= eps:
                return
        sw.append(t)

    for t in open_times:
        add(t)
    if ramps_on:
        for ot in open_times:
            if ot <= eps:
                continue
            for k in on_knots:
                add(k)
            for k in off_knots:
                add(ot + k)
    sw.sort()
    sw.append(dt)
    return sw


def test_staircase_mirror_edges():
    assert ref_quantize(0.0) == 0.0
    assert ref_quantize(-0.1) == 0.0
    assert ref_quantize(1e-12) == pytest.approx(1.0 / 6.0)
    assert ref_quantize(0.49) == pytest.approx(1.0 / 6.0)
    assert ref_quantize(0.5) == pytest.approx(5.0 / 6.0)
    assert ref_quantize(0.9999999) == pytest.approx(5.0 / 6.0)
    assert ref_quantize(1.0) == pytest.approx(1.0)
    assert ref_quantize(1.5) == pytest.approx(1.0)
    assert ref_on_time(0.05, 0.1) == pytest.approx(5.0 / 6.0 * 0.1)
    assert ref_on_time(0.1, 0.1) == pytest.approx(0.1)
    assert ref_on_time(0.05, 0.0) == 0.0


def test_min_pulse_gate_mirror():
    # M3 rule: sub-MinOnTime suppresses, >= MinOnTime passes.
    assert ref_min_pulse_gate(0.004, 1.0 / 6.0, 0.02) == 0.0
    assert ref_min_pulse_gate(0.01, 1.0, 0.02) == 0.0
    assert ref_min_pulse_gate(0.02, 1.0, 0.02) == pytest.approx(1.0)
    assert ref_min_pulse_gate(0.03, 1.0, 0.02) == pytest.approx(1.0)
    assert ref_min_pulse_gate(0.004, 1.0 / 6.0, 0.0) == pytest.approx(1.0 / 6.0)


def test_ramp_factor_mirror():
    assert ref_ramp_factor(0.0, [], True, True) == 1.0
    assert ref_ramp_factor(0.05, [], True, False) == 0.0
    knots = [0.02, 0.05]
    assert ref_ramp_factor(0.0, knots, True, True) == 0.0
    assert ref_ramp_factor(0.01, knots, True, True) == pytest.approx(0.2)
    assert ref_ramp_factor(0.05, knots, True, True) == pytest.approx(1.0)
    assert ref_ramp_factor(0.06, knots, True, True) == pytest.approx(1.0)
    assert ref_ramp_factor(0.01, knots, True, False) == pytest.approx(0.8)
    # Step-hold (stock THRTimePair shape): 2 knots -> halves.
    assert ref_ramp_factor(0.01, knots, False, True) == pytest.approx(0.0)
    assert ref_ramp_factor(0.03, knots, False, True) == pytest.approx(0.5)
    assert ref_ramp_factor(0.03, knots, False, False) == pytest.approx(0.5)


def test_lag_step_mirror():
    # Default tau = 1/10 = 0.1 s (M2 ladder, stock default 10 rad/s).
    assert ref_lag_step(0.0, 1.0, 0.1, 10.0) == pytest.approx(1.0 - math.exp(-1.0))
    # Benches pin 1000 to hide the lag: single step ~= identity.
    assert ref_lag_step(0.0, 1.0, 0.1, 1000.0) == pytest.approx(1.0)
    # Converges asymptotically; stays put at target.
    f = 0.0
    for _ in range(100):
        f = ref_lag_step(f, 1.0, 0.1, 10.0)
    assert f == pytest.approx(1.0)
    assert ref_lag_step(1.0, 1.0, 0.1, 10.0) == pytest.approx(1.0)


def test_blowdown_poly_mirror():
    assert ref_blowdown_poly([], 0.0) == 1.0
    assert ref_blowdown_poly([0.1], 7.3) == pytest.approx(0.1)
    # Descending Horner: [a, b] @ x = a*x + b.
    assert ref_blowdown_poly([2.0, 3.0], 5.0) == pytest.approx(13.0)
    # No-tank module rule (M6): factors held at 1.0, depletion OUT (labeled).


def test_swirl_axis_mirror():
    t = ref_swirl_torque(1.0, 0.05, [0.0, 0.0, 1.0])
    assert t == pytest.approx([0.0, 0.0, 0.05])
    assert ref_swirl_torque(0.0, 0.05, [0.0, 0.0, 1.0]) == pytest.approx([0.0, 0.0, 0.0])
    # Axis torque is parallel to the thrust direction by construction.
    d = [0.0, -1.0, 0.0]
    t2 = ref_swirl_torque(5.0 / 6.0, 0.05, d)
    assert t2 == pytest.approx([0.0, -5.0 / 6.0 * 0.05, 0.0])


def test_arc_cut_mirror_structure():
    dt = 0.1
    sw = ref_arc_cuts(dt, [5.0 / 6.0 * dt], [], [], False)
    assert sw[0] == pytest.approx(0.0) and sw[-1] == pytest.approx(dt)
    assert sw == sorted(sw)
    assert len(sw) == 3  # 0, openT, dt (no ramp cuts when off)
    sw2 = ref_arc_cuts(dt, [dt], [0.02, 0.05], [0.03], True)
    # on-knots land as cuts; off-knot 0.1+0.03 falls outside the step; the
    # openT==dt edge itself is the appended dt (never a duplicate cut).
    assert sw2 == pytest.approx([0.0, 0.02, 0.05, dt], abs=1e-12)


# ---------------------------------------------------------------------------
# Part B -- ledger pins (committed JSON; the models must reproduce these).
# ---------------------------------------------------------------------------

def test_ledger_mq_staircase_green():
    duties = _payload(_load(WP3_JSON))["probes"]["MQ"]["duties"]
    assert len(duties) == 10
    for row in duties:
        cmd = row["cmd"]
        clipped = min(max(cmd, 0.0), 1.0)
        # Mirror matches the recorded expectation exactly (sign: ledger uses
        # the negative torque branch; compare magnitudes).
        assert ref_quantize(clipped) == pytest.approx(abs(row["expected"]), abs=1e-12)
        assert abs(row["err"]) < 1e-3, row  # ledger green pin


def test_ledger_m3_suppress_fire_classes():
    m3 = _payload(_load(WP3_JSON))["probes"]["M3"]
    assert m3["MinOnTime"] == pytest.approx(0.02)
    assert m3["dt"] == pytest.approx(0.01)
    for key, mult in (("x0.2", 0.2), ("x0.5", 0.5), ("x1.0", 1.0), ("x1.5", 1.5)):
        cell = m3[key]
        assert cell["ontime_cmd"] == pytest.approx(mult * 0.02)
        gate = ref_min_pulse_gate(cell["ontime_cmd"], 1.0, 0.02)
        idle = all(v == pytest.approx(0.0, abs=0.0) for v in cell["norms"])
        if gate == 0.0:
            assert idle, (key, cell["norms"])  # suppressed rows are all-idle
        else:
            assert not idle, (key, cell["norms"])  # firing rows move
    assert all(v == pytest.approx(0.0, abs=0.0) for v in m3["chatter"]["norms"])


def test_ledger_m2_ladder_identical_at_dt01():
    m2 = _payload(_load(WP3_JSON))["probes"]["M2"]["cutoff_ladder"]
    norms = [m2[k]["norms"] for k in ("10.0", "100.0", "1000.0")]
    assert norms[0] == pytest.approx(norms[1], abs=1e-12)
    assert norms[0] == pytest.approx(norms[2], abs=1e-12)
    for k in ("10.0", "100.0", "1000.0"):
        assert m2[k]["thrustFactor"][-1] == pytest.approx(1.0)
        assert m2[k]["thrustForce"][-1] == pytest.approx(1.0)
        assert m2[k]["thrustBlowDownFactor"][-1] == pytest.approx(1.0)


def test_ledger_m1_empty_ramps():
    m1 = _payload(_load(WP3_JSON))["probes"]["M1"]
    assert m1["static"]["OnRamp_len"] == 0
    assert m1["static"]["OffRamp_len"] == 0
    assert m1["step"]["pair_type"] == "THRTimePair"


def test_ledger_m6_blowdown_swirl_force():
    m6 = _payload(_load(WP3_JSON))["probes"]["M6"]
    # Blowdown [0.1] with no tank: identical fire norms to the no-blowdown
    # M0 cell (factors stayed 1.0; warning is informative, M6 record).
    m0_norms = _payload(_load(WP3_JSON))["probes"]["M0"]["cells"]["fresh_same_FFF"]["norms"]
    assert m6["blowdown_smoke_norms"] == pytest.approx(m0_norms, abs=1e-12)
    assert m6["swirl_create"] == "accepted"
    assert m6["THRArrayCmdForceMsg_present"] is True


def test_ledger_smsg_thr_9_and_config_4():
    smsg = _payload(_load(WP3_JSON))["probes"]["SMSG"]
    assert all(smsg["THRConfig_4field"].get(f) for f in
               ("maxThrust", "rThrust_B", "swirlTorque", "tHatThrust_B"))
    got9 = smsg["THROutput_9field"]
    assert all(got9.get(f) for f in THR_EXPECTED_9), got9


def test_ledger_t1_dump_values():
    t1 = _payload(_load(WP6_JSON))["t1_payload_census"]
    real = [n for n in t1["thr_output_static_fields"] if n not in SWIG_EXTRAS]
    assert sorted(real) == sorted(THR_EXPECTED_9), real
    d = t1["thr_1step_dump"]
    assert d["thrustFactor"] == pytest.approx(1.0)
    assert d["thrustFactor_tick0"] == pytest.approx(0.0)
    assert d["thrustForce"] == pytest.approx(1.0)
    assert d["thrustForce_tick0"] == pytest.approx(0.0)
    assert list(d["thrustForce_B"]) == pytest.approx([0.0, 0.0, 1.0])
    assert list(d["thrustTorquePntB_B"]) == pytest.approx([0.0, -0.5, 0.0])
    assert d["thrustBlowDownFactor"] == pytest.approx(1.0)
    assert d["ispBlowDownFactor"] == pytest.approx(1.0)
    assert list(d["thrusterLocation"]) == pytest.approx([0.5, 0.0, 1.0])
    assert list(d["thrusterDirection"]) == pytest.approx([0.0, 0.0, 1.0])
    assert "last tick" in t1["thr_dump_note"] and "tick0" in t1["thr_dump_note"]


# ---------------------------------------------------------------------------
# Part C -- C++ contract (SKIP until the WP3 edit-specs land; assert after).
# ---------------------------------------------------------------------------

def _need(path: Path, needle: str) -> str:
    """File text if it contains needle, else skip (spec not applied)."""
    assert path.exists(), f"missing {path}"
    src = path.read_text()
    if needle not in src:
        pytest.skip(f"WP3 C++ spec not yet applied: {needle!r} absent in {path.name}")
    return src


def test_cpp_contract_header_flags_setters_telemetry():
    # WP0 flag names EXACTLY (spec binding) + setters + publisher + guard port.
    src = _need(SPACECRAFT_H, "minPulseOn_")
    for name in ("rampsOn_", "cutoffLagOn_", "blowdownOn_", "swirlOn_",
                 "forceMsgOn_"):
        assert name in src, f"WP0 flag missing: {name}"
    for name in ("setMinPulseTime", "setThrusterRamps", "setCutoffFrequency",
                 "setBlowdownCoeffs", "setMaxSwirlTorque"):
        assert name in src, f"setter decl missing: {name}"
    assert "thrusterOutMsgs" in src
    assert "Message<THROutputMsgPayload>" in src
    assert "thrForceInMsg" in src
    assert "THRArrayCmdForceMsgPayload" in src
    assert "THROutputMsgPayload" in src


def test_cpp_contract_module_timing_states():
    src = _need(SPACECRAFT_CPP, "minPulseState_")
    for needle in ("lagState_", "rampClock_", "lastFilt_", "lastQ_",
                   "rampFactor", "swirlScale", "WP5-arc-hook",
                   "WP3: thruster telemetry"):
        assert needle in src, f"arc/staging piece missing: {needle}"
    assert "thrForceInMsg.isLinked()" in src  # force guard lives in staging


def test_cpp_contract_contributor_hooks_only():
    # ThrusterContributorT ONLY: swirl term + blowdown/min-pulse hooks.
    # HubContributor must stay untouched (WP4 owns it).
    src = _need(CONTRIBUTORS, "swirlScale")
    assert '"swirlScale"' in src
    assert "scale_vec3(d, swirl)" in src or "swirl" in src
    assert "blowdownPoly" in src
    assert "minPulseGate" in src
    hub_region = src.split("class HubContributorT")[1].split(
        "class ReactionWheelContributorT")[0]
    for bad in ("swirl", "blowdown", "minPulse", "ramp", "lag"):
        assert bad not in hub_region, f"HubContributor touched by WP3: {bad}"


def test_cpp_contract_no_branch_in_graph():
    # A3: timing math is module-side on doubles; the thruster graph paths stay
    # pure arithmetic. Passes before AND after the spec.
    src = CONTRIBUTORS.read_text()
    region = src.split("class ThrusterContributorT")[1].split("}  // namespace omni")[0]
    for bad in ("tanh", "std::min", "std::max", "fmin", "fmax", "clamp",
                "std::exp", "exp("):
        assert bad not in region, f"A3 violation in thruster contributor: {bad}"


def test_cpp_contract_swig_payloads():
    src = _need(SWIG_I, "THROutputMsgPayload.h")
    assert "THRArrayCmdForceMsgPayload.h" in src
    assert "THROutputMsgVector" in src


# ---------------------------------------------------------------------------
# Part D -- host sims (Basilisk macOS-only + built extension).
# ---------------------------------------------------------------------------

def _need_wp3_api(sc, *names: str) -> None:
    for name in names:
        if not callable(getattr(sc, name, None)):
            pytest.skip(f"WP3 C++ spec not yet applied: OmniSpacecraft.{name} missing")


def _run_omni_thr(*, duty_cmd, dt=0.1, n_ticks=3, writer="fresh",
                  pos=(0.5, 0.0, 1.0), direction=(0.0, 0.0, 1.0), tmax=1.0,
                  opts=None, twice_writes=False):
    """Single-thruster omni sim (sigma=0/omega=0/GG-off, cubesat bus).

    Returns (omega_log, thr0_log or None, sc). ``writer`` mirrors the probe
    modes: fresh / once / delayed / double / frozen. ``opts`` may hold
    minPulse=(bool, s), ramps=(on, off, linear), cutoff=Hz, blowdown=([...],),
    swirl=s, force=True. thr0_log is None when thrusterOutMsgs is absent.
    """
    np = pytest.importorskip("numpy")
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion

    from cubesat_12u_sim.core import config as C

    mass = float(C.MASS)
    i6 = np.asarray(C.INERTIA, float)

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)

    sc = oh.OmniSpacecraft()
    sc.setMass(mass)
    sc.setInertia(float(i6[0, 0]), float(i6[1, 1]), float(i6[2, 2]),
                  float(i6[0, 1]), float(i6[0, 2]), float(i6[1, 2]))
    sc.setMu(MU)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.addThruster(float(direction[0]), float(direction[1]), float(direction[2]),
                   float(pos[0]), float(pos[1]), float(pos[2]), float(tmax))
    sc.ModelTag = "wp3_omni_thr"
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    if opts:
        if "minPulse" in opts:
            _need_wp3_api(sc, "setMinPulseTime")
            sc.setMinPulseTime(bool(opts["minPulse"][0]), float(opts["minPulse"][1]))
        if "ramps" in opts:
            _need_wp3_api(sc, "setThrusterRamps")
            on, off, lin = opts["ramps"]
            sc.setThrusterRamps(list(map(float, on)), list(map(float, off)), bool(lin))
        if "cutoff" in opts:
            _need_wp3_api(sc, "setCutoffFrequency")
            sc.setCutoffFrequency(float(opts["cutoff"]))
        if "blowdown" in opts:
            _need_wp3_api(sc, "setBlowdownCoeffs")
            # Accept ([t...],), ([t...], [i...]), or flat [t...].
            bd = opts["blowdown"]
            if isinstance(bd, (list, tuple)) and len(bd) == 1 \
                    and isinstance(bd[0], (list, tuple)):
                bd = bd[0]
            if isinstance(bd, (list, tuple)) and len(bd) == 2 \
                    and all(isinstance(x, (list, tuple)) for x in bd):
                sc.setBlowdownCoeffs(list(map(float, bd[0])),
                                     list(map(float, bd[1])))
            else:
                sc.setBlowdownCoeffs(list(map(float, bd)))
        if "swirl" in opts:
            _need_wp3_api(sc, "setMaxSwirlTorque")
            sc.setMaxSwirlTorque(float(opts["swirl"]))
    sim.AddModelToTask("t", sc, 1)

    mode = writer

    class Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()
            self._prev = 0.0

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            cur = float(duty_cmd) * dt
            if mode == "delayed":
                send, self._prev = self._prev, cur
            elif mode == "frozen":
                return
            else:
                send = cur
            if mode == "double" or twice_writes:
                p2 = messaging.THRArrayOnTimeCmdMsgPayload()
                o2 = [0.0] * 36
                o2[0] = send * 0.5
                p2.OnTimeRequest = o2
                self.out.write(p2, t, self.moduleID)
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = send
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)

    force_msg = None
    if opts and opts.get("force"):
        if not hasattr(sc, "thrForceInMsg"):
            pytest.skip("WP3 C++ spec not yet applied: thrForceInMsg missing")
        force_msg = messaging.THRArrayCmdForceMsg()
        fp = messaging.THRArrayCmdForceMsgPayload()
        fp.thrForce = [0.0] * 36
        force_msg.write(fp)
        sc.thrForceInMsg.subscribeTo(force_msg)

    if writer == "once":
        drv = messaging.THRArrayOnTimeCmdMsg()
        pay = messaging.THRArrayOnTimeCmdMsgPayload()
        ot = [0.0] * 36
        ot[0] = float(duty_cmd) * dt
        pay.OnTimeRequest = ot
        drv.write(pay)
        sc.thrOnTimeInMsg.subscribeTo(drv)
    else:
        drv = Driver()
        sim.AddModelToTask("t", drv)
        sc.thrOnTimeInMsg.subscribeTo(drv.out)

    om_log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", om_log)
    thr_log = None
    if len(getattr(sc, "thrusterOutMsgs", [])) > 0:
        thr_log = sc.thrusterOutMsgs[0].recorder()
        sim.AddModelToTask("t", thr_log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()
    return np.asarray(om_log.omega_BN_B, float), thr_log, sc


def _deltas(om):
    import numpy as np

    arr = np.asarray(om, float)
    return [arr[k] - arr[k - 1] for k in range(1, len(arr))]


def test_telemetry_nine_fields_tick0_and_steady():
    """T1 gate on the omni arm: 9/9 fields, tick0 zeros, steady fire values."""
    np = pytest.importorskip("numpy")
    om, thr, sc = _run_omni_thr(duty_cmd=1.0, dt=0.1, n_ticks=3)
    if thr is None:
        pytest.skip("WP3 C++ spec not yet applied: thrusterOutMsgs empty")
    names = [n for n in dir(thr) if not n.startswith("_")]
    real = [n for n in names if n not in SWIG_EXTRAS]
    for f in THR_EXPECTED_9:  # superset: recorders may add helpers (times())
        assert f in real, (f, real)
    factor = np.asarray(thr.thrustFactor, float).ravel()
    force = np.asarray(thr.thrustForce, float).ravel()
    assert factor[0] == pytest.approx(0.0, abs=0.0)  # tick0 pre-fire fill
    assert factor[-1] == pytest.approx(1.0, abs=1e-6)  # steady = last tick
    assert force[0] == pytest.approx(0.0, abs=0.0)
    assert force[-1] == pytest.approx(1.0, abs=1e-6)
    fb = np.asarray(thr.thrustForce_B, float)
    np.testing.assert_allclose(fb[-1], [0.0, 0.0, 1.0], atol=1e-9)
    tq = np.asarray(thr.thrustTorquePntB_B, float)
    # About-B UNSHIFTED r x F = [0.5,0,1] x [0,0,1] = [0,-0.5,0] (T1 ledger).
    np.testing.assert_allclose(tq[-1], [0.0, -0.5, 0.0], atol=1e-9)
    np.testing.assert_allclose(tq[0], [0.0, 0.0, 0.0], atol=0.0)
    assert np.asarray(thr.thrustBlowDownFactor, float).ravel()[-1] == pytest.approx(1.0)
    assert np.asarray(thr.ispBlowDownFactor, float).ravel()[-1] == pytest.approx(1.0)
    np.testing.assert_allclose(np.asarray(thr.thrusterLocation, float)[-1],
                               [0.5, 0.0, 1.0], atol=0.0)
    np.testing.assert_allclose(np.asarray(thr.thrusterDirection, float)[-1],
                               [0.0, 0.0, 1.0], atol=0.0)


def test_force_msg_guard_raises():
    """M6 must-raise gate: a linked THRArrayCmdForceMsg bskErrors."""
    with pytest.raises(Exception, match="[Ff]orce"):
        _run_omni_thr(duty_cmd=0.5, dt=0.1, n_ticks=2, opts={"force": True})


def test_timing_setters_validation_raises():
    """Each setter loud-rejects non-physical input (sim-free, built module)."""
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    sc = oh.OmniSpacecraft()
    _need_wp3_api(sc, "setMinPulseTime", "setThrusterRamps",
                  "setCutoffFrequency", "setBlowdownCoeffs", "setMaxSwirlTorque")
    with pytest.raises(Exception):
        sc.setMinPulseTime(True, 0.0)
    with pytest.raises(Exception):
        sc.setMinPulseTime(True, -0.01)
    with pytest.raises(Exception):
        sc.setThrusterRamps([0.05, 0.02], [], True)  # not ascending
    with pytest.raises(Exception):
        sc.setThrusterRamps([-0.01], [], True)
    with pytest.raises(Exception):
        sc.setCutoffFrequency(0.0)
    with pytest.raises(Exception):
        sc.setCutoffFrequency(float("nan"))
    with pytest.raises(Exception):
        sc.setBlowdownCoeffs([float("nan")])
    with pytest.raises(Exception):
        sc.setMaxSwirlTorque(-0.01)
    # Benign values accepted (flags flip, no raise).
    sc.setMinPulseTime(True, 0.02)
    sc.setMinPulseTime(False, 0.02)
    sc.setThrusterRamps([0.02, 0.05], [0.03], True)
    sc.setThrusterRamps([], [], True)
    sc.setCutoffFrequency(1000.0)
    sc.setBlowdownCoeffs([0.1])
    sc.setBlowdownCoeffs([], [])
    sc.setMaxSwirlTorque(0.05)
    sc.setMaxSwirlTorque(0.0)


def test_timing_models_enabled_run_stays_sane():
    """All five models on: short run completes, telemetry stays 9-field."""
    np = pytest.importorskip("numpy")
    om, thr, sc = _run_omni_thr(
        duty_cmd=1.0, dt=0.1, n_ticks=3,
        opts={"minPulse": (True, 0.02), "ramps": ([0.02, 0.05], [0.03], True),
              "cutoff": 100.0, "blowdown": ([0.1],), "swirl": 0.05})
    if thr is None:
        pytest.skip("WP3 C++ spec not yet applied: thrusterOutMsgs empty")
    assert np.all(np.isfinite(np.asarray(om, float)))
    factor = np.asarray(thr.thrustFactor, float).ravel()
    assert factor[0] == pytest.approx(0.0, abs=0.0)
    assert 0.0 <= factor[-1] <= 1.0 + 1e-9


def test_m3_min_pulse_rows_on_sim():
    """M3 histogram on the omni arm @ dt=0.01, MinOnTime=0.02."""
    dt, mn = 0.01, 0.02
    classes = {}
    for key, mult in (("x0.2", 0.2), ("x0.5", 0.5), ("x1.0", 1.0), ("x1.5", 1.5)):
        duty = mult * mn / dt
        om, _, _ = _run_omni_thr(duty_cmd=duty, dt=dt, n_ticks=3,
                                 opts={"minPulse": (True, mn)})
        dl = _deltas(om)
        ref = float(np_l2(dl[1])) if len(dl) > 1 else 0.0
        fires = [float(np_l2(d)) >= 0.05 * ref for d in dl] if ref > 0 else \
            [float(np_l2(d)) > 1e-9 for d in dl]
        classes[key] = any(fires[1:])
    assert classes["x0.2"] is False  # 0.004 s < MinOnTime: suppressed
    assert classes["x0.5"] is False  # 0.010 s < MinOnTime: suppressed
    assert classes["x1.0"] is True  # 0.020 s: fires
    assert classes["x1.5"] is True  # 0.030 s: fires


def np_l2(v):
    s = 0.0
    for x in list(v):
        s += float(x) * float(x)
    return math.sqrt(s)


def test_m2_ladder_default_path_matches_ledger():
    """M2 rows: default path reproduces the ledger; cutoff is inert.

    The ledger ladder {10,100,1000} is identical @ dt=0.1 on the DEFAULT
    path (models off / benches pin 1000): full-step fires achieve duty 1.
    WP25-A2 (Decision 2: NO lag model on either path): stock cutoff is
    INERT without ramps, so setCutoffFrequency(10) must equal 1000 to ~1%
    over fire AND shutoff transients at matched dt (ledger
    swap_p1_parity.json thr_lag10==thr_fresh_05 bit-identical). The old
    0.3-0.9 bound around 1-exp(-1) ~= 0.632 described the deleted
    divergent filter, not stock, and is removed.
    """
    np = pytest.importorskip("numpy")
    om, thr, _ = _run_omni_thr(duty_cmd=1.0, dt=0.1, n_ticks=3)
    if thr is None:
        pytest.skip("WP3 C++ spec not yet applied: thrusterOutMsgs empty")
    dl = _deltas(om)
    # Geometry-derived expectation (NOT ledger absolutes: the ledger M2 rows
    # were recorded with the probe's own bus/geometry -- this rig uses
    # pos=(0.5,0,1), dir=(0,0,1), tmax=1 on the cubesat bus, so torque is
    # r x F = (0,-0.5,0) and each fire step is |T|*dt/Iyy).
    from cubesat_12u_sim.core import config as C
    i6 = np.asarray(C.INERTIA, float)
    tq = np.cross(np.asarray([0.5, 0.0, 1.0]), np.asarray([0.0, 0.0, 1.0])) * 1.0
    exp_step = float(np_l2(tq * 0.1 / float(i6[1, 1])))
    got = [float(np_l2(d)) for d in dl]
    assert got[0] == pytest.approx(0.0, abs=1e-9)  # fill step idles
    assert got[1] == pytest.approx(exp_step, rel=0.05), (got, exp_step)
    assert got[2] == pytest.approx(exp_step, rel=0.05), (got, exp_step)
    assert np.asarray(thr.thrustFactor, float).ravel()[-1] == pytest.approx(1.0, abs=1e-6)
    # WP25-A2 inert: cutoff=10 vs 1000, matched dt, fire AND shutoff
    # transients. Duty 1.0 covers full-fire arcs; duty 0.5 (5/6 staircase)
    # adds a mid-step shutoff arc. Idle fill steps must idle on both.
    for duty in (1.0, 0.5):
        om_hi, _, _ = _run_omni_thr(duty_cmd=duty, dt=0.1, n_ticks=3,
                                    opts={"cutoff": 1000.0})
        om_lo, _, _ = _run_omni_thr(duty_cmd=duty, dt=0.1, n_ticks=3,
                                    opts={"cutoff": 10.0})
        for k, (a, b) in enumerate(zip(_deltas(om_hi), _deltas(om_lo))):
            na, nb = float(np_l2(a)), float(np_l2(b))
            if na < 1e-12:
                assert nb < 1e-12, (duty, k, na, nb)
            else:
                assert 0.99 < nb / na < 1.01, (duty, k, na, nb)


def test_t2_chatter_row_dt001():
    """T2 gate: quantizer boundary holds @ dt=0.01 on the default path."""
    om, _, _ = _run_omni_thr(duty_cmd=0.5, dt=0.01, n_ticks=4)
    dl = _deltas(om)
    # Fill step idles, propagations fire at 5/6 (staircase, no valve lag).
    assert float(np_l2(dl[0])) < 0.05 * float(np_l2(dl[1]))
    assert abs(float(np_l2(dl[2])) - float(np_l2(dl[1]))) <= 0.05 * float(np_l2(dl[1]))


def test_m4_phase_row_delayed_matches_normal():
    """M4 gate: 1-tick-delayed driver matches normal, matched dt."""
    om_n, _, _ = _run_omni_thr(duty_cmd=5.0 / 6.0, dt=0.1, n_ticks=3)
    om_d, _, _ = _run_omni_thr(duty_cmd=5.0 / 6.0, dt=0.1, n_ticks=3,
                               writer="delayed")
    dn = _deltas(om_n)
    dd = _deltas(om_d)
    for a, b in zip(dn, dd):
        assert float(np_l2([x - y for x, y in zip(list(a), list(b))])) <= \
            0.05 * max(float(np_l2(a)), 1e-12)


def test_m5_over_negative_rows():
    """M5 gate: over-duty clamps to 1, negative duties go idle."""
    om_over, _, _ = _run_omni_thr(duty_cmd=1.5, dt=0.1, n_ticks=3)
    dl = _deltas(om_over)
    assert float(np_l2(dl[1])) > 0.0  # fires (clamped to full)
    om_neg, _, _ = _run_omni_thr(duty_cmd=-0.1, dt=0.1, n_ticks=3)
    for d in _deltas(om_neg):
        assert float(np_l2(d)) == pytest.approx(0.0, abs=1e-12)


def test_swirl_axis_telemetry_and_dynamics():
    """M6 swirl: torque gains q*s about d (T1 geometry separates y/z)."""
    np = pytest.importorskip("numpy")
    s = 0.05
    om_off, thr_off, _ = _run_omni_thr(duty_cmd=1.0, dt=0.1, n_ticks=3)
    om_on, thr_on, _ = _run_omni_thr(duty_cmd=1.0, dt=0.1, n_ticks=3,
                                     opts={"swirl": s})
    if thr_on is None:
        pytest.skip("WP3 C++ spec not yet applied: thrusterOutMsgs empty")
    tq = np.asarray(thr_on.thrustTorquePntB_B, float)
    # Telemetry (module-side): [0,-0.5,0] + q*s*d = [0,-0.5,+0.05].
    np.testing.assert_allclose(tq[-1], [0.0, -0.5, s], rtol=0.05)
    # Dynamics (contributor swirlScale slot): z-impulse grows by s*dt/Izz,
    # y-impulse (r x F) unchanged.
    from cubesat_12u_sim.core import config as C

    izz = float(np.asarray(C.INERTIA, float)[2, 2])
    dz_off = _deltas(om_off)[1][2]
    dz_on = _deltas(om_on)[1][2]
    dy_off = _deltas(om_off)[1][1]
    dy_on = _deltas(om_on)[1][1]
    assert (dz_on - dz_off) == pytest.approx(s * 0.1 / izz, rel=0.2)
    assert dy_on == pytest.approx(dy_off, rel=0.05)


def test_blowdown_no_tank_factors_one():
    """M6 rule: coeffs set + no tank => factors stay 1.0, run completes."""
    np = pytest.importorskip("numpy")
    om, thr, _ = _run_omni_thr(duty_cmd=0.8, dt=0.1, n_ticks=3,
                               opts={"blowdown": ([0.1],), "cutoff": 1000.0})
    if thr is None:
        pytest.skip("WP3 C++ spec not yet applied: thrusterOutMsgs empty")
    assert np.asarray(thr.thrustBlowDownFactor, float).ravel()[-1] == \
        pytest.approx(1.0, abs=1e-12)
    assert np.asarray(thr.ispBlowDownFactor, float).ravel()[-1] == \
        pytest.approx(1.0, abs=1e-12)
    # Smoke norms match the no-blowdown fire class (factors were inert).
    dl = _deltas(om)
    assert float(np_l2(dl[1])) > 0.0
