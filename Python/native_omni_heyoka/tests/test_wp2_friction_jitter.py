"""WP2 — Stribeck friction (iteration-1) + JitterSimple-distributional + voltage reuse.

Binding design (PLAN_ENVELOPE.md Rev 1, WP2):

* Friction torque is computed MODULE-SIDE each tick on doubles from the wheel
  speed ``W_i`` (iteration-1 law, see :func:`stribeck_iter1`) and injected via
  a SECOND input slot per wheel. Slot-name decision (locked here): the second
  slot is named ``tauF{i}`` (1-based, e.g. ``tauF1..tauFN``), declared via
  ``ReactionWheelContributorT::input_names()`` next to ``u{i}``. The
  contributor reads the slot verbatim (like ``u``) and adds it to ``vecRot``
  AND the spin equation with the same sign convention as ``u``; the hub
  reaction sign follows stock (F4: react-on-hub) with a ``frictionHubSign_``
  hook defaulting to +1.
* ``JitterSimple`` is additive noise on the SAME second slot from the module's
  own RNG (``std::mt19937_64`` + ``normal_distribution``; per-wheel stream
  split by index hashing; seed via ``setJitterSeed``). DISTRIBUTIONAL parity
  only — never a bit-exact claim. ``JitterFullyCoupled`` (theta states) is out
  of torque-slot scope and must ``bskError``.
* Voltage: NO omni change. Reuse wiring is stock ``MotorVoltageInterface``
  with ``motorTorqueOutMsg -> rwMotorCmdInMsg``; ``voltageOn_`` is
  informational only.
* Off-state: factory-off zeroes ONLY ``fCoulomb`` (stray ``cViscous``/``fStatic``
  persist). Our off is NOT the factory off: our off is ``tauF`` identically 0
  plus a zeroed slot (documented deviation, asserted in
  :func:`test_off_state_is_identically_zero`).

Graph purity (A3): no ``tanh``/``min``/``max`` inside
``updateContributions``/``state_derivatives`` — friction is module-side on
doubles. Layout (A2): new slots via ``input_names``/``par_names`` only, never
hard-coded indices (this file uses names throughout).

Host verify (no execution here — Linux, Basilisk is macOS-only):
``rebuild && pytest native_omni_heyoka/tests -q`` (this file) plus
``bench_rw_omni.py`` friction rows (F1 stiction band, F2 equilibrium,
F3 spin-down, F4 hub-reaction sign, off-by-default bit-exact vs the current
sweep) once the C++ edit-specs below are applied on the macOS host.

C++ edit-specs are NOT applied here (parallel-agent ownership); the
``test_cpp_contract_*`` tests below SKIP until the WP2 C++ spec lands so the
current sweep stays green, and assert the contract once present.
"""

from __future__ import annotations

import json
import math
import random
import statistics
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "wp2_friction_jitter_voltage.json"
CONTRIBUTORS = REPO / "include" / "omni" / "contributors.hpp"
SYSTEM = REPO / "include" / "omni" / "system.hpp"
SPACECRAFT_H = REPO / "include" / "omni" / "spacecraft.hpp"
SPACECRAFT_CPP = REPO / "src" / "omni_spacecraft.cpp"

# Slot-name decision (locked): second RW input slot is ``tauF{i}`` (1-based).
TAUF_PREFIX = "tauF"


def tauF_name(i_1based: int) -> str:
    """Name of wheel i's friction-torque input slot (1-based)."""
    return f"{TAUF_PREFIX}{i_1based}"


# ---------------------------------------------------------------------------
# Reference model: Stribeck iteration-1 (module-side, doubles only).
# tauF = -fC * tanh(W / k) - cV * W,  with k from omegaLimitCycle.
# Iteration-2 hook (budgeted, not implemented): betaStatic/fStatic shaping of
# the velocity-weakening branch + u_min interplay; accepted here as explicit
# keyword args so the call signature is forward-compatible and the iter-1
# truncation is visible, not silent.
# ---------------------------------------------------------------------------

def stribeck_iter1(W: float, fCoulomb: float, cViscous: float,
                   omegaLimitCycle: float, *,
                   fStatic: float = 0.0, betaStatic: float = 0.0,
                   u_min: float = 0.0) -> float:
    """Iteration-1 friction torque ON THE WHEEL (opposes motion).

    ``fStatic``/``betaStatic``/``u_min`` are iteration-2 hooks: accepted and
    documented, intentionally unused in iteration-1.
    """
    del fStatic, betaStatic, u_min  # iteration-2 hooks (see docstring)
    k = omegaLimitCycle
    assert k > 0.0, "omegaLimitCycle must be positive (width of the tanh zone)"
    assert fCoulomb >= 0.0 and cViscous >= 0.0
    return -fCoulomb * math.tanh(W / k) - cViscous * W


def _jitter_stream(seed: int, wheel_index_0based: int) -> random.Random:
    """Per-wheel stream split by index hashing (mirrors the C++ spec).

    C++ uses ``std::mt19937_64(seed ^ splitmix64(index + 1))``; here the same
    split with ``random.Random`` — distributional mirror only, never bit-exact.
    """
    x = (wheel_index_0based + 1) & 0xFFFFFFFFFFFFFFFF
    x = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
    x = ((x ^ (x >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
    x = (x ^ (x >> 31)) & 0xFFFFFFFFFFFFFFFF
    return random.Random((seed ^ x) & 0xFFFFFFFFFFFFFFFF)


# ---------------------------------------------------------------------------
# Part 0 — slot naming decision
# ---------------------------------------------------------------------------

def test_tauf_slot_names_decided():
    assert tauF_name(1) == "tauF1"
    assert tauF_name(3) == "tauF3"
    assert TAUF_PREFIX == "tauF"  # locked; alternatives tF{i}/fT{i} rejected


# ---------------------------------------------------------------------------
# Part 1 — iteration-1 reference math (always runs; no C++ needed)
# ---------------------------------------------------------------------------

def test_iter1_zero_at_rest():
    assert stribeck_iter1(0.0, 0.005, 1e-5, 1.0) == pytest.approx(0.0, abs=1e-18)


def test_iter1_odd_symmetry():
    for W in (0.01, 0.5, 3.0, 100.0):
        a = stribeck_iter1(W, 0.005, 1e-5, 1.0)
        b = stribeck_iter1(-W, 0.005, 1e-5, 1.0)
        assert a == pytest.approx(-b, rel=1e-12)


def test_iter1_opposes_motion():
    # Friction torque on the wheel always opposes W (both branches).
    for W in (0.001, 0.1, 1.0, 10.0, 500.0):
        assert stribeck_iter1(W, 0.005, 1e-5, 1.0) < 0.0
        assert stribeck_iter1(-W, 0.005, 1e-5, 1.0) > 0.0


def test_iter1_coulomb_plateau():
    # |W| >> k: tanh -> +/-1, tauF -> -/+fC - cV*W (F2 equilibrium branch).
    fC, cV, k = 0.005, 1e-5, 1.0
    assert stribeck_iter1(50.0, fC, cV, k) == pytest.approx(-fC - cV * 50.0, rel=1e-9)
    assert stribeck_iter1(-50.0, fC, cV, k) == pytest.approx(fC + cV * 50.0, rel=1e-9)


def test_iter1_limit_cycle_zone_slope():
    # |W| << k: tanh(W/k) ~= W/k, slope -(fC/k + cV) (F3 spin-down zone).
    fC, cV, k = 0.005, 1e-5, 1.0
    h = 1e-6
    slope = (stribeck_iter1(h, fC, cV, k) - stribeck_iter1(-h, fC, cV, k)) / (2 * h)
    assert slope == pytest.approx(-(fC / k + cV), rel=1e-6)


def test_iter1_k_from_omega_limit_cycle():
    # k IS omegaLimitCycle: wider k flattens the low-speed branch (F3 note:
    # "omegaLimitCycle zone flattens near 0").
    W = 0.5
    narrow = abs(stribeck_iter1(W, 0.005, 0.0, 0.1))
    wide = abs(stribeck_iter1(W, 0.005, 0.0, 5.0))
    assert narrow > wide


def test_iter1_iteration2_hooks_accepted_but_inert():
    a = stribeck_iter1(2.0, 0.005, 1e-5, 1.0)
    b = stribeck_iter1(2.0, 0.005, 1e-5, 1.0,
                       fStatic=0.008, betaStatic=2.0, u_min=0.015)
    assert a == b  # iter-1 ignores the iter-2 shaping knobs (documented)


def test_off_state_is_identically_zero():
    # OUR off (deviation from factory documented in module docstring):
    # tauF identically 0 + zeroed slot, regardless of stray params.
    for W in (0.0, 1.0, -300.0):
        assert stribeck_iter1(W, 0.0, 0.0, 1.0) == 0.0
    # Factory-off (zeroes ONLY fCoulomb) would still drag via stray cViscous;
    # our off must NOT reproduce that stray drag.
    assert stribeck_iter1(100.0, 0.0, 1e-5, 1.0) != 0.0  # factory remnant
    # ...hence our off zeroes the slot, not just fCoulomb (spec: slot write 0.0
    # when frictionOn_ is false).


# ---------------------------------------------------------------------------
# Part 2 — measured ledger (J1 deterministic; F1–F6 ok; V1–V4 ok)
# ---------------------------------------------------------------------------

def _ledger():
    assert LEDGER.exists(), f"missing ledger {LEDGER}"
    return json.loads(LEDGER.read_text())


def test_ledger_j1_deterministic():
    j1 = _ledger()["payload"]["jitter"]["probes"]["J1"]
    assert j1["ok"] is True
    assert j1["deterministic"] is True
    assert j1["max_abs_delta_u_cmd"] == 0.0
    assert j1["max_abs_delta_wheelSpeeds"] == 0.0  # 2x400-tick, fixed RNGSeed


def test_ledger_friction_probes_ok():
    probes = _ledger()["payload"]["friction"]["probes"]
    for pid in ("F1", "F2", "F3", "F4", "F5", "F6"):
        assert probes[pid]["ok"] is True, f"{pid} not ok"


def test_ledger_friction_baseline_param_set():
    base = _ledger()["payload"]["friction"]["baseline"]
    # Live param address per task brief: RWConfigPayload carries
    # fCoulomb/fStatic/betaStatic/cViscous/omegaLimitCycle. The ledger baseline
    # records the subset below (betaStatic readback null in F-section, set-path
    # under review — does not block iteration-1).
    for key in ("fCoulomb", "fStatic", "cViscous", "omegaLimitCycle"):
        assert key in base, f"baseline missing {key}"
    assert base["useRWfriction"] is True


def test_ledger_f4_hub_reaction_sign():
    f4 = _ledger()["payload"]["friction"]["probes"]["F4"]
    assert "opposite sign" in f4["reaction_sign"]  # react-on-hub, F4 hook
    # frictionHubSign_ default +1 encodes exactly this (see C++ spec).


def test_ledger_voltage_probes_ok():
    probes = _ledger()["payload"]["voltage"]["probes"]
    for pid in ("V1", "V2", "V3", "V4"):
        assert probes[pid]["ok"] is True, f"{pid} not ok"


def test_ledger_voltage_reuse_address():
    v4 = _ledger()["payload"]["voltage"]["probes"]["V4"]
    assert "voltage" in v4["fields"]["ArrayMotorVoltageMsgPayload"]
    # Reuse wiring (no omni change): stock MotorVoltageInterface
    # motorTorqueOutMsg -> rwMotorCmdInMsg (V2 static note in ledger).


# ---------------------------------------------------------------------------
# Part 3 — JitterSimple distributional mirror (never bit-exact)
# ---------------------------------------------------------------------------

def test_jitter_streams_split_per_wheel():
    r0 = _jitter_stream(20260511, 0)
    r1 = _jitter_stream(20260511, 1)
    assert r0.random() != r1.random()  # distinct streams per wheel


def test_jitter_seed_reproducible_distributionally():
    # Same seed+index replays: draw 3 from ONE stream, twice.
    a = list(_jitter_draws(20260511, 0, 1.0, 3))
    b = list(_jitter_draws(20260511, 0, 1.0, 3))
    assert a == b  # distributional harness, not stock
    # ...while a different wheel index splits the stream.
    c = list(_jitter_draws(20260511, 1, 1.0, 3))
    assert c != a


def _jitter_draws(seed, index, sigma, n):
    rng = _jitter_stream(seed, index)
    return [rng.gauss(0.0, sigma) for _ in range(n)]


def test_jitter_envelope_mean_var():
    # J1 used 2x400 ticks; check the mirror's envelope at that horizon.
    sigma = 1e-4
    draws = _jitter_draws(20260511, 0, sigma, 800)
    assert abs(statistics.mean(draws)) < 5 * sigma / math.sqrt(len(draws))
    assert statistics.pvariance(draws) == pytest.approx(sigma ** 2, rel=0.25)


def test_jitter_zero_sigma_is_silent():
    assert _jitter_stream(7, 2).gauss(0.0, 0.0) == 0.0


# ---------------------------------------------------------------------------
# Part 4 — C++ contract (SKIP until the WP2 edit-specs land; assert after)
# ---------------------------------------------------------------------------

def _need(path: Path, needle: str):
    """Return file text if it contains needle, else skip (spec not applied)."""
    assert path.exists(), f"missing {path}"
    src = path.read_text()
    if needle not in src:
        pytest.skip(f"WP2 C++ spec not yet applied: {needle!r} absent in {path.name}")
    return src


def test_cpp_contract_rw_second_slot():
    src = _need(CONTRIBUTORS, '"tauF"')
    assert "n_inputs() const override { return 2; }" in src
    assert "tauF" in src


def test_cpp_contract_no_tanh_in_graph():
    # A3: friction tanh lives module-side on doubles; the contributor graph
    # paths stay + - * / sqrt. Passes before AND after the spec.
    # NOTE: comments are stripped first ("NO tanh in the graph" mentions tanh).
    import re
    src = CONTRIBUTORS.read_text()
    for cls_anchor in ("class ReactionWheelContributorT",):
        assert cls_anchor in src
    # The RW contributor must never gain tanh/min/max/clamp in its graph code.
    rw_region = src.split("class ReactionWheelContributorT")[1].split(
        "class CentralGravityContributorT")[0]
    code = re.sub(r"//.*", "", rw_region)
    code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
    for bad in ("tanh", "std::min", "std::max", "fmin", "fmax", "clamp"):
        assert bad not in code, f"A3 violation in RW contributor: {bad}"


def test_cpp_contract_friction_offsets_helper():
    src = _need(SYSTEM, "friction_input_offsets")
    assert "wheel_input_offsets" in src  # mirrors the wheel helper


def test_cpp_contract_spacecraft_flags():
    # WP0 flag names EXACTLY (spec binding).
    src = _need(SPACECRAFT_H, "frictionOn_")
    assert "jitterOn_" in src
    assert "voltageOn_" in src
    assert "setJitterSeed" in src
    assert "frictionHubSign_" in src


def test_cpp_contract_module_friction_block():
    src = _need(SPACECRAFT_CPP, "fricIn")
    assert "wheelIn" in src  # applies ON TOP of WP1's RW input block


def test_cpp_contract_fully_coupled_rejects():
    # JitterFullyCoupled (theta states) is out of torque-slot scope: loud reject.
    for path in (SPACECRAFT_H, SPACECRAFT_CPP):
        text = path.read_text()
        if "FullyCoupled" in text or "JitterFullyCoupled" in text:
            assert "bskError" in text
            return
    pytest.skip("WP2 C++ spec not yet applied: no FullyCoupled guard yet")


def test_cpp_contract_no_voltage_port():
    # Voltage reuse = NO omni change: no voltage *input slot* may appear in the
    # symbolic contributors or layout. NOTE: spacecraft.hpp legitimately hosts
    # setVoltageEnabled/voltageOn_ (informational reuse flag) and comments, so
    # it is excluded — this test guards the graph/layout surface only.
    import re
    for path in (CONTRIBUTORS, SYSTEM):
        src = path.read_text()
        code = re.sub(r"//.*", "", src)
        code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
        assert "volt" not in code.lower(), f"voltage port leaked into {path.name}"
    # And no volt-named input/par slot anywhere in contributor declarations.
    names = re.findall(r'"([^"]*[Vv][Oo][Ll][Tt][^"]*)"', CONTRIBUTORS.read_text())
    assert names == [], f"volt-named slots: {names}"
