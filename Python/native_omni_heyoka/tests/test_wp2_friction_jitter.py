"""WP2 / WP25-B3 — Stribeck friction (ported stock-exact) + JitterSimple-distributional + voltage reuse.

Binding design (PLAN_ENVELOPE.md + PLAN_PHASE25.md WP25-B3; Decision 5 locked: PORT path):

* Vec owns stock-exact Stribeck (vec law + ``rw3_fric`` 1e-14 gate, untouched).
  SysModel PORTS the flag/latch/``omegaBefore``/limit-cycle law module-side per
  tick (``src/omni_spacecraft.cpp`` UpdateState block + per-wheel
  ``Impl::fricState_`` latch state, fresh on Reset; reference
  ``src/omni_vec_integrator.cpp:828-876``, read-only). One GLOBAL param set is
  shared by every wheel (vec keeps per-wheel recs; single-valued configs
  coincide exactly); ``betaStatic <= 0`` keeps the stiction band off.
* Off-state: factory-off zeroes ONLY ``fCoulomb`` (stray ``cViscous``/``fStatic``
  persist). Our off is NOT the factory off: our off is ``tauF`` identically 0
  plus a zeroed slot (documented deviation, asserted in
  :func:`test_off_state_is_identically_zero`).
* ``JitterSimple`` is additive noise on the SAME second slot from the module's
  own RNG (``std::mt19937_64`` + ``normal_distribution``; per-wheel stream
  split by index hashing; seed via ``setJitterSeed``). TRUE 2x400-tick
  determinism (probe rev 2, ``min(n,40)`` cap removed); the recorded rev-1
  ledger ran 2x40 -- labeled, and the horizon gate below is ``plan_rev``-aware.
  DISTRIBUTIONAL parity only — never a bit-exact claim (the local
  :func:`_jitter_stream` is a distributional analogue, not a mirror of the C++
  bit stream). ``JitterFullyCoupled`` (theta states) is out of torque-slot
  scope: SysModel rejects SETTER-GATED ONLY (no auto-read of a stock wheel's
  ``RWModel`` string; vec-side auto-reject lives in the swap mirror).
  ``wheelThetas`` are zeros on both omni paths (no theta states registered).
* Voltage: NO omni change. Reuse wiring is stock ``MotorVoltageInterface``
  with ``motorTorqueOutMsg -> rwMotorCmdInMsg``; ``voltageOn_`` is
  informational only.
* F-ledger: F1–F6 ``ok:true`` means "sim ran", NEVER "law matched";
  quantitative Stribeck-law evidence is SUPERSEDED to the vec ``rw3_fric``
  cell (see :func:`test_ledger_friction_law_evidence_superseded`).
* Min-pulse (M8): suppress-only + dead-memory label (``carryover`` /
  ``lastFireStep`` / ``fireCounter`` write-only; static gate below).
* ``u_min``/``P_max`` (Decision 6): B3 does NOT model them -- the B1
  bucket-(a) fatal raises (``setUMin``/``setPMax`` arming, checked at the
  Reset + UpdateState heads) stay in place (reject path); telemetry echoes the
  disarmed defaults. Guard behavior is owned by ``tests/test_wp0_guards.py``,
  not this file.

Graph purity (A3): no ``tanh``/``min``/``max`` inside
``updateContributions``/``state_derivatives`` — friction is module-side on
doubles (tanh/exp live in UpdateState bodies on both paths). Layout (A2): new slots via ``input_names``/``par_names`` only, never
hard-coded indices (this file uses names throughout).

Host verify (no execution here — Linux, no python3/Basilisk in this container):
``rebuild && pytest native_omni_heyoka/tests -q`` (this file) plus
``bench_rw_omni.py`` friction rows (F1 stiction band, F2 equilibrium,
F3 spin-down, F4 hub-reaction sign, off-by-default bit-exact vs the current
sweep) once the C++ edit-specs below are applied on the macOS host.
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
VEC_CPP = REPO / "src" / "omni_vec_integrator.cpp"  # READ-ONLY reference; never edited by B3

# Slot-name decision (locked): second RW input slot is ``tauF{i}`` (1-based).
TAUF_PREFIX = "tauF"


def tauF_name(i_1based: int) -> str:
    """Name of wheel i's friction-torque input slot (1-based)."""
    return f"{TAUF_PREFIX}{i_1based}"


# ---------------------------------------------------------------------------
# Reference model A: Stribeck iteration-1 (module-side, doubles only).
# HISTORICAL as of WP25-B3 (superseded on the SysModel path by the ported
# stock-exact law below; kept to document the iter-1 truncation explicitly).
# tauF = -fC * tanh(W / k) - cV * W,  with k from omegaLimitCycle.
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
    """Per-wheel stream split by index hashing (DISTRIBUTIONAL analogue only).

    NOT a bit-exact mirror of the C++ spec: C++ seeds
    ``std::mt19937_64(seed + GOLDEN + (i+1))`` then splitmix64s, while this
    harness splitmix64s ``(index+1)`` and draws from ``random.Random`` with a
    different PRNG -- bit-level equality is neither asserted nor expected
    (and omni's own ``normal_distribution`` inherits the libc++ portability
    caveat cited against stock). Envelope/seed-split properties below are the
    whole claim.
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
# Part 1b — WP25-B3 ported stock-exact law (SysModel shipped law).
# Pure-Python transliteration of src/omni_spacecraft.cpp UpdateState friction
# block (itself a port of src/omni_vec_integrator.cpp:828-876, read-only).
# Property tests only (oddness where the latch holds, band rescale, latch
# drop, beta<=0 band-off); cross-path numeric parity is owned by the vec
# rw3_fric cell, not duplicated here.
# ---------------------------------------------------------------------------

class _PortState:
    """Per-wheel latch state mirror (Impl::FricState). Fresh on Reset."""

    def __init__(self):
        self.stribeck = 0
        self.omegaBefore = 0.0


def stribeck_stock_port(W: float, fCoulomb: float, fStatic: float,
                        betaStatic: float, cViscous: float,
                        omegaLimitCycle: float, state: _PortState) -> float:
    """Ported stock-exact Stribeck torque ON THE WHEEL (opposes motion).

    All five knobs are LIVE (unlike iter-1 above). Mutates ``state`` exactly
    like the C++ port (stribeck flag + omegaBefore update).
    """
    oL = omegaLimitCycle if omegaLimitCycle > 0.0 else 1.0
    if abs(W) < 0.10 * oL and betaStatic > 0.0:
        state.stribeck = 1
    signO = (1.0 if W > 0.0 else 0.0) - (1.0 if W < 0.0 else 0.0)
    omegaDot = W - state.omegaBefore
    signOD = (1.0 if omegaDot > 0.0 else 0.0) - (1.0 if omegaDot < 0.0 else 0.0)
    if state.stribeck == 1 and abs(signO - signOD) < 2.0 and betaStatic > 0.0:
        state.stribeck = 1
    else:
        state.stribeck = 0
    if state.stribeck == 1:
        e = math.exp(-(W / betaStatic) ** 2 / 2.0)
        kk = math.sqrt(2.0 * math.exp(1.0))
        frictionForce = (kk * (fStatic - fCoulomb) * e * W / (betaStatic * math.sqrt(2.0))
                         + fCoulomb * math.tanh(W * 10.0 / betaStatic)
                         + cViscous * W)
        eL = math.exp(-(oL / betaStatic) ** 2 / 2.0)
        frictionForceAtLimitCycle = (
            kk * (fStatic - fCoulomb) * eL * oL / (betaStatic * math.sqrt(2.0))
            + fCoulomb * math.tanh(oL * 10.0 / betaStatic)
            + cViscous * oL)
    else:
        frictionForce = signO * fCoulomb + cViscous * W
        frictionForceAtLimitCycle = fCoulomb + cViscous * oL
    if abs(W) < oL and oL > 0.0:
        frictionForce = frictionForceAtLimitCycle / oL * W
    state.omegaBefore = W
    return -frictionForce


_PORT_DEFAULTS = dict(fCoulomb=0.005, fStatic=0.008, betaStatic=2.0,
                      cViscous=1e-5, omegaLimitCycle=1.0)


def test_port_zero_at_rest():
    assert stribeck_stock_port(0.0, **_PORT_DEFAULTS, state=_PortState()) == pytest.approx(0.0, abs=1e-18)


def test_port_opposes_motion_outside_band():
    assert stribeck_stock_port(50.0, **_PORT_DEFAULTS, state=_PortState()) == pytest.approx(
        -(0.005 + 1e-5 * 50.0), rel=1e-12)
    assert stribeck_stock_port(-50.0, **_PORT_DEFAULTS, state=_PortState()) == pytest.approx(
        0.005 + 1e-5 * 50.0, rel=1e-12)


def test_port_beta_nonpositive_disables_band():
    # Vec OFF convention (betaStatic = -1.0): no stribeck branch anywhere;
    # sub-limit-cycle torque is the linear rescale of the Coulomb branch.
    kw = dict(_PORT_DEFAULTS, betaStatic=-1.0)
    s = _PortState()
    tau = stribeck_stock_port(0.05, **kw, state=s)
    assert s.stribeck == 0
    assert tau == pytest.approx(-(0.005 + 1e-5) * 0.05, rel=1e-12)


def test_port_limit_cycle_rescale_linear():
    # Two rising in-band steps: latch holds, both torques are the same
    # F_LC/oL slope through the origin (ratio == speed ratio exactly).
    s = _PortState()
    t1 = stribeck_stock_port(0.03, **_PORT_DEFAULTS, state=s)
    assert s.stribeck == 1
    t2 = stribeck_stock_port(0.05, **_PORT_DEFAULTS, state=s)
    assert s.stribeck == 1
    assert t2 / t1 == pytest.approx(0.05 / 0.03, rel=1e-12)


def test_port_latch_drops_on_deceleration():
    # Rising step engages the band; a same-sign slowdown flips sign(W -
    # omegaBefore) so |signO - signOD| == 2 and the latch drops.
    s = _PortState()
    stribeck_stock_port(0.05, **_PORT_DEFAULTS, state=s)
    assert s.stribeck == 1
    tau = stribeck_stock_port(0.02, **_PORT_DEFAULTS, state=s)
    assert s.stribeck == 0
    assert tau == pytest.approx(-(0.005 + 1e-5) * 0.02, rel=1e-9)


# ---------------------------------------------------------------------------
# Part 2 — measured ledger (J1 deterministic at labeled horizon; F1–F6
# sim-ran + rw3_fric supersede pointer; V1–V4 ok)
# ---------------------------------------------------------------------------

def _ledger():
    assert LEDGER.exists(), f"missing ledger {LEDGER}"
    return json.loads(LEDGER.read_text())


def test_ledger_j1_deterministic():
    j1 = _ledger()["payload"]["jitter"]["probes"]["J1"]
    assert j1["ok"] is True
    assert j1["deterministic"] is True
    assert j1["max_abs_delta_u_cmd"] == 0.0
    assert j1["max_abs_delta_wheelSpeeds"] == 0.0  # fixed RNGSeed; horizon labeled below


def test_ledger_j1_horizon_matches_plan_rev():
    # WP25-B3: J1 determinism must hold at the CLAIMED horizon. Probe rev 1
    # recorded 2x40 (min(n,40) cap); probe rev >= 2 runs TRUE 2x400.
    doc = _ledger()
    rev = doc.get("payload", {}).get("plan_rev", 1)
    j1 = doc["payload"]["jitter"]["probes"]["J1"]
    n1 = j1["run1"]["n_ticks"]
    n2 = j1["run2"]["n_ticks"]
    assert n1 == n2
    if rev >= 2:
        assert (n1, n2) == (400, 400)
        assert "TRUE 2x400" in " ".join(j1.get("notes", []))
    else:
        assert (n1, n2) == (40, 40)  # pre-B3 cap, honestly recorded


def test_ledger_friction_probes_ok():
    # "Sim ran" only -- NOT "law matched" (see supersede test below).
    probes = _ledger()["payload"]["friction"]["probes"]
    for pid in ("F1", "F2", "F3", "F4", "F5", "F6"):
        assert probes[pid]["ok"] is True, f"{pid} not ok"


def test_ledger_friction_law_evidence_superseded():
    # WP25-B3 F-ledger (M13/F-ledger): no unamended "law matched" reading.
    # Quantitative Stribeck-law evidence lives ONLY in the vec rw3_fric cell;
    # the probe records that pointer from rev 2 on. Pre-B3 ledgers (rev 1,
    # all-null F1-F6 readbacks) are grandfathered ONLY with this skip, which
    # states the sim-ran-only scope -- the skip message itself is the label.
    fric = _ledger()["payload"]["friction"]
    if "law_evidence" not in fric:
        pytest.skip("probe rev<2 (recorded pre-WP25-B3): regenerate on host; "
                    "F1-F6 read as sim-ran ONLY, law evidence pending rw3_fric pointer")
    assert "rw3_fric" in fric["law_evidence"]
    assert "SUPERSEDED" in fric["law_evidence"]


def test_ledger_friction_baseline_param_set():
    base = _ledger()["payload"]["friction"]["baseline"]
    # Live param address per task brief: RWConfigPayload carries
    # fCoulomb/fStatic/betaStatic/cViscous/omegaLimitCycle. The ledger baseline
    # records the subset below (fStatic readback null in the F-section rows,
    # set-path under review — does not block the ported law, which takes all
    # five knobs live from setFriction).
    for key in ("fCoulomb", "fStatic", "cViscous", "omegaLimitCycle"):
        assert key in base, f"baseline missing {key}"
    assert base["useRWfriction"] is True


def test_ledger_f4_hub_reaction_sign():
    f4 = _ledger()["payload"]["friction"]["probes"]["F4"]
    assert "opposite sign" in f4["reaction_sign"]  # react-on-hub, F4 hook
    # frictionHubSign_ default +1 encodes exactly this (see C++ spec).
    # WP25-B3 scope: QUALITATIVE ONLY -- either the rev-1 "to be confirmed"
    # wording or the rev-2 QUALITATIVE label must be present; a bare
    # quantitative reading is banned (law evidence is rw3_fric, above).
    scoped = ("to be confirmed" in f4.get("reaction_sign", "")) or any(
        "QUALITATIVE" in (n or "") for n in f4.get("notes", []))
    assert scoped


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
    # J1 horizon is 2x400 (probe rev >= 2; rev-1 ledger recorded 2x40, see
    # test_ledger_j1_horizon_matches_plan_rev); check the analogue envelope
    # at the full claimed horizon either way.
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


def test_cpp_contract_sysmodel_stribeck_port():
    # WP25-B3 Decision 5 PORT path: the SysModel per-tick block carries the
    # stock-exact law tokens (flag/latch/omegaBefore/limit-cycle state); the
    # iter-1 one-liner is gone and fStatic/betaStatic are live, not inert.
    src = _need(SPACECRAFT_CPP, "fricState_")
    for tok in ("omegaBefore", "stribeck", "frictionForceAtLimitCycle", "0.10"):
        assert tok in src, f"port token missing: {tok}"
    assert "tanh(Wi / k)" not in src  # iter-1 line removed by the port


def test_cpp_contract_no_thetas_written():
    # wheelThetas-zeros label gate (M13): neither omni path registers theta
    # states nor writes wheelThetas (JitterFullyCoupled rejected) -- zeros by
    # construction. Any future thetas write fails this and must update the label.
    # NOTE: VEC_CPP is a READ-ONLY reference here, never edited by B3.
    for path in (SPACECRAFT_CPP, VEC_CPP):
        assert "wheelThetas" not in path.read_text(), f"thetas write in {path.name}"


def test_cpp_contract_fully_coupled_setter_gated_only():
    # WP25-B3 documented limitation: SysModel FullyCoupled rejection is
    # setter-gated (setRWModelJitterCoupled always fatal + bucket-(a) guard);
    # the path never auto-reads a stock wheel's RWModel string, so no
    # stock-side misconfiguration is auto-detected here (vec-side IS, via the
    # swap mirror). Static form of that claim: RWModel appears only as the
    # telemetry echo assignment -- no ==/!= string-compare auto-detect.
    import re
    src = SPACECRAFT_CPP.read_text()
    code = re.sub(r"//.*", "", src)
    code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
    assert "FullyCoupled" in code  # setter + bucket-(a) guard present
    assert "bskError" in code
    for line in code.splitlines():
        if "RWModel" in line:
            assert "==" not in line and "!=" not in line, \
                f"RWModel auto-detect? {line.strip()}"


def test_cpp_contract_min_pulse_dead_memory():
    # M8 label gate (suppress-only + dead memory): carryover/lastFireStep/
    # fireCounter are WRITE-ONLY in src/omni_spacecraft.cpp (decls + st8
    # writes); the gate never feeds them back into openT/thrStaged. Any
    # future read fails this and must update the M8 label first.
    import re
    src = SPACECRAFT_CPP.read_text()
    code = re.sub(r"//.*", "", src)
    code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
    ok_pat = re.compile(r"(struct MinPulseState|int64_t lastFireStep|"
                        r"double carryover|long fireCounter|"
                        r"st8\.(carryover|lastFireStep|fireCounter)\s*(\+=|=|\+\+))")
    for lineno, line in enumerate(code.splitlines(), 1):
        if re.search(r"\b(carryover|lastFireStep|fireCounter)\b", line):
            assert ok_pat.search(line), \
                f"M8 dead-memory label violated at line {lineno}: {line.strip()}"


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
