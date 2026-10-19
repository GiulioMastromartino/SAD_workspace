"""WP25-B1 guards: amend-then-enforce (behavioral raise tests).

Covers PLAN_ENVELOPE.md:472-487 bucket (a) (fatal-today) + Ground rule 7,
Decision 3 locked. Every bucket-(a) item raises ``bskError`` at the Reset +
UpdateState heads (``OmniSpacecraft::checkUnsupported()``) before first
propagation; disarmed (default) configs still run.

HYGIENE ONLY (not guard evidence — CI/probe-skip/keyword-in-text is never
cited as proof): ``test_ci_envelope_checks_pass_on_current_tree``,
``test_layout_name_api_present``, ``test_plan_trap_list_documented``.
``audit_bundle/results/wp0_guards.json`` silent-trap ``ok:true``-by-skip
rows are likewise hygiene, not evidence; the tests below are the evidence.

Host-only sims (Basilisk + built ``omni_heyoka``) via importorskip: skipped
(no failure) on containers without them, e.g. Linux CI without python3.
"""

from __future__ import annotations

import importlib.util
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
CI_SCRIPT = REPO / "scripts" / "ci_envelope_checks.sh"
LAYOUT_PY = REPO / "omniheyoka" / "layout.py"
PLAN = REPO / "PLAN_ENVELOPE.md"

MU = 3.986004418e14
MASS = 750.0
IXX, IYY, IZZ = 900.0, 800.0, 600.0
DT = 0.1

# --- hygiene keyword list (presence only; NOT guard proof) -------------------
TRAP_KEYWORDS = [
    "useRWfriction",      # Stribeck on
    "jitter",             # Jitter on (numRWJitter / U_s,U_d / theta states)
    "voltage",            # Voltage wired
    "u_min",              # u_min / P_max non-default
    "P_max",
    "useMinPulseTime",    # Min-pulse on
    "ThrusterOnRamp",     # Ramps set
    "cutoffFrequency",    # Default cutoff
    "blowdown",           # Blowdown / fuel / swirl / force-msg
    "swirl",
    "force-msg",
    "USE_J2",             # Harmonics / J2 on
    "r_BcB_B",            # Offset nonzero
    "bskError",           # Required behavior until modeled
    "Loud rejection",     # Ground rule 7
    "JIT budget",         # Ground rule 8
    "MC dispersion",      # Ground rule 9
]

B1_SETTERS = (
    "setUMin", "setPMax", "setMaxWheelAcceleration",
    "setLargeTorqueThreshold", "setMtbGtMatrix", "setSRPFacetRotHat",
    "setPrescribedMotionEnabled", "setFlexEnabled", "setSloshEnabled",
    "addFuelTank",
)


def _load_layout():
    """Load omniheyoka/layout.py by path (numpy only, no Basilisk/heyoka)."""
    pytest.importorskip("numpy")
    assert LAYOUT_PY.exists(), f"missing {LAYOUT_PY}"
    spec = importlib.util.spec_from_file_location(
        "omniheyoka_layout_wp0", str(LAYOUT_PY))
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_ci_envelope_checks_pass_on_current_tree():
    """HYGIENE ONLY: CI greps green (layout/purity gates, not guard proof)."""
    assert CI_SCRIPT.exists(), f"missing {CI_SCRIPT}"
    if shutil.which("bash") is None:
        pytest.skip("bash missing -- cannot run ci_envelope_checks.sh here")
    proc = subprocess.run(
        ["bash", str(CI_SCRIPT)], cwd=str(REPO),
        capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, (
        f"ci_envelope_checks.sh failed (rc={proc.returncode}):\n"
        f"--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}")


def test_layout_name_api_present():
    """HYGIENE ONLY: name-based layout API present (not guard proof)."""
    layout = _load_layout()
    for name in ("par_names", "par_index", "sigma_slice"):
        assert callable(getattr(layout, name, None)), (
            f"omniheyoka/layout.py lacks required name-based API {name!r}")
    # Functional smoke: resolution by name, never by hard-coded count.
    names = ["u1", "u2", "u3", "mass", "Ixx"]
    assert layout.Layout(names).index("Ixx") == 4
    assert layout.par_index("Ixx", names=names) == 4
    hub = ["rx", "ry", "rz", "vx", "vy", "vz",
           "s1", "s2", "s3", "omx", "omy", "omz", "W1"]
    assert layout.sigma_slice(names=hub) == slice(6, 9)


def test_plan_trap_list_documented():
    """HYGIENE ONLY: trap keywords present in PLAN text (not guard proof)."""
    assert PLAN.exists(), f"missing {PLAN}"
    text = PLAN.read_text(encoding="utf-8")
    missing = [k for k in TRAP_KEYWORDS if k not in text]
    assert not missing, f"PLAN_ENVELOPE.md omits trap keywords: {missing}"


# --- behavioral guard helpers (host-only) -----------------------------------

def _need_b1_api(sc, *names):
    missing = [n for n in names if not callable(getattr(sc, n, None))]
    if missing:
        pytest.skip(f"WP25-B1 C++ spec not yet applied: {missing} missing")


def _fresh_hub_wheel():
    """Minimal hub + 1-wheel omni sim. Returns (sim, sc, macros)."""
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))

    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)

    sc = oh.OmniSpacecraft()
    sc.setMass(MASS)
    sc.setInertia(IXX, IYY, IZZ)
    sc.setMu(MU)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    sc.addWheel(1.0, 0.0, 0.0, 0.07957747154594767, 0.2)
    sc.ModelTag = "wp25b1_guard"
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sc.setWheelSpeedsInitRPM([0.0])
    sim.AddModelToTask("t", sc)
    return sim, sc, macros


def _run_two_ticks(sim, macros):
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(DT * 2))
    sim.ExecuteSimulation()


# --- bucket (a) behavioral raise tests (one per item) -------------------------

def test_guard_force_msg_raises_and_disarmed_runs():
    """Bucket (a): linked THRArrayCmdForceMsg raises pre-propagation."""
    pytest.importorskip("Basilisk.architecture.messaging",
                        reason="host-only: needs Basilisk messaging")
    from Basilisk.architecture import messaging
    # Armed: linked force path raises at Reset (before first propagation).
    sim, sc, _ = _fresh_hub_wheel()
    assert hasattr(sc, "thrForceInMsg"), "thrForceInMsg missing"
    force_msg = messaging.THRArrayCmdForceMsg()
    fp = messaging.THRArrayCmdForceMsgPayload()
    fp.thrForce = [0.0] * 36
    force_msg.write(fp)
    sc.thrForceInMsg.subscribeTo(force_msg)
    with pytest.raises(Exception, match="[Ff]orce"):
        sim.InitializeSimulation()
    # Disarmed: unlinked force path runs.
    sim2, sc2, macros2 = _fresh_hub_wheel()
    _run_two_ticks(sim2, macros2)


def test_guard_umin_raises_and_disarmed_runs():
    """Bucket (a): non-default u_min raises; default 0.0 runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setUMin")
    sc.setUMin(1e-6)
    with pytest.raises(Exception, match="uMin"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setUMin(0.0)
    _run_two_ticks(sim2, macros2)


def test_guard_pmax_raises_and_disarmed_runs():
    """Bucket (a): non-default P_max raises; default 200.0 runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setPMax")
    sc.setPMax(100.0)
    with pytest.raises(Exception, match="pMax"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setPMax(200.0)
    _run_two_ticks(sim2, macros2)


def test_guard_max_wheel_accel_raises_and_disarmed_runs():
    """Bucket (a): non-default maxWheelAcceleration raises; 1e6 runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setMaxWheelAcceleration")
    sc.setMaxWheelAcceleration(100.0)
    with pytest.raises(Exception, match="maxWheelAcceleration"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setMaxWheelAcceleration(1.0e6)
    _run_two_ticks(sim2, macros2)


def test_guard_large_torque_threshold_raises_and_disarmed_runs():
    """Bucket (a): non-default largeTorqueThreshold raises; 10.0 runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setLargeTorqueThreshold")
    sc.setLargeTorqueThreshold(1.0)
    with pytest.raises(Exception, match="largeTorqueThreshold"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setLargeTorqueThreshold(10.0)
    _run_two_ticks(sim2, macros2)


def test_guard_jitter_fully_coupled_setter_raises():
    """Bucket (a): JitterFullyCoupled raises at its setter.

    SETTER-GATED LIMITATION (documented, not a gap in the guard): the
    SysModel config surface carries no ``RWModel`` member, so there is
    nothing to detect at setup time — the setter itself is the only arming
    path and it raises immediately. Stock-side ``RWModel`` values are
    rejected vec-side by ``omniheyoka/swap.py`` (unmodified reference).
    Disarmed (default BalancedWheels) configs run.
    """
    _, sc, _ = _fresh_hub_wheel()
    assert callable(getattr(sc, "setRWModelJitterCoupled", None))
    with pytest.raises(Exception, match="JitterFullyCoupled"):
        sc.setRWModelJitterCoupled()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    _run_two_ticks(sim2, macros2)


def test_guard_mtb_gt_raises_and_identity_runs():
    """Bucket (a): non-identity Gt raises; identity runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setMtbGtMatrix")
    sc.setMtbGtMatrix([2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    with pytest.raises(Exception, match="non-identity"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setMtbGtMatrix([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    _run_two_ticks(sim2, macros2)


def test_guard_srp_rot_hat_raises_and_zero_runs():
    """Bucket (a): nonzero SRP facet rotHat raises; zero runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setSRPFacetRotHat")
    sc.setSRPFacetRotHat(0.0, 0.0, 1.0)
    with pytest.raises(Exception, match="rotHat"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setSRPFacetRotHat(0.0, 0.0, 0.0)
    _run_two_ticks(sim2, macros2)


def test_guard_nbodies_gt2_raises_and_two_runs():
    """Bucket (a): 3rd env body raises (no per-index msg vector); 2 run."""
    MU_SUN = 1.32712440018e20
    AU = 1.495978707e11
    sim, sc, _ = _fresh_hub_wheel()
    sc.addEnvironmentBody(MU, 0.0, 0.0, 0.0)
    sc.addEnvironmentBody(MU_SUN, AU, 0.0, 0.0)
    sc.addEnvironmentBody(4.9048695e12, 0.0, 3.844e8, 0.0)
    with pytest.raises(Exception, match="nBodies"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.addEnvironmentBody(MU, 0.0, 0.0, 0.0)
    sc2.addEnvironmentBody(MU_SUN, AU, 0.0, 0.0)
    _run_two_ticks(sim2, macros2)


def test_guard_prescribed_raises_and_disarmed_runs():
    """Bucket (a): prescribed motion enabled raises; disabled runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setPrescribedMotionEnabled")
    sc.setPrescribedMotionEnabled(True)
    with pytest.raises(Exception, match="prescribed"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setPrescribedMotionEnabled(False)
    _run_two_ticks(sim2, macros2)


def test_guard_flex_raises_and_disarmed_runs():
    """Bucket (a): flex enabled raises; disabled runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setFlexEnabled")
    sc.setFlexEnabled(True)
    with pytest.raises(Exception, match="flex"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setFlexEnabled(False)
    _run_two_ticks(sim2, macros2)


def test_guard_slosh_raises_and_disarmed_runs():
    """Bucket (a): slosh enabled raises; disabled runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "setSloshEnabled")
    sc.setSloshEnabled(True)
    with pytest.raises(Exception, match="slosh"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setSloshEnabled(False)
    _run_two_ticks(sim2, macros2)


def test_guard_multi_tank_raises_and_single_runs():
    """Bucket (a): 2nd tank registration raises; single-tank runs."""
    sim, sc, _ = _fresh_hub_wheel()
    _need_b1_api(sc, "addFuelTank")
    sc.setFuelParams(10.0, 10.0, 250.0)
    sc.addFuelTank(5.0, 5.0, 250.0)
    with pytest.raises(Exception, match="multiTank"):
        sim.InitializeSimulation()
    sim2, sc2, macros2 = _fresh_hub_wheel()
    sc2.setFuelParams(10.0, 10.0, 250.0)
    _run_two_ticks(sim2, macros2)


def test_guard_update_state_head_raises_post_init():
    """Bucket (a): arming AFTER init raises at the UpdateState head.

    Both heads call the same ``checkUnsupported()``; this test proves the
    UpdateState side (the per-item tests above prove the Reset side).
    """
    sim, sc, macros = _fresh_hub_wheel()
    _need_b1_api(sc, "setUMin", "setSloshEnabled")
    sim.InitializeSimulation()
    sc.setUMin(1e-6)
    sim.ConfigureStopTime(macros.sec2nano(DT * 1))
    with pytest.raises(Exception, match="unmodeled feature"):
        sim.ExecuteSimulation()


def test_guard_default_config_runs():
    """Bucket (a) disarmed baseline: stock-default config runs 2 ticks."""
    sim, _, macros = _fresh_hub_wheel()
    _run_two_ticks(sim, macros)


# --- WP25-C2 M5-rest layout hygiene (APPEND-ONLY home for printer pin) -------
# checked_par_count (include/omni/system.hpp) parses heyoka's par printer
# output expecting the `p<i>` form (s[0]=='p' + digits). If heyoka ever
# changes that format, the parse silently yields an empty set — this test
# fails loudly instead (source-contract half always runs; live-printer half
# runs wherever heyoka is importable, skips otherwise).

def test_printer_par_index_format_pinned():
    """M5-rest: heyoka prints par[i] as 'p<i>' (checked_par_count parses it)."""
    src = (REPO / "include" / "omni" / "system.hpp").read_text(encoding="utf-8")
    assert "s[0] != 'p'" in src, (
        "checked_par_count no longer parses the 'p<i>' printer form -- "
        "update this pin and the parser together")
    assert "heyoka::par[i]" in src, (
        "par vector no longer built from heyoka::par -- re-verify the print assumption")
    heyoka = pytest.importorskip("heyoka")
    assert str(heyoka.par[0]) == "p0", str(heyoka.par[0])
    assert str(heyoka.par[12]) == "p12", str(heyoka.par[12])
