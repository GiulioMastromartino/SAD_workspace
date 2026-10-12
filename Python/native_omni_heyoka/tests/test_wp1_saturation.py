"""WP1 RW momentum-saturation parity (PLAN_ENVELOPE.md Rev 1 WP1).

Stock ledger (``audit_bundle/results/wp1_sat.json``, BSK 2.12.0, HR16 demo
bus 750 kg / diag(900, 800, 600), Driver rewrite-every-tick)::

  approach-below  5950 RPM, u=+0.2, dt=0.1, 60 ticks -> overshoot 0.40 RPM,
                                                 plateau 6000.40 RPM
  from-above      6050 RPM, u=+0.2                  -> holds 6050, slope 0
  reverse-descent 6050 RPM, u=-0.2                  -> spins down
  dt=0.02         5950 RPM, u=+0.2, 60 x 0.02 s      -> 5978.3, never caps

Locked tolerance: overshoot <= 0.5 RPM @ dt=0.1, plateau +/-0.05 RPM.
Config contract: lowercase ``custom`` type; ``Omega_max`` setter-only
(``RWArrayConfigMsgPayload`` carries no ``Omega_max``); ``Omega_max <= 0``
means OFF (stock ``custom`` -1 convention); ``addWheel`` (custom) never
inherits rated defaults.

Scope: saturation ONLY. ``maxWheelAcceleration`` / ``largeTorqueThreshold``
semantics (log vs clamp) are OUT of WP1 -- WP0-guard territory. No
``satProbeDone_`` flag or threshold member is added here.

Runtime: 7 short sims (~210 ticks total + 7 JIT builds), < 3 min on host.
Every sim test import-skips Basilisk AND the built wheel, so the file
collects (as skips) where the extension is not built -- e.g. Linux CI,
where Basilisk/heyoka wheels are macOS-only.
"""
from __future__ import annotations

import math

import pytest

RPM_TO_RADS = 2.0 * math.pi / 60.0
RADS_TO_RPM = 60.0 / (2.0 * math.pi)
MU = 3.986004418e14
MASS = 750.0
IXX, IYY, IZZ = 900.0, 800.0, 600.0
DT = 0.1
CAP_RPM = 6000.0

# Locked stock ledger values (approach-below, dt=0.1).
LEDGER_OVERSHOOT_RPM = 0.40
LEDGER_PLATEAU_RPM = 6000.40
# First-step spin-up rate pins Js behaviorally: u*dt/Js in RPM
# (5952.400212225355 - 5950.0 from the ledger trace).
LEDGER_FIRST_STEP_RPM = 2.400212225355


def _run_omni_wheel(*, omega_start_rpm, u_cmd, dt=DT, n_ticks=60,
                    gs=(1.0, 0.0, 0.0), rated=(50.0, 6000.0, 0.2),
                    custom=None, omega_max_rpm=None):
    """Single-wheel omni sim with a rewrite-every-tick torque driver.

    ``rated=(maxMomentum, omegaMaxRPM, uMax)`` uses ``addRatedWheel``;
    ``custom=(Js, uMax)`` uses ``addWheel`` (cap OFF unless ``omega_max_rpm``
    is passed via the setter-only ``setWheelOmegaMax``). Returns per-tick
    wheel-0 speeds in RPM (sample[0] is the IC tick, matching the stock
    probe convention).
    """
    np = pytest.importorskip("numpy")
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion

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
    sc.setMass(MASS)
    sc.setInertia(IXX, IYY, IZZ)
    sc.setMu(MU)
    sc.setUseGravityGradient(False)
    sc.setTol(1e-12)
    if rated is not None:
        sc.addRatedWheel(float(gs[0]), float(gs[1]), float(gs[2]),
                         float(rated[0]), float(rated[1]), float(rated[2]))
    else:
        assert custom is not None
        sc.addWheel(float(gs[0]), float(gs[1]), float(gs[2]),
                    float(custom[0]), float(custom[1]))
    if omega_max_rpm is not None:
        sc.setWheelOmegaMax(0, float(omega_max_rpm))
    sc.ModelTag = "wp1_omni_sat"
    sc.setR_CN_NInit(*map(float, r0))
    sc.setV_CN_NInit(*map(float, v0))
    sc.setSigmaBNInit(0.0, 0.0, 0.0)
    sc.setOmegaBNInit(0.0, 0.0, 0.0)
    sc.setWheelSpeedsInitRPM([float(omega_start_rpm)])
    sim.AddModelToTask("t", sc)

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

    drv = _Driver()
    sim.AddModelToTask("t", drv)
    sc.rwMotorCmdInMsg.subscribeTo(drv.out)

    log = sc.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("t", log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()

    w = np.asarray(log.wheelSpeeds, float)
    assert w.ndim == 2 and w.shape[0] >= 2, f"recorder shape {w.shape}"
    return (w[:, 0] * RADS_TO_RPM).tolist()


# --- sim-free unit tests (need the built wheel + Basilisk, but no sim) ---

def test_add_rated_wheel_js_derivation_no_sim():
    """``addRatedWheel`` registers one wheel; Js contract is locked.

    C++ implements ``Js = maxMomentum / omegaMaxRad`` with the RPM->rad/s
    conversion (stock ``rwFactory`` parity; both+Js is a hard error in
    stock, derived-only here). No Js getter exists by design (WP1 adds no
    telemetry), so the numeric contract is locked here and verified
    behaviorally by the spin-up slope assert in
    ``test_approach_below_overshoot_and_plateau``.
    """
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    sc = oh.OmniSpacecraft()
    assert sc.nWheels() == 0
    sc.addRatedWheel(1.0, 0.0, 0.0, 50.0, 6000.0, 0.2)
    assert sc.nWheels() == 1
    expected_js = 50.0 / (6000.0 * RPM_TO_RADS)
    assert expected_js == pytest.approx(0.07957747154594767, rel=1e-12)
    # Setter-only path smoke (OFF round-trip, sim-free).
    sc.setWheelOmegaMax(0, 6000.0)
    sc.setWheelOmegaMax(0, -1.0)
    assert sc.nWheels() == 1


# --- stock-ledger parity sims ---

def test_approach_below_overshoot_and_plateau():
    """Ledger gate: overshoot <= 0.5 RPM @ dt=0.1, plateau 6000.40 +/- 0.05."""
    rpm = _run_omni_wheel(omega_start_rpm=5950.0, u_cmd=0.2, dt=0.1, n_ticks=60)
    assert len(rpm) >= 40, f"short run: {len(rpm)} samples"
    assert rpm[0] == pytest.approx(5950.0, abs=0.05)
    # Behavioral Js pin: first torque step matches the ledger spin-up rate.
    assert (rpm[1] - rpm[0]) == pytest.approx(LEDGER_FIRST_STEP_RPM, abs=0.05)
    overshoot = max(rpm) - CAP_RPM
    assert 0.0 <= overshoot <= 0.5, f"overshoot {overshoot}"
    plateau = sum(rpm[-10:]) / 10.0
    assert plateau == pytest.approx(LEDGER_PLATEAU_RPM, abs=0.05), plateau
    # Silent + coast after cut: tail is flat.
    tail = rpm[-10:]
    assert max(tail) - min(tail) <= 0.05, tail


def test_negative_spin_mirror():
    """Direction-aware cut, negative arm (symmetry gate; ledger pins + arm)."""
    rpm = _run_omni_wheel(omega_start_rpm=-5950.0, u_cmd=-0.2, dt=0.1, n_ticks=60)
    assert len(rpm) >= 40, f"short run: {len(rpm)} samples"
    undershoot = -CAP_RPM - min(rpm)
    assert 0.0 <= undershoot <= 0.5, f"undershoot {undershoot}"
    plateau = sum(rpm[-10:]) / 10.0
    assert plateau == pytest.approx(-LEDGER_PLATEAU_RPM, abs=0.05), plateau


def test_from_above_hold():
    """Ledger gate: 6050 RPM + spin-up torque holds (cut + coast, slope 0)."""
    rpm = _run_omni_wheel(omega_start_rpm=6050.0, u_cmd=0.2, dt=0.1, n_ticks=20)
    assert len(rpm) >= 10, f"short run: {len(rpm)} samples"
    assert max(rpm) <= 6050.5, max(rpm)
    assert min(rpm) >= 6049.0, min(rpm)
    assert rpm[-1] == pytest.approx(6050.0, abs=1.0)


def test_reverse_descent():
    """Ledger gate: 6050 RPM + reverse torque spins down (never saturates)."""
    rpm = _run_omni_wheel(omega_start_rpm=6050.0, u_cmd=-0.2, dt=0.1, n_ticks=30)
    assert len(rpm) >= 15, f"short run: {len(rpm)} samples"
    assert max(rpm) <= 6050.5, max(rpm)
    assert rpm[-1] < rpm[0] - 5.0, (rpm[0], rpm[-1])


def test_setter_only_cap_via_setWheelOmegaMax():
    """Custom wheel + setter-only cap behaves like the rated path."""
    js_hr16 = 50.0 / (6000.0 * RPM_TO_RADS)
    rpm = _run_omni_wheel(omega_start_rpm=5950.0, u_cmd=0.2, dt=0.1, n_ticks=25,
                          rated=None, custom=(js_hr16, 0.2),
                          omega_max_rpm=6000.0)
    assert len(rpm) >= 20, f"short run: {len(rpm)} samples"
    overshoot = max(rpm) - CAP_RPM
    assert 0.0 <= overshoot <= 0.5, f"overshoot {overshoot}"


def test_custom_default_off():
    """``addWheel`` cap defaults OFF: sails through 6000 RPM with +torque."""
    js_hr16 = 50.0 / (6000.0 * RPM_TO_RADS)
    rpm = _run_omni_wheel(omega_start_rpm=6050.0, u_cmd=0.2, dt=0.1, n_ticks=15,
                          rated=None, custom=(js_hr16, 0.2))
    assert len(rpm) >= 10, f"short run: {len(rpm)} samples"
    # 14 torque steps x ~2.4 RPM with no cut (stock `custom` w/o Omega_max
    # stays off, never inherits 6000 RPM).
    assert rpm[-1] == pytest.approx(6050.0 + 14 * LEDGER_FIRST_STEP_RPM, abs=3.0)
    assert rpm[-1] > 6050.0 + 5.0
