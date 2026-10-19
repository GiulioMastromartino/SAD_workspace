"""Upstream-demo hijack: run the EXACT pinned Basilisk scenario with omni.

Pipeline (per demo, all machine-checked, no hand transcription):
  1. PIN: fetch ``basilisk/examples/<demo>.py`` at the pinned tag from
     ``raw.githubusercontent.com``; verify sha256 against ``UPSTREAM``.
  2. PORT: generate ``<demo>_omni.py`` = upstream + exactly one marked swap
     block (inserted before the first ``InitializeSimulation()`` call) +
     one marked dump block (after the last ``ExecuteSimulation()``) + the
     entry call with the first ``True`` (always ``show_plots``) flipped to
     ``False``. Insertions only -- no upstream line is modified or deleted.
  3. DIFF-GATE (self-check, fails loudly): strip the marked regions from the
     port; the remainder must equal upstream byte-for-byte.
  4. DUAL RUN: stock arm (``OMNI_HIJACK=off``) + omni arm (``=on``) as
     subprocesses, each dumping ``r/v/sigma/omega[/wheels][/onTime]`` to npz.
     Reject-mode demos expect the omni arm to raise the loud mirror reject
     instead of integrating.
  5. COMPARE: final-state diffs vs the demo's tol class + liveness vacuity.

The swap block closes the SAME FSW loop the demo built (stock FSW modules,
stock messages); only the integrator + mirrored config change. Upstream files
are never edited by hand -- they live read-only under
``audit_bundle/upstream/`` and any local modification breaks the sha256 gate.

The omni arm mirrors through the repo's ``omniheyoka/swap.py`` loaded BY FILE
PATH (env ``OMNI_REPO_SWAP``, set by ``_run_port``) alongside the wheel's
``OmniVecIntegrator`` -- the same "repo swap.py by path + wheel integrator"
pairing every env probe uses. The installed wheel's ``swap.py`` copy lags the
repo (it has no env readers and rejects drag/SRP/mag outright), so a leg that
couples one of those would otherwise fail on a stale artifact rather than on
the code under test. The path import touches no sys.path entry, so the wheel
still supplies ``omni_heyoka`` + the compiled extension.

Usage:
    python audit_bundle/probes/hijack_demo.py --demo scenarioAttitudeFeedbackRW
    python audit_bundle/probes/hijack_demo.py --list-demos
Host-only (Basilisk + built wheel); without them records
``status: skipped-no-basilisk`` and exits 0. ``--list-demos`` is stdlib-only.
"""

from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Repo root (Python/) -- identical to every other probe (common.py,
# bench_stock_vs_omni.py, ...). Do NOT insert parents[2]
# (native_omni_heyoka/): its sourceless omniheyoka tree copy would shadow
# the installed wheel (conftest.py rationale). The arms run as subprocesses
# with PYTHONPATH=Python/ (see _run_port); the repo's swap.py reaches them by
# FILE PATH via OMNI_REPO_SWAP, which needs no sys.path entry.
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from common import RESULTS_DIR, log_text, save_result, safe_import_basilisk  # noqa: E402

PROBES_DIR = Path(__file__).resolve().parent
UPSTREAM_DIR = PROBES_DIR.parent / "upstream"

TAG = "v2.12.0"
BASE_URL = ("https://raw.githubusercontent.com/AVSLab/basilisk/"
            "%s/examples/" % TAG)

# ---------------------------------------------------------------------------
# Full upstream inventory: every .py file under basilisk/examples @ the
# pinned tag (187 files: 180 scenario scripts + SunLineKF helper + 6
# in-directory support scripts), each with a bench disposition. Row shape:
# path -> (bench, hijack).
# bench:  live:<bench-cells>      benched live in bench_stock_vs_omni.py
#         reject:<cell>           proves the loud reject (out-of-envelope)
#         covered:<cells/ledger>  same dynamics already benched (mechanism)
#         out:<reason>            not benched, with the reason
#         audit:<question>        needs a file read before disposition
# hijack: parity | reject | excluded:<reason> | na:<reason>
# Support libraries, masters, plotting helpers, paper tooling, sweep
# drivers, data dirs, model dirs and message dumps are out:infra BY CLASS
# (documented here, not per-file): BSK_masters / BSK_*models / BSK_*plotting
# / plotting* / tleData / xml models / run_MC_IC/*.json.
# ---------------------------------------------------------------------------
UPSTREAM_ALL = {
    # --- root: orbital mechanics --------------------------------------
    "scenarioBasicOrbit.py": (
        "live:demo_basicOrbit_LEO,demo_basicOrbit_GTO,demo_basicOrbit_GEO",
        "parity"),
    "scenarioOrbitManeuver.py": (
        "covered:hub coast cells (DV sequencing is script-level "
        "stop/restart, not an integrator surface)", "na:script-level DV"),
    "scenarioHohmann.py": (
        "covered:hub coast cells (same reason as scenarioOrbitManeuver)",
        "na:script-level DV"),
    "scenarioPatchedConics.py": ("out:multi-arc multi-central bodies",
                                 "na:N-body"),
    "scenarioFlybySpice.py": ("out:SPICE ephemeris", "na:N-body"),
    "scenarioHelioTransSpice.py": ("out:SPICE ephemeris", "na:N-body"),
    "scenarioJupiterArrival.py":
        ("covered:demo_basicOrbit_LEO (same hub-only Kepler mechanism; "
         "hyperbolic ICs unaudited)", "parity"),
    "scenarioCentralBody.py":
        ("out:mid-run central-body switch is a config mutation the "
         "setup-time mirror cannot foresee", "na:config mutation"),
    "scenarioCustomGravBody.py": ("out:custom (non-central) gravity",
                                  "na:N-body"),
    "scenarioOrbitMultiBody.py": ("out:N-body", "na:N-body"),
    "scenarioLagrangePointOrbit.py": ("out:N-body", "na:N-body"),
    "scenarioHaloOrbit.py": ("out:N-body", "na:N-body"),
    "scenarioOrbitFromOmm.py": ("out:SPICE/OEM ephemeris ICs unaudited",
                                "na:ephemeris ICs"),
    "scenarioSpiceSpacecraft.py": ("out:SPICE ephemeris", "na:ephemeris"),
    "scenarioSpiceReconstruction.py": ("out:SPICE ephemeris", "na:ephemeris"),
    "scenarioVestaOrientation.py": ("out:mission SPICE", "na:mission"),
    "scenarioAsteroidArrival.py": ("out:mission (arrival targeting)",
                                   "na:mission"),
    "scenarioAerocapture.py": ("out:mission + deflected panels",
                               "na:mission+branching"),
    "scenarioAerobrake.py": ("out:mission + deflected panels",
                             "na:mission+branching"),
    "scenarioGroundTracks.py": ("out:viz (ground-track rendering)",
                                "na:visualization"),
    "scenarioOrbitConsistencyVerification.py":
        ("out:verifies spherical-harmonic (J2) propagation; central-J2 "
         "mirror path unverified", "na:central-J2"),
    "scenarioInertialSpiral.py":
        ("covered:hub (spiral guidance law; FSW wiring unaudited)",
         "reject"),
    "scenarioGroundLocationImaging.py":
        ("covered:hub/rw pointing class (imaging task is FSW; wiring "
         "unaudited)", "reject"),
    # --- root: attitude regulation / guidance --------------------------
    "scenarioAttitudeFeedbackRW.py": ("live:demo_attFeedbackRW", "parity"),
    "scenarioAttitudeFeedback.py": ("covered:ext torque now mirrorable "
                                    "(hijack_scenarioAttitudeFeedback)",
                                    "parity"),
    "scenarioAttitudeFeedback2T.py":
        ("covered:ext torque now mirrorable (hijack_scenarioAttitudeFeedback2T)",
         "parity"),
    "scenarioAttitudeFeedback2T_TH.py":
        ("covered:ext torque mirrored; FSW onTime scheduling unverified",
         "reject"),
    "scenarioAttitudeFeedback2T_stateEffTH.py":
        ("covered:ext torque mirrored; state-effector firing path "
         "unvalidated", "reject"),
    "scenarioAttitudeFeedbackNoEarth.py":
        ("covered:ext torque present; no-central-body question moot",
         "reject"),
    "scenarioAttitudeFeedbackNumba.py":
        ("covered:rw3_pd class (Numba FSW variant; same dynamics)",
         "na:FSW implementation"),
    "scenarioAttitudeFeedbackRWPower.py":
        ("covered:rw3_pd class (power bookkeeping is not dynamics)",
         "parity"),
    "scenarioAttitudeGG.py": ("live:demo_attitudeGG", "parity"),
    "scenarioAttitudeGuidance.py":
        ("covered:hub/rw pointing class (guidance-law variant; FSW wiring "
         "unaudited)", "reject"),
    "scenarioAttitudePointing.py":
        ("covered:hub/rw pointing class (same note as guidance)",
         "reject"),
    "scenarioAttitudePointingNumba.py":
        ("covered:hub/rw pointing class (Numba FSW variant)",
         "na:FSW implementation (numba not installed on the host)"),
    "scenarioAttitudePointingPy.py":
        ("covered:hub/rw pointing class (Python FSW variant)",
         "reject"),
    "scenarioAttitudePrescribed.py":
        ("out:prescribed motion (bucket-a loud reject, never modeled)",
         "na:prescribed"),
    "scenarioAttitudeSteering.py":
        ("covered:hub/rw (steering-law variant; FSW wiring unaudited)",
         "parity"),
    "scenarioAttLocPoint.py":
        ("covered:hub/rw pointing class (FSW wiring unaudited)",
         "reject"),
    "scenarioAttGuideHyperbolic.py":
        ("covered:hub/rw (hyperbolic guidance variant; FSW unaudited)",
         "reject"),
    "scenarioAttitudeConstrainedManeuver.py":
        ("covered:hub/rw (keep-out guidance is FSW; dynamics identical)",
         "na:guidance variant"),
    "scenarioAttitudeConstraintViolation.py":
        ("covered:hub/rw (same note as constrained maneuver)",
         "na:guidance variant"),
    "scenarioSweepingSpacecraft.py":
        ("covered:hub/rw pointing class (sweep maneuver is FSW)",
         "reject"),
    "scenarioStripImaging.py":
        ("covered:hub/rw pointing class (imaging task is FSW)",
         "na:guidance variant"),
    "scenarioRotatingPanel.py":
        ("out:articulated/rotating panel (unmodeled topology)",
         "na:articulated"),
    "scenarioRotTransArrayDeployment.py":
        ("out:deploying array (unmodeled topology)", "na:articulated"),
    "scenarioExtendingBoom.py":
        ("out:extending boom (unmodeled topology)", "na:articulated"),
    "scenarioDeployingPanel.py":
        ("out:deploying panel (unmodeled topology)", "na:articulated"),
    "scenarioDeployingSolarArrays.py":
        ("out:deploying arrays (unmodeled topology)", "na:articulated"),
    # --- root: actuators ------------------------------------------------
    "scenarioMomentumDumping.py": ("live:demo_momentumDumping",
                                   "excluded:N-body (sun third body + "
                                   "SPICE; env-bodies NO-GO, list.md A1)"),
    "scenarioSepMomentumManagement.py":
        ("covered:demo_momentumDumping (same desat class)",
         "na:same class as live demo cell"),
    "scenarioThrusterArm.py":
        ("out:two-axis articulated thruster arm (unmodeled topology)",
         "na:articulated"),
    "scenarioRoboticArm.py":
        ("out:robotic arm NDOF effector (unmodeled)", "na:articulated"),
    "scenarioRoboticGrappling.py":
        ("out:branched grappling arm (unmodeled topology)",
         "na:branched"),
    "scenarioHingedRigidBody.py":
        ("out:hinged rigid body (unmodeled)", "na:articulated"),
    "scenarioSpinningBodiesTwoDOF.py":
        ("out:spinning 2-DOF bodies (unmodeled)", "na:spinning"),
    "scenarioPrescribedMotionWithRotationBranching.py":
        ("out:prescribed motion with branching (bucket-a loud reject, "
         "never modeled)", "na:prescribed"),
    "scenarioPrescribedMotionWithTranslationBranching.py":
        ("out:prescribed motion with branching (same note)",
         "na:prescribed"),
    "scenarioPrescribedScrewMotion.py":
        ("out:prescribed helical motion (same bucket-a reject)",
         "na:prescribed"),
    "scenarioMtbMomentumManagement.py":
        ("covered:env_parity_mag (identity-Gt MTB + centered dipole)",
         "na:same class as mag parity"),
    "scenarioMtbMomentumManagementSimple.py":
        ("covered:env_parity_mag (same class)", "na:same class"),
    "scenarioMonteCarloAttRW.py":
        ("covered:mc tier (seeded Js/IC dispersion campaign)",
         "na:same class"),
    "scenarioMonteCarloSpice.py":
        ("covered:mc tier (Spice-setup variant of the same campaign)",
         "na:same class"),
    "scenarioTAM.py": ("covered:env_parity_mag (TAM + field class)",
                        "parity"),
    "scenarioTAMcomparison.py":
        ("covered:env_parity_mag (TAM comparison is FSW/sensor)",
         "na:same class"),
    # --- root: environments ---------------------------------------------
    "scenarioDragDeorbit.py": ("live:demo_dragDeorbit",
                               "parity:capped-horizon leg (stop time clamped "
                               "to 1200 s; see CONFIG note)"),
    "scenarioDragRendezvous.py":
        ("covered:env_drag cells (drag env) + swap_multi (proximity ops "
         "are FSW, not integrator)", "na:ops variant"),
    "scenarioDragSensitivity.py":
        ("covered:env_drag cells (sensitivity harness over the same drag "
         "dynamics)", "na:study harness"),
    "scenarioMagneticFieldCenteredDipole.py":
        ("covered:hub circular-Earth (field read adds no dynamics; "
         "torque needs an MTB: mag-parity class)",
         "parity"),
    "scenarioMagneticFieldWMM.py":
        ("out:message-only WMM stays unmirrorable (staged reader pins "
         "the reject)", "na:message-only model"),
    "scenarioAlbedo.py": ("out:albedo (no contributor)", "na:no model"),
    "scenarioStochasticDragSpacecraft.py":
        ("out:stochastic density (Taylor cannot do SDE)",
         "na:stochastic"),
    "scenarioGaussMarkovRandomWalk.py":
        ("out:stochastic process demo (analysis harness, not dynamics)",
         "na:stochastic"),
    # --- root: formation / proximity / mission ops (FSW, not integrator) --
    "scenarioFormationBasic.py":
        ("covered:swap_multi (sync mechanism); control laws are FSW",
         "na:ops variant"),
    "scenarioFormationMeanOEFeedback.py":
        ("covered:swap_multi (same note)", "na:ops variant"),
    "scenarioFormationReconfig.py":
        ("covered:swap_multi (same note)", "na:ops variant"),
    "scenarioRendezVous.py": ("out:proximity ops (not an integrator "
                              "surface)", "na:ops"),
    "scenarioDebrisReorbitET.py":
        ("out:electrostatic tractor (unmodeled actuation)", "na:ops"),
    "scenarioSatelliteConstellation.py":
        ("covered:swap_multi (sync mechanism; constellation ops are FSW)",
         "na:ops variant"),
    "scenarioSmallBodyFeedbackControl.py":
        ("out:small-body prox-ops + waypoint FSW (not integrator)",
         "na:ops"),
    "scenarioSmallBodyLandmarks.py": ("out:landmark nav (not dynamics)",
                                      "na:navigation"),
    "scenarioSmallBodyNav.py": ("out:navigation (not dynamics)",
                                "na:navigation"),
    "scenarioSmallBodyNavUKF.py": ("out:navigation (not dynamics)",
                                   "na:navigation"),
    "scenarioLambertSolver.py": ("out:impulsive Lambert DV (no "
                                 "continuous dynamics leg)", "na:impulsive"),
    "scenarioOrbitManeuverTH.py":
        ("covered:ext torque mirrored (phase-1 capped leg, RKF45)",
         "parity"),
    "scenarioImpact.py": ("out:contact dynamics (not modeled)",
                          "na:contact"),
    "scenarioTwoChargedSC.py": ("out:charged-spacecraft MSM (not "
                                "modeled)", "na:charged"),
    # --- root: sub-systems / sensors / viz / meta (not dynamics) ---------
    "scenarioCSS.py": ("out:coarse sun sensors (not dynamics)",
                       "na:sensors"),
    "scenarioCSSFilters.py": ("out:sensor filters (not dynamics)",
                              "na:sensors"),
    "scenarioPowerDemo.py": ("out:power sub-system (bookkeeping, not "
                             "dynamics)", "na:sub-system"),
    "scenarioSensorThermal.py": ("out:thermal (not dynamics)",
                                 "na:sub-system"),
    "scenarioTempMeasurementAttitude.py":
        ("out:thermal + noise (not dynamics)", "na:sub-system"),
    "scenarioGroundDownlink.py": ("out:comms downlink (not dynamics)",
                                  "na:comms"),
    "scenarioGroundMapping.py": ("out:mapping task (FSW, not dynamics)",
                                 "reject"),
    "scenarioMovingCamera.py": ("out:viz camera (not dynamics)",
                                "na:visualization"),
    "scenarioQuadMaps.py": ("out:viz quad maps (not dynamics)",
                            "na:visualization"),
    "scenarioSpacecraftLocation.py":
        ("out:location utility (not dynamics)", "na:utility"),
    "scenarioDataDemo.py": ("out:data handling demo (not dynamics)",
                            "na:data"),
    "scenarioDataToViz.py": ("out:viz plumbing (not dynamics)",
                             "na:visualization"),
    "scenarioVizPoint.py": ("out:viz helper (not dynamics)",
                            "na:visualization"),
    "scenarioBskLog.py": ("out:logging utility demo (not dynamics)",
                          "na:utility"),
    "scenarioBasicOrbitStream.py":
        ("covered:hub (streaming transport variant of the same orbit)",
         "na:transport variant"),
    "scenarioBenchmarkNumba.py": ("out:meta-benchmark harness (omni IS "
                                  "the comparison subject)", "na:meta"),
    "scenarioIntegrators.py":
        ("out:meta-integrator demo (omni IS the comparison subject)",
         "na:meta"),
    "scenarioIntegratorsComparison.py": ("out:meta (same note)",
                                         "na:meta"),
    "scenarioVariableTimeStepIntegrators.py": ("out:meta (same note)",
                                               "na:meta"),
    "scenarioBranchingVerification.py":
        ("out:effector-branching verification (unmodeled topology)",
         "na:branched"),
    "scenarioConstrainedDynamics.py":
        ("out:holonomic-constraint dynamics (unmodeled)", "na:constrained"),
    "scenarioConstrainedDynamicsComponentAnalysis.py":
        ("out:constrained (same note)", "na:constrained"),
    "scenarioConstrainedDynamicsFrequencyAnalysis.py":
        ("out:constrained (same note)", "na:constrained"),
    "scenarioConstrainedDynamicsManeuverAnalysis.py":
        ("out:constrained (same note)", "na:constrained"),
    "scenarioFuelSlosh.py": ("out:fuel slosh (unmodeled)",
                             "na:slosh"),
    "scenarioFlexiblePanel.py": ("out:flexible panels (unmodeled)",
                                 "na:flex"),
    # --- BskSim/scenarios (13) -------------------------------------------
    "BskSim/scenarios/scenario_AttEclipse.py":
        ("covered:env_parity_eclipse (sun-pointing + eclipse, extended)",
         "na:same class"),
    "BskSim/scenarios/scenario_AttFeedback.py":
        ("covered:demo_attFeedbackRW (same hub+RW dynamics; FSW-mode "
         "variation)", "na:FSW variant"),
    "BskSim/scenarios/scenario_AttGuidance.py":
        ("covered:demo_attFeedbackRW (same note)", "na:FSW variant"),
    "BskSim/scenarios/scenario_AttModes.py":
        ("covered:demo_attFeedbackRW (same note)", "na:FSW variant"),
    "BskSim/scenarios/scenario_AttSteering.py":
        ("covered:demo_attFeedbackRW (same note)", "na:FSW variant"),
    "BskSim/scenarios/scenario_FeedbackRW.py":
        ("covered:demo_attFeedbackRW (same note)", "na:FSW variant"),
    "BskSim/scenarios/scenario_AttGuidHyperbolic.py":
        ("covered:demo_attFeedbackRW (hyperbolic guidance variant)",
         "na:guidance variant"),
    "BskSim/scenarios/scenario_BasicOrbit.py":
        ("covered:demo_basicOrbit_LEO (same hub-only class)",
         "na:same class"),
    "BskSim/scenarios/scenario_BasicOrbitFormation.py":
        ("covered:swap_multi (two-spacecraft sync mechanism)",
         "na:sync variant"),
    "BskSim/scenarios/scenario_RelativePointingFormation.py":
        ("covered:swap_multi (same note)", "na:sync variant"),
    "BskSim/scenarios/scenario_AddRWFault.py":
        ("out:mid-run RW fault injection (config mutation the "
         "setup-time mirror cannot foresee)", "na:fault injection"),
    "BskSim/scenarios/scenario_FaultList.py":
        ("out:fault-injection list (same note)", "na:fault injection"),
    "BskSim/scenarios/scenario_LambertGuidance.py":
        ("out:impulsive Lambert guidance (no continuous dynamics leg)",
         "na:impulsive"),
    # --- MultiSatBskSim/scenariosMultiSat (5) ------------------------------
    "MultiSatBskSim/scenariosMultiSat/scenario_AttGuidMultiSat.py":
        ("covered:swap_multi (multi sync mechanism; guidance modes are "
         "FSW)", "na:sync variant"),
    "MultiSatBskSim/scenariosMultiSat/scenario_BasicOrbitMultiSat.py":
        ("covered:swap_multi (same note)", "na:sync variant"),
    "MultiSatBskSim/scenariosMultiSat/scenario_BasicOrbitMultiSat_MT.py":
        ("covered:swap_multi (multi-threaded variant of the same sync)",
         "na:sync variant"),
    "MultiSatBskSim/scenariosMultiSat/scenario_StationKeepingMultiSat.py":
        ("covered:swap_multi + fuel cells (per-object depletion; "
         "stationkeeping law is FSW)", "na:sync+fuel variant"),
    "MultiSatBskSim/scenariosMultiSat/scenario_constellationFromTle.py":
        ("covered:swap_multi (constellation ops are FSW; TLE is an IC "
         "format)", "na:sync variant"),
    # --- OpNavScenarios (9 + 3 + 2; optical navigation, not dynamics) -----
    "OpNavScenarios/scenariosOpNav/scenario_CNNAttOD.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_OpNavAttOD.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_OpNavAttODLimb.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_OpNavHeading.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_OpNavOD.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_OpNavODLimb.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_OpNavPoint.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_OpNavPointLimb.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/scenario_faultDetOpNav.py":
        ("out:OpNav + fault detection (not dynamics)", "na:OpNav+fault"),
    "OpNavScenarios/scenariosOpNav/OpNavMC/MonteCarlo.py":
        ("out:OpNav MC image-set harness (not a dynamics leg)",
         "na:OpNav harness"),
    "OpNavScenarios/scenariosOpNav/OpNavMC/scenario_LimbAttOD.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/OpNavMC/scenario_OpNavAttODMC.py":
        ("out:optical navigation (not dynamics)", "na:OpNav"),
    "OpNavScenarios/scenariosOpNav/CNN_ImageGen/OpNavMonteCarlo.py":
        ("out:CNN training-image harness (not dynamics)", "na:OpNav harness"),
    "OpNavScenarios/scenariosOpNav/CNN_ImageGen/scenario_CNNImages.py":
        ("out:CNN image generation (not dynamics)", "na:OpNav harness"),
    # --- MonteCarloExamples (3) --------------------------------------------
    "MonteCarloExamples/scenarioBskSimAttFeedbackMC.py":
        ("covered:mc tier (seeded Js/IC dispersion campaign)",
         "na:same class"),
    "MonteCarloExamples/scenarioRerunMonteCarlo.py":
        ("covered:mc tier (rerun harness over the same campaign class)",
         "na:same class"),
    "MonteCarloExamples/scenarioVisualizeMonteCarlo.py":
        ("out:MC visualization (not dynamics)", "na:visualization"),
    # --- mujoco (18 scenarios; separate engine, never mirrored) ------------
    "mujoco/scenarioArmWithThrusters.py":
        ("out:MuJoCo engine", "na:MuJoCo engine"),
    "mujoco/scenarioAsteroidLanding.py":
        ("out:MuJoCo engine + contact", "na:MuJoCo engine"),
    "mujoco/scenarioAttitudeFeedbackRWMuJoCo.py":
        ("out:MuJoCo engine (stock class: demo_attFeedbackRW)",
         "na:MuJoCo engine"),
    "mujoco/scenarioBranchingPanels.py":
        ("out:MuJoCo engine + branching", "na:MuJoCo engine"),
    "mujoco/scenarioDeployPanels.py":
        ("out:MuJoCo engine + deployment", "na:MuJoCo engine"),
    "mujoco/scenarioFlexiblePanelMuJoCo.py":
        ("out:MuJoCo engine + flex", "na:MuJoCo engine"),
    "mujoco/scenarioFormationFlyingWithDrag.py":
        ("out:MuJoCo engine", "na:MuJoCo engine"),
    "mujoco/scenarioHingedRigidBodyMuJoCo.py":
        ("out:MuJoCo engine + hinge", "na:MuJoCo engine"),
    "mujoco/scenarioMJEarthMoonGravity.py":
        ("out:MuJoCo engine + N-body", "na:MuJoCo engine"),
    "mujoco/scenarioMJSceneVizard.py":
        ("out:MuJoCo engine + viz", "na:MuJoCo engine"),
    "mujoco/scenarioMomentumDumpingMuJoCo.py":
        ("out:MuJoCo engine (stock class: demo_momentumDumping)",
         "na:MuJoCo engine"),
    "mujoco/scenarioReactionWheel.py":
        ("out:MuJoCo engine (stock class: rw cells)", "na:MuJoCo engine"),
    "mujoco/scenarioSRPInPanels.py":
        ("out:MuJoCo engine + faceted SRP", "na:MuJoCo engine"),
    "mujoco/scenarioSimpleDocking.py":
        ("out:MuJoCo engine + contact", "na:MuJoCo engine"),
    "mujoco/scenarioStochasticDrag.py":
        ("out:MuJoCo engine + stochastic", "na:MuJoCo engine"),
    "mujoco/scenarioThrArmControl.py":
        ("out:MuJoCo engine + thruster arms", "na:MuJoCo engine"),
    "mujoco/scenarioThrArmTracking.py":
        ("out:MuJoCo engine + thruster arms", "na:MuJoCo engine"),
    "mujoco/scenarioUnbalancedThrusters.py":
        ("out:MuJoCo engine (stock class: fuel_offset unbalanced-torque)",
         "na:MuJoCo engine"),
    "mujoco/BSK_mujocoMasters.py":
        ("out:infra MuJoCo master file (not a sim)", "na:infra"),
    # --- dynamicsComparison (8 compare + 6 sweep research tooling) ---------
    "dynamicsComparison/scenarioCompareOrbit.py":
        ("covered:hub (Keplerian baseline leg)", "na:research harness"),
    "dynamicsComparison/scenarioCompareOrbitMultibody.py":
        ("out:N-body comparison leg", "na:N-body"),
    "dynamicsComparison/scenarioCompareTorque.py":
        ("covered:rw_diag3 (constant-torque rigid-body leg)",
         "na:research harness"),
    "dynamicsComparison/scenarioCompareRwPanels.py":
        ("out:wheels + hinged panels (flex unmodeled)",
         "na:research harness + flex"),
    "dynamicsComparison/scenarioCompareFlexPanels.py":
        ("out:flexible panels (unmodeled)", "na:research harness + flex"),
    "dynamicsComparison/scenarioCompareParetoFlexPanels.py":
        ("out:flexible panels (unmodeled)", "na:research harness + flex"),
    "dynamicsComparison/scenarioCompareParetoRwPanels.py":
        ("out:wheels + hinged panels (flex unmodeled)",
         "na:research harness + flex"),
    "dynamicsComparison/scenarioCompareVariableMass.py":
        ("out:variable-mass slosh/depletion (unmodeled)",
         "na:research harness"),
    "dynamicsComparison/sweepOrbitDt.py":
        ("covered:ladder (dt-sweep class, same switching-residual "
         "measurement)", "na:research harness"),
    "dynamicsComparison/sweepRwBookkeepingMass.py":
        ("covered:rw cells (RW mass-bookkeeping class)",
         "na:research harness"),
    "dynamicsComparison/sweepTorqueArtifact.py":
        ("covered:ladder (torque-artifact class, Update-M4 family)",
         "na:research harness"),
    "dynamicsComparison/sweepTorqueMechanismChecks.py":
        ("covered:ladder (same family)", "na:research harness"),
    "dynamicsComparison/sweepSloshDisplacement.py":
        ("out:slosh metric (unmodeled)", "na:research harness + slosh"),
    "dynamicsComparison/sweepVariableMassPendulumInertia.py":
        ("out:variable-mass pendulum (unmodeled)",
         "na:research harness"),
    "dynamicsComparison/_comparePlots.py":
        ("out:infra paper-plotting helper (not a sim)", "na:infra"),
    "dynamicsComparison/_comparisonValidation.py":
        ("out:infra validation helper (not a sim)", "na:infra"),
    "dynamicsComparison/_runtimeTable.py":
        ("out:infra runtime-table helper (not a sim)", "na:infra"),
    "dynamicsComparison/paperFigures.py":
        ("out:infra paper-figure helper (not a sim)", "na:infra"),
    "dynamicsComparison/runAllComparisons.py":
        ("out:infra comparison driver (not a sim)", "na:infra"),
    # --- helper (not a scenario) -------------------------------------------
    "SunLineKF_test_utilities.py": ("out:infra test utility (not a sim)",
                                    "na:infra"),
}

# Per-demo hijack config. var names verified against the v2.12.0 tag sources
# (single source of truth for the generator; tests pin every key present).
# mode: parity (A/B compare) | reject (omni arm must raise the loud mirror
#   reject; the stock arm must still complete) | excluded (documented reason).
CONFIG = {
    "scenarioAttitudeFeedbackRW": {
        "file": "scenarioAttitudeFeedbackRW.py",
        "sha256": "aed4ac66131236f406aea65636329b97760e34c25b8de0b2e306602cd"
                  "6574252",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": "rwStateEffector", "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 1e-9, "v_BN_N": 1e-9, "sigma_BN": 1e-9,
                "omega_BN_B": 1e-9, "wheels": 1e-4},
        "note": "upstream default runs WITH voltage IO (kept verbatim); "
                "wheels carry the coupled-state switching bound",
    },
    "scenarioMomentumDumping": {
        "file": "scenarioMomentumDumping.py",
        "sha256": "f6d6a7feb1e5e1a0c923c5791dd8a90039f54a5b9a22cf6c920a9b6a6b9"
                  "600bb",
        "sim": None, "sc": None, "task": None,
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "excluded",
        "note": "upstream setup carries a sun third body + SPICE ephemeris "
                "(createBodies('earth','sun')); the mirror requires exactly "
                "the central body and the env-bodies N-body path is an "
                "honest NO-GO (list.md A1: stock-reader loud raise + "
                "~2.4 km central-J2 shadow divergence). Dropping the sun "
                "would silently diverge the arms by sun-tide order "
                "(~6 cm over the leg), so no hijack until the N-body path "
                "goes green. Covered by the central-earth bench cell "
                "demo_momentumDumping (both arms share the adapted setup).",
    },
    "scenarioBasicOrbit": {
        "file": "scenarioBasicOrbit.py",
        "sha256": "879d93e8d82adcbbe622dfa40f92e616760a2e0914b055511b25ad6f514"
                  "21449",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 5e-2, "v_BN_N": 1e-4, "sigma_BN": 1e-9,
                "omega_BN_B": 1e-9},
        "note": "point-mass LEO, 3-orbit horizon at the demo's dt=10 s "
                "(measured class: 9.3e-3 m / 1.1e-5 m/s -- stock-RK4 "
                "discretization error vs near-exact Taylor, growing with "
                "horizon x coarse steps; torque-free attitude stays "
                "QUIET). J2 variants stay out: central-J2 mirror "
                "unverified",
    },
    "scenarioAttitudeGG": {
        "file": "scenarioAttitudeGG.py",
        "sha256": "ca82f80b01c0a97f6488421038c92e8e20e6b3572a05dc6b58100d48b8"
                  "c3a461",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "ext_torque_msg": "mrpControl.cmdTorqueOutMsg",
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 1e-9, "v_BN_N": 1e-9, "sigma_BN": 1e-6,
                "omega_BN_B": 1e-6},
        "note": "extForceTorque (msg-driven) mirrored; GG already passes "
                "the guard",
    },
    "scenarioDragDeorbit": {
        "file": "scenarioDragDeorbit.py",
        "sha256": "d8218f52473ed6804310039eebd3b0fd991ff05737efae00aa67c81c4"
                  "1c0454f",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "cap_s": 1200.0,
        "set_central": "planet",
        "tol": {"r_BN_N": 1.0e3, "v_BN_N": 2.0, "sigma_BN": 1e-9,
                "omega_BN_B": 1e-9},
        "note": "Upstream default runs model='msis' over a 100-orbit horizon "
                "with a 100 km terminal event. Two documented deviations, "
                "both injected inside the marked block (no upstream line "
                "touched): (1) stop time clamped to 1200 s so both arms fly "
                "the same ~0.22-orbit arc (the terminal event never fires); "
                "(2) planet.isCentralBody=True, which upstream omits "
                "(createEarth() defaults False in v2.12.0) and the mirror's "
                "stage-1 needs -- verified bit-identical for the stock arm. "
                "Residual class = coarse-dt density sampling: the mirrored "
                "drag consumes the atmosphere's once-per-step density "
                "message while stock samples it across RK4 substeps, so the "
                "arms differ by the within-step density variation at the "
                "demo's own dt=15 s. That is FIRST-order in dt (measured "
                "2.8e2/1.4e2/7.1e1/3.5e1 m as dt halves 15->1.875 s), and "
                "it tracks the atmosphere's variation rate: the identical "
                "leg at model='exponential' gives 7.1e-1 m, and the bench's "
                "capped-horizon replica at dt=0.1 agrees to 7.4e-6 m -- a "
                "consistent discretization difference, not a model error "
                "(everything -> 0 as dt -> 0). Attitude is exactly quiet "
                "(hub-only, drag comOffset 0: no torque).",
    },
    "scenarioJupiterArrival": {
        "file": "scenarioJupiterArrival.py",
        "sha256": "64f583d9ca57675fdd9056f7e7bedad6f2801f107468922a3afbb9da0"
                  "5bbf9f1",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 5e-2, "v_BN_N": 1e-4, "sigma_BN": 1e-9,
                "omega_BN_B": 1e-9},
        "note": "single central body (Jupiter) hyperbolic arrival: same "
                "horizon-class tol as the basicOrbit hijack (coarse-dt "
                "stock discretization vs near-exact Taylor)",
    },
    "scenarioAttitudeFeedback2T": {
        "file": "scenarioAttitudeFeedback2T.py",
        "sha256": "906817e3c469ce0c76a100d934d94a086d0ed73ab1301ff50a217914"
                  "07521cbf",
        "sim": "scSim", "sc": "scObject", "task": "dynTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "ext_torque_msg": "mrpControl.cmdTorqueOutMsg",
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 1e-9, "v_BN_N": 1e-9, "sigma_BN": 1e-6,
                "omega_BN_B": 1e-6},
        "note": "extForceTorque (const + msg-driven) now mirrored",
    },
    "scenarioAttitudeFeedback2T_TH": {
        "file": "scenarioAttitudeFeedback2T_TH.py",
        "sha256": "beb0a283404025b66e6e60de5b120f02af347102b6fe742df2537c77"
                  "7ca7f32b",
        "sim": "scSim", "sc": "scObject", "task": "dynTaskName",
        "rw": None, "thr_factory": "thFactory", "thr_cmd": "thrFiringSchmittObj.onTimeOutMsg",
        "ext_torque_msg": "mrpControl.cmdTorqueOutMsg",
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "extForceTorque mirrored, but the FSW onTimeOutMsg "
                "scheduling (0.5 s FSW step vs 0.1 s dyn) is not yet "
                "verified on the mirror; parity fails (sigma 0.25 rad). "
                "Re-enable once the thruster cmd path is validated.",
    },
    "scenarioAttitudeFeedback2T_stateEffTH": {
        "file": "scenarioAttitudeFeedback2T_stateEffTH.py",
        "sha256": "7f671fd54e03588780429eb241922e153e8a6f82d0cf531dbf506f72"
                  "1ff93ec1",
        "sim": "scSim", "sc": "scObject", "task": "dynTaskName",
        "rw": None, "thr_factory": "thFactory",
        "thr_cmd": "thrFiringSchmittObj.onTimeOutMsg",
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "ext torque now mirrored, but the state-effector thruster "
                "FIRES (FSW-driven) and the kappa lag is only modeled in "
                "the idle case; firing fidelity (target window) not yet "
                "validated. Re-enable once Feature 2 firing path lands.",
    },
    "scenarioOrbitManeuverTH": {
        "file": "scenarioOrbitManeuverTH.py",
        "sha256": "5e56acbd1dc2154692d13d505e8fcbd4d7979f29f1246ef573033c693578"
                  "d87d",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": "thFactory", "thr_cmd": "thrOnTimeMsg",
        "ext_torque_msg": "mrpControl.cmdTorqueOutMsg",
        "dump": None,
        "mode": "parity",
        "cap_s": 1457.1,
        "tol": {"r_BN_N": 1e3, "v_BN_N": 2.0, "sigma_BN": 1e-6,
                "omega_BN_B": 1e-6},
        "note": "phase-1 coast capped at 0.25 P (1457 s); ext torque "
                "mirrored (live msg); thruster idle over the cap. "
                "Stock arm = RKF45 at dt=2 s (tolerance-driven).",
    },
    "scenarioAttitudeFeedbackNoEarth": {
        "file": "scenarioAttitudeFeedbackNoEarth.py",
        "sha256": "c4ee6dc86d101417c662b1d9ee49179788ecb876504e1940b8234a35c"
                  "dd8b8a3",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "extForceTorque present so the same guard fires first; the "
                "no-central-body question stays moot",
    },
    # --- promotion batch: dynamics demos pulled from "covered"/"out" into
    #     live hijack legs. Parity entries exercise the exact upstream file
    #     against the mirror (tol class named per entry); reject entries
    #     prove the loud guard on the exact file. Var names verified against
    #     the v2.12.0 tag sources; sha256 pins the same bytes.
    "scenarioAttitudeFeedbackRWPower": {
        "file": "scenarioAttitudeFeedbackRWPower.py",
        "sha256": "5c75c03625928182e071c596cbec1397f8db2f3c0cef9250e492cb8"
                  "bc37dec46",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": "rwStateEffector", "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 1e-9, "v_BN_N": 1e-9, "sigma_BN": 1e-9,
                "omega_BN_B": 1e-9, "wheels": 1e-4},
        "note": "same hub+3RW MRP closed loop as scenarioAttitudeFeedbackRW "
                "(10 min @ 0.1 s); the added RW power/battery modules are "
                "SysModels (no dynamics) and run identically on both arms, "
                "so the measured RW-class tol holds",
    },
    "scenarioMagneticFieldCenteredDipole": {
        "file": "scenarioMagneticFieldCenteredDipole.py",
        "sha256": "1bd20de7307df901fe39d66d34db67c379c0348b01091cea102e5a4c"
                  "b78cfd00",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 5e-2, "v_BN_N": 1e-4, "sigma_BN": 1e-9,
                "omega_BN_B": 1e-9},
        "note": "default circular-Earth centered dipole READ (no MTB, no "
                "torque): exact-file test of the field-only mag mirror "
                "path; dynamics is 1-orbit dt=10 Kepler, so the basicOrbit "
                "hijack horizon-class tol applies",
    },
    "scenarioTAM": {
        "file": "scenarioTAM.py",
        "sha256": "b77a84d1f9db20fe7057c4c70346ade46c67632cd0a3608ddfa8eb77"
                  "928d989f",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 5e-2, "v_BN_N": 1e-4, "sigma_BN": 1e-9,
                "omega_BN_B": 1e-9},
        "note": "default Jupiter elliptical centered-dipole + magnetometer "
                "READ (no MTB, no torque): exact-file field-only mag mirror "
                "test; central-Jupiter 1-orbit dt=10 -> basicOrbit "
                "horizon-class tol",
    },
    "scenarioAttitudeFeedback": {
        "file": "scenarioAttitudeFeedback.py",
        "sha256": "e7cbfcc8afd9917e44e2fb8f73c45ad1847f91aded94b64bd3468644"
                  "97d4b145",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "ext_torque_msg": "mrpControl.cmdTorqueOutMsg",
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 1e-9, "v_BN_N": 1e-9, "sigma_BN": 1e-6,
                "omega_BN_B": 1e-6},
        "note": "extForceTorque (msg-driven) now mirrored via the new engine "
                "term; attitude loop closes identically on both arms",
    },
    "scenarioAttitudeGuidance": {
        "file": "scenarioAttitudeGuidance.py",
        "sha256": "d08a9fce2c623bedc5ca924d26526cbc0aad0942a40a69cb93d6d653"
                  "df9ec64e",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "guidance-law PD closes through extForceTorque: same guard",
    },
    "scenarioAttitudePointing": {
        "file": "scenarioAttitudePointing.py",
        "sha256": "b0649309324af4ce1f128447affe1e79a951df8499cb8f1c34a327b4"
                  "10cd1ff9",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "pointing PD via extForceTorque (no gravity: liveness is "
                "the attitude motion); guard fires first",
    },
    "scenarioAttitudePointingPy": {
        "file": "scenarioAttitudePointingPy.py",
        "sha256": "8b3189f13a303528dc59354b820a03547332372520a4d844a0156eb2"
                  "b1b5eb34",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "pure-Python pointing PD via extForceTorque (no gravity); "
                "same guard",
    },
    "scenarioAttLocPoint": {
        "file": "scenarioAttLocPoint.py",
        "sha256": "e9dad8be336e3ebb11230cb8ca2582f0256ab1fc9ddb0dbf524fe53f"
                  "bad509bd",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "location-pointing PD via extForceTorque: same guard",
    },
    "scenarioAttGuideHyperbolic": {
        "file": "scenarioAttGuideHyperbolic.py",
        "sha256": "f109be23b0dcaba09e5b50084b3ee3aea3e28d345f5830c55ee1ac58"
                  "9db9ecac",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "hyperbolic guidance PD via extForceTorque: same guard",
    },
    "scenarioSweepingSpacecraft": {
        "file": "scenarioSweepingSpacecraft.py",
        "sha256": "8b6914f651764925c5e422bd4a1316d35459504aa7818261eccfc599"
                  "09be0b06",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "sweep maneuver PD via extForceTorque: same guard",
    },
    "scenarioInertialSpiral": {
        "file": "scenarioInertialSpiral.py",
        "sha256": "460bae249fce3fbb71be6cf55002d0ad6a0c2ef3da01c8c447ee7fe1"
                  "a17df06b",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "spiral guidance PD via extForceTorque: same guard",
    },
    "scenarioGroundLocationImaging": {
        "file": "scenarioGroundLocationImaging.py",
        "sha256": "08ec58465e44aedbc97ad4594f7270869d84564a1fa893f4ca477834"
                  "25110d53",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "imaging task PD via extForceTorque: same guard",
    },
    "scenarioGroundMapping": {
        "file": "scenarioGroundMapping.py",
        "sha256": "bdfddaa600a12ff27272dddc75b4dea421fffadb259a63e789a04748"
                  "0076df77",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": None, "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "reject",
        "tol": {},
        "note": "mapping task PD via extForceTorque: same guard",
    },
    "scenarioAttitudeSteering": {
        "file": "scenarioAttitudeSteering.py",
        "sha256": "8f3b9b0fc8bb0ba4ed9a28aa4553e0905a0d67ec5103b412ab57575b"
                  "67a0bece",
        "sim": "scSim", "sc": "scObject", "task": "simTaskName",
        "rw": "rwStateEffector", "thr_factory": None, "thr_cmd": None,
        "dump": None,
        "mode": "parity",
        "tol": {"r_BN_N": 1e-9, "v_BN_N": 1e-9, "sigma_BN": 1e-6,
                "omega_BN_B": 1e-6, "wheels": 1e-4},
        "note": "extForceTorque (const disturbance) now mirrored; RWs "
                "mirrored",
    },
}

SWAP_BEGIN = "# --- OMNI-HIJACK BEGIN"
SWAP_END = "# --- OMNI-HIJACK END"
DUMP_BEGIN = "# --- OMNI-HIJACK-DUMP BEGIN"
DUMP_END = "# --- OMNI-HIJACK-DUMP END"


def _swap_block(cfg):
    """Render the swap block (indentation matched to the anchor at use)."""
    lines = [
        SWAP_BEGIN + " (generated by hijack_demo.py; only delta vs "
        "upstream pin %s)" % TAG,
        "import os as _os",
        '_hijack_on = _os.environ.get("OMNI_HIJACK", "on") == "on"',
        '_hijack_out = _os.environ.get("HIJACK_OUT", "hijack_out.npz")',
    ]
    if cfg.get("set_central"):
        # Upstream omits the central-body flag (``createEarth()`` defaults to
        # False in this Basilisk); the mirror's stage-1 wants exactly one
        # central. Flag it identically in BOTH arms -- outside the hijack
        # guard, and before the mirror call below.
        lines.append("# Central-body flag upstream omits (deviation: see note).")
        lines.append("{v}.isCentralBody = True".format(v=cfg["set_central"]))
    lines += [
        "if _hijack_on:",
        "    import importlib.util as _ilu",
        "    from omniheyoka.omni_heyoka import OmniVecIntegrator",
        "    # mirror_stock_config comes from the CURRENT repo source, loaded by",
        "    # file path (OMNI_REPO_SWAP). The installed wheel's swap.py copy lags",
        "    # the repo (no env readers: it rejects drag/SRP/mag outright). A path",
        "    # import -- never a sys.path insert -- keeps the wheel supplying",
        "    # omni_heyoka + the compiled extension (conftest.py rationale).",
        '    _swap_p = _os.environ.get("OMNI_REPO_SWAP")',
        "    if not _swap_p:",
        "        raise RuntimeError(",
        '            "OMNI_REPO_SWAP must point at the repo omniheyoka/swap.py")',
        '    _swap_spec = _ilu.spec_from_file_location("_hijack_repo_swap",',
        "                                              _swap_p)",
        "    _swap = _ilu.module_from_spec(_swap_spec)",
        "    _swap_spec.loader.exec_module(_swap)",
        "    mirror_stock_config = _swap.mirror_stock_config",
        "    _oi = OmniVecIntegrator({sc})".format(sc=cfg["sc"]),
    ]
    _call_kwargs = []
    if cfg["thr_factory"]:
        _call_kwargs.append("thr_factory={tf}".format(tf=cfg["thr_factory"]))
    _call_kwargs.append("thr_cmd_msg={cmd}".format(cmd=cfg["thr_cmd"]))
    if cfg.get("ext_torque_msg"):
        _call_kwargs.append("ext_torque_msg={m}".format(
            m=cfg["ext_torque_msg"]))
    if cfg.get("ext_force_body_msg"):
        _call_kwargs.append("ext_force_body_msg={m}".format(
            m=cfg["ext_force_body_msg"]))
    if cfg.get("ext_force_inertial_msg"):
        _call_kwargs.append("ext_force_inertial_msg={m}".format(
            m=cfg["ext_force_inertial_msg"]))
    if _call_kwargs:
        lines.append("    mirror_stock_config({sim}, {sc}, _oi, {kw})".format(
            sim=cfg["sim"], sc=cfg["sc"], kw=", ".join(_call_kwargs)))
    else:
        lines.append("    mirror_stock_config({sim}, {sc}, _oi)".format(
            sim=cfg["sim"], sc=cfg["sc"]))
    lines.append("    {sc}.setIntegrator(_oi)".format(sc=cfg["sc"]))
    lines.append("_hjk_state = {sc}.scStateOutMsg.recorder()".format(
        sc=cfg["sc"]))
    lines.append("{sim}.AddModelToTask({task}, _hjk_state)".format(
        sim=cfg["sim"], task=cfg["task"]))
    if cfg["rw"]:
        lines.append("_hjk_wheels = {rw}.rwSpeedOutMsg.recorder()".format(
            rw=cfg["rw"]))
        lines.append("{sim}.AddModelToTask({task}, _hjk_wheels)".format(
            sim=cfg["sim"], task=cfg["task"]))
    if cfg["dump"]:
        lines.append("_hjk_dump = ({dump}).recorder()".format(
            dump=cfg["dump"]))
        lines.append("{sim}.AddModelToTask({task}, _hjk_dump)".format(
            sim=cfg["sim"], task=cfg["task"]))
    if cfg.get("cap_s"):
        # Bench horizon cap (deviation; the CONFIG note owns the reason).
        # Both arms must clamp identically -- the block sits OUTSIDE the
        # ``if _hijack_on:`` guard so the stock arm is capped too -- and it
        # must run before the upstream StopTime/Execute pair, which the
        # insertion anchor (before ``InitializeSimulation()``) guarantees.
        cap_ns = int(round(float(cfg["cap_s"]) * 1e9))
        lines.append("# Bench horizon cap (deviation: see CONFIG note).")
        lines.append("_hjk_cap_ns = %d" % cap_ns)
        lines.append("_hjk_orig_stop = {sim}.ConfigureStopTime".format(
            sim=cfg["sim"]))
        lines.append("def _hjk_cap_stop(_t):")
        lines.append("    _hjk_orig_stop(min(int(_t), _hjk_cap_ns))")
        lines.append("{sim}.ConfigureStopTime = _hjk_cap_stop".format(
            sim=cfg["sim"]))
    lines.append(SWAP_END)
    return lines


def _dump_block(cfg):
    """Render the npz-dump block (inserted after the last Execute)."""
    lines = [
        DUMP_BEGIN,
        "import numpy as _np2",
        "_hjk_d = {\"t\": _hjk_state.times()}",
        "for _k in (\"r_BN_N\", \"v_BN_N\", \"sigma_BN\", \"omega_BN_B\"):",
        "    _hjk_d[_k] = getattr(_hjk_state, _k)",
    ]
    if cfg["rw"]:
        lines.append("_hjk_d[\"wheelSpeeds\"] = _hjk_wheels.wheelSpeeds")
    if cfg["dump"]:
        lines.append("_hjk_d[\"onTime\"] = _hjk_dump.OnTimeRequest")
    lines.append("_np2.savez(_hijack_out, **_hjk_d)")
    lines.append(DUMP_END)
    return lines


def fetch_upstream(name, cfg):
    """Download (if absent) + sha256-verify the pinned upstream file."""
    UPSTREAM_DIR.mkdir(parents=True, exist_ok=True)
    path = UPSTREAM_DIR / cfg["file"]
    if not path.exists():
        url = BASE_URL + cfg["file"]
        req = urllib.request.Request(url, headers={"User-Agent": "bsk-bench"})
        with urllib.request.urlopen(req, timeout=120) as resp:
            data = resp.read()
        path.write_bytes(data)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != cfg["sha256"]:
        raise ValueError(
            "sha256 mismatch for %s: got %s, pin has %s (upstream moved "
            "or download corrupt -- refusing to run)" % (
                cfg["file"], digest, cfg["sha256"]))
    return path


def generate_port(name, cfg, upstream_text):
    """Port = upstream + swap block + dump block + show_plots flip.

    Insertions only: the swap block goes before the first
    ``InitializeSimulation()`` (end of ``run()`` setup, where every local
    exists); the dump block goes after the last ``ExecuteSimulation()``;
    the ``__main__`` tail is reproduced verbatim except the first ``True``
    (always ``show_plots``) flipped to ``False``. Anchor uniqueness is
    guaranteed by the sha256 pin: the generator only ever runs on
    byte-identical input (verified tag sources have exactly one
    InitializeSimulation() call site each).
    """
    lines = upstream_text.splitlines()
    anchor = next(i for i, ln in enumerate(lines)
                  if ".InitializeSimulation()" in ln)
    indent = lines[anchor][:len(lines[anchor]) - len(lines[anchor].lstrip())]
    swap = ["%s%s" % (indent, ln) for ln in _swap_block(cfg)]
    out = lines[:anchor] + swap + lines[anchor:]
    last_exec = max(i for i, ln in enumerate(out)
                    if ".ExecuteSimulation()" in ln)
    indent2 = out[last_exec][:len(out[last_exec])
                             - len(out[last_exec].lstrip())]
    dump = ["%s%s" % (indent2, ln) for ln in _dump_block(cfg)]
    out = out[:last_exec + 1] + dump + out[last_exec + 1:]
    main_at = next(i for i, ln in enumerate(out)
                   if ln.strip() == 'if __name__ == "__main__":')
    tail = "\n".join(out[main_at:]).replace("True", "False", 1)
    out = out[:main_at] + tail.split("\n")
    return "\n".join(out) + "\n"


def diff_gate(name, upstream_text, port_text):
    """Prove the port differs from upstream ONLY in the marked regions.

    Strip the marked blocks from the port; the remainder must equal upstream
    with exactly the entry-call ``True``->``False`` flip. Returns the marked
    block count. Raises ValueError with a diff excerpt otherwise.
    """
    stripped, in_mark = [], False
    for ln in port_text.splitlines():
        s = ln.strip()
        # startswith: the BEGIN lines carry a "(generated...)" suffix.
        if s.startswith(SWAP_BEGIN) or s.startswith(DUMP_BEGIN):
            in_mark = True
            continue
        if s == SWAP_END or s == DUMP_END:
            in_mark = False
            continue
        if not in_mark:
            stripped.append(ln)
    up_lines = upstream_text.splitlines()
    main_at = next(i for i, ln in enumerate(up_lines)
                   if ln.strip() == 'if __name__ == "__main__":')
    expected = up_lines[:]
    tail = "\n".join(expected[main_at:]).replace("True", "False", 1)
    expected = expected[:main_at] + tail.split("\n")
    if stripped != expected:
        import difflib
        diff = list(difflib.unified_diff(
            expected, stripped, "upstream-expected", "port-stripped",
            lineterm=""))[:24]
        raise ValueError("port differs from upstream outside marked blocks "
                         "(%d/%d lines differ):\n%s" % (
                             sum(1 for _ in diff), len(expected),
                             "\n".join(diff)))
    return sum(1 for ln in port_text.splitlines()
               if ln.strip().startswith((SWAP_BEGIN, DUMP_BEGIN)))


def _dcm(sigma):
    import numpy as _np

    def skew(c):
        return _np.array([[0.0, -c[2], c[1]], [c[2], 0.0, -c[0]],
                          [-c[1], c[0], 0.0]])
    s = _np.asarray(sigma, float)
    s2 = float(s @ s)
    S = skew(s)
    return _np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2


def _run_port(port_path, on, out_npz, timeout_s=1800):
    """Execute the ported demo in one mode. Returns (exit, wall, err_tail)."""
    import os
    env = dict(os.environ)
    env["OMNI_HIJACK"] = "on" if on else "off"
    env["HIJACK_OUT"] = str(out_npz)
    env["MPLBACKEND"] = "Agg"
    # The port file is generated, not hand-placed: it cannot rely on the
    # probe's sys.path surgery. The installed omniheyoka wheel needs its
    # cubesat_12u_sim sibling (repo root) on the path -- same convention as
    # run_all.sh's PYTHONPATH export. Env-level fix keeps the port diff
    # limited to the marked blocks.
    repo_root = str(Path(__file__).resolve().parents[3])
    env["PYTHONPATH"] = (repo_root + os.pathsep
                         + env.get("PYTHONPATH", ""))
    # The omni arm mirrors through the repo's swap.py (current source), not
    # the wheel's lagging copy: same "repo swap.py by path + wheel
    # OmniVecIntegrator" pairing every env probe uses. Env-passed so the
    # generated port stays free of host paths.
    env["OMNI_REPO_SWAP"] = str(Path(__file__).resolve().parents[2]
                                / "omniheyoka" / "swap.py")
    t0 = time.perf_counter()
    try:
        p = subprocess.run([sys.executable, str(port_path)],
                           capture_output=True, text=True,
                           timeout=float(timeout_s), env=env,
                           cwd=str(RESULTS_DIR))
    except subprocess.TimeoutExpired:
        return "timeout", round(time.perf_counter() - t0, 1), ""
    wall = round(time.perf_counter() - t0, 1)
    err = (p.stderr or "")[-1500:]
    return int(p.returncode), wall, err


def _load_npz(path):
    import numpy as _np
    with _np.load(str(path)) as z:
        return {k: _np.asarray(z[k]) for k in z.files}


def _compare(stock, omni, tol):
    """Final-state diffs + dcm_max over shared keys. Returns (diffs, fails)."""
    import numpy as _np
    diffs, fails = {}, []
    for f in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
        if f in stock and f in omni:
            v = float(_np.abs(_np.asarray(omni[f])[-1]
                              - _np.asarray(stock[f])[-1]).max())
            diffs[f] = v
            if f in tol and not v < tol[f]:
                fails.append("%s=%.3e" % (f, v))
    if "wheelSpeeds" in stock and "wheelSpeeds" in omni:
        v = float(_np.abs(_np.asarray(omni["wheelSpeeds"])[-1]
                          - _np.asarray(stock["wheelSpeeds"])[-1]).max())
        diffs["wheels"] = v
        if "wheels" in tol and not v < tol["wheels"]:
            fails.append("wheels=%.3e" % v)
    if "dcm_max" in tol and "sigma_BN" in stock and "sigma_BN" in omni:
        sa, sb = _np.asarray(stock["sigma_BN"]), _np.asarray(omni["sigma_BN"])
        n = min(len(sa), len(sb))
        if n > 0:
            diffs["dcm_max"] = float(max(
                _np.abs(_dcm(sa[k]) - _dcm(sb[k])).max() for k in range(n)))
            if not diffs["dcm_max"] < tol["dcm_max"]:
                fails.append("dcm_max=%.3e" % diffs["dcm_max"])
    return diffs, fails


def _vacuity(name, cfg, stock):
    """Liveness: the stock arm must actually run with live dynamics.

    Translation OR attitude motion counts. Deep-space attitude demos
    (``scenarioAttitudeFeedbackNoEarth``, ``scenarioAttitudePointing*``)
    fly without gravity and translate ~statically by design, so their
    liveness is the attitude motion the demo exists to show; a demo whose
    hub neither translates nor rotates is genuinely vacuous.
    """
    import numpy as _np
    bad = []
    r = _np.asarray(stock["r_BN_N"])
    disp = float(_np.linalg.norm(r[-1] - r[0]))
    s = _np.asarray(stock["sigma_BN"], float)
    att_span = float(_np.abs(s - s[0]).max()) if len(s) else 0.0
    if disp < 0.01 and att_span < 1e-9:
        bad.append("%s.vacuous disp=%.3e att_span=%.3e" % (
            name, disp, att_span))
    if cfg["rw"] and "wheelSpeeds" in stock:
        w = _np.asarray(stock["wheelSpeeds"], float)
        span = float(_np.abs(w - w[0]).max())
        if not span > 1e-3:
            bad.append("%s.vacuous FSW span=%.3e" % (name, span))
    if cfg["dump"] and "onTime" in stock:
        if not bool((_np.asarray(stock["onTime"], float) > 0).any()):
            bad.append("%s.vacuous no-dump-firing" % name)
    return bad


def run_demo(name, timeout_s=1800):
    """Full hijack pipeline for one demo. Returns the ledger payload."""
    cfg = CONFIG[name]
    payload = {"demo": name, "tag": TAG, "mode": cfg["mode"],
               "scenario": cfg["file"], "note": cfg["note"]}
    if cfg["mode"] == "excluded":
        payload.update({"status": "skipped-excluded",
                        "reason": cfg["note"]})
        return payload
    path = fetch_upstream(name, cfg)
    upstream_text = path.read_text(encoding="utf-8")
    port_text = generate_port(name, cfg, upstream_text)
    port_path = RESULTS_DIR / ("%s_omni.py"
                               % Path(cfg["file"]).stem)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    port_path.write_text(port_text, encoding="utf-8")
    payload["port_path"] = str(port_path)
    payload["port_bytes"] = len(port_text.encode("utf-8"))
    try:
        payload["marked_blocks"] = diff_gate(name, upstream_text, port_text)
    except ValueError as exc:
        payload.update({"status": "fail",
                        "verdict": "PORT DIRTY: %s" % exc})
        return payload
    stock_npz = RESULTS_DIR / ("%s_stock.npz"
                               % Path(cfg["file"]).stem)
    omni_npz = RESULTS_DIR / ("%s_omni.npz" % Path(cfg["file"]).stem)
    exit_s, wall_s, _ = _run_port(port_path, False, stock_npz, timeout_s)
    payload["stock_exit"] = exit_s
    payload["stock_wall_s"] = wall_s
    if exit_s != 0:
        payload.update({"status": "fail",
                        "verdict": "STOCK PORT FAILED (exit %s) -- setup "
                                   "broken, not a parity signal" % exit_s})
        return payload
    stock = _load_npz(stock_npz)
    payload["vacuity"] = _vacuity(name, cfg, stock)
    if cfg["mode"] == "reject":
        exit_o, wall_o, err_o = _run_port(port_path, True, omni_npz,
                                          timeout_s)
        payload["omni_exit"] = exit_o
        payload["omni_wall_s"] = wall_o
        loud = ("mirror_stock_config" in err_o)
        payload["omni_reject_loud"] = bool(loud)
        payload["omni_err_tail"] = err_o[-500:]
        ok = bool(loud) and not payload["vacuity"]
        payload["verdict"] = ("reject loud" if ok
                              else "REJECT FAIL loud=%s vacuity=%s" % (
                                  loud, payload["vacuity"]))
        payload["status"] = "ok" if ok else "fail"
        return payload
    exit_o, wall_o, err_o = _run_port(port_path, True, omni_npz, timeout_s)
    payload["omni_exit"] = exit_o
    payload["omni_wall_s"] = wall_o
    if exit_o != 0:
        payload.update({"status": "fail",
                        "verdict": "OMNI PORT FAILED (exit %s): %s" % (
                            exit_o, err_o[-500:])})
        return payload
    # Cold-start accounting: the first omni build in a fresh interpreter pays
    # one-time LLVM/JIT work; over multiples/MC this amortizes away. Re-run
    # the omni arm once more in a fresh process: the persistent heyoka
    # kernel cache makes this the warm number. Parity is judged on the warm
    # run (deterministic -- identical trajectories either way); both walls
    # are recorded so single-shot vs steady-state stay distinct.
    _, wall_warm, _ = _run_port(port_path, True, omni_npz, timeout_s)
    payload["omni_cold_wall_s"] = wall_o
    payload["omni_wall_s"] = wall_warm
    payload["speedup_cold"] = (round(wall_s / wall_o, 3)
                               if wall_o else None)
    payload["speedup"] = (round(wall_s / wall_warm, 3)
                          if wall_warm else None)
    omni = _load_npz(omni_npz)
    diffs, fails = _compare(stock, omni, cfg["tol"])
    payload["diffs"] = diffs
    payload["tol"] = dict(cfg["tol"])
    bad = list(fails) + list(payload["vacuity"])
    payload["verdict"] = ("parity green" if not bad
                          else "PARITY FAIL: %s" % bad)
    payload["status"] = "ok" if not bad else "fail"
    return payload


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Upstream-demo hijack: exact pinned scenario + omni.")
    ap.add_argument("--demo", default=None, choices=list(CONFIG),
                    help="upstream demo key (default: all runnable)")
    ap.add_argument("--list-demos", action="store_true",
                    help="print the hijack registry (no Basilisk needed)")
    ap.add_argument("--timeout", type=float, default=1800.0,
                    help="per-arm timeout [s] (default: 1800)")
    return ap.parse_args(argv)


def list_demos():
    for key in sorted(CONFIG):
        c = CONFIG[key]
        print("%-32s mode=%-8s file=%s" % (key, c["mode"], c["file"]))
        print("    %s" % c["note"])


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.list_demos:
        list_demos()
        return 0
    names = [args.demo] if args.demo else [k for k, c in CONFIG.items()
                                           if c["mode"] != "excluded"]
    if not safe_import_basilisk():
        save_result("hijack_demo",
                    {"status": "skipped-no-basilisk",
                     "reason": "Basilisk not importable on this host"})
        print("hijack_demo: skipped (Basilisk unimportable)")
        return 0
    overall, code, speedups = [], 0, {}
    for name in names:
        try:
            payload = run_demo(name, args.timeout)
        except Exception as exc:
            import traceback as _tb
            payload = {"demo": name, "status": "error",
                       "error": "%s: %s" % (type(exc).__name__, exc),
                       "traceback": _tb.format_exc(limit=15)}
        ledger = "hijack_%s" % Path(CONFIG[name]["file"]).stem
        save_result(ledger, payload)
        log_text(ledger, "[%s] mode=%s status=%s verdict=%s" % (
            name, payload.get("mode"), payload.get("status"),
            payload.get("verdict", payload.get("reason", "?"))))
        sp = payload.get("speedup")
        if isinstance(sp, (int, float)):
            speedups[name] = sp
        print("[hijack] %-28s mode=%-8s status=%-6s verdict=%s" % (
            name, payload.get("mode"), payload.get("status"),
            payload.get("verdict", payload.get("reason", "?"))))
        if sp is not None:
            print("         stock=%.3fs omni_cold=%.3fs omni_warm=%.3fs "
                  "speedup=%s (warm)" % (
                      payload.get("stock_wall_s"),
                      payload.get("omni_cold_wall_s"),
                      payload.get("omni_wall_s"), sp))
        overall.append((name, payload.get("status")))
        if payload.get("status") not in ("ok", "skipped-excluded",
                                         "skipped-no-basilisk"):
            code = 1
    if len(names) > 1:
        import math as _math
        vals = [v for v in speedups.values() if v and v > 0]
        summary = {"n_timed": len(vals),
                   "geomean_speedup": round(_math.exp(
                       sum(_math.log(v) for v in vals) / len(vals)), 3)
                   if vals else None,
                   "per_demo_speedup": dict(speedups)}
        save_result("hijack_demo",
                    {"status": "ok" if code == 0 else "fail",
                     "demos": {n: s for n, s in overall},
                     "timing_summary": summary})
        print("hijack_demo verdict: %s"
              % ("all green" if code == 0 else "FAILURES PRESENT"))
        if vals:
            print("hijack timing: geomean speedup %.3f over %d demo(s): %s"
                  % (summary["geomean_speedup"], len(vals),
                     ", ".join("%s=%s" % kv
                               for kv in sorted(speedups.items()))))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
