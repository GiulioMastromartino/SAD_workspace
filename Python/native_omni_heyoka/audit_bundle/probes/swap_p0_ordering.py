"""Phase-0 probe 4 (swap): stock tick-ordering record.

Static record (pinned to upstream v2.12.0 sources, same version as the
host BSK build) of the exact per-tick call order an OmniVecIntegrator
must reproduce, plus runtime confirmation of the default integrator.
Upstream files:
  src/simulation/dynamics/spacecraft/spacecraft.cpp      (Reset/UpdateState/
      initializeDynamics/equationsOfMotion/pre+postIntegration)
  src/simulation/dynamics/_GeneralModuleFiles/dynamicObject.cpp
      (setIntegrator/integrateState ownership + ordering)
  src/simulation/dynamics/reactionWheels/reactionWheelStateEffector.cpp
      (registerStates names, saturation rule, coupled spin, friction)

Run: ../../env/bin/python audit_bundle/probes/swap_p0_ordering.py
Writes: audit_bundle/results/swap_p0_ordering.json
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import log_text, save_result, safe_import_basilisk  # noqa: E402

BASE = ("https://raw.githubusercontent.com/AVSLab/basilisk/"
        "v2.12.0/src/simulation/dynamics")


def main() -> int:
    payload: dict = {
        "status": "ok",
        "upstream_tag": "v2.12.0",
        "sources": {
            "spacecraft.cpp": f"{BASE}/spacecraft/spacecraft.cpp",
            "dynamicObject.cpp": f"{BASE}/_GeneralModuleFiles/dynamicObject.cpp",
            "reactionWheelStateEffector.cpp":
                f"{BASE}/reactionWheels/reactionWheelStateEffector.cpp",
        },
        # Verified line-level against the fetched sources (2026-10-12).
        "ctor": "Spacecraft() sets integrator = new svIntegratorRK4(this) "
                "-> the stock DEFAULT integrator is RK4 (the swap target).",
        "UpdateState_order": [
            "gravField.UpdateState",
            "integrateState (= preIntegration[all dynPtrs] -> "
            "integrator->integrate(timeBefore, timeStep) -> "
            "postIntegration[all dynPtrs]; skipped if isDynamicsSynced)",
            "readOptionalRefMsg (att/trans ref override AFTER integration)",
            "gravField.updateInertialPosAndVel",
            "writeOutputStateMessages (+ each stateEffector)",
        ],
        "Reset_order": [
            "hub.validateConfiguration",
            "gravField.Reset",
            "initializeDynamics (= createProperty mass/time props -> "
            "gravField.registerProperties -> hub.registerStates -> "
            "effector registerStates -> linkInStates -> updateSCMassProps "
            "-> B/C r/v correction -> effector linkInStates -> "
            "readOptionalRefMsg -> equationsOfMotion(0, 1))",
            "gravField.updateInertialPosAndVel",
            "writeOutputStateMessages",
        ],
        "postIntegration_order": [
            "updateSCMassProps",
            "DV accumulators (CN_B/BN_B/CN_N) + nonConservativeAccel + "
            "omegaDot finite-difference",
            "computeEnergyMomentum",
            "hub.modifyStates (MRP shadow switch) + effector modifyStates",
            "calcForceTorqueFromStateEffectors",
        ],
        "setIntegrator_rules": [
            "takes ownership (destructor deletes); warns (not errors) if "
            "isDynamicsSynced",
            "bskError on null; bskError unless new integrator was created "
            "with THIS DynamicObject (dynPtrs[0] check, no dynamic_cast)",
            "old integrator's dynPtrs take priority (multi-sc sync preserved)",
        ],
        "integrate_args": "integrate(timeBefore, timeStep) — NOT the tick "
                          "time; the swap must use exactly these.",
        "rw_registered_states": {
            "reactionWheelOmegas": "numRW x 1 (always)",
            "reactionWheelThetas": "numRWJitter x 1 (only if any jitter "
                                   "wheel; JitterSimple or JitterFullyCoupled)",
        },
        "stock_saturation_rule": "|Omega| >= Omega_max && Omega_max > 0 && "
            "Omega*u_cmd >= 0 -> cut (ConfigureRWRequests; matches the "
            "omni direction-aware cut semantics).",
        "stock_spin_coupling": "OmegasDot = (u_current+frictionTorque)/Js "
            "- gs^T*omegaDot (coupled, matches omni Update-5).",
        "stock_rw_matrixD": "matrixD -= Js*gs*gs^T (total-inertia "
            "semantics, matches omni Update-5).",
        "balanced_mass_props": "BalancedWheels/JitterSimple contribute ZERO "
            "mass props (mEff=0); only JitterFullyCoupled adds mass/inertia.",
        "maxWheelAcceleration": "CLAMP (not log-only): |OmegasDot| capped "
            "with BSK_WARNING; u_current back-solved for consistency. "
            "Answers the WP1 log-vs-clamp open question from source.",
        "hub_fast_path": "useHubOnlyFastPath (no effectors + zero offset) "
            "takes computeHubOnlyDerivatives; pointMassTranslationalOnly is "
            "a separate translational path.",
        "diffusion": "equationsOfMotionDiffusion delegates to "
            "equationsOfMotion (no real diffusion model) — stochastic "
            "integrators stay out of scope as planned.",
    }
    if safe_import_basilisk():
        from Basilisk.simulation import spacecraft  # noqa: E402
        sc = spacecraft.Spacecraft()
        payload["runtime_default_integrator_proxy"] = type(
            sc.integrator).__name__
    else:
        payload["runtime_default_integrator_proxy"] = "skipped (no Basilisk)"
    payload["verdict"] = "ordering + ownership + RW-source facts recorded"
    save_result("swap_p0_ordering", payload)
    log_text("swap_p0_ordering", f"verdict={payload['verdict']}")
    print(f"probe4 verdict: {payload['verdict']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
