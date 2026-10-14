"""WP-H probe (closed-loop fuel, single-tank scope): PD + burn + depletion in the loop.

Mission leg (matched dt=0.1, T=600 s, ONE leg pair -- not a sweep):
  hub (740 kg dry + 10 kg prop) + 3x HR16 wheels under a pure-Python PD
  attitude law (same K=3.5/P=30/u_max=0.2 PD class as the swap_p1 rw3_pd
  cell) + torque-balanced stationkeeping burn (single TEST_Thruster, 5 N,
  duty 0.5 fresh-write driver, line of action through the hub CoM so the
  0.2 N.m wheels keep authority) + ONE CENTERED depleting fuel tank
  (the fuel_centered precedent: mass/I feedback isolated, c_B == 0
  exactly) + central gravity. Stock-RK4 arm vs omni-vec arm
  (OmniVecIntegrator via omniheyoka.swap.mirror_stock_config).

Leg-design note (measured, recorded not guessed): the first candidate leg
(unbalanced fuel-cell thruster geometry + offset tank) DIVERGES under PD:
r=1.1, v=6.1e-3, sigma=2.3e-2, dcm=0.116, wheels=0.24 at 600 s with MRP
27/27 and bit-exact fuel. Mechanism: the 2.5 N.m-class burn torque
saturates the 0.2 N.m wheels, and saturated rail-chatter amplifies
micro-seeds through the wheel/hub coupling (O(1) wheel split); the offset
tank adds a stock-only c-drift torque ~F*|dc| ~ 1e-3 N.m the static omni
arm never sees. The accepted leg removes both mechanisms by construction
(torque-free burn line + centered tank), so the loop tests exactly what
WP-H claims: PD control + stationkeeping firings + shrinking mass/inertia
with depletion IN the loop. The rejected candidate is RECORDED IN THE JSON
as a documented non-parity cell (`cells.rejected`, status
"rejected-non-parity" with its divergence numbers, NOT parity-gated); it
is NOT a parity failure of the vec path (open-loop fuel_offset with the
same geometry is green).

Dynamics envelope: hub + 3RW + staircase thruster + central gravity + ONE
fuel tank -- all mirrored today; anything else is LOUDLY rejected by
mirror_stock_config (kept that way: no drag/SRP/extra effectors here).

Rotation is reported BOTH ways: sigma-vector final diff (switching class --
the PD law stabilizes the leg so no shadow-set crossing is expected) AND
the switch-invariant DCM trajectory-max (fuel-cell class, forensics-proof
if the leg ever tumbles). Wheels carry the coupled-state switching bound
(test_swap_multi class: wheel deltas track hub-omega through the spin
coupling; a double-step would diverge O(1)). Fuel mass is near-bit-exact
(depletion law is algebraic in achieved duties).

Vacuity guards: burned > 0.1 kg (burn live) and both-arm translational
displacement > 1e5 m (trajectories live, not zeros-vs-zeros). MRP switch
totals recorded per arm (+-2 gate, test_swap_multi mechanism).

Run: ../../env/bin/python audit_bundle/probes/closed_loop_fuel.py
Writes: audit_bundle/results/closed_loop_fuel.json
Host-only (needs Basilisk macOS build + built omni_heyoka).
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # Python/ for
# omniheyoka + cubesat_12u_sim sibling imports (BUILD_STATUS recipe)
from common import log_text, save_result, safe_import_basilisk  # noqa: E402

DT = 0.1
T_LEG = 600.0
TICKS = int(round(T_LEG / DT))
MU = 3.986004418e14
MASS_DRY = 740.0
PROP0 = 10.0
I6 = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
R_TB = [0.0, 0.0, 0.0]  # centered tank: fuel_centered precedent
# (mass/I feedback isolated, c_B == 0 exactly -- no stock-only c-torque).
THR_LOC = [0.0, 0.0, 1.0]  # torque-balanced stationkeeping line: thrust
THR_DIR = [0.0, 0.0, -1.0]  # axis passes through the hub CoM (r x F == 0).
# Rejected-candidate geometry (M17: unbalanced fuel-cell thruster geometry
# + offset tank -- the leg that DIVERGES under PD; recorded in JSON as a
# documented non-parity cell, never parity-gated).
REJ_R_TB = [0.3, 0.0, 0.2]
REJ_THR_LOC = [0.5, 0.0, 1.0]
REJ_THR_DIR = [0.0, 0.0, -1.0]
REJ_STATUS = "rejected-non-parity"

# Pre-registered gates (swap_p1 classes, margins per precedent):
#   r/v/omega/dcm: fuel-cell switching class (5 N, 600 s burn).
#   sigma: thruster switching class TOL_THR_SIG (PD stabilizes the leg).
#   wheels: coupled-state switching bound (test_swap_multi class).
TOL_R = 2e-2
TOL_V = 1e-4
TOL_SIG = 2e-4
TOL_OM = 1e-4
TOL_DCM = 2e-3
TOL_WHEEL = 1e-4
TOL_FUEL_REL = 1e-9
TOL_BURNED = 0.1
TOL_DISP = 1e5


def _base_sim(tag):
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                    simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = MASS_DRY
    sc.hub.IHubPntBc_B = [list(r) for r in I6]
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R, 347.8 * macros.D2R,
                                85.3 * macros.D2R)
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    sc.hub.sigma_BNInit = [0.1, -0.2, 0.3]
    sc.hub.omega_BN_BInit = [0.01, -0.02, 0.03]
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    return sim, sc, list(r0)


def _pd_driver(K=3.5, P=30.0, u_max=0.2):
    """PD attitude law (swap_p1 rw3_pd class, verbatim gains)."""
    import numpy as np
    from Basilisk.architecture import messaging, sysModel

    class PD(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()
            self.sc_in = messaging.SCStatesMsgReader()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            s = np.asarray(self.sc_in().sigma_BN, float)
            w = np.asarray(self.sc_in().omega_BN_B, float)
            u = np.clip(-K * s - P * w, -u_max, u_max)
            m = [0.0] * 36
            for i in range(3):
                m[i] = float(u[i])
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    return PD()


def _burn_driver(duty=0.5):
    """Stationkeeping burn profile: staircase fresh-write driver at fixed
    duty (the _env_fuel_cell burn fixture -- proven parity class)."""
    from Basilisk.architecture import messaging, sysModel

    class Burn(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = duty * DT
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)
    return Burn()


def _dcm(sigma):
    """MRP->DCM (avsEigenMRP semantics, DESIGN constraint 2)."""
    import numpy as np

    def skew(c):
        return np.array([[0., -c[2], c[1]], [c[2], 0., -c[0]],
                         [-c[1], c[0], 0.]])
    s = np.asarray(sigma, float)
    s2 = float(s @ s)
    S = skew(s)
    return np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2


def _build(use_omni, r_tb=None, thr_loc=None, thr_dir=None):
    """One arm of the closed-loop leg. Returns handles dict.

    Defaults are the accepted leg (centered tank + torque-free burn line);
    the rejected candidate passes the unbalanced geometry explicitly.
    """
    import numpy as np  # noqa: F401
    from Basilisk.simulation import (fuelTank, reactionWheelStateEffector,
                                     thrusterDynamicEffector)
    from Basilisk.utilities import simIncludeRW, simIncludeThruster
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config

    r_tb = list(R_TB) if r_tb is None else list(r_tb)
    thr_loc = list(THR_LOC) if thr_loc is None else list(thr_loc)
    thr_dir = list(THR_DIR) if thr_dir is None else list(thr_dir)

    sim, sc, r0 = _base_sim(("omni-" if use_omni else "rk-") + "clfuel")
    # 3 wheels (PD-controlled).
    fac = simIncludeRW.rwFactory()
    for gs in ([1., 0, 0], [0, 1., 0], [0, 0, 1.]):
        fac.create("Honeywell_HR16", list(gs), maxMomentum=50.0, Omega=100.0)
    rw = reactionWheelStateEffector.ReactionWheelStateEffector()
    fac.addToSpacecraft("rw", rw, sc)
    sim.AddModelToTask("t", rw)
    pd = _pd_driver()
    sim.AddModelToTask("t", pd)
    rw.rwMotorCmdInMsg.subscribeTo(pd.out)
    pd.sc_in.subscribeTo(sc.scStateOutMsg)
    # Stationkeeping thruster + depleting tank (_env_fuel_cell wiring).
    tf = simIncludeThruster.thrusterFactory()
    tf.create("TEST_Thruster", list(thr_loc), list(thr_dir), MaxThrust=5.0,
              cutoffFrequency=1000.0, useMinPulseTime=False)
    thr = thrusterDynamicEffector.ThrusterDynamicEffector()
    tf.addToSpacecraft("thr", thr, sc)
    sim.AddModelToTask("t", thr, 2)
    burn = _burn_driver(0.5)
    sim.AddModelToTask("t", burn)
    thr.cmdsInMsg.subscribeTo(burn.out)
    tank = fuelTank.FuelTank()
    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = PROP0
    tank.setTankModel(model)
    tank.setNameOfMassState("fuelMass")
    tank.setR_TB_B(list(r_tb))
    tank.addThrusterSet(thr)
    sc.addStateEffector(tank)
    sim.AddModelToTask("t", tank)
    oi = None
    if use_omni:
        oi = OmniVecIntegrator(sc)
        for fn in ("getMRPSwitchCount", "getBuildSeconds",
                   "getStepCountForObject", "getIntegrateCallCount"):
            if not callable(getattr(oi, fn, None)):
                raise RuntimeError(
                    f"closed_loop_fuel: oi.{fn} missing -- vec C++ spec "
                    f"not applied to this wheel build.")
        mirror_stock_config(sim, sc, oi, thr_factory=tf,
                            thr_cmd_msg=burn.out)
        sc.setIntegrator(oi)
    scLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", scLog)
    spdLog = rw.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("t", spdLog)
    return {"sim": sim, "sc": sc, "r0": r0, "scLog": scLog,
            "spdLog": spdLog, "oi": oi}


def _run(sim):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T_LEG))
    sim.ExecuteSimulation()


def _final_diff(a, b, fields):
    import numpy as np
    out = {}
    for f in fields:
        xa = np.asarray(getattr(a, f))[-1]
        xb = np.asarray(getattr(b, f))[-1]
        out[f] = float(np.abs(xa - xb).max())
    return out


def _loadavg():
    """Host load averages for build_s provenance (L9). Never raises."""
    try:
        import os
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return "unavailable"


def _leg():
    """Stock-vs-omni closed-loop leg. Returns the cell dict."""
    import numpy as np
    arm_s = _build(False)
    _run(arm_s["sim"])
    arm_o = _build(True)
    _run(arm_o["sim"])
    oi = arm_o["oi"]
    log_s, log_o = arm_s["scLog"], arm_o["scLog"]
    cell: dict = {"ticks": TICKS, "t_leg": T_LEG,
                  "fixture": {"mHub_dry": MASS_DRY, "prop0": PROP0,
                              "r_TB_B": list(R_TB), "thrust": 5.0,
                              "thr_loc": list(THR_LOC),
                              "thr_dir": list(THR_DIR),
                              "duty": 0.5, "pd": {"K": 3.5, "P": 30.0,
                                                  "u_max": 0.2},
                              "wheels": 3, "ic": "tumbling-SIG0/OM0"}}
    cell.update(_final_diff(log_s, log_o,
                            ["r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"]))
    sa = np.asarray(log_s.sigma_BN)
    sb = np.asarray(log_o.sigma_BN)
    cell["dcm_max"] = float(max(np.abs(_dcm(sa[k]) - _dcm(sb[k])).max()
                                for k in range(len(sa))))
    wa = np.asarray(arm_s["spdLog"].wheelSpeeds)[-1]
    wb = np.asarray(arm_o["spdLog"].wheelSpeeds)[-1]
    cell["wheels"] = float(np.abs(wa - wb).max())
    fa = float(np.asarray(
        arm_s["sc"].dynManager.getStateObject("fuelMass").getState()
        ).ravel()[0])
    fb = float(np.asarray(
        arm_o["sc"].dynManager.getStateObject("fuelMass").getState()
        ).ravel()[0])
    cell["fuel_stock"] = fa
    cell["fuel_omni"] = fb
    cell["fuel_rel_err"] = abs(fa - fb) / fa if fa else 0.0
    cell["fuel_burned"] = PROP0 - fa
    cell["mrp_stock"] = int(np.asarray(
        getattr(log_s, "MRPSwitchCount")).ravel()[-1])
    cell["mrp_omni"] = int(oi.getMRPSwitchCount())
    cell["disp_stock"] = float(np.linalg.norm(
        np.asarray(log_s.r_BN_N)[-1] - np.asarray(arm_s["r0"], float)))
    cell["disp_omni"] = float(np.linalg.norm(
        np.asarray(log_o.r_BN_N)[-1] - np.asarray(arm_o["r0"], float)))
    cell["nPars"] = int(oi.nPars())
    cell["nStates"] = int(oi.nStates())
    cell["build_s"] = round(float(oi.getBuildSeconds()), 4)
    # L14 step-count audit (single-spacecraft path): exactly one integrate
    # step and one integrate call per tick -- no double-step.
    cell["steps"] = int(oi.getStepCountForObject(0))
    cell["calls"] = int(oi.getIntegrateCallCount())
    # L9 build provenance: host contention inflates build_s (this config
    # has no JIT tier -- build_s is recorded, never gated).
    cell["loadavg"] = _loadavg()
    return cell


def _rejected_leg():
    """M17 rejected candidate (unbalanced geometry) as a documented
    NON-PARITY cell: status marker + divergence numbers, never gated
    against the parity bounds (it exceeds them by construction)."""
    import numpy as np
    arm_s = _build(False, REJ_R_TB, REJ_THR_LOC, REJ_THR_DIR)
    _run(arm_s["sim"])
    arm_o = _build(True, REJ_R_TB, REJ_THR_LOC, REJ_THR_DIR)
    _run(arm_o["sim"])
    oi = arm_o["oi"]
    log_s, log_o = arm_s["scLog"], arm_o["scLog"]
    cell: dict = {
        "status": REJ_STATUS,
        "scope": ("documented divergence, NOT a parity cell: the 2.5 N.m-class "
                  "burn torque saturates the 0.2 N.m wheels and the offset tank "
                  "adds a stock-only c-drift torque -- exceeds the fuel-cell "
                  "switching bounds by construction; open-loop fuel_offset with "
                  "the same geometry is green"),
        "ticks": TICKS, "t_leg": T_LEG,
        "fixture": {"mHub_dry": MASS_DRY, "prop0": PROP0,
                    "r_TB_B": list(REJ_R_TB), "thrust": 5.0,
                    "thr_loc": list(REJ_THR_LOC),
                    "thr_dir": list(REJ_THR_DIR),
                    "duty": 0.5, "pd": {"K": 3.5, "P": 30.0, "u_max": 0.2},
                    "wheels": 3, "ic": "tumbling-SIG0/OM0"}}
    cell.update(_final_diff(log_s, log_o,
                            ["r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"]))
    sa = np.asarray(log_s.sigma_BN)
    sb = np.asarray(log_o.sigma_BN)
    cell["dcm_max"] = float(max(np.abs(_dcm(sa[k]) - _dcm(sb[k])).max()
                                for k in range(len(sa))))
    wa = np.asarray(arm_s["spdLog"].wheelSpeeds)[-1]
    wb = np.asarray(arm_o["spdLog"].wheelSpeeds)[-1]
    cell["wheels"] = float(np.abs(wa - wb).max())
    fa = float(np.asarray(
        arm_s["sc"].dynManager.getStateObject("fuelMass").getState()
        ).ravel()[0])
    fb = float(np.asarray(
        arm_o["sc"].dynManager.getStateObject("fuelMass").getState()
        ).ravel()[0])
    cell["fuel_stock"] = fa
    cell["fuel_omni"] = fb
    cell["fuel_rel_err"] = abs(fa - fb) / fa if fa else 0.0
    cell["fuel_burned"] = PROP0 - fa
    cell["mrp_stock"] = int(np.asarray(
        getattr(log_s, "MRPSwitchCount")).ravel()[-1])
    cell["mrp_omni"] = int(oi.getMRPSwitchCount())
    cell["disp_stock"] = float(np.linalg.norm(
        np.asarray(log_s.r_BN_N)[-1] - np.asarray(arm_s["r0"], float)))
    cell["disp_omni"] = float(np.linalg.norm(
        np.asarray(log_o.r_BN_N)[-1] - np.asarray(arm_o["r0"], float)))
    cell["nPars"] = int(oi.nPars())
    cell["nStates"] = int(oi.nStates())
    cell["build_s"] = round(float(oi.getBuildSeconds()), 4)
    cell["steps"] = int(oi.getStepCountForObject(0))
    cell["calls"] = int(oi.getIntegrateCallCount())
    cell["loadavg"] = _loadavg()
    # Scope-boundary pin: which parity bounds this leg exceeds (the test
    # cites at least one -- the boundary is explicit, not implied).
    cell["exceeds"] = []
    for f, tol in (("r_BN_N", TOL_R), ("v_BN_N", TOL_V),
                   ("sigma_BN", TOL_SIG), ("omega_BN_B", TOL_OM),
                   ("dcm_max", TOL_DCM), ("wheels", TOL_WHEEL)):
        if not cell[f] < tol:
            cell["exceeds"].append(f"{f}={cell[f]:.3e} >= {tol:.1e}")
    return cell


def main() -> int:
    if not safe_import_basilisk():
        save_result("closed_loop_fuel",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    try:
        payload: dict = {"status": "ok", "cells": {}, "dt": DT,
                         "t_leg": T_LEG, "ticks": TICKS,
                         "loadavg": _loadavg()}
        payload["cells"]["leg"] = _leg()
        # M17: rejected fixture recorded alongside (documented non-parity;
        # never enters the parity gates below, which bind cells["leg"] only).
        payload["cells"]["rejected"] = _rejected_leg()
        payload["loadavg_end"] = _loadavg()
        fails = []
        c = payload["cells"]["leg"]
        for f, tol in (("r_BN_N", TOL_R), ("v_BN_N", TOL_V),
                       ("sigma_BN", TOL_SIG), ("omega_BN_B", TOL_OM),
                       ("dcm_max", TOL_DCM), ("wheels", TOL_WHEEL)):
            if not c[f] < tol:
                fails.append(f"leg.{f}={c[f]:.3e}")
        if not c["fuel_rel_err"] < TOL_FUEL_REL:
            fails.append(f"leg.fuel_rel_err={c['fuel_rel_err']:.3e}")
        if not abs(c["mrp_omni"] - c["mrp_stock"]) <= 2:
            fails.append(f"leg.mrp omni={c['mrp_omni']} "
                         f"stock={c['mrp_stock']}")
        if not c["fuel_burned"] > TOL_BURNED:
            fails.append(f"leg.vacuous burned={c['fuel_burned']:.3e}")
        for arm in ("disp_stock", "disp_omni"):
            if not c[arm] > TOL_DISP:
                fails.append(f"leg.vacuous {arm}={c[arm]:.3e}")
        payload["verdict"] = ("parity green" if not fails
                              else f"PARITY FAIL: {fails}")
        if fails:
            payload["status"] = "fail"
    except Exception as exc:
        from common import write_exception  # noqa: E402
        write_exception("closed_loop_fuel", exc)
        print(f"closed_loop_fuel ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("closed_loop_fuel", payload)
    c = payload["cells"]["leg"]
    log_text("closed_loop_fuel",
             "[leg] " + " ".join(
                 f"{k}={v:.2e}" if isinstance(v, float) else f"{k}={v}"
                 for k, v in c.items()))
    r = payload["cells"]["rejected"]
    log_text("closed_loop_fuel",
             "[rejected:%s] " % r.get("status") + " ".join(
                 f"{k}={v:.2e}" if isinstance(v, float) else f"{k}={v}"
                 for k, v in r.items() if k not in ("scope", "exceeds"))
             + " exceeds=%s" % (r.get("exceeds"),))
    print(f"closed_loop_fuel verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
