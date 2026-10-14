"""WP25-A4 probe (swap): two-spacecraft synced sync path.

Cells (all matched-dt DT=0.1, pre-registered tolerances in verdicts):
  leg_shared  two objects, IDENTICAL configs (shared symbolic graph),
              distinct ICs, hub+3RW+staircase-thruster mission leg (600 s)
  leg_split   same mission leg, but object B differs (2 wheels, not 3), so
              per-object graphs are built (fingerprints differ)
  reject_unmirrored  slot 0 mirrored explicitly + follower synced + 2 ticks:
              the C++ build must fail LOUDLY naming object 1 (object B must
              never silently inherit object A's graph)

Each leg runs a stock-stock arm (both objects on stock RK4, synced) vs an
omni arm (ONE OmniVecIntegrator with both slots mirrored via
mirror_sync_configs, setIntegrator on the leader, then
scA.syncDynamicsIntegration(scB)). Per-object final-state deltas are gated
in the switching class (same class as the swap_p1 THR cells at dt=0.1);
wheels carry a coupled-state switching bound (wheel deltas track hub-omega
deltas through the gs^T spin coupling, while a double-step diverges O(1),
so the margin is honest). Rotation is compared by DCM trajectory-max
(switch-invariant: a 600 s tumble crosses the MRP shadow set, where raw
sigma-vector comparison is meaningless -- same metric as the swap_p1 fuel
cells); sigma-vector diffs are recorded for forensics only.

The step-count audit (per-object steps == ticks AND integrate calls ==
ticks) proves no double-step on synced objects. Per-object build seconds +
shared_graph flag + config fingerprints are recorded (probe-first on build
cost; the Ground-8 JIT tier < 5.0 s binds every build). MRP switch totals
per object are recorded vs stock (+-2 gate; the B2.5 +-1 mechanism on a new
unmeasured config, kept honest-loose until a host measures it).

Ordering compliance (swap_p0_ordering.py:47,73): stock runs
preIntegration[all dynPtrs] -> integrate(timeBefore, timeStep) ->
postIntegration[all dynPtrs], skipped if isDynamicsSynced. The probe
asserts scB.isDynamicsSynced (and NOT scA's) right after the sync call, so
a reversed sync direction fails loudly on host instead of silently.

Run: ../../env/bin/python audit_bundle/probes/swap_multi.py
Writes: audit_bundle/results/swap_multi.json
Host-only (needs Basilisk build + built omni_heyoka).
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
MASS = 750.0
I6 = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]

# Pre-registered switching-class bounds at dt=0.1 (same class as the swap_p1
# THR cells: the stock-RK4-across-the-valve-kink residual dominates).
# LEG-LENGTH SCALING (measured 2026-10-14, recorded not guessed): the 30 s
# swap_p1 THR cells read r=3.05e-4 / v=4.06e-6; the 600 s (20x) mission leg
# reads r=4.49e-3 (15x) / v=2.36e-5 (6x) on objA — sublinear accumulation of
# the same per-tick systematic residual (~4e-9 m/s: 4e-9*6000 = 2.4e-5 v),
# relative error ~1e-9 vs 3.7e6 m displacement, no divergent mechanism
# (rotation/wheels/steps/MRP exact; objB green at 1.6e-3/8.4e-6). Bounds carry
# >=5x margins on the measured maxima (repo discipline, cf. swap_p1).
TOL_R = 2.5e-2  # measured max 4.49e-3 -> 5.6x margin
TOL_V = 1.2e-4  # measured max 2.36e-5 -> 5.1x margin
TOL_OM = 1e-5
TOL_DCM = 2e-3  # switch-invariant rotation metric (fuel-cell class)
TOL_WHEEL = 1e-4  # coupled-state switching bound (see module docstring)
TOL_BUILD_S = 5.0  # Ground-8 JIT tier, same as the swap_p1 hub gate
TOL_IC_SEP = 1e5  # distinct-ICs proof (m)
TOL_LIVE_DISP = 1e5  # vacuity: every object displaced on both arms (m)

OE_A = dict(a=1.0e7, e=0.01, i=33.3, Omega=48.2, omega=347.8, f=85.3)
OE_B = dict(a=1.02e7, e=0.02, i=41.7, Omega=12.9, omega=301.4, f=125.3)
SIG_A = [0.1, -0.2, 0.3]
OM_A = [0.01, -0.02, 0.03]
SIG_B = [-0.3, 0.15, 0.05]
OM_B = [0.02, 0.01, -0.015]


def _oe_rv(oe):
    from Basilisk.utilities import macros, orbitalMotion
    o = orbitalMotion.ClassicElements()
    o.a, o.e = oe["a"], oe["e"]
    o.i = oe["i"] * macros.D2R
    o.Omega, o.omega, o.f = (oe["Omega"] * macros.D2R,
                             oe["omega"] * macros.D2R,
                             oe["f"] * macros.D2R)
    return orbitalMotion.elem2rv(MU, o)


def _const_rw_driver(u, n):
    from Basilisk.architecture import messaging, sysModel

    class Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            m = [0.0] * 36
            for i in range(n):
                m[i] = u
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    return Driver()


def _thr_driver(duty):
    from Basilisk.architecture import messaging, sysModel

    class Thr(sysModel.SysModel):
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
    return Thr()


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


def _build_pair(tag, use_omni, wheels_b=3, sync=True):
    """Two-spacecraft sim; omni arm mirrors BOTH objects into ONE oi.

    wheels_b=3 -> identical configs (shared graph); wheels_b=2 -> object B
    differs (per-object graphs). sync=False leaves the pair unsynced (the
    negative cell syncs explicitly after installing its integrator).
    Returns dict of handles.
    """
    from Basilisk.simulation import (reactionWheelStateEffector,
                                     spacecraft,
                                     thrusterDynamicEffector)
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                     simIncludeGravBody, simIncludeRW,
                                     simIncludeThruster)
    from omniheyoka.swap import mirror_sync_configs

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True

    objects = []
    for tag_i, oe, sig, om, u, nw in (
            ("A", OE_A, SIG_A, OM_A, 0.5, 3),
            ("B", OE_B, SIG_B, OM_B, -0.3, wheels_b)):
        sc = spacecraft.Spacecraft()
        sc.ModelTag = f"{tag}-{tag_i}"
        sc.hub.mHub = MASS
        sc.hub.IHubPntBc_B = [list(r) for r in I6]
        r0, v0 = _oe_rv(oe)
        sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
        sc.hub.sigma_BNInit = list(sig)
        sc.hub.omega_BN_BInit = list(om)
        sim.AddModelToTask("t", sc)
        grav.addBodiesTo(sc)
        fac = simIncludeRW.rwFactory()
        axes = [[1., 0, 0], [0, 1., 0], [0, 0, 1.]][:nw]
        for gs in axes:
            fac.create("Honeywell_HR16", list(gs), maxMomentum=50.0,
                       Omega=100.0)
        rw = reactionWheelStateEffector.ReactionWheelStateEffector()
        fac.addToSpacecraft("rw", rw, sc)
        sim.AddModelToTask("t", rw)
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", [0.5, 0, 1.0], [0, 0, -1.0], MaxThrust=1.0,
                  cutoffFrequency=1000.0, useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        rw_drv = _const_rw_driver(u, nw)
        sim.AddModelToTask("t", rw_drv)
        rw.rwMotorCmdInMsg.subscribeTo(rw_drv.out)
        thr_drv = _thr_driver(0.5)
        sim.AddModelToTask("t", thr_drv)
        thr.cmdsInMsg.subscribeTo(thr_drv.out)
        objects.append({"sc": sc, "rw": rw, "tf": tf, "thr": thr,
                        "thr_cmd": thr_drv.out, "r0": list(r0)})

    scA, scB = objects[0]["sc"], objects[1]["sc"]
    oi = None
    if use_omni:
        from omniheyoka.omni_heyoka import OmniVecIntegrator
        oi = OmniVecIntegrator(scA)
        for fn in ("getStepCountForObject", "getIntegrateCallCount",
                   "sharedGraphUsed", "configFingerprintForObject",
                   "getBuildSecondsForObject", "getMRPSwitchCountForObject"):
            if not callable(getattr(oi, fn, None)):
                raise RuntimeError(
                    f"swap_multi: oi.{fn} missing -- WP25-A4 C++ spec not "
                    f"applied to this wheel build.")
        mirror_sync_configs(sim, oi, [
            {"sc": scA, "rw_models": [objects[0]["rw"]],
             "thr_models": [objects[0]["thr"]],
             "thr_factory": objects[0]["tf"],
             "thr_cmd_msg": objects[0]["thr_cmd"],
             "fuel_models": [], "use_gg": False},
            {"sc": scB, "rw_models": [objects[1]["rw"]],
             "thr_models": [objects[1]["thr"]],
             "thr_factory": objects[1]["tf"],
             "thr_cmd_msg": objects[1]["thr_cmd"],
             "fuel_models": [], "use_gg": False},
        ])
        scA.setIntegrator(oi)
    # Sync direction (upstream dynamicObject.cpp, v2.12.0): leader.sync(fol-
    # lower) pushes the follower into the LEADER's integrator and marks the
    # FOLLOWER synced. Asserted below -- a reversal fails loudly on host.
    if sync:
        scA.syncDynamicsIntegration(scB)
        assert bool(scB.isDynamicsSynced) and not bool(scA.isDynamicsSynced), (
            "swap_multi: sync flags wrong post-sync "
            f"(A={scA.isDynamicsSynced}, B={scB.isDynamicsSynced}); refusing to "
            f"run a silently unsynced leg.")
    logs = []
    for ob in objects:
        sc_log = ob["sc"].scStateOutMsg.recorder()
        sim.AddModelToTask("t", sc_log)
        spd_log = ob["rw"].rwSpeedOutMsg.recorder()
        sim.AddModelToTask("t", spd_log)
        logs.append((sc_log, spd_log))
    return {"sim": sim, "objects": objects, "logs": logs, "oi": oi,
            "scA": scA, "scB": scB}


def _run(sim, T):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T))
    sim.ExecuteSimulation()


def _final_diff(a, b, fields):
    import numpy as np
    out = {}
    for f in fields:
        xa = np.asarray(getattr(a, f))[-1]
        xb = np.asarray(getattr(b, f))[-1]
        out[f] = float(np.abs(xa - xb).max())
    return out


def _dcm_max(a, b):
    import numpy as np
    sa = np.asarray(a.sigma_BN)
    sb = np.asarray(b.sigma_BN)
    return float(max(np.abs(_dcm(sa[k]) - _dcm(sb[k])).max()
                     for k in range(len(sa))))


def _disp(log, r0):
    import numpy as np
    return float(np.linalg.norm(np.asarray(log.r_BN_N)[-1]
                                - np.asarray(r0, float)))


def _leg(wheels_b):
    """One stock-vs-omni mission leg. Returns the cell dict."""
    import numpy as np
    arm_s = _build_pair("rk", False, wheels_b)
    _run(arm_s["sim"], T_LEG)
    arm_o = _build_pair("omni", True, wheels_b)
    _run(arm_o["sim"], T_LEG)
    oi = arm_o["oi"]
    cell: dict = {"ticks": TICKS, "wheels_b": wheels_b}
    r0a = np.asarray(_oe_rv(OE_A)[0], float)
    r0b = np.asarray(_oe_rv(OE_B)[0], float)
    cell["ic_separation"] = float(np.linalg.norm(r0a - r0b))
    for i, tag in ((0, "objA"), (1, "objB")):
        log_s, spd_s = arm_s["logs"][i]
        log_o, spd_o = arm_o["logs"][i]
        d = _final_diff(log_s, log_o,
                        ["r_BN_N", "v_BN_N", "omega_BN_B"])
        d["dcm_max"] = _dcm_max(log_s, log_o)
        # sigma-vector diff is forensics only (shadow-set crossings make it
        # switch-timing-sensitive over a 600 s tumble).
        d["sigma_BN"] = _final_diff(log_s, log_o, ["sigma_BN"])["sigma_BN"]
        wa = np.asarray(spd_s.wheelSpeeds)[-1]
        wb = np.asarray(spd_o.wheelSpeeds)[-1]
        d["wheels"] = float(np.abs(wa - wb).max())
        d["disp_stock"] = _disp(log_s, arm_s["objects"][i]["r0"])
        d["disp_omni"] = _disp(log_o, arm_o["objects"][i]["r0"])
        d["mrp_stock"] = int(np.asarray(
            getattr(log_s, "MRPSwitchCount")).ravel()[-1])
        d["steps"] = int(oi.getStepCountForObject(i))
        d["mrp_omni"] = int(oi.getMRPSwitchCountForObject(i))
        d["build_s"] = round(float(oi.getBuildSecondsForObject(i)), 4)
        cell[tag] = d
    cell["calls"] = int(oi.getIntegrateCallCount())
    cell["shared_graph"] = bool(oi.sharedGraphUsed())
    cell["fp0"] = oi.configFingerprintForObject(0)
    cell["fp1"] = oi.configFingerprintForObject(1)
    cell["fp_equal"] = (cell["fp0"] == cell["fp1"])
    return cell


def _reject_unmirrored_cell():
    """Slot 0 mirrored explicitly, follower synced, 2 ticks: must raise
    loudly naming object 1 (anti-silent-inherit gate)."""
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    arm = _build_pair("rej", False, 3, sync=False)
    sim, objects = arm["sim"], arm["objects"]
    scA, scB = arm["scA"], arm["scB"]
    oi = OmniVecIntegrator(scA)
    # Explicit slot-0 mirror ONLY (per-object path): slot 1 stays unmirrored.
    mirror_stock_config(sim, scA, oi, object_index=0,
                        thr_factory=objects[0]["tf"],
                        thr_cmd_msg=objects[0]["thr_cmd"],
                        rw_models=[objects[0]["rw"]],
                        thr_models=[objects[0]["thr"]],
                        fuel_models=[], use_gg=False)
    scA.setIntegrator(oi)
    scA.syncDynamicsIntegration(scB)
    try:
        _run(sim, 2 * DT)
    except Exception as exc:  # C++ bskError surfaces here (p1 precedent)
        return {"raised": f"{type(exc).__name__}: {exc}"[:300]}
    return {"raised": False}


def main() -> int:
    if not safe_import_basilisk():
        save_result("swap_multi",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    # L15 log hygiene: truncate the run log at probe start so it keeps the
    # last run only (log_text appends; without this every run piles onto
    # the previous ones while the JSON keeps just the latest).
    try:
        _log_path = Path(__file__).resolve().parent.parent / "results" / "swap_multi.log"
        _log_path.write_text("", encoding="utf-8")
    except Exception:
        pass
    try:
        payload: dict = {"status": "ok", "cells": {}, "dt": DT,
                         "t_leg": T_LEG, "ticks": TICKS}
        payload["cells"]["leg_shared"] = _leg(3)
        payload["cells"]["leg_split"] = _leg(2)
        payload["cells"]["reject_unmirrored"] = _reject_unmirrored_cell()

        fails = []
        for leg in ("leg_shared", "leg_split"):
            c = payload["cells"][leg]
            for tag in ("objA", "objB"):
                d = c[tag]
                for f, tol in (("r_BN_N", TOL_R), ("v_BN_N", TOL_V),
                               ("omega_BN_B", TOL_OM),
                               ("dcm_max", TOL_DCM),
                               ("wheels", TOL_WHEEL)):
                    if not d[f] < tol:
                        fails.append(f"{leg}.{tag}.{f}={d[f]:.3e}")
                if not d["build_s"] < TOL_BUILD_S:
                    fails.append(f"{leg}.{tag}.build_s={d['build_s']:.3f}")
                if not d["steps"] == TICKS:
                    fails.append(f"{leg}.{tag}.steps={d['steps']}")
                if not abs(d["mrp_omni"] - d["mrp_stock"]) <= 2:
                    fails.append(f"{leg}.{tag}.mrp omni={d['mrp_omni']} "
                                 f"stock={d['mrp_stock']}")
                for arm in ("disp_stock", "disp_omni"):
                    if not d[arm] > TOL_LIVE_DISP:
                        fails.append(f"{leg}.{tag}.{arm}={d[arm]:.3e}")
            if not c["calls"] == TICKS:
                fails.append(f"{leg}.calls={c['calls']}")
            if not c["ic_separation"] > TOL_IC_SEP:
                fails.append(f"{leg}.ic_separation={c['ic_separation']:.3e}")
        sh = payload["cells"]["leg_shared"]
        if not (sh["shared_graph"] and sh["fp_equal"]):
            fails.append("leg_shared.expected shared graph + equal fingerprints")
        sp = payload["cells"]["leg_split"]
        if not ((not sp["shared_graph"]) and (not sp["fp_equal"])):
            fails.append("leg_split.expected split graphs + differing fingerprints")
        rej = payload["cells"]["reject_unmirrored"].get("raised")
        if not (isinstance(rej, str) and "object 1" in rej):
            fails.append(f"reject_unmirrored.not-loud: {rej!r}")
        payload["verdict"] = ("parity green" if not fails
                              else f"PARITY FAIL: {fails}")
        if fails:
            payload["status"] = "fail"
    except Exception as exc:
        from common import write_exception  # noqa: E402
        write_exception("swap_multi", exc)
        print(f"swap_multi ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("swap_multi", payload)
    for leg, c in payload["cells"].items():
        if leg == "reject_unmirrored":
            log_text("swap_multi", f"[{leg}] raised={c.get('raised')!r}")
            continue
        for tag in ("objA", "objB"):
            d = c[tag]
            log_text("swap_multi",
                     f"[{leg}.{tag}] " + " ".join(
                         f"{k}={v:.2e}" if isinstance(v, float) else f"{k}={v}"
                         for k, v in d.items()))
        log_text("swap_multi",
                 f"[{leg}] calls={c['calls']} shared={c['shared_graph']} "
                 f"fp_equal={c['fp_equal']}")
    print(f"swap_multi verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
