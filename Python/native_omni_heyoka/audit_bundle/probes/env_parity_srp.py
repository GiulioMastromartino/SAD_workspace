"""Env-parity probe: SRP-cannonball family (PLAN_ENV_MIRROR §3, list.md §A1).

Stock-vs-omni comparison at MATCHED dt through the WIRED
``mirror_stock_config`` path (stock arm = pure stock sim; omni arm = same
sim with the two integrator lines exchanged: repo ``swap.py`` by path +
wheel ``OmniVecIntegrator``).

Fixture (``swap_p1_parity._env_srp_cell`` verbatim, now wired): LEO
400 km circular still ICs, cannonball A=10 m² stock C_R=1.5
(``setUseCannonballModel``) -> omni cr = C_R - 1 = 0.5, static pre-written
sun message (REQUIRED sun link), eclipse unlinked (full sun both arms),
matched dt=0.1, T=600 s (6000 ticks).

Gates (measured basis, cited in tests/test_env_parity_srp.py): state
norms within the swap_p1 ``env_srp`` class; single-tick SRP-only dv within
10% of the analytic P*A*C_R*dt/m (vacuity, swap_p1 pattern); steps ==
calls == ticks audit (L14).

Run from Python/:  ./env/bin/python native_omni_heyoka/audit_bundle/probes/env_parity_srp.py
Writes: audit_bundle/results/env_parity_srp.json + .log
Host-only (needs Basilisk 2.12.0 + built omni_heyoka).
On-demand Phase-3 evidence (NOT in the run_all.sh glob).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # Python/
from common import log_text, save_result, safe_import_basilisk  # noqa: E402

REPO = Path(__file__).resolve().parents[2]  # native_omni_heyoka/

DT = 0.1
T_LEG = 600.0
TICKS = int(round(T_LEG / DT))
MU = 3.986004418e14
R_EARTH = 6378137.0
MASS = 750.0
R_LEO = R_EARTH + 400e3
V_LEO = float(__import__("math").sqrt(MU / R_LEO))
SUN = [1.5e11, 0.0, 0.0]
AREA = 10.0
C_R = 1.5
P_AU = 4.5398178907e-6  # env_const ledger value (swap_p1)
AU = 1.495978707e11

# swap_p1 env_srp class (tests/test_swap_p1.py::test_p1_env_parity).
TOL = {"r_BN_N": 1e-6, "v_BN_N": 1e-8, "sigma_BN": 1e-9,
       "omega_BN_B": 1e-9}


def _repo_swap():
    spec = importlib.util.spec_from_file_location(
        "env_parity_swap_srp", REPO / "omniheyoka" / "swap.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _loadavg():
    try:
        import os
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return "unavailable"


def _sun_msg():
    from Basilisk.architecture import messaging
    sm = messaging.SpicePlanetStateMsg()
    p = messaging.SpicePlanetStateMsgPayload()
    p.PositionVector = list(SUN)
    p.VelocityVector = [0.0, 0.0, 0.0]
    sm.write(p)
    return sm


def _build(use_omni):
    import numpy as np  # noqa: F401
    from Basilisk.simulation import radiationPressure, spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                     simIncludeGravBody)
    sm = _sun_msg()
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = ("omni-" if use_omni else "rk-") + "envsrp"
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
    sc.hub.r_CN_NInit = [R_LEO, 0.0, 0.0]
    sc.hub.v_CN_NInit = [0.0, V_LEO, 0.0]
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    srp = radiationPressure.RadiationPressure()
    srp.area = AREA
    srp.coefficientReflection = C_R
    srp.setUseCannonballModel()
    srp.sunEphmInMsg.subscribeTo(sm)
    sc.addDynamicEffector(srp)
    sim.AddModelToTask("t", srp)
    oi = None
    mirror_rec = None
    if use_omni:
        from omniheyoka.omni_heyoka import OmniVecIntegrator
        oi = OmniVecIntegrator(sc)
        mirror_rec = _repo_swap().mirror_stock_config(sim, sc, oi)
        sc.setIntegrator(oi)
    scLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", scLog)
    # Retain the sun writer for the sim lifetime (eclipse-probe precedent).
    return {"sim": sim, "sc": sc, "scLog": scLog, "oi": oi,
            "mirror_rec": mirror_rec, "retain": [sm]}


def _run(sim, T):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T))
    sim.ExecuteSimulation()


def _sanitize(node):
    if node is None or isinstance(node, (bool, int, float, str)):
        return node
    if isinstance(node, dict):
        return {str(k): _sanitize(v) for k, v in node.items()}
    if isinstance(node, (list, tuple)):
        return [_sanitize(v) for v in node]
    return repr(node)


def _leg():
    import numpy as np
    arm_s = _build(False)
    _run(arm_s["sim"], T_LEG)
    arm_o = _build(True)
    _run(arm_o["sim"], T_LEG)
    oi = arm_o["oi"]
    log_s, log_o = arm_s["scLog"], arm_o["scLog"]
    cell: dict = {
        "fixture": {"orbit": "LEO 400 km circular still", "area": AREA,
                    "C_R": C_R, "cr": C_R - 1.0, "sun": "static",
                    "eclipse": "unlinked/full-sun"},
        "dt": DT, "ticks": TICKS, "t_leg": T_LEG}
    for f in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
        xa = np.asarray(getattr(log_s, f))[-1]
        xb = np.asarray(getattr(log_o, f))[-1]
        cell[f] = float(np.abs(xa - xb).max())
    # Single-tick analytic vacuity (swap_p1 _env_srp_cell pattern verbatim):
    # |SRP-only dv| vs P*A*C_R*dt/m over one tick.
    arm_1 = _build(True)
    _run(arm_1["sim"], DT)
    sim0, sc0 = _bare_base()
    log0 = sc0.scStateOutMsg.recorder()
    sim0.AddModelToTask("t", log0)
    _run(sim0, DT)
    sun_d = float(np.linalg.norm(np.asarray(SUN, float)
                                 - np.asarray([R_LEO, 0.0, 0.0])))
    analytic = P_AU * (AU / sun_d) ** 2 * AREA * C_R * DT / MASS
    meas = float(np.linalg.norm(np.asarray(arm_1["scLog"].v_BN_N)[-1]
                                - np.asarray(log0.v_BN_N)[-1]))
    cell["srp_single_tick_meas"] = meas
    cell["srp_single_tick_analytic"] = analytic
    cell["srp_single_tick_ratio"] = meas / analytic if analytic else 0.0
    cell["steps"] = int(oi.getStepCountForObject(0))
    cell["calls"] = int(oi.getIntegrateCallCount())
    cell["nStates"] = int(oi.nStates())
    cell["build_s"] = round(float(oi.getBuildSeconds()), 4)
    cell["loadavg"] = _loadavg()
    cell["mirror"] = _sanitize(arm_o["mirror_rec"])
    return cell


def _bare_base():
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                     simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "srp0"
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
    sc.hub.r_CN_NInit = [R_LEO, 0.0, 0.0]
    sc.hub.v_CN_NInit = [0.0, V_LEO, 0.0]
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    return sim, sc


def main() -> int:
    if not safe_import_basilisk():
        save_result("env_parity_srp",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    try:
        from common import write_exception  # noqa: E402
        payload: dict = {"status": "ok", "cells": {}, "dt": DT,
                         "t_leg": T_LEG, "ticks": TICKS,
                         "loadavg": _loadavg()}
        payload["cells"]["leg"] = _leg()
        payload["loadavg_end"] = _loadavg()
        c = payload["cells"]["leg"]
        fails = []
        for f, tol in TOL.items():
            if not c[f] < tol:
                fails.append(f"leg.{f}={c[f]:.3e} >= {tol:.0e}")
        if not 0.9 < c["srp_single_tick_ratio"] < 1.1:
            fails.append("leg.vacuous single-tick ratio="
                         f"{c['srp_single_tick_ratio']:.4f}")
        if not c["steps"] == TICKS:
            fails.append(f"leg.steps={c['steps']}")
        if not c["calls"] == TICKS:
            fails.append(f"leg.calls={c['calls']}")
        m = c["mirror"]["env"]["srp"]
        entry = m["srp"][0] if m["srp"] else {}
        if not (entry.get("kind") == "cannonball"
                and entry.get("args") == [AREA, C_R - 1.0]
                and m.get("sun_link") == "sunStateInMsg.subscribeTo"):
            fails.append(f"leg.mirror-record {m}")
        payload["verdict"] = ("parity green" if not fails
                              else f"PARITY FAIL: {fails}")
        if fails:
            payload["status"] = "fail"
    except Exception as exc:
        write_exception("env_parity_srp", exc)
        print(f"env_parity_srp ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("env_parity_srp", payload)
    c = payload["cells"]["leg"]
    log_text("env_parity_srp",
             "[srp] " + " ".join(
                 f"{k}={v:.3e}" if isinstance(v, float) else f"{k}={v}"
                 for k, v in c.items() if k not in ("mirror", "fixture")))
    print(f"env_parity_srp verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
