"""Env-parity probe: drag family (PLAN_ENV_MIRROR §2, list.md §A1 parity leg).

Stock-vs-omni comparison at MATCHED dt through the WIRED
``mirror_stock_config`` path (stock arm = pure stock sim; omni arm = same
sim with the two integrator lines exchanged: repo ``swap.py`` by path +
wheel ``OmniVecIntegrator``).

Fixture (LEO 350 km, still ICs, com=[0,0,0], wind-linked corotation --
``leo_drag_sim.py`` shape at probe-matched density: baseDensity 1e-9 /
scaleHeight 50 km / planetRadius R_EARTH, CdA = 2.2 * 5.0 = 11):
matched dt=0.1, T=600 s (6000 ticks). The omni arm keeps the stock drag
effector in the task (routed, never removed -- sc detachment); the atmo
(REQUIRED) + wind (corotation) links are forwarded by the wired reader.

Gates (measured basis, cited in tests/test_env_parity_drag.py):
state norms within the swap_p1 ``env_drag_still`` class; density_max above
the ``DENSITY_FIRING_GATE`` (effect live); macroscopic decay; steps ==
calls == ticks audit (L14 single-spacecraft path).

Run from Python/:  ./env/bin/python native_omni_heyoka/audit_bundle/probes/env_parity_drag.py
Writes: audit_bundle/results/env_parity_drag.json + .log
Host-only (needs Basilisk 2.12.0 + built omni_heyoka).
On-demand Phase-3 evidence (NOT in the run_all.sh glob -- same class as
the leo_drag / stationkeeping_stock probes).
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
R_LEO = R_EARTH + 350e3
V_LEO = float(__import__("math").sqrt(MU / R_LEO))
BASE_DENSITY = 1.0e-9
SCALE_HEIGHT = 50.0e3
AREA = 5.0
CD = 2.2
FIRING_GATE = 1.0e-13  # leo_drag_sim.DENSITY_FIRING_GATE (L1 vacuity)
EARTH_ROT_RATE = 7.2921159e-5  # leo_drag_sim / wp5_env corotation constant

# swap_p1 env_drag_still class (tests/test_swap_p1.py::test_p1_env_parity).
TOL = {"r_BN_N": 2e-5, "v_BN_N": 1e-7, "sigma_BN": 1e-7,
       "omega_BN_B": 1e-9}


def _repo_swap():
    spec = importlib.util.spec_from_file_location(
        "env_parity_swap_drag", REPO / "omniheyoka" / "swap.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _loadavg():
    try:
        import os
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return "unavailable"


def _build(use_omni):
    import numpy as np  # noqa: F401
    from Basilisk.architecture import messaging
    from Basilisk.simulation import (dragDynamicEffector,
                                     exponentialAtmosphere, spacecraft,
                                     zeroWindModel)
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                     simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = ("omni-" if use_omni else "rk-") + "envdrag"
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
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.baseDensity = BASE_DENSITY
    atmo.scaleHeight = SCALE_HEIGHT
    atmo.planetRadius = R_EARTH
    atmo.addSpacecraftToModel(sc.scStateOutMsg)
    sim.AddModelToTask("t", atmo)
    wind = zeroWindModel.ZeroWindModel()
    wind.addSpacecraftToModel(sc.scStateOutMsg)
    earth_state = messaging.SpicePlanetStateMsg()
    earth_state.write(messaging.SpicePlanetStateMsgPayload())
    wind.planetPosInMsg.subscribeTo(earth_state)
    wind.setUseSpiceOmegaFlag(False)
    wind.setPlanetOmega_N([0.0, 0.0, EARTH_ROT_RATE])
    sim.AddModelToTask("t", wind)
    drag = dragDynamicEffector.DragDynamicEffector()
    drag.coreParams.projectedArea = AREA
    drag.coreParams.dragCoeff = CD
    drag.coreParams.comOffset = [0.0, 0.0, 0.0]
    drag.cannonballDrag()
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
    drag.windVelInMsg.subscribeTo(wind.envOutMsgs[-1])
    sc.addDynamicEffector(drag)
    sim.AddModelToTask("t", drag)
    oi = None
    mirror_rec = None
    if use_omni:
        from omniheyoka.omni_heyoka import OmniVecIntegrator
        oi = OmniVecIntegrator(sc)
        mirror_rec = _repo_swap().mirror_stock_config(sim, sc, oi)
        sc.setIntegrator(oi)
    scLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", scLog)
    dLog = atmo.envOutMsgs[-1].recorder()
    sim.AddModelToTask("t", dLog)
    # Retain writer messages for the sim lifetime (eclipse-probe
    # precedent: free-floating _build locals must not be collected
    # mid-leg; numeric links are unaffected, retained anyway).
    return {"sim": sim, "sc": sc, "scLog": scLog, "dLog": dLog, "oi": oi,
            "mirror_rec": mirror_rec, "retain": [earth_state]}


def _run(sim):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T_LEG))
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
    _run(arm_s["sim"])
    arm_o = _build(True)
    _run(arm_o["sim"])
    oi = arm_o["oi"]
    log_s, log_o = arm_s["scLog"], arm_o["scLog"]
    cell: dict = {
        "fixture": {"orbit": "LEO 350 km circular still", "CdA": CD * AREA,
                    "comOffset": [0.0, 0.0, 0.0],
                    "atmo": "exponential 1e-9/50km", "wind": "corotation",
                    "shape": "leo_drag_sim links at probe density"},
        "dt": DT, "ticks": TICKS, "t_leg": T_LEG}
    for f in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
        xa = np.asarray(getattr(log_s, f))[-1]
        xb = np.asarray(getattr(log_o, f))[-1]
        cell[f] = float(np.abs(xa - xb).max())
    rho = np.asarray(arm_o["dLog"].neutralDensity, float)
    cell["density_max"] = float(rho.max()) if rho.size else float("nan")
    rn = np.linalg.norm(np.asarray(log_s.r_BN_N, float), axis=1)
    cell["decay_m"] = float(rn[0] - rn[-1])
    cell["steps"] = int(oi.getStepCountForObject(0))
    cell["calls"] = int(oi.getIntegrateCallCount())
    cell["nStates"] = int(oi.nStates())
    cell["build_s"] = round(float(oi.getBuildSeconds()), 4)
    cell["loadavg"] = _loadavg()
    cell["mirror"] = _sanitize(arm_o["mirror_rec"])
    return cell


def main() -> int:
    if not safe_import_basilisk():
        save_result("env_parity_drag",
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
        if not c["density_max"] > FIRING_GATE:
            fails.append(f"leg.vacuous density_max={c['density_max']:.3e}")
        if not c["decay_m"] > 0.0:
            fails.append(f"leg.vacuous decay={c['decay_m']:.3e}")
        if not c["steps"] == TICKS:
            fails.append(f"leg.steps={c['steps']}")
        if not c["calls"] == TICKS:
            fails.append(f"leg.calls={c['calls']}")
        m = c["mirror"]["env"]["drag"]
        if not (m["n_drag"] == 1 and abs(m["CdA"] - CD * AREA) < 1e-12
                and m["atmo"] == "linked" and m["wind"] == "forwarded"):
            fails.append(f"leg.mirror-record {m}")
        payload["verdict"] = ("parity green" if not fails
                              else f"PARITY FAIL: {fails}")
        if fails:
            payload["status"] = "fail"
    except Exception as exc:
        write_exception("env_parity_drag", exc)
        print(f"env_parity_drag ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("env_parity_drag", payload)
    c = payload["cells"]["leg"]
    log_text("env_parity_drag",
             "[drag] " + " ".join(
                 f"{k}={v:.3e}" if isinstance(v, float) else f"{k}={v}"
                 for k, v in c.items() if k not in ("mirror", "fixture")))
    print(f"env_parity_drag verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
