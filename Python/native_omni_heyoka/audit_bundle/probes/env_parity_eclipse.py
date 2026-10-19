"""Env-parity probe: eclipse-link family (PLAN_ENV_MIRROR §5, list.md §A1).

Stock-vs-omni comparison at MATCHED dt through the WIRED
``mirror_stock_config`` path (stock arm = pure stock sim; omni arm = same
sim with the two integrator lines exchanged: repo ``swap.py`` by path +
wheel ``OmniVecIntegrator``).

Fixture (LEO 400 km circular still ICs, SRP carrier): cannonball
A=10 m² C_R=1.5 with the eclipse reader linked
(``sunEclipseInMsg`` <- ``Eclipse.eclipseOutMsgs[0]``) + the Eclipse field
module (earth planet message WITH PlanetName, sun message, single
registered spacecraft). The leg starts at 100° from the sun line so the
600 s leg at matched dt=0.1 (6000 ticks) contains the penumbra/umbra
ENTRY plus both regimes (full sun + umbra); the shadow series itself is
computed stock-side in both arms (omni only consumes shadowFactor), so
edge timing is common-mode.

Gates (measured basis, cited in tests/test_env_parity_eclipse.py): state
norms within the FSW-loop switching class (shadow-edge switching leg);
shadow vacuity (umbra reached AND full sun in-leg -- a missed entry fails
loudly, never records vacuous parity); single-tick SRP-only dv within 10%
of analytic (carrier live, swap_p1 pattern); steps == calls == ticks (L14).

Run from Python/:  ./env/bin/python native_omni_heyoka/audit_bundle/probes/env_parity_eclipse.py
Writes: audit_bundle/results/env_parity_eclipse.json + .log
Host-only (needs Basilisk 2.12.0 + built omni_heyoka).
On-demand Phase-3 evidence (NOT in the run_all.sh glob).
"""

from __future__ import annotations

import importlib.util
import math
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
V_LEO = float(math.sqrt(MU / R_LEO))
SUN_POS = [1.496e11, 0.0, 0.0]
AREA = 10.0
C_R = 1.5
P_AU = 4.5398178907e-6  # env_const ledger value (swap_p1)
AU = 1.495978707e11
TH0 = math.radians(100.0)  # leg start: full sun, entry in-leg
R0 = [R_LEO * math.cos(TH0), R_LEO * math.sin(TH0), 0.0]
V0 = [-V_LEO * math.sin(TH0), V_LEO * math.cos(TH0), 0.0]

# FSW-loop switching class (tests/test_fsw_loop.py docstring bounds).
TOL = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
       "omega_BN_B": 1e-4, "dcm_max": 2e-3}


def _repo_swap():
    spec = importlib.util.spec_from_file_location(
        "env_parity_swap_eclipse", REPO / "omniheyoka" / "swap.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _loadavg():
    try:
        import os
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return "unavailable"


def _dcm(sigma):
    import numpy as np

    def skew(c):
        return np.array([[0., -c[2], c[1]], [c[2], 0., -c[0]],
                         [-c[1], c[0], 0.]])
    s = np.asarray(sigma, float)
    s2 = float(s @ s)
    S = skew(s)
    return np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2


def _planet_msg(pos, name=""):
    from Basilisk.architecture import messaging
    sm = messaging.SpicePlanetStateMsg()
    p = messaging.SpicePlanetStateMsgPayload()
    p.PositionVector = list(pos)
    p.VelocityVector = [0.0, 0.0, 0.0]
    if name:
        p.PlanetName = name
    sm.write(p)
    return sm


def _build(use_omni):
    import numpy as np  # noqa: F401
    from Basilisk.simulation import (eclipse as EC, radiationPressure as RP,
                                     spacecraft)
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                     simIncludeGravBody)
    sun_msg = _planet_msg(SUN_POS, "sun")
    earth_msg = _planet_msg([0.0, 0.0, 0.0], "earth")
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = ("omni-" if use_omni else "rk-") + "enveclipse"
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
    sc.hub.r_CN_NInit = list(R0)
    sc.hub.v_CN_NInit = list(V0)
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    srp = RP.RadiationPressure()
    srp.area = AREA
    srp.coefficientReflection = C_R
    srp.setUseCannonballModel()
    srp.sunEphmInMsg.subscribeTo(sun_msg)
    ecl = EC.Eclipse()
    ecl.addSpacecraftToModel(sc.scStateOutMsg)
    ecl.addPlanetToModel(earth_msg)
    ecl.sunInMsg.subscribeTo(sun_msg)
    sim.AddModelToTask("t", ecl)
    srp.sunEclipseInMsg.subscribeTo(ecl.eclipseOutMsgs[0])
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
    eLog = ecl.eclipseOutMsgs[0].recorder()
    sim.AddModelToTask("t", eLog)
    # Retain every writer message for the sim lifetime: free-floating
    # messages are _build locals, and the eclipse planet-name lookup fails
    # mid-leg once the Python wrappers are collected (fixed here; the
    # numeric-payload links never noticed).
    return {"sim": sim, "sc": sc, "scLog": scLog, "eLog": eLog, "oi": oi,
            "mirror_rec": mirror_rec, "retain": [sun_msg, earth_msg]}


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
        "fixture": {"orbit": "LEO 400 km circular still, start 100deg",
                    "carrier": "SRP cannonball A=10 C_R=1.5",
                    "eclipse": "single-sc, earth-planet, entry in-leg"},
        "dt": DT, "ticks": TICKS, "t_leg": T_LEG}
    for f in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
        xa = np.asarray(getattr(log_s, f))[-1]
        xb = np.asarray(getattr(log_o, f))[-1]
        cell[f] = float(np.abs(xa - xb).max())
    sa = np.asarray(log_s.sigma_BN)
    sb = np.asarray(log_o.sigma_BN)
    cell["dcm_max"] = float(max(np.abs(_dcm(sa[k]) - _dcm(sb[k])).max()
                                for k in range(len(sa))))
    sh = np.asarray(arm_s["eLog"].shadowFactor, float).ravel()
    cell["shadow_min"] = float(sh.min()) if sh.size else float("nan")
    cell["shadow_max"] = float(sh.max()) if sh.size else float("nan")
    below = [k for k in range(len(sh)) if sh[k] < 1.0]
    cell["first_shadowed_tick"] = int(below[0]) if below else None
    cell["shadowed_frac"] = float(len(below) / len(sh)) if len(sh) else 0.0
    # Carrier vacuity: single-tick SRP-only dv vs analytic (swap_p1
    # pattern; leg starts in full sun so the tick is unshadowed).
    arm_1 = _build(True)
    _run(arm_1["sim"], DT)
    sim0, sc0 = _bare_base()
    log0 = sc0.scStateOutMsg.recorder()
    sim0.AddModelToTask("t", log0)
    _run(sim0, DT)
    sun_d = float(np.linalg.norm(np.asarray(SUN_POS, float)
                                 - np.asarray(R0)))
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
    sc.ModelTag = "ecl0"
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
    sc.hub.r_CN_NInit = list(R0)
    sc.hub.v_CN_NInit = list(V0)
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
        save_result("env_parity_eclipse",
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
        # Shadow vacuity: the entry must be IN the leg (both regimes).
        if not c["shadow_min"] < 0.5:
            fails.append(f"leg.vacuous no umbra shadow_min={c['shadow_min']}")
        if not c["shadow_max"] > 0.99:
            fails.append(f"leg.vacuous no full sun shadow_max={c['shadow_max']}")
        if not 0.9 < c["srp_single_tick_ratio"] < 1.1:
            fails.append("leg.vacuous single-tick ratio="
                         f"{c['srp_single_tick_ratio']:.4f}")
        if not c["steps"] == TICKS:
            fails.append(f"leg.steps={c['steps']}")
        if not c["calls"] == TICKS:
            fails.append(f"leg.calls={c['calls']}")
        m = c["mirror"]["env"]
        if not (m["eclipse"]["eclipse_link"] == "eclipseInMsg.subscribeTo"
                and m["eclipse"]["n_eclipse"] == 1
                and m["srp"].get("sun_link") == "sunStateInMsg.subscribeTo"):
            fails.append(f"leg.mirror-record {m['eclipse']}")
        payload["verdict"] = ("parity green" if not fails
                              else f"PARITY FAIL: {fails}")
        if fails:
            payload["status"] = "fail"
    except Exception as exc:
        write_exception("env_parity_eclipse", exc)
        print(f"env_parity_eclipse ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("env_parity_eclipse", payload)
    c = payload["cells"]["leg"]
    log_text("env_parity_eclipse",
             "[eclipse] " + " ".join(
                 f"{k}={v:.3e}" if isinstance(v, float) else f"{k}={v}"
                 for k, v in c.items() if k not in ("mirror", "fixture")))
    print(f"env_parity_eclipse verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
