"""Env-parity probe: env-bodies+J2 family (PLAN_ENV_MIRROR §6, list.md §A1).

ATTEMPTED stock-vs-omni comparison at MATCHED dt through the WIRED
``mirror_stock_config`` path. Both sub-legs are HOST-BLOCKED by staged-
reader gaps (REAL model gaps, not probe bugs -- the jitter-spike NO-GO
precedent applies: record divergence + cause, keep the loud behavior,
never weaken the reader to force green):

* N-body cell (earth + sun + moon, static ephemeris messages, task
  ``GravityEffector``): the wired mirror RAISES on host -- the staged
  reader cannot recover the writer Message behind the stock READER functor
  (``env_mirror_envbodies._body_record`` raises "writer Message is
  unrecoverable"), and BSK ``subscribeTo`` needs the writer Message, not
  the reader functor (sibling drag/mag readers recover it via
  ``_get_source_owner``). Loud host raise, recorded verbatim. Firing
  proof: stock-vs-earth-only tide displacement (sun+moon live).
* J2 cell (central spherical harmonics maxDeg=2, Cbar20=-4.8417e-4):
  the wired mirror RAISES on host -- the central body's concrete gravity
  law lives in the ``_pyGravityModel`` Python shadow and is lost across
  the C++ ``gravBodies`` copy (``census_shadow`` all False on host), so
  the reader rejects the unknowable law instead of assuming point-mass
  (PLAN §1.2, never a silent drop). The historical silent no-action ran
  omni without J2 while stock felt the SH field (~2.4 km / 600 s); that
  divergence is now a loud reject. Firing proof: the stock J2
  displacement is macroscopic.

Fixture: LEO 400 km circular still ICs, matched dt=0.1, T=600 s.
Verdict is NO-GO by record: BOTH cells raise loudly (the reader refuses
the unrepresentable N-body link and the unknowable central J2 law). Probe
status stays "ok" = ran as designed.

Run from Python/:  ./env/bin/python native_omni_heyoka/audit_bundle/probes/env_parity_envbodies.py
Writes: audit_bundle/results/env_parity_envbodies.json + .log
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
SUN_MU = 1.32712440018e20
MOON_POS = [3.844e8, 0.0, 0.0]
MOON_MU = 4.9028e12
CBAR20 = -4.8417e-4
J2_EARTH = -math.sqrt(5.0) * CBAR20


def _repo_swap():
    spec = importlib.util.spec_from_file_location(
        "env_parity_swap_envbodies", REPO / "omniheyoka" / "swap.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _loadavg():
    try:
        import os
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return "unavailable"


def _base(tag):
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                     simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
    sc.hub.r_CN_NInit = [R_LEO, 0.0, 0.0]
    sc.hub.v_CN_NInit = [0.0, V_LEO, 0.0]
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc)
    fac = simIncludeGravBody.gravBodyFactory()
    return sim, sc, fac


def _static_msg(pos):
    from Basilisk.architecture import messaging
    sm = messaging.SpicePlanetStateMsg()
    p = messaging.SpicePlanetStateMsgPayload()
    p.PositionVector = list(pos)
    p.VelocityVector = [0.0, 0.0, 0.0]
    sm.write(p)
    _RETAIN.append(sm)  # sim-lifetime retention (eclipse-probe precedent)
    return sm


_RETAIN: list = []


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


def _nbody_cell():
    """Sun+moon extras: stock tide liveness + wired-mirror host raise."""
    import numpy as np
    from Basilisk.simulation import gravityEffector as GE

    def build(extras, tag):
        sim, sc, fac = _base(tag)
        e = fac.createEarth()
        e.isCentralBody = True
        bodies = [e]
        if "sun" in extras:
            s = fac.createSun()
            s.isCentralBody = False
            s.planetBodyInMsg.subscribeTo(_static_msg(SUN_POS))
            bodies.append(s)
        if "moon" in extras:
            mm = fac.createMoon()
            mm.isCentralBody = False
            mm.planetBodyInMsg.subscribeTo(_static_msg(MOON_POS))
            bodies.append(mm)
        fac.addBodiesTo(sc)
        eff = None
        if extras:
            eff = GE.GravityEffector()
            for b in bodies:
                eff.addGravBody(b)
            sim.AddModelToTask("t", eff)
        log = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", log)
        return sim, sc, eff, log

    cell: dict = {
        "fixture": {"orbit": "LEO 400 km circular still",
                    "extras": ["sun", "moon"], "ephemeris": "static msgs",
                    "effector": "task GravityEffector (factory bodies)"},
        "dt": DT, "ticks": TICKS, "t_leg": T_LEG}
    simA, _, _, logA = build(("sun", "moon"), "nb-stock")
    _run(simA)
    sim0, _, _, log0 = build((), "nb-base")
    _run(sim0)
    rA = np.asarray(logA.r_BN_N)[-1]
    r0 = np.asarray(log0.r_BN_N)[-1]
    cell["tide_effect_m"] = float(np.linalg.norm(rA - r0))
    # Wired-mirror attempt on a fresh identical sim (must raise LOUDLY,
    # recorded verbatim). On host the whole census is rejected before the
    # N-body link is even reached: the central body's concrete law lives in
    # the _pyGravityModel shadow and is lost across the C++ addBodiesTo copy,
    # so the central-law gate fires first. The sibling per-body link gate
    # (linked reader with an unrecoverable writer Message) is pinned
    # host-free by tests/test_env_mirror_envbodies.py.
    simM, scM, _, _ = build(("sun", "moon"), "nb-mirror")
    try:
        from omniheyoka.omni_heyoka import OmniVecIntegrator
        oi = OmniVecIntegrator(scM)
        _repo_swap().mirror_stock_config(simM, scM, oi)
        cell["status"] = "UNEXPECTED-GREEN"
        cell["mirror_raise"] = None
    except Exception as exc:  # noqa: BLE001 (loud host behavior is the record)
        cell["status"] = "mirror-raises-on-host"
        cell["mirror_raise"] = f"{type(exc).__name__}: {exc}"
    cell["cause"] = ("host rejects the census before the N-body link: the "
                     "central body's concrete law lives in the "
                     "_pyGravityModel shadow and is lost across the C++ "
                     "addBodiesTo copy, so the central-law gate fires first "
                     "(the linked-reader/unrecoverable-writer-Message link "
                     "gate is pinned host-free by "
                     "test_reject_linked_but_sourceless_link).")
    cell["loadavg"] = _loadavg()
    return cell


def _j2_cell():
    """Central J2: stock feels it; the wired mirror REJECTS it loudly (NO-GO)."""
    import numpy as np
    from Basilisk.simulation import (gravityEffector as GE,
                                     sphericalHarmonicsGravityModel as SH)

    def sh_model():
        m = SH.SphericalHarmonicsGravityModel()
        m.cBar = [[0.0], [0.0, 0.0], [CBAR20, 0.0, 0.0]]
        m.sBar = [[0.0], [0.0, 0.0], [0.0, 0.0, 0.0]]
        m.maxDeg = 2
        m.muBody = MU
        m.radEquator = R_EARTH
        return m

    def build(tag, with_j2, with_eff):
        sim, sc, fac = _base(tag)
        e = fac.createEarth()
        e.isCentralBody = True
        if with_j2:
            e.gravityModel = sh_model()
        fac.addBodiesTo(sc)
        eff = None
        if with_eff:
            eff = GE.GravityEffector()
            eff.addGravBody(e)  # factory body (keeps the model shadow)
            sim.AddModelToTask("t", eff)
        log = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", log)
        return sim, sc, eff, log

    cell: dict = {
        "fixture": {"orbit": "LEO 400 km circular still",
                    "central": "SH maxDeg=2 Cbar20=-4.8417e-4",
                    "J2_rule": "-sqrt(5)*Cbar20 = 1.0826e-3 (wp5_env:262-292)",
                    "effector": "task GravityEffector (factory body)"},
        "dt": DT, "ticks": TICKS, "t_leg": T_LEG,
        "J2_expected": J2_EARTH}
    # Mechanism evidence: C++-vector-iterated stock bodies lost the shadow.
    simP, _, effP, _ = build("j2-probe", True, True)
    iterated = list(effP.gravBodies)
    cell["census_shadow"] = {
        "n_bodies": len(iterated),
        "iterated_has_py_shadow": [
            getattr(b, "_pyGravityModel", "MISSING") != "MISSING"
            for b in iterated],
    }
    simS, _, _, logS = build("j2-stock", True, True)
    _run(simS)
    sim0, _, _, log0 = build("j2-base", False, False)
    _run(sim0)
    rS = np.asarray(logS.r_BN_N)[-1]
    r0 = np.asarray(log0.r_BN_N)[-1]
    cell["j2_effect_m"] = float(np.linalg.norm(rS - r0))
    # Mirrored-omni arm: the reader must REJECT the unknowable central law
    # loudly (the shadow died in the C++ copy -- census_shadow below). If it
    # ever succeeds, record the outcome + the divergence it produces so a
    # reader fix is visible here (move the cell to the parity suite).
    simM, scM, _, logM = build("j2-omni", True, True)
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    oi = OmniVecIntegrator(scM)
    try:
        rec = _repo_swap().mirror_stock_config(simM, scM, oi)
        cell["status"] = "UNEXPECTED-GREEN"
        cell["mirror_raise"] = None
        try:
            cell["mirror_j2"] = _sanitize(rec["env"]["envbodies"]["j2"])
        except Exception:
            cell["mirror_j2"] = None
        scM.setIntegrator(oi)
        _run(simM)
        rM = np.asarray(logM.r_BN_N)[-1]
        vS = np.asarray(logS.v_BN_N)[-1]
        vM = np.asarray(logM.v_BN_N)[-1]
        cell["divergence"] = {"r_BN_N": float(np.abs(rS - rM).max()),
                              "v_BN_N": float(np.abs(vS - vM).max())}
    except Exception as exc:  # noqa: BLE001 (loud host behavior is the record)
        cell["status"] = "mirror-raises-on-host"
        cell["mirror_raise"] = f"{type(exc).__name__}: {exc}"
    cell["cause"] = ("staged _grav_model_name keys on the _pyGravityModel "
                     "shadow; host stock bodies iterated from C++ vectors "
                     "lost it (census_shadow all false), so the concrete "
                     "law is unreadable and the reader REJECTS (never "
                     "assumes point-mass, PLAN §1.2). Before the reject "
                     "landed the silent no-action ran omni without J2 while "
                     "stock felt the SH field (~2.4 km / 600 s).")
    cell["loadavg"] = _loadavg()
    return cell


def main() -> int:
    if not safe_import_basilisk():
        save_result("env_parity_envbodies",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    try:
        from common import write_exception  # noqa: E402
        payload: dict = {"status": "ok", "cells": {}, "dt": DT,
                         "t_leg": T_LEG, "ticks": TICKS,
                         "loadavg": _loadavg()}
        payload["cells"]["nbody"] = _nbody_cell()
        payload["cells"]["j2"] = _j2_cell()
        payload["loadavg_end"] = _loadavg()
        n, j = payload["cells"]["nbody"], payload["cells"]["j2"]
        problems = []
        if n["status"] != "mirror-raises-on-host":
            problems.append(f"nbody: {n['status']} (expected the loud raise)")
        if not n["tide_effect_m"] > 1e-2:
            problems.append(f"nbody vacuous tide={n['tide_effect_m']:.3e}")
        if j["status"] != "mirror-raises-on-host":
            problems.append(f"j2: {j['status']} (expected the loud reject; "
                            "move to the parity suite if a reader fix "
                            "landed)")
        if "mirror_stock_config" not in (j.get("mirror_raise") or ""):
            problems.append(
                f"j2 reject signature lost: {j.get('mirror_raise')}")
        if not j["j2_effect_m"] > 1.0:
            problems.append(
                f"j2 stock effect vacuous ({j['j2_effect_m']:.3e})")
        payload["verdict"] = ("NO-GO (envbodies reader host gaps: the "
                              "N-body sun/moon link's writer Message is "
                              "unrecoverable -- loud raise; the central J2 "
                              "law is unreadable across the C++ gravBodies "
                              "copy -- loud reject, never assumed "
                              "point-mass)"
                              if not problems
                              else f"UNEXPECTED: {problems}")
        if problems:
            payload["status"] = "fail"
    except Exception as exc:
        write_exception("env_parity_envbodies", exc)
        print(f"env_parity_envbodies ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("env_parity_envbodies", payload)
    for name, c in payload["cells"].items():
        log_text("env_parity_envbodies",
                 f"[{name}] " + " ".join(
                     f"{k}={v:.3e}" if isinstance(v, float) else f"{k}={v}"
                     for k, v in c.items() if k != "mirror_j2"))
    print(f"env_parity_envbodies verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
