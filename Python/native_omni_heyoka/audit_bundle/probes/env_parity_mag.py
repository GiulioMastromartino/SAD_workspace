"""Env-parity probe: mag/MTB family (PLAN_ENV_MIRROR §4, list.md §A1).

Stock-vs-omni comparison at MATCHED dt through the WIRED
``mirror_stock_config`` path (stock arm = pure stock sim; omni arm = same
sim with the two integrator lines exchanged: repo ``swap.py`` by path +
wheel ``OmniVecIntegrator``).

Fixture (LEO 400 km circular still ICs): single-bar identity-Gt
``MtbEffector`` (numMTB=1, Gt=[1,0,0]+zeros, maxMtbDipoles=[0.8]+zeros)
with a LATCHED pre-written dipole command (0.5 x-Am²) +
``MagneticFieldCenteredDipole`` (Tesla coefficients, planetRadius set,
planet message carrying the identity J20002Pfix -- a zeroed DCM maps the
dipole to a silent B=0, recon precedent) feeding the REQUIRED mag link;
matched dt=0.1, T=600 s (6000 ticks).

Gates (measured basis, cited in tests/test_env_parity_mag.py): state
norms within the FSW-loop switching class (this family has no swap_p1
quiet-leg precedent); torque/attitude vacuity (effect live: latched
dipole x live B field); steps == calls == ticks audit (L14).

Run from Python/:  ./env/bin/python native_omni_heyoka/audit_bundle/probes/env_parity_mag.py
Writes: audit_bundle/results/env_parity_mag.json + .log
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
# Tesla dipole coefficients (SI both paths; parity itself pins the shared
# convention -- an nT-scale input spins the hub to chaos, recon precedent).
G10, G11, H11 = -29404.8e-9, -1450.9e-9, 4652.9e-9
DIPOLE_CMD = 0.5
MAX_DIPOLE = 0.8
ID3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

# FSW-loop switching class (tests/test_fsw_loop.py docstring bounds).
TOL = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
       "omega_BN_B": 1e-4, "dcm_max": 2e-3}


def _repo_swap():
    spec = importlib.util.spec_from_file_location(
        "env_parity_swap_mag", REPO / "omniheyoka" / "swap.py")
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


def _build(use_omni):
    import numpy as np  # noqa: F401
    from Basilisk.architecture import messaging
    from Basilisk.simulation import (MtbEffector as ME,
                                     magneticFieldCenteredDipole as MF,
                                     spacecraft)
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                     simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = ("omni-" if use_omni else "rk-") + "envmag"
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
    field = MF.MagneticFieldCenteredDipole()
    field.g10, field.g11, field.h11 = G10, G11, H11
    field.planetRadius = R_EARTH
    planet_msg = messaging.SpicePlanetStateMsg()
    pp = messaging.SpicePlanetStateMsgPayload()
    pp.PositionVector = [0.0, 0.0, 0.0]
    pp.VelocityVector = [0.0, 0.0, 0.0]
    pp.J20002Pfix = [list(r) for r in ID3]
    planet_msg.write(pp)
    field.planetPosInMsg.subscribeTo(planet_msg)
    field.addSpacecraftToModel(sc.scStateOutMsg)
    sim.AddModelToTask("t", field)
    cmd = messaging.MTBCmdMsg()
    cp = messaging.MTBCmdMsgPayload()
    cp.mtbDipoleCmds = [DIPOLE_CMD] + [0.0] * 35
    cmd.write(cp)
    cfg = messaging.MTBArrayConfigMsg()
    qp = messaging.MTBArrayConfigMsgPayload()
    qp.numMTB = 1
    qp.GtMatrix_B = [1.0, 0.0, 0.0] + [0.0] * 105
    qp.maxMtbDipoles = [MAX_DIPOLE] + [0.0] * 35
    cfg.write(qp)
    mtb = ME.MtbEffector()
    mtb.mtbCmdInMsg.subscribeTo(cmd)
    mtb.mtbParamsInMsg.subscribeTo(cfg)
    mtb.magInMsg.subscribeTo(field.envOutMsgs[0])
    sc.addDynamicEffector(mtb)
    sim.AddModelToTask("t", mtb)
    oi = None
    mirror_rec = None
    if use_omni:
        from omniheyoka.omni_heyoka import OmniVecIntegrator
        oi = OmniVecIntegrator(sc)
        mirror_rec = _repo_swap().mirror_stock_config(sim, sc, oi)
        sc.setIntegrator(oi)
    scLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", scLog)
    mLog = mtb.mtbOutMsg.recorder()
    sim.AddModelToTask("t", mLog)
    fLog = field.envOutMsgs[0].recorder()
    sim.AddModelToTask("t", fLog)
    # Retain writer messages for the sim lifetime (eclipse-probe precedent).
    return {"sim": sim, "sc": sc, "scLog": scLog, "mLog": mLog,
            "fLog": fLog, "oi": oi, "mirror_rec": mirror_rec,
            "retain": [planet_msg, cmd, cfg]}


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
        "fixture": {"orbit": "LEO 400 km circular still",
                    "dipole_cmd": [DIPOLE_CMD, 0.0, 0.0],
                    "max_dipole": MAX_DIPOLE, "Gt": "single-bar identity",
                    "dipole_coeffs_T": [G10, G11, H11]},
        "dt": DT, "ticks": TICKS, "t_leg": T_LEG}
    for f in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
        xa = np.asarray(getattr(log_s, f))[-1]
        xb = np.asarray(getattr(log_o, f))[-1]
        cell[f] = float(np.abs(xa - xb).max())
    sa = np.asarray(log_s.sigma_BN)
    sb = np.asarray(log_o.sigma_BN)
    cell["dcm_max"] = float(max(np.abs(_dcm(sa[k]) - _dcm(sb[k])).max()
                                for k in range(len(sa))))
    tau = np.asarray(arm_s["mLog"].mtbNetTorque_B, float)
    cell["tau_max_stock"] = float(np.abs(tau).max())
    cell["sigF_norm_stock"] = float(
        np.linalg.norm(np.asarray(log_s.sigma_BN)[-1]))
    bfield = np.asarray(arm_s["fLog"].magField_N, float)
    cell["B_max_stock_T"] = float(np.abs(bfield).max())
    cell["steps"] = int(oi.getStepCountForObject(0))
    cell["calls"] = int(oi.getIntegrateCallCount())
    cell["nStates"] = int(oi.nStates())
    cell["build_s"] = round(float(oi.getBuildSeconds()), 4)
    cell["loadavg"] = _loadavg()
    cell["mirror"] = _sanitize(arm_o["mirror_rec"])
    return cell


def main() -> int:
    if not safe_import_basilisk():
        save_result("env_parity_mag",
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
        # Vacuity (in-leg measured effect, >=10x margin): latched dipole x
        # live Tesla field must torque the hub (measured ~1.2e-5 N.m).
        if not c["tau_max_stock"] > 1e-6:
            fails.append(f"leg.vacuous tau_max={c['tau_max_stock']:.3e}")
        if not c["sigF_norm_stock"] > 1e-6:
            fails.append(f"leg.vacuous sigF={c['sigF_norm_stock']:.3e}")
        if not c["steps"] == TICKS:
            fails.append(f"leg.steps={c['steps']}")
        if not c["calls"] == TICKS:
            fails.append(f"leg.calls={c['calls']}")
        m = c["mirror"]["env"]["mag"]
        if not (m["n_mtb"] == 1 and m["n_magfield"] == 1
                and m["magnetic_moment"] == [DIPOLE_CMD, 0.0, 0.0]
                and m["mag_msg_wired"] is True
                and m["latched_dipole"] is True):
            fails.append(f"leg.mirror-record {m}")
        payload["verdict"] = ("parity green" if not fails
                              else f"PARITY FAIL: {fails}")
        if fails:
            payload["status"] = "fail"
    except Exception as exc:
        write_exception("env_parity_mag", exc)
        print(f"env_parity_mag ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("env_parity_mag", payload)
    c = payload["cells"]["leg"]
    log_text("env_parity_mag",
             "[mag] " + " ".join(
                 f"{k}={v:.3e}" if isinstance(v, float) else f"{k}={v}"
                 for k, v in c.items() if k not in ("mirror", "fixture")))
    print(f"env_parity_mag verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
