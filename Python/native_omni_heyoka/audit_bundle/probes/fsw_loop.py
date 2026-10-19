"""WP-C4 probe (FSW-in-the-loop ledger): PD closed-loop leg loop comparison.

Mission leg (matched dt=0.1, T=600 s, ONE leg pair -- not a sweep): the
accepted closed_loop_fuel.py leg verbatim (hub 740 kg dry + 10 kg prop, 3x
HR16 wheels under the same K=3.5/P=30/u_max=0.2 PD class, torque-balanced
stationkeeping burn single TEST_Thruster 5 N duty 0.5 fresh-write driver
with the line of action through the hub CoM, ONE CENTERED depleting fuel
tank with c_B == 0 exactly, central gravity). Stock-RK4 arm vs omni-vec
arm (OmniVecIntegrator via omniheyoka.swap.mirror_stock_config).

Loop measurements beyond the fuel leg (list.md section C4 -- pieces
existed, the tick-by-tick loop comparison had no ledger):
  MRP-switch trace: full per-tick stock MRPSwitchCount series from the
    scStateOutMsg recorder + leg totals both arms (stock final vs
    oi.getMRPSwitchCount()). No omni per-tick poller is added: per-tick
    polling (test_vec_mrp_switch.py _SwitchPoller) is NOT a
    closed_loop_fuel.py pattern, so the omni side records the leg total
    only (B2.5 provenance: tick-exact equality is NOT required --
    saturation-edge timing jitter can flip individual ticks).
  saturation edges: first tick each wheel's PD command hits u_max (measured
    per arm from the ArrayMotorTorqueMsg recorder) + first tick each
    wheel's speed hits 95% of the STOCK leg-max for that wheel (relative
    edge -- closed_loop_fuel.py carries no OmegaMax constant; the 0.95
    fraction is INVENTED-relative, flagged, and applied with the
    stock-derived threshold on BOTH arms so the tick comparison is fair).
  allocator chatter: THRArrayOnTime command-change count per leg per arm
    (successive-tick transitions of OnTimeRequest[0]) + firing-tick count
    (ticks with OnTimeRequest[0] > 0) from the THRArrayOnTimeCmdMsg
    recorder. The constant-duty driver writes duty*DT every tick, so the
    honest expectation is chatter 0 / fire ~ticks on both arms.

Rotation is reported BOTH ways (switching class -- the PD law stabilizes
the leg so no shadow-set crossing is expected): sigma-vector final diff
AND switch-invariant DCM trajectory-max. Wheels carry the coupled-state
switching bound (test_swap_multi class). Steps==ticks AND calls==ticks
audit (L14 single-spacecraft path -- no double-step).

Recorder note (composed, not copied -- see RETURN manifest): pd.out /
burn.out recorders reuse ONLY closed_loop_fuel.py elements (message names
ArrayMotorTorqueMsg / THRArrayOnTimeCmdMsg + the .recorder() +
AddModelToTask pattern); no new factory/message/recorder call is
introduced. If the host rejects .recorder() on a driver message, the
copied write_exception path records it loudly (never silent).

Run: ../../env/bin/python audit_bundle/probes/fsw_loop.py
Writes: audit_bundle/results/fsw_loop.json
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

# Loop-ledger additions (stated tolerances, INVENTED values -- B2.5 jitter
# provenance: tick-exact equality is NOT required, saturation-edge timing
# can flip individual ticks; the 5-tick window on a 6000-tick leg is 0.08%):
U_MAX = 0.2  # == PD u_max fixture below (copied, not chosen)
U_EPS = 1e-9  # saturation-hit band (INVENTED; float-write exactness guard)
W_FRAC = 0.95  # INVENTED-relative: no OmegaMax constant in the source leg
SAT_TOL_TICKS = 5  # INVENTED value; mechanism per B2.5 (see above)


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


def _build(use_omni):
    """One arm of the FSW loop leg. Returns handles dict."""
    import numpy as np  # noqa: F401
    from Basilisk.simulation import (fuelTank, reactionWheelStateEffector,
                                     thrusterDynamicEffector)
    from Basilisk.utilities import simIncludeRW, simIncludeThruster
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config

    sim, sc, r0 = _base_sim(("omni-" if use_omni else "rk-") + "fswloop")
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
    tf.create("TEST_Thruster", list(THR_LOC), list(THR_DIR), MaxThrust=5.0,
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
    tank.setR_TB_B(list(R_TB))
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
                    f"fsw_loop: oi.{fn} missing -- vec C++ spec "
                    f"not applied to this wheel build.")
        mirror_stock_config(sim, sc, oi, thr_factory=tf,
                            thr_cmd_msg=burn.out)
        sc.setIntegrator(oi)
    scLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", scLog)
    spdLog = rw.rwSpeedOutMsg.recorder()
    sim.AddModelToTask("t", spdLog)
    # Loop recorders: same .recorder()+AddModelToTask pattern on the
    # already-present driver messages (composed use -- see docstring).
    pdLog = pd.out.recorder()
    sim.AddModelToTask("t", pdLog)
    thrLog = burn.out.recorder()
    sim.AddModelToTask("t", thrLog)
    return {"sim": sim, "sc": sc, "r0": r0, "scLog": scLog,
            "spdLog": spdLog, "pdLog": pdLog, "thrLog": thrLog, "oi": oi}


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


def _sat_u_edges(pd_log):
    """First tick each wheel's |command| hits u_max (None if never)."""
    import numpy as np
    m = np.asarray(pd_log.motorTorque, float)
    edges = []
    for i in range(3):
        hit = [k for k in range(m.shape[0]) if abs(float(m[k][i]))
               >= U_MAX - U_EPS]
        edges.append(int(hit[0]) if hit else None)
    return edges


def _w_edges(spd_stock, spd_omni):
    """First tick each wheel hits W_FRAC of the STOCK leg-max (both arms).

    Thresholds are stock-derived and applied to both arms (fair tick
    comparison). INVENTED-relative -- no OmegaMax constant in the source.
    """
    import numpy as np
    a = np.asarray(spd_stock.wheelSpeeds, float)
    b = np.asarray(spd_omni.wheelSpeeds, float)
    thresh, wmax_s, wmax_o, es, eo = [], [], [], [], []
    for i in range(3):
        ca = np.abs(a[:, i])
        cb = np.abs(b[:, i])
        ms, mo = float(ca.max()), float(cb.max())
        th = W_FRAC * ms if ms > 0.0 else 0.0
        hs = [k for k in range(ca.shape[0]) if float(ca[k]) >= th]
        ho = [k for k in range(cb.shape[0]) if float(cb[k]) >= th]
        thresh.append(float(th))
        wmax_s.append(ms)
        wmax_o.append(mo)
        es.append(int(hs[0]) if hs else None)
        eo.append(int(ho[0]) if ho else None)
    return thresh, wmax_s, wmax_o, es, eo


def _thr_stats(thr_log):
    """Firing-tick count + command-change count for OnTimeRequest[0]."""
    import numpy as np
    ot = np.asarray(thr_log.OnTimeRequest, float)[:, 0]
    fire = int((ot > 0.0).sum())
    changes = int((ot[1:] != ot[:-1]).sum()) if ot.shape[0] > 1 else 0
    change_ticks = [int(k) for k in range(1, ot.shape[0])
                    if float(ot[k]) != float(ot[k - 1])]
    return fire, changes, change_ticks


def _edges_match(es, eo, tol):
    """Tick-match allowing None==None (never saturated on either arm)."""
    for a, b in zip(es, eo):
        if a is None and b is None:
            continue
        if a is None or b is None:
            return False
        if abs(a - b) > tol:
            return False
    return True


def _leg():
    """Stock-vs-omni FSW loop leg. Returns the cell dict."""
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
                              "wheels": 3, "ic": "tumbling-SIG0/OM0"},
                  "sat_tol_ticks": SAT_TOL_TICKS, "w_frac": W_FRAC}
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
    cell["fuel_burned_omni"] = PROP0 - fb
    # MRP-switch trace: full per-tick stock series + totals both arms.
    trace = [int(v) for v in np.asarray(
        getattr(log_s, "MRPSwitchCount")).ravel().tolist()]
    cell["stock_mrp_trace"] = trace
    cell["mrp_trace_len"] = len(trace)
    cell["mrp_stock"] = int(trace[-1]) if trace else 0
    cell["mrp_omni"] = int(oi.getMRPSwitchCount())
    # Saturation edges (u_max measured per arm; wheel edge stock-relative).
    cell["sat_u_stock"] = _sat_u_edges(arm_s["pdLog"])
    cell["sat_u_omni"] = _sat_u_edges(arm_o["pdLog"])
    th, wms, wmo, ews, ewo = _w_edges(arm_s["spdLog"], arm_o["spdLog"])
    cell["w_edge_thresh"] = th
    cell["w_max_stock"] = wms
    cell["w_max_omni"] = wmo
    cell["sat_w_stock"] = ews
    cell["sat_w_omni"] = ewo
    # Allocator chatter per arm.
    fs, cs, cts = _thr_stats(arm_s["thrLog"])
    fo, co, cto = _thr_stats(arm_o["thrLog"])
    cell["thr_fire_stock"] = fs
    cell["thr_fire_omni"] = fo
    cell["thr_chatter_stock"] = cs
    cell["thr_chatter_omni"] = co
    cell["thr_change_ticks_stock"] = cts
    cell["thr_change_ticks_omni"] = cto
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


def main() -> int:
    if not safe_import_basilisk():
        save_result("fsw_loop",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    try:
        payload: dict = {"status": "ok", "cells": {}, "dt": DT,
                         "t_leg": T_LEG, "ticks": TICKS,
                         "loadavg": _loadavg()}
        payload["cells"]["leg"] = _leg()
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
        if not _edges_match(c["sat_u_stock"], c["sat_u_omni"],
                            SAT_TOL_TICKS):
            fails.append(f"leg.sat_u stock={c['sat_u_stock']} "
                         f"omni={c['sat_u_omni']}")
        if not _edges_match(c["sat_w_stock"], c["sat_w_omni"],
                            SAT_TOL_TICKS):
            fails.append(f"leg.sat_w stock={c['sat_w_stock']} "
                         f"omni={c['sat_w_omni']}")
        if not c["steps"] == TICKS:
            fails.append(f"leg.steps={c['steps']}")
        if not c["calls"] == TICKS:
            fails.append(f"leg.calls={c['calls']}")
        for arm in ("thr_fire_stock", "thr_fire_omni"):
            if not c[arm] > 0:
                fails.append(f"leg.vacuous {arm}={c[arm]}")
        if not c["thr_chatter_stock"] == c["thr_chatter_omni"]:
            fails.append(f"leg.chatter stock={c['thr_chatter_stock']} "
                         f"omni={c['thr_chatter_omni']}")
        payload["verdict"] = ("parity green" if not fails
                              else f"PARITY FAIL: {fails}")
        if fails:
            payload["status"] = "fail"
    except Exception as exc:
        from common import write_exception  # noqa: E402
        write_exception("fsw_loop", exc)
        print(f"fsw_loop ERROR: {type(exc).__name__}: {exc}")
        return 1
    save_result("fsw_loop", payload)
    c = payload["cells"]["leg"]
    log_text("fsw_loop",
             "[leg] " + " ".join(
                 f"{k}={v:.2e}" if isinstance(v, float) else f"{k}={v}"
                 for k, v in c.items()
                 if k not in ("stock_mrp_trace", "thr_change_ticks_stock",
                              "thr_change_ticks_omni")))
    log_text("fsw_loop",
             "[loop] mrp_stock=%s mrp_omni=%s trace_len=%s "
             "sat_u_stock=%s sat_u_omni=%s sat_w_stock=%s sat_w_omni=%s "
             "thr_fire=%s/%s chatter=%s/%s steps=%s calls=%s" % (
                 c["mrp_stock"], c["mrp_omni"], c["mrp_trace_len"],
                 c["sat_u_stock"], c["sat_u_omni"], c["sat_w_stock"],
                 c["sat_w_omni"], c["thr_fire_stock"], c["thr_fire_omni"],
                 c["thr_chatter_stock"], c["thr_chatter_omni"],
                 c["steps"], c["calls"]))
    print(f"fsw_loop verdict: {payload['verdict']}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
