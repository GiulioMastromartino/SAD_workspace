"""Track B debug — tilt-W3 closed-loop divergence forensics (host-only).

Context: tilt-W3 closed loop shows att 4.8e-05 deg (PASS), rate PASS, but
wheels +0.26 RPM apart and transient 0.0067 deg; allocator commands diverge
(max|du| ~2e-4 Nm) from tick 1 while yaw-10deg agrees to 1e-13 everywhere
and open-loop diagonal dynamics agree at 1e-8. Open question: FSW inputs
(speeds/config) vs allocator vs dynamics under closed-loop profiles.

This probe (short 10-tick pair, prints tables, saves JSON):
 1. Diffs the two legs' FSW RWConfigMsg content field-by-field
    (GsMatrix_B, JsList, uMax, Omega_max, numRW) — kills the "config differs"
    hypothesis statically.
 2. Dumps per-tick (10 ticks) u_cmd[0..2] + W[0..2] + sigma for BOTH arms —
    shows who moves first (commands with identical speeds => FSW-side;
    speeds with identical commands => dynamics-side).

USE (from native_omni_heyoka/):
    ../../env/bin/python audit_bundle/probes/wp7_tilt_debug.py

Matches run_all.sh `wp*.py` glob; guarded try/except; never raises.
"""
from __future__ import annotations

import math
import os
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Standalone-proof: run_all.sh exports the repo Python/ dir on PYTHONPATH,
# but direct invocation (python path/to/probe.py) does not -- omniheyoka's
# __init__ needs the sibling cubesat_12u_sim package. probes/ -> audit_bundle/
# -> native_omni_heyoka/ -> Python/ is three parents up.
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

try:
    from common import env_info, log_text, save_result
except Exception:
    try:
        from probes.common import env_info, log_text, save_result  # type: ignore
    except Exception:
        def env_info() -> dict:  # type: ignore
            return {"fallback": True}

        def log_text(_n: str, _t: str) -> str:  # type: ignore
            return ""

        def save_result(_n: str, _p: dict) -> str:  # type: ignore
            return ""

PROBE_ID = "wp7_tilt_debug"
DT = 0.1
N_TICKS = 10
# LONG mode (env WP7_TILT_DEBUG_LONG=1): 600 s closed-loop pair with
# per-100-tick snapshots. The 10-tick table shows LINEAR early growth
# (du ~5e-8/tick) that extrapolates to 1e-4 RPM -- but the 600 s wheel gap
# is 0.26 RPM, so a discrete mid-run event (shadow switch? clipping?
# reversal?) must amplify it. Snapshots locate the event tick + channel.
LONG = os.environ.get("WP7_TILT_DEBUG_LONG", "0") == "1"
LONG_TICKS = 6000
SNAP_EVERY = 100
# Effective horizon: short table by default, full-run snapshots on LONG.
N_RUN = LONG_TICKS if LONG else N_TICKS
# SWEEP mode (env WP7_TILT_DEBUG_SWEEP=1): configuration bisection. The plant
# is NOT hyper-sensitive (stock-vs-stock+1e-12 stays 1e-12 class), open-loop
# diagonal dynamics agree (1e-8), FSW construction is mirrored -- yet tilt-W3
# closed loop leaves wheels 0.26 RPM apart. Sweep varies ONE factor at a time
# (rates/gains/angle/diagonal-position) to find the trigger. Endpoint metrics
# only (att/dw/rate/transient), printed as a table + saved JSON.
SWEEP = os.environ.get("WP7_TILT_DEBUG_SWEEP", "0") == "1"
HR16_JS = 0.07957747154594767
MC_RPM0 = (100.0, 200.0, 300.0)
N_TILT = 1.0 / math.sqrt(3.0)
GS_TILT_W3 = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (N_TILT, N_TILT, N_TILT)]


def _dump_msg_fields(msg, prefix=""):
    out = {}
    try:
        names = [a for a in dir(msg) if not a.startswith("_")]
    except Exception:
        return {"error": "dir-failed"}
    for nm in names:
        try:
            v = getattr(msg, nm)
        except Exception:
            continue
        try:
            import numpy as _np
            arr = _np.asarray(v, dtype=float).ravel().tolist()
            out[prefix + nm] = arr
        except Exception:
            try:
                out[prefix + nm] = float(v)
            except Exception:
                out[prefix + nm] = str(v)[:80]
    return out


def main() -> dict:
    print("[wp7_tilt_debug] start (10-tick tilt-W3 closed-loop pair)",
          flush=True)
    res: dict = {"probe": PROBE_ID}
    try:
        res["env"] = env_info()
    except Exception:
        res["env"] = {}
    try:
        from Basilisk.utilities import simIncludeRW
        import numpy as np
    except Exception as e:
        res["status"] = "env-missing"
        res["error"] = f"{type(e).__name__}: {e}"
        print(f"[wp7_tilt_debug] env-missing: {type(e).__name__}: {e}",
              flush=True)
        try:
            save_result(PROBE_ID, res)
        except Exception:
            pass
        return res

    # ---- 1. FSW message content diff (static, no sim) ----
    try:
        f1 = simIncludeRW.rwFactory()
        for g, rpm in zip(GS_TILT_W3, MC_RPM0):
            f1.create("custom", [float(x) for x in g], Js=float(HR16_JS),
                      u_max=0.2, Omega=float(rpm))
        m1 = f1.getConfigMessage()
        f2 = simIncludeRW.rwFactory()
        for g, rpm in zip(GS_TILT_W3, MC_RPM0):
            f2.create("custom", [float(x) for x in g], Js=float(HR16_JS),
                      u_max=0.2, Omega=float(rpm))
        m2 = f2.getConfigMessage()
        d1, d2 = _dump_msg_fields(m1), _dump_msg_fields(m2)
        diff = {}
        for k in sorted(set(d1) | set(d2)):
            a, b = d1.get(k), d2.get(k)
            try:
                same = bool(np.allclose(np.asarray(a, dtype=float),
                                        np.asarray(b, dtype=float),
                                        rtol=0, atol=0))
            except Exception:
                same = (a == b)
            if not same:
                diff[k] = {"legA": a, "legB": b}
        res["fsw_msg_diff"] = {"n_diffs": len(diff), "diffs": diff,
                               "keys": sorted(set(d1) | set(d2))}
    except Exception as e:
        res["fsw_msg_diff"] = {"error": f"{type(e).__name__}: {e}"}

    # ---- 2. Short closed-loop pair with per-tick traces ----
    try:
        from Basilisk.architecture import messaging, sysModel
        from Basilisk.fswAlgorithms import (attTrackingError, inertial3D,
                                            mrpFeedback, rwMotorTorque)
        from Basilisk.simulation import (reactionWheelStateEffector,
                                         spacecraft)
        from Basilisk.simulation import simpleNav
        from Basilisk.utilities import (SimulationBaseClass, macros,
                                        orbitalMotion, simHelpers,
                                        simIncludeGravBody)
        import omniheyoka.omni_heyoka as oh
    except Exception as e:
        res["status"] = "env-missing"
        res["error"] = f"{type(e).__name__}: {e}"
        print(f"[wp7_tilt_debug] env-missing (section 2 imports): "
              f"{type(e).__name__}: {e}", flush=True)
        try:
            save_result(PROBE_ID, res)
        except Exception:
            pass
        return res

    def _fsw(sim, task, gs_list, k=3.5, p=30.0):
        nav = simpleNav.SimpleNav()
        sim.AddModelToTask(task, nav)
        ref = inertial3D.inertial3D()
        sim.AddModelToTask(task, ref)
        ref.sigma_R0N = [0.0, 0.0, 0.0]
        err = attTrackingError.attTrackingError()
        sim.AddModelToTask(task, err)
        ctl = mrpFeedback.mrpFeedback()
        sim.AddModelToTask(task, ctl)
        ctl.K = float(k)
        ctl.Ki = -1
        ctl.P = float(p)
        ctl.integralLimit = 2.0 / ctl.Ki * 0.1
        alloc = rwMotorTorque.rwMotorTorque()
        sim.AddModelToTask(task, alloc)
        alloc.controlAxes_B = [float(x) for gs in gs_list for x in gs]
        vc = messaging.VehicleConfigMsg().write(
            messaging.VehicleConfigMsgPayload(ISCPntB_B=[
                900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]))
        return nav, err, ctl, alloc, vc, ref

    def _ics(mu):
        oe = orbitalMotion.ClassicElements()
        oe.a, oe.e = 1.0e7, 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R, 347.8 * macros.D2R,
                                    85.3 * macros.D2R)
        return orbitalMotion.elem2rv(mu, oe)

    traces = {}
    for leg in ("stock", "omni"):
        try:
            sim = SimulationBaseClass.SimBaseClass()
            sim.SetProgressBar(False)
            proc = sim.CreateNewProcess("p")
            proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
            if leg == "stock":
                sc = spacecraft.Spacecraft()
                sc.ModelTag = "bsk-tilt-dbg"
                sc.hub.mHub = 750.0
                sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
                # FLAT 9-list (np2EigenMatrix3d indexes mat[0..8]; a list of
                # diag rows dies at mat[3]).
                sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(
                    [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0])
                sim.AddModelToTask("t", sc, 1)
                grav = simIncludeGravBody.gravBodyFactory()
                earth = grav.createEarth()
                earth.isCentralBody = True
                mu = earth.mu
                grav.addBodiesTo(sc)
                fac = simIncludeRW.rwFactory()
                for g, rpm in zip(GS_TILT_W3, MC_RPM0):
                    fac.create("custom", [float(x) for x in g],
                               Js=float(HR16_JS), u_max=0.2, Omega=float(rpm))
                rw = reactionWheelStateEffector.ReactionWheelStateEffector()
                fac.addToSpacecraft("rw-dbg", rw, sc)
                sim.AddModelToTask("t", rw, 2)
                cmd_src = rw
            else:
                sc = oh.OmniSpacecraft()
                sc.ModelTag = "omni-tilt-dbg"
                sc.setMass(750.0)
                sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
                grav = simIncludeGravBody.gravBodyFactory()
                earth = grav.createEarth()
                earth.isCentralBody = True
                mu = earth.mu
                sc.setMu(float(mu))
                sc.setUseGravityGradient(False)
                sc.setTol(1e-12)
                for g in GS_TILT_W3:
                    sc.addWheel(float(g[0]), float(g[1]), float(g[2]),
                                float(HR16_JS))
                sim.AddModelToTask("t", sc, 1)
                # NO grav.addBodiesTo(sc): OmniSpacecraft is a SysModel, not a
                # Spacecraft (no gravField); it carries its own gravity via
                # setMu, exactly like the test's _run_omni_leg.
                cmd_src = sc
            nav, err, ctl, alloc, vc, ref = _fsw(sim, "t", GS_TILT_W3)
            fac2 = simIncludeRW.rwFactory()
            for g, rpm in zip(GS_TILT_W3, MC_RPM0):
                fac2.create("custom", [float(x) for x in g],
                            Js=float(HR16_JS), u_max=0.2, Omega=float(rpm))
            fsw_msg = fac2.getConfigMessage()
            r0, v0 = _ics(mu)
            if leg == "stock":
                sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
                sc.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
                sc.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]
            else:
                sc.setR_CN_NInit(*map(float, r0))
                sc.setV_CN_NInit(*map(float, v0))
                sc.setSigmaBNInit(0.1, 0.2, -0.3)
                sc.setOmegaBNInit(0.001, -0.01, 0.03)
                sc.setWheelSpeedsInitRPM([float(x) for x in MC_RPM0])
            nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
            err.attNavInMsg.subscribeTo(nav.attOutMsg)
            err.attRefInMsg.subscribeTo(ref.attRefOutMsg)
            ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
            ctl.vehConfigInMsg.subscribeTo(vc)
            ctl.rwParamsInMsg.subscribeTo(fsw_msg)
            ctl.rwSpeedsInMsg.subscribeTo(
                rw.rwSpeedOutMsg if leg == "stock" else sc.rwSpeedOutMsg)
            alloc.rwParamsInMsg.subscribeTo(fsw_msg)
            alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
            (rw.rwMotorCmdInMsg if leg == "stock"
             else sc.rwMotorCmdInMsg).subscribeTo(alloc.rwMotorTorqueOutMsg)
            slog = sc.scStateOutMsg.recorder()
            wlog = (rw.rwSpeedOutMsg if leg == "stock"
                    else sc.rwSpeedOutMsg).recorder()
            clog = alloc.rwMotorTorqueOutMsg.recorder()
            sim.AddModelToTask("t", slog)
            sim.AddModelToTask("t", wlog)
            sim.AddModelToTask("t", clog)
            sim.InitializeSimulation()
            sim.ConfigureStopTime(macros.sec2nano(DT * N_RUN))
            sim.ExecuteSimulation()
            try:
                cmds = np.asarray(clog.motorTorque, float).tolist()
            except Exception:
                try:
                    _cn = [a for a in dir(clog) if not a.startswith("_")]
                    cmds = {"unreadable-field": _cn[:20]}
                except Exception:
                    cmds = "unreadable"
            traces[leg] = {
                "sigma": np.asarray(slog.sigma_BN, float).tolist(),
                "omega": np.asarray(slog.omega_BN_B, float).tolist(),
                "wheels": (np.asarray(wlog.wheelSpeeds, float)[:, :3]
                           * 60.0 / (2.0 * math.pi)).tolist(),
                "cmds": cmds,
            }
        except Exception as e:
            traces[leg] = {"error": f"{type(e).__name__}: {e}",
                           "trace": traceback.format_exc(limit=5)}
    res["traces"] = traces
    for leg in ("stock", "omni"):
        if "error" in traces.get(leg, {}):
            print(f"[wp7_tilt_debug] {leg} leg FAILED: "
                  f"{traces[leg]['error']}", flush=True)
    # Per-tick command + wheel deltas (short mode prints all ticks).
    try:
        o, s = traces["omni"], traces["stock"]
        rows = []
        for k in range(min(len(o["cmds"]), len(s["cmds"]))):
            du = [abs(a - b) for a, b in zip(o["cmds"][k], s["cmds"][k])]
            dw = [abs(a - b) for a, b in zip(o["wheels"][k], s["wheels"][k])]
            rows.append({"tick": k, "max_du": max(du), "du": du,
                         "max_dw_RPM": max(dw), "dw": dw})
        res["per_tick"] = rows if not LONG else rows[:11]
        print("\ntick max|du|[Nm] du[0..2] max|dW|[RPM]", flush=True)
        for r in (rows if not LONG else rows[:11]):
            print("t=%2d du=%.3e %s dW=%.3e %s" % (
                r["tick"], r["max_du"],
                "[" + ",".join("%.1e" % v for v in r["du"]) + "]",
                r["max_dw_RPM"],
                "[" + ",".join("%.1e" % v for v in r["dw"]) + "]"), flush=True)
    except Exception as e:
        res["per_tick_error"] = f"{type(e).__name__}: {e}"
    # LONG snapshots: per-100-tick att/dW/du + max-jump tick per channel.
    # Event-tick + channel localization for the mid-run amplification.
    if LONG:
        try:
            import numpy as _np

            def _att(a, b):
                d = _np.asarray(a, float) - _np.asarray(b, float)
                return float(_np.linalg.norm(d))

            o, s = traces["omni"], traces["stock"]
            n = min(len(o["sigma"]), len(s["sigma"]),
                    len(o["cmds"]), len(s["cmds"]),
                    len(o["wheels"]), len(s["wheels"]))
            snaps = []
            for k in range(0, n, SNAP_EVERY):
                du = max(abs(a - b) for a, b in zip(o["cmds"][k], s["cmds"][k]))
                dw = max(abs(a - b) for a, b in zip(o["wheels"][k], s["wheels"][k]))
                snaps.append({"tick": k, "att_sig_norm": _att(o["sigma"][k], s["sigma"][k]),
                              "max_du": du, "max_dw_RPM": dw})
            res["snapshots"] = snaps
            print("\nLONG snapshots every %d ticks: tick |sig| max|du| max|dW|" % SNAP_EVERY, flush=True)
            for r in snaps:
                print("t=%5d sig=%.3e du=%.3e dW=%.3e" % (
                    r["tick"], r["att_sig_norm"], r["max_du"],
                    r["max_dw_RPM"]), flush=True)
            # Jump detection: largest single-interval growth per channel.
            for key in ("att_sig_norm", "max_du", "max_dw_RPM"):
                vals = [r[key] for r in snaps]
                jumps = [(b - a, snaps[i + 1]["tick"])
                         for i, (a, b) in enumerate(zip(vals, vals[1:]))]
                if jumps:
                    jv, jt = max(jumps, key=lambda t: t[0])
                    print(f"[wp7_tilt_debug] max {key} jump +{jv:.3e} at tick {jt}",
                          flush=True)
                    res[f"max_{key}_jump"] = {"dtick": jt, "jump": jv}
        except Exception as e:
            res["snapshot_error"] = f"{type(e).__name__}: {e}"

    # SWEEP mode: configuration bisection (one factor at a time). Endpoint
    # metrics only; the LONG table above already characterized tilt-W3.
    if SWEEP:
        try:
            import numpy as _snp
            from Basilisk.architecture import messaging as _msg2
            from Basilisk.architecture import sysModel as _sys2
            from Basilisk.simulation import reactionWheelStateEffector as _rw2
            from Basilisk.simulation import spacecraft as _sc2
            from Basilisk.utilities import (SimulationBaseClass as _SBC,
                                            macros as _mac, orbitalMotion as _om,
                                            simHelpers as _sh,
                                            simIncludeGravBody as _gb,
                                            simIncludeRW as _rwf)

            _nt = 1.0 / math.sqrt(3.0)

            def _sweep_pair(gs_list, rpm_list, omega0, k, p, ticks=6000,
                            tol=1e-12, omni_tol_b=None,
                            sigma0=(0.1, 0.2, -0.3)):
                # omni_tol_b set => the "stock" slot runs a SECOND omni build
                # at that tol (omni-vs-omni integrator-sensitivity control).
                outs = {}
                legs = ("omniA", "omniB") if omni_tol_b is not None else ("stock", "omni")
                for _leg in legs:
                    _sim = _SBC.SimBaseClass()
                    _sim.SetProgressBar(False)
                    _proc = _sim.CreateNewProcess("p")
                    _proc.addTask(
                        _sim.CreateNewTask("t", _mac.sec2nano(DT)))
                    if _leg == "stock":
                        _sc = _sc2.Spacecraft()
                        _sc.ModelTag = "swp-bsk"
                        _sc.hub.mHub = 750.0
                        _sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
                        _sc.hub.IHubPntBc_B = _sh.np2EigenMatrix3d(
                            [900.0, 0.0, 0.0, 0.0, 800.0, 0.0,
                             0.0, 0.0, 600.0])
                        _sim.AddModelToTask("t", _sc, 1)
                        _grav = _gb.gravBodyFactory()
                        _earth = _grav.createEarth()
                        _earth.isCentralBody = True
                        _grav.addBodiesTo(_sc)
                        _fac = _rwf.rwFactory()
                        for _g, _rpm in zip(gs_list, rpm_list):
                            _fac.create(
                                "custom", [float(x) for x in _g],
                                Js=float(HR16_JS), u_max=0.2,
                                Omega=float(_rpm))
                        _rw = _rw2.ReactionWheelStateEffector()
                        _fac.addToSpacecraft("swp", _rw, _sc)
                        _sim.AddModelToTask("t", _rw, 2)
                    else:
                        _sc = oh.OmniSpacecraft()
                        _sc.ModelTag = "swp-omni"
                        _sc.setMass(750.0)
                        _sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
                        _grav = _gb.gravBodyFactory()
                        _earth = _grav.createEarth()
                        _earth.isCentralBody = True
                        _sc.setMu(float(_earth.mu))
                        _sc.setUseGravityGradient(False)
                        _sc.setTol(float(omni_tol_b if _leg == "omniB" else tol))
                        for _g in gs_list:
                            _sc.addWheel(float(_g[0]), float(_g[1]),
                                         float(_g[2]), float(HR16_JS))
                        _sim.AddModelToTask("t", _sc, 1)
                    _nav, _err, _ctl, _alloc, _vc, _ref = _fsw(
                        _sim, "t", gs_list, k=k, p=p)
                    _fac2 = _rwf.rwFactory()
                    for _g, _rpm in zip(gs_list, rpm_list):
                        _fac2.create(
                            "custom", [float(x) for x in _g],
                            Js=float(HR16_JS), u_max=0.2, Omega=float(_rpm))
                    _fsw_msg = _fac2.getConfigMessage()
                    _oe = _om.ClassicElements()
                    _oe.a, _oe.e = 1.0e7, 0.01
                    _oe.i = 33.3 * _mac.D2R
                    _oe.Omega, _oe.omega, _oe.f = (
                        48.2 * _mac.D2R, 347.8 * _mac.D2R, 85.3 * _mac.D2R)
                    _r0, _v0 = _om.elem2rv(float(_earth.mu), _oe)
                    if _leg == "stock":
                        _sc.hub.r_CN_NInit = list(_r0)
                        _sc.hub.v_CN_NInit = list(_v0)
                        _sc.hub.sigma_BNInit = [[sigma0[0]], [sigma0[1]],
                                                  [sigma0[2]]]
                        _sc.hub.omega_BN_BInit = [
                            [omega0[0]], [omega0[1]], [omega0[2]]]
                    else:
                        _sc.setR_CN_NInit(*map(float, _r0))
                        _sc.setV_CN_NInit(*map(float, _v0))
                        _sc.setSigmaBNInit(float(sigma0[0]),
                                           float(sigma0[1]),
                                           float(sigma0[2]))
                        _sc.setOmegaBNInit(*map(float, omega0))
                        _sc.setWheelSpeedsInitRPM([float(x) for x in rpm_list])
                    _nav.scStateInMsg.subscribeTo(_sc.scStateOutMsg)
                    _err.attNavInMsg.subscribeTo(_nav.attOutMsg)
                    _err.attRefInMsg.subscribeTo(_ref.attRefOutMsg)
                    _ctl.guidInMsg.subscribeTo(_err.attGuidOutMsg)
                    _ctl.vehConfigInMsg.subscribeTo(_vc)
                    _ctl.rwParamsInMsg.subscribeTo(_fsw_msg)
                    _ctl.rwSpeedsInMsg.subscribeTo(
                        _rw.rwSpeedOutMsg if _leg == "stock"
                        else _sc.rwSpeedOutMsg)
                    _alloc.rwParamsInMsg.subscribeTo(_fsw_msg)
                    _alloc.vehControlInMsg.subscribeTo(_ctl.cmdTorqueOutMsg)
                    ((_rw.rwMotorCmdInMsg if _leg == "stock"
                      else _sc.rwMotorCmdInMsg).subscribeTo(
                          _alloc.rwMotorTorqueOutMsg))
                    _slog = _sc.scStateOutMsg.recorder()
                    _wlog = (_rw.rwSpeedOutMsg if _leg == "stock"
                             else _sc.rwSpeedOutMsg).recorder()
                    _sim.AddModelToTask("t", _slog)
                    _sim.AddModelToTask("t", _wlog)
                    # Applied-torque recorders (u_current per wheel): the
                    # missing datum -- commands (allocator output) may agree
                    # while APPLIED torques differ (saturation/clipping path)
                    # or vice versa. Errors are RECORDED (never silent).
                    _ulogs = []
                    _ucur_error = None
                    try:
                        if _leg == "stock":
                            _msgs = list(_rw.rwOutMsgs)
                        else:
                            # Omni rwOutMsgs is a VALUE vector: list() may hand
                            # back COPIES (recorders would freeze at zeros --
                            # measured sat_omni=0.0/ducur=0.2 exactly). Prefer
                            # the live-object pointer accessor; fall back to
                            # list() only if the accessor is absent.
                            try:
                                _msgs = [_sc.rwLogOutMsg(i)
                                         for i in range(len(gs_list))]
                                _ucur_via = "rwLogOutMsg(i)"
                            except Exception:
                                _msgs = list(_sc.rwOutMsgs)
                                _ucur_via = "list(rwOutMsgs)"
                        for _m in _msgs[:len(gs_list)]:
                            _r = _m.recorder()
                            _sim.AddModelToTask("t", _r)
                            _ulogs.append(_r)
                        if _leg != "stock":
                            res.setdefault("ucur_via", {})[_leg] = _ucur_via
                    except Exception as _e:
                        _ucur_error = (f"recorder-setup: {type(_e).__name__}: "
                                       f"{_e}")
                        _ulogs = []
                    _sim.InitializeSimulation()
                    _sim.ConfigureStopTime(_mac.sec2nano(DT * ticks))
                    _sim.ExecuteSimulation()
                    _ucur = None
                    try:
                        if _ulogs:
                            _ucur = _snp.asarray(
                                [_u.u_current for _u in _ulogs],
                                float).T.tolist()
                        elif _ucur_error is None:
                            _ucur_error = "no-recorders"
                    except Exception as _e2:
                        _ucur_error = (f"readback: {type(_e2).__name__}: "
                                       f"{_e2}")
                        _ucur = None
                    outs[_leg] = {
                        "sg": _snp.asarray(_slog.sigma_BN, float),
                        "om": _snp.asarray(_slog.omega_BN_B, float),
                        "wh": (_snp.asarray(_wlog.wheelSpeeds, float)
                               [:, :len(gs_list)] * 60.0 / (2.0 * math.pi)),
                        "ucur": _ucur,
                        "ucur_error": _ucur_error,
                    }
                return outs

            def _epair_metric(o, s):
                import math as _m

                # Principal-error via MRP composition (shadow-safe).
                def _att(x, y):
                    a = _snp.asarray(x, float).reshape(3)
                    b = _snp.asarray(y, float).reshape(3)
                    n1, n2 = float(a @ a), float(b @ b)
                    num = ((1 - n2) * a - (1 - n1) * b
                           - 2 * _snp.cross(a, b))
                    den = 1 + n1 * n2 + 2 * float(a @ b)
                    return float(_snp.degrees(
                        4.0 * _m.atan(float(_snp.linalg.norm(num / den)))))
                n = min(len(o["sg"]), len(s["sg"]))
                tr = [_att(o["sg"][k], s["sg"][k]) for k in range(n)]
                dw = [max(abs(a - b) for a, b in zip(o["wh"][k], s["wh"][k]))
                      for k in range(n)]
                # Applied-torque gap + saturation footprint (None when
                # a leg exposes no ucur recorders -- with the WHY attached).
                ducur, sat_o, sat_s = None, None, None
                ucur_err = {"omni": o.get("ucur_error"),
                            "stock": s.get("ucur_error")}
                try:
                    if o.get("ucur") is not None and s.get("ucur") is not None:
                        m = min(len(o["ucur"]), len(s["ucur"]))
                        ducur = max(
                            max(abs(a - b) for a, b in zip(o["ucur"][k], s["ucur"][k]))
                            for k in range(m))
                        sat_o = sum(
                            1 for k in range(m) for v in o["ucur"][k] if abs(v) >= 0.199) / max(1, 3 * m)
                        sat_s = sum(
                            1 for k in range(m) for v in s["ucur"][k] if abs(v) >= 0.199) / max(1, 3 * m)
                except Exception:
                    pass
                return tr[-1], max(tr), max(dw), ducur, sat_o, sat_s, ucur_err

            _sweep_cfgs = [
                ("base tilt-W3", GS_TILT_W3, list(MC_RPM0),
                 (0.001, -0.01, 0.03), 3.5, 30.0),
                ("zerorate", GS_TILT_W3, list(MC_RPM0),
                 (0.0, 0.0, 0.0), 3.5, 30.0),
                ("halfgain", GS_TILT_W3, list(MC_RPM0),
                 (0.001, -0.01, 0.03), 1.75, 15.0),
                ("yaw30", [(math.cos(math.radians(30)), math.sin(math.radians(30)), 0.0),
                           (-math.sin(math.radians(30)), math.cos(math.radians(30)), 0.0),
                           (0.0, 0.0, 1.0)], list(MC_RPM0),
                 (0.001, -0.01, 0.03), 3.5, 30.0),
                # NOTE: no 4-wheel config here: stock rwMotorTorque enforces
                # len(controlAxes_B) == 3*3 (ValueError above), so 4-wheel
                # closed-loop parity cannot run in this FSW harness at all.
                # 4-wheel tilted coverage comes from the Update-2 pipeline
                # record + open-loop isolator, not from this sweep.
                ("aligned", [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)],
                 list(MC_RPM0), (0.001, -0.01, 0.03), 3.5, 30.0),
                # smallangle: SAME diagonal geometry, 10x smaller maneuver
                # (torques stay below the 0.2 clip: NOTHING saturates, so
                # clip-edge timing jitter cannot seed anything). Decisive
                # control: green at 1e-9 class PROVES the tilt-W3 wheel gap
                # is saturation-edge jitter (harness regime), not a model
                # gap; red reopens the contributor.
                ("smallangle", GS_TILT_W3, list(MC_RPM0),
                 (0.001, -0.01, 0.03), 3.5, 30.0, 1e-12, None,
                 (0.01, 0.02, -0.03)),
                # tolsens: omni-vs-omni at tol 1e-12 vs 1e-14 (NO stock leg).
                # Decides integrator-mismatch vs model bug: if these two
                # differ at ~0.1 deg class, the trajectory is integrator-
                # sensitive at that level regardless of model correctness
                # (gate becomes envelope); if they agree ~1e-9, the stock
                # gap is a MODEL gap.
                ("tolsens", GS_TILT_W3, list(MC_RPM0),
                 (0.001, -0.01, 0.03), 3.5, 30.0, 1e-12, 1e-14),
            ]
            _sweep_rows = []
            print("\nSWEEP tilt-W3 bisection (600 s endpoint metrics):", flush=True)
            print(f"{'cfg':>14} {'att[deg]':>12} {'trans[deg]':>12} "
                  f"{'wheels[RPM]':>12} {'maxdUcur':>12} {'satO':>7} {'satS':>7}",
                  flush=True)
            for _cfg in _sweep_cfgs:
                _tag, _gs, _rpm, _om0, _k, _p = _cfg[:6]
                _tol_b = _cfg[7] if len(_cfg) > 7 else None
                _tol_a = _cfg[6] if len(_cfg) > 6 else 1e-12
                _sig0 = _cfg[8] if len(_cfg) > 8 else (0.1, 0.2, -0.3)
                try:
                    _pair = _sweep_pair(_gs, _rpm, _om0, _k, _p,
                                        tol=_tol_a, omni_tol_b=_tol_b,
                                        sigma0=_sig0)
                    if _tol_b is None:
                        _x, _y = _pair["omni"], _pair["stock"]
                    else:
                        # tolsens: omni(tol_a) vs omni(tol_b), no stock leg.
                        _x, _y = _pair["omniB"], _pair["omniA"]
                    _m = _epair_metric(_x, _y)
                    _a, _t, _d = _m[0], _m[1], _m[2]
                    _du, _so, _ss, _uerr = _m[3], _m[4], _m[5], _m[6]
                    _sweep_rows.append({"cfg": _tag, "att": _a, "trans": _t,
                                        "dw": _d, "ducur": _du,
                                        "sat_omni": _so, "sat_stock": _ss,
                                        "ucur_error": _uerr})
                    print(f"{_tag:>14} {_a:12.3e} {_t:12.3e} {_d:12.3e} "
                          f"{str(_du):>12} {str(_so):>7} {str(_ss):>7}",
                          flush=True)
                    if _du is None:
                        print(f"  ucur_error: {_uerr}", flush=True)
                except Exception as _e:
                    import traceback as _tb
                    _tb_tail = _tb.format_exc(limit=6)[-600:]
                    _sweep_rows.append({"cfg": _tag,
                                        "error": f"{type(_e).__name__}: {_e}",
                                        "trace": _tb_tail})
                    print(f"{_tag:>14} FAILED {type(_e).__name__}: {_e}",
                          flush=True)
                    print(f"  trace: {_tb_tail}", flush=True)
            res["sweep"] = _sweep_rows
        except Exception as e:
            res["sweep_error"] = f"{type(e).__name__}: {e}"
    res["status"] = "ok"
    try:
        path = save_result(PROBE_ID, res)
        print(f"[wp7_tilt_debug] done, saved {path}", flush=True)
    except Exception as e:
        print(f"[wp7_tilt_debug] save failed: {type(e).__name__}: {e}",
              flush=True)
    return res


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise
