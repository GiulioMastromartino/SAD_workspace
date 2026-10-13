"""WP4 aggregate-CoM offset probes (PLAN Rev 1, P-WP4.1..P-WP4.4).

Results-only, stock-Basilisk measurements. All sims <= 30 ticks, guarded,
never raises. Relative paths only. Requires stdlib + numpy + Basilisk.
"""
from __future__ import annotations

import pathlib
import traceback

try:
    from probes.common import env_info, log_text, save_result, write_exception
except Exception:  # never raise; local fallback when harness absent
    def env_info() -> dict:
        return {"fallback": True}

    def log_text(_name: str, _text: str) -> None:
        return None

    def save_result(_pid: str, _data: dict) -> None:
        return None

    def write_exception(_pid: str, _tag: str, _exc: BaseException) -> None:
        return None

try:
    import numpy as np
except Exception:
    np = None  # type: ignore[assignment]

DT = 0.1
NTICKS_SHORT = 20  # <= 30 hard cap for every sim in this file
NTICKS_TINY = 10
OFFSET_B = [0.1, 0.0, 0.0]

# PLAN Rev 1 established contract: 23-field RWConfigLogMsgPayload.
RW_LOG_FIELDS_23 = [
    "rWB_B", "gsHat_B", "w2Hat0_B", "w3Hat0_B", "mass", "theta", "Omega",
    "Js", "Jt", "Jg", "U_s", "U_d", "d", "J13", "u_current",
    "frictionTorque", "u_max", "u_min", "u_f", "Omega_max", "P_max",
    "linearFrictionRatio", "RWModel",
]

PROBE_ID = "wp4_offset"


# Compatibility: call sites below use write_exception(PROBE_ID, tag, exc)
# (3-arg) while probes.common provides write_exception(name, exc) (2-arg).
# Normalize once so every call site works with either helper.
try:
    import inspect as _inspect

    _sig = _inspect.signature(write_exception)
    if len(_sig.parameters) == 2:
        _real_write_exception = write_exception

        def write_exception(pid, tag_or_exc, exc=None):  # noqa: F811
            if exc is None:
                return _real_write_exception(pid, tag_or_exc)
            return _real_write_exception(pid, tag_or_exc if isinstance(tag_or_exc, BaseException) else exc)

except Exception:
    pass


def _wexc(tag: str, exc: BaseException) -> None:
    """2-arg/3-arg tolerant exception save (real common vs shim)."""
    try:
        write_exception(PROBE_ID, exc)  # type: ignore
    except TypeError:
        try:
            write_exception(PROBE_ID, tag, exc)  # type: ignore
        except Exception:
            pass
    except Exception:
        pass


def _payload_names(payload_cls) -> list:
    try:
        tmp = payload_cls()
        names = [a for a in dir(tmp) if not a.startswith("_")]
        return sorted(names)
    except Exception:
        return []


def _repo_root() -> pathlib.Path:
    # audit_bundle/probes/<file> -> native_omni_heyoka root (relative only).
    return pathlib.Path(__file__).resolve().parents[2]


def _grep_consumer_hits(keywords: list) -> dict:
    """Grep cubesat_12u_sim source as text; record per-keyword file hits."""
    out: dict = {}
    try:
        sib = _repo_root().parents[0] / "cubesat_12u_sim"
        roots = [sib] if sib.is_dir() else [_repo_root()]
        for kw in keywords:
            hits: list = []
            for root in roots:
                for fp in root.rglob("*.py"):
                    try:
                        txt = fp.read_text(errors="ignore")
                    except Exception:
                        continue
                    if kw in txt:
                        try:
                            rel = str(fp.relative_to(_repo_root().parents[0]))
                        except Exception:
                            rel = fp.name
                        hits.append(rel)
                        if len(hits) >= 25:
                            break
            out[kw] = {"n_files": len(hits), "files": hits[:25]}
    except Exception as exc:
        out["error"] = f"{type(exc).__name__}: {exc}"
    return out


def p_wp41_rw_census() -> dict:
    """P-WP4.1: stock 3-wheel closed loop, full rwOutMsgs + RWSpeedMsg dump."""
    res: dict = {"id": "P-WP4.1", "ticks": NTICKS_SHORT, "dt": DT}
    try:
        from Basilisk.architecture import messaging
        from Basilisk.simulation import reactionWheelStateEffector as rwEff
        from Basilisk.simulation import spacecraft
        from Basilisk.utilities import (
            SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody,
            simIncludeRW,
        )
        res["rw_log_expected_23"] = list(RW_LOG_FIELDS_23)
        try:
            names = _payload_names(messaging.RWConfigLogMsgPayload)
            res["rw_log_static_fields"] = names
            res["rw_log_static_count"] = len(names)
            res["rw_log_missing_vs_23"] = [f for f in RW_LOG_FIELDS_23 if f not in names]
            res["rw_log_extra_vs_23"] = [f for f in names if f not in RW_LOG_FIELDS_23]
        except Exception as exc:
            res["static_census_error"] = f"{type(exc).__name__}: {exc}"
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp41-stock"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
        oe = orbitalMotion.ClassicElements()
        oe.a, oe.e = 1.0e7, 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
        r0, v0 = orbitalMotion.elem2rv(3.986004418e14, oe)
        sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = r0, v0
        sc.hub.sigma_BNInit = [0.1, 0.2, -0.3]
        sc.hub.omega_BN_BInit = [0.001, -0.01, 0.03]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        rwf = simIncludeRW.rwFactory()
        for gs in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            rwf.create("Honeywell_HR16", gs, maxMomentum=50.0, Omega=100.0)
        rw = rwEff.ReactionWheelStateEffector()
        rwf.addToSpacecraft("rw", rw, sc)
        sim.AddModelToTask("t", rw, 2)
        cmd = messaging.ArrayMotorTorqueMsg()
        pay = messaging.ArrayMotorTorqueMsgPayload()
        pay.motorTorque = [0.02, -0.015, 0.01] + [0.0] * 33
        cmd.write(pay)
        rw.rwMotorCmdInMsg.subscribeTo(cmd)
        recs = []
        try:
            for m in list(rw.rwOutMsgs):
                r = m.recorder()
                sim.AddModelToTask("t", r)
                recs.append(r)
        except Exception as exc:
            res["rw_out_recorder_error"] = f"{type(exc).__name__}: {exc}"
        spd = rw.rwSpeedOutMsg.recorder()
        sim.AddModelToTask("t", spd)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS_SHORT))
        sim.ExecuteSimulation()
        res["n_rw_out_msgs"] = len(recs)
        sample: dict = {}
        try:
            for i, r in enumerate(recs[:3]):
                tick: dict = {}
                for f in ("u_current", "Omega", "frictionTorque", "theta", "Js"):
                    try:
                        tick[f] = float(np.asarray(getattr(r, f)).ravel()[0])
                    except Exception:
                        tick[f] = "unreadable"
                sample[f"wheel_{i}"] = tick
        except Exception as exc:
            sample["error"] = f"{type(exc).__name__}: {exc}"
        res["sample_tick_0"] = sample
        try:
            res["rw_speed_tick_0"] = [float(v) for v in np.asarray(spd.wheelSpeeds)[0][:3]]
            res["rw_speed_nticks"] = int(np.asarray(spd.wheelSpeeds).shape[0])
        except Exception as exc:
            res["rw_speed_error"] = f"{type(exc).__name__}: {exc}"
        res["status"] = "ok"
    except Exception as exc:
        res["status"] = "guarded-fail"
        res["error"] = f"{type(exc).__name__}: {exc}"
        try:
            write_exception(PROBE_ID, "P-WP4.1", exc)
        except Exception:
            pass
    return res


def p_wp42_b_vs_c() -> dict:
    """P-WP4.2: hub offset r_BcB_B=[0.1,0,0]; log BN vs CN + reader grep."""
    res: dict = {"id": "P-WP4.2", "offset_cmd": list(OFFSET_B), "ticks": NTICKS_TINY, "dt": DT}
    try:
        from Basilisk.simulation import spacecraft
        from Basilisk.utilities import (
            SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody,
        )
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp42-stock"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
        if hasattr(sc.hub, "r_BcB_B"):
            sc.hub.r_BcB_B = list(OFFSET_B)
            res["offset_attr"] = "set"
        else:
            res["offset_attr"] = "offset-attr-missing"
        oe = orbitalMotion.ClassicElements()
        oe.a, oe.e = 1.0e7, 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
        r0, v0 = orbitalMotion.elem2rv(3.986004418e14, oe)
        sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = r0, v0
        sc.hub.sigma_BNInit = [0.05, -0.05, 0.02]
        sc.hub.omega_BN_BInit = [0.01, 0.0, 0.0]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        log = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", log)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS_TINY))
        sim.ExecuteSimulation()
        try:
            rBN = np.asarray(log.r_BN_N, float)
            rCN = np.asarray(log.r_CN_N, float)
            res["r_BN_tick0"] = [float(v) for v in rBN[0]]
            res["r_CN_tick0"] = [float(v) for v in rCN[0]]
            res["rBN_minus_rCN_tick0"] = [float(v) for v in (rBN[0] - rCN[0])]
            res["rBN_minus_rCN_norm"] = float(np.linalg.norm(rBN[0] - rCN[0]))
        except Exception as exc:
            res["bn_cn_error"] = f"{type(exc).__name__}: {exc}"
        try:
            sigBN = np.asarray(log.sigma_BN, float)
            res["sigma_BN_tick0"] = [float(v) for v in sigBN[0]]
        except Exception as exc:
            res["sigma_BN_error"] = f"{type(exc).__name__}: {exc}"
        # WP25-A1 signed-component gate (same logic as
        # tests/test_wp4_offset.py:test_signed_component_gate_static_attitude:
        # per-axis r_BN - r_CN + R^T c == 0; norm-only is sign-blind, kept
        # above as secondary).
        try:
            s = np.asarray(res["sigma_BN_tick0"], float)
            S = np.array([[0.0, -s[2], s[1]],
                          [s[2], 0.0, -s[0]],
                          [-s[1], s[0], 0.0]])
            s2 = float(s @ s)
            R = np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2
            rB = np.asarray(res["r_BN_tick0"], float)
            rC = np.asarray(res["r_CN_tick0"], float)
            sres = (rB - rC) + R.T @ np.asarray(OFFSET_B, float)
            res["signed_residual_tick0"] = [float(v) for v in sres]
            res["signed_residual_norm"] = float(np.linalg.norm(sres))
            # 1e-8 here, not 1e-9: double rounding of the |r|~1e7 published
            # rows floors the residual at a few e-9; the binding <=1e-9 gate
            # is the small-magnitude static-attitude fixture in
            # tests/test_wp4_offset.py.
            res["signed_residual_pass_1e8"] = bool(np.all(np.abs(sres) <= 1e-8))
            res["signed_gate_note"] = ("vec `-` wins (WP25-A1): residual "
                                       "r_BN-r_CN+R^T c ~0 per axis; `+` would "
                                       "read 2*R^T c ~= 0.2 m")
        except Exception as exc:
            res["signed_residual_error"] = f"{type(exc).__name__}: {exc}"
        for f in ("omegaDot_BN_B", "TotalAccumDV_BN_B", "TotalAccumDVBdy",
                  "TotalAccumDV_CN_N", "nonConservativeAccelpntB_B",
                  "v_BN_N", "v_CN_N"):
            try:
                res[f + "_tick0"] = [float(v) for v in np.asarray(getattr(log, f))[0]]
            except Exception:
                res[f + "_tick0"] = "missing-or-unreadable"
        res["status"] = "ok"
    except Exception as exc:
        res["status"] = "guarded-fail"
        res["error"] = f"{type(exc).__name__}: {exc}"
        try:
            write_exception(PROBE_ID, "P-WP4.2", exc)
        except Exception:
            pass
    try:
        res["downstream_consumers"] = _grep_consumer_hits(
            ["r_BN_N", "r_CN_N", "scStateInMsg", "attOutMsg", "imuSensor",
             "coarseSunSensor", "simpleNav", "r_BcB_B"])
    except Exception as exc:
        res["consumer_grep_error"] = f"{type(exc).__name__}: {exc}"
    return res


def p_wp43_aggregate() -> dict:
    """P-WP4.3: offset hub + offset wheels rWB_B!=0 + RW mass; mass-props + fuel."""
    res: dict = {"id": "P-WP4.3", "ticks": NTICKS_TINY, "dt": DT}
    try:
        from Basilisk.architecture import messaging
        from Basilisk.simulation import reactionWheelStateEffector as rwEff
        from Basilisk.simulation import spacecraft
        from Basilisk.utilities import (
            SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody,
            simIncludeRW,
        )
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp43-stock"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
        try:
            if hasattr(sc.hub, "r_BcB_B"):
                sc.hub.r_BcB_B = list(OFFSET_B)
                res["hub_offset"] = "set"
            else:
                res["hub_offset"] = "offset-attr-missing"
        except Exception as exc:
            res["hub_offset"] = f"set-fail: {exc}"
        oe = orbitalMotion.ClassicElements()
        oe.a, oe.e = 1.0e7, 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
        r0, v0 = orbitalMotion.elem2rv(3.986004418e14, oe)
        sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = r0, v0
        sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
        sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        rwf = simIncludeRW.rwFactory()
        try:
            rwf.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=50.0,
                       rWB_B=[0.2, 0.0, 0.0])
            rwf.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=50.0,
                       rWB_B=[0.0, 0.2, 0.0])
            res["wheel_offsets"] = "set rWB_B!=0 x2"
        except Exception as exc:
            res["wheel_offsets"] = f"set-fail: {type(exc).__name__}: {exc}"
        rw = rwEff.ReactionWheelStateEffector()
        rwf.addToSpacecraft("rw", rw, sc)
        sim.AddModelToTask("t", rw, 2)
        cmd = messaging.ArrayMotorTorqueMsg()
        pay = messaging.ArrayMotorTorqueMsgPayload()
        pay.motorTorque = [0.0, 0.0] + [0.0] * 34
        cmd.write(pay)
        rw.rwMotorCmdInMsg.subscribeTo(cmd)
        mass_log = None
        try:
            mass_log = sc.massPropsOutMsg.recorder()
            sim.AddModelToTask("t", mass_log)
            res["mass_props_msg"] = "subscribed"
        except Exception as exc:
            res["mass_props_msg"] = f"record-missing: {type(exc).__name__}: {exc}"
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS_TINY))
        sim.ExecuteSimulation()
        if mass_log is not None:
            try:
                for f in ("massSC", "CoM_B", "c_B", "massHub", "ISCPntB_B"):
                    try:
                        res["massprops_" + f] = np.asarray(getattr(mass_log, f)).tolist()
                    except Exception:
                        res["massprops_" + f] = "missing-or-unreadable"
            except Exception as exc:
                res["massprops_read_error"] = f"{type(exc).__name__}: {exc}"
        else:
            res["massprops_read"] = "record-missing"
        try:
            names = _payload_names(messaging.FuelTankMsgPayload)
            res["fuel_msg_fields"] = names
            res["cPrime_note"] = ("record-missing: no depletion run here; "
                                  "cPrime smoke needs fuelTank + thruster burn")
        except Exception as exc:
            res["fuel_msg_error"] = f"{type(exc).__name__}: {exc}"
        res["status"] = "ok"
    except Exception as exc:
        res["status"] = "guarded-fail"
        res["error"] = f"{type(exc).__name__}: {exc}"
        try:
            write_exception(PROBE_ID, "P-WP4.3", exc)
        except Exception:
            pass
    return res


def p_wp44_moment_arm() -> dict:
    """P-WP4.4: single-thruster step with offset; about-B tel vs about-CoM dyn."""
    res: dict = {"id": "P-WP4.4", "ticks": NTICKS_TINY, "dt": DT}
    try:
        from Basilisk.architecture import messaging
        from Basilisk.simulation import spacecraft, thrusterDynamicEffector
        from Basilisk.utilities import (
            SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody,
            simIncludeThruster,
        )
        r_thr = [0.5, 0.0, 1.0]
        t_hat = [0.0, 0.0, 1.0]
        fmax = 1.0
        res["r_thr_B"] = list(r_thr)
        res["t_hat"] = list(t_hat)
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp44-stock"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
        try:
            if hasattr(sc.hub, "r_BcB_B"):
                sc.hub.r_BcB_B = list(OFFSET_B)
                res["hub_offset"] = "set"
            else:
                res["hub_offset"] = "offset-attr-missing"
        except Exception as exc:
            res["hub_offset"] = f"set-fail: {exc}"
        oe = orbitalMotion.ClassicElements()
        oe.a, oe.e = 1.0e7, 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
        r0, v0 = orbitalMotion.elem2rv(3.986004418e14, oe)
        sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = r0, v0
        sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
        sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", list(r_thr), list(t_hat),
                  MaxThrust=fmax, cutoffFrequency=1000.0, useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        on = messaging.THRArrayOnTimeCmdMsg()
        pay = messaging.THRArrayOnTimeCmdMsgPayload()
        pay.OnTimeRequest = [DT] + [0.0] * 35  # 36 slots
        on.write(pay)
        thr.cmdsInMsg.subscribeTo(on)
        try:
            tlog = thr.thrusterOutMsgs[0].recorder()
            sim.AddModelToTask("t", tlog)
            res["thr_out_subscribed"] = True
        except Exception as exc:
            tlog = None
            res["thr_out_subscribed"] = f"fail: {type(exc).__name__}: {exc}"
        slog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", slog)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS_TINY))
        sim.ExecuteSimulation()
        if tlog is not None:
            try:
                # tick0 is pre-fire (staged mailbox idles the fill step);
                # steady values are the LAST tick.
                res["thrustTorquePntB_B_tick0"] = [float(v) for v in np.asarray(tlog.thrustTorquePntB_B)[0]]
                res["thrustTorquePntB_B_steady"] = [float(v) for v in np.asarray(tlog.thrustTorquePntB_B)[-1]]
                res["thrustForce_B_tick0"] = [float(v) for v in np.asarray(tlog.thrustForce_B)[0]]
                res["thrustForce_B_steady"] = [float(v) for v in np.asarray(tlog.thrustForce_B)[-1]]
                res["thrustFactor_tick0"] = float(np.asarray(tlog.thrustFactor).ravel()[0])
                res["thrustFactor_steady"] = float(np.asarray(tlog.thrustFactor).ravel()[-1])
            except Exception as exc:
                res["thr_read_error"] = f"{type(exc).__name__}: {exc}"
        # WP25-C1 firing discipline + supersede pointer: tick0 rows are IDLE
        # (fill step) and are SUPERSEDED by the steady-tick host tests in
        # tests/test_wp4_offset.py:306-333 (steady-tick anchor/bench region:
        # test_stock_finite_difference_anchor_host +
        # test_omni_offset_bench_openloop_both_arms_host, LAST-tick sampling).
        # Probe tick0/idle rows must NEVER be cited as arm evidence; the arm
        # verdict below reads the STEADY row only, and only when firing is
        # proven (thrustFactor_steady > 0 and |thrustForce_steady| > 0 --
        # otherwise the verdict is WITHHELD, never defaulted).
        res["idle_rows_superseded_by"] = (
            "tests/test_wp4_offset.py:306-333 steady-tick anchor/bench "
            "(test_stock_finite_difference_anchor_host + "
            "test_omni_offset_bench_openloop_both_arms_host); probe "
            "tick0/idle rows are not arm evidence")
        try:
            ffac = float(np.asarray(tlog.thrustFactor).ravel()[-1])
            fmag = float(np.linalg.norm(np.asarray(tlog.thrustForce_B, float)[-1]))
            res["firing_proven_steady"] = bool(ffac > 0.0 and fmag > 0.0)
            if not res["firing_proven_steady"]:
                res["verdict_withheld"] = (
                    "leg idle at steady tick (thrustFactor/thrustForce ~0): "
                    "no arm verdict may be read")
        except Exception as exc:
            res["firing_proven_steady"] = False
            res["firing_read_error"] = f"{type(exc).__name__}: {exc}"
        try:
            F = np.array(t_hat, float) * fmax
            expect_unshifted = np.cross(np.array(r_thr, float), F).tolist()
            res["expect_unshifted_r_thr_x_F"] = [float(v) for v in expect_unshifted]
            try:
                if not res.get("firing_proven_steady", False):
                    res["tel_minus_expect"] = "withheld (leg idle)"
                else:
                    tel = np.asarray(tlog.thrustTorquePntB_B, float)[-1]
                    res["tel_minus_expect"] = [float(v) for v in (tel - np.array(expect_unshifted))]
            except Exception:
                res["tel_minus_expect"] = "unreadable"
        except Exception as exc:
            res["expect_error"] = f"{type(exc).__name__}: {exc}"
        try:
            om = np.asarray(slog.omega_BN_B, float)
            res["omega_BN_B_tick0"] = [float(v) for v in om[0]]
            res["omega_BN_B_tick1"] = [float(v) for v in om[1]] if len(om) > 1 else "short"
            res["omegaDot_fd_tick0"] = [float(v) for v in ((om[1] - om[0]) / DT)] if len(om) > 1 else "short"
            res["arm_note"] = ("telemetry about B must equal unshifted r_thr x F; "
                               "achieved omegaDot is about CoM, shifted by (r_thr-c_B) x F")
        except Exception as exc:
            res["omega_read_error"] = f"{type(exc).__name__}: {exc}"
        res["status"] = "ok"
    except Exception as exc:
        res["status"] = "guarded-fail"
        res["error"] = f"{type(exc).__name__}: {exc}"
        try:
            write_exception(PROBE_ID, "P-WP4.4", exc)
        except Exception:
            pass
    return res


def run() -> dict:
    """Run all WP4 probes; never raises."""
    out: dict = {"probe": PROBE_ID}
    try:
        out["env"] = env_info()
    except Exception:
        out["env"] = {"error": "env_info-failed"}
    for fn in (p_wp41_rw_census, p_wp42_b_vs_c, p_wp43_aggregate, p_wp44_moment_arm):
        try:
            out[fn.__name__] = fn()
        except Exception as exc:
            out[fn.__name__] = {"status": "guarded-fail", "error": f"{type(exc).__name__}: {exc}"}
            try:
                write_exception(PROBE_ID, fn.__name__, exc)
            except Exception:
                pass
    try:
        log_text(PROBE_ID, f"WP4 done: {[out.get(k, {}).get('status') for k in ('p_wp41_rw_census', 'p_wp42_b_vs_c', 'p_wp43_aggregate', 'p_wp44_moment_arm')]}")
    except Exception:
        pass
    try:
        save_result(PROBE_ID, out)
    except Exception:
        pass
    return out


def main() -> dict:
    try:
        return run()
    except Exception as exc:
        try:
            write_exception(PROBE_ID, "main", exc)
        except Exception:
            pass
        return {"probe": PROBE_ID, "status": "guarded-fail",
                "error": f"{type(exc).__name__}: {exc}",
                "trace": traceback.format_exc(limit=5)}


if __name__ == "__main__":
    main()
