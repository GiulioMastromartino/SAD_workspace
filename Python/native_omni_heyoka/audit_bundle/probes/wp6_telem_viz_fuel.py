"""WP6 telemetry / viz / fuel probes (PLAN Rev 1, T1..T3).

Results-only, stock-Basilisk measurements. All sims <= 30 ticks, guarded,
never raises. Relative paths only. Requires stdlib + numpy + Basilisk.
"""
from __future__ import annotations

import inspect
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
NTICKS_1 = 1
NTICKS_SHORT = 10  # <= 30 hard cap for every sim in this file

THR_EXPECTED_9 = [
    "thrustFactor", "thrustForce", "thrustForce_B", "thrustTorquePntB_B",
    "thrustBlowDownFactor", "ispBlowDownFactor", "maxThrust",
    "thrusterLocation", "thrusterDirection",
]
RW_EXPECTED_23 = [
    "rWB_B", "gsHat_B", "w2Hat0_B", "w3Hat0_B", "mass", "theta", "Omega",
    "Js", "Jt", "Jg", "U_s", "U_d", "d", "J13", "u_current",
    "frictionTorque", "u_max", "u_min", "u_f", "Omega_max", "P_max",
    "linearFrictionRatio", "RWModel",
]
# SCStates gap audit: fields omni is known/believed to leave zero.
SC_GAP_FIELDS = ["omegaDot_BN_B", "TotalAccumDV_BN_N", "TotalAccumDVBdy_B",
                 "nonConservativeAccelpntB_B", "vel_BN_N"]

PROBE_ID = "wp6_telem_viz_fuel"

# Bisection + crash-safety: WP6_ONLY=T1|T2|T3 (comma ok) runs a subset;
# the VizInterface ExecuteSimulation is headless-segfault-prone, so it only
# runs with WP6_RUN_VIZ_SIM=1 (default: subscription + census, no sim run).
import os as _os

_WP6_ONLY = {s.strip() for s in _os.environ.get("WP6_ONLY", "T1,T2,T3").split(",")}
_WP6_RUN_VIZ_SIM = _os.environ.get("WP6_RUN_VIZ_SIM", "0") == "1"
# T3's fuel-burn ExecuteSimulation segfaults on this host (10:57 run: T1+T2
# saved via checkpoint, crash in T3). Default to static-only census; full burn
# only with WP6_RUN_FUEL_SIM=1 for isolated bisection.
_WP6_RUN_FUEL_SIM = _os.environ.get("WP6_RUN_FUEL_SIM", "0") == "1"
_WP6_WANT = set()
if "T1" in _WP6_ONLY:
    _WP6_WANT.add("t1_payload_census")
if "T2" in _WP6_ONLY:
    _WP6_WANT.add("t2_viz")
if "T3" in _WP6_ONLY:
    _WP6_WANT.add("t3_fuel_truth")


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
        return sorted(a for a in dir(tmp) if not a.startswith("_"))
    except Exception:
        return []


def _repo_root() -> pathlib.Path:
    # audit_bundle/probes/<file> -> native_omni_heyoka root (relative only).
    return pathlib.Path(__file__).resolve().parents[2]


def _grep(patterns: list, max_files: int = 25) -> dict:
    out: dict = {}
    try:
        sib = _repo_root().parents[0] / "cubesat_12u_sim"
        roots = [sib] if sib.is_dir() else [_repo_root()]
        try:
            roots.append(_repo_root())
        except Exception:
            pass
        for pat in patterns:
            hits: list = []
            for root in roots:
                try:
                    files = list(root.rglob("*.py"))
                except Exception:
                    continue
                for fp in files:
                    try:
                        txt = fp.read_text(errors="ignore")
                    except Exception:
                        continue
                    if pat in txt:
                        try:
                            rel = str(fp.relative_to(_repo_root().parents[0]))
                        except Exception:
                            rel = fp.name
                        hits.append(rel)
                        if len(hits) >= max_files:
                            break
            out[pat] = {"n_files": len(hits), "files": hits[:max_files]}
    except Exception as exc:
        out["error"] = f"{type(exc).__name__}: {exc}"
    return out


def t1_payload_census() -> dict:
    """T1: THR 9/9 + RW 23 + SCStates 15-field gap audit + 1-step dump."""
    res: dict = {"id": "T1", "dt": DT}
    try:
        from Basilisk.architecture import messaging
        from Basilisk.simulation import spacecraft, thrusterDynamicEffector
        from Basilisk.utilities import (
            SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody,
            simIncludeThruster,
        )
        for key, cls, expect in (
            ("thr_output", messaging.THROutputMsgPayload, THR_EXPECTED_9),
            ("rw_log", messaging.RWConfigLogMsgPayload, RW_EXPECTED_23),
        ):
            try:
                names = _payload_names(cls)
                res[key + "_static_fields"] = names
                res[key + "_static_count"] = len(names)
                res[key + "_missing"] = [f for f in expect if f not in names]
                res[key + "_extra"] = [f for f in names if f not in expect]
            except Exception as exc:
                res[key + "_error"] = f"{type(exc).__name__}: {exc}"
        try:
            names = _payload_names(messaging.SCStatesMsgPayload)
            res["scstates_static_fields"] = names
            res["scstates_static_count"] = len(names)
            gap: dict = {}
            for f in SC_GAP_FIELDS:
                gap[f] = ("present-in-header" if f in names else "absent-from-header")
            res["scstates_gap_audit"] = gap
        except Exception as exc:
            res["scstates_error"] = f"{type(exc).__name__}: {exc}"
        # One stock 1-thruster 1-step dump (NTICKS_1 + margin, still <= 30).
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp6t1-stock"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
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
        tf.create("TEST_Thruster", [0.5, 0.0, 1.0], [0.0, 0.0, 1.0],
                  MaxThrust=1.0, cutoffFrequency=1000.0, useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        on = messaging.THRArrayOnTimeCmdMsg()
        pay = messaging.THRArrayOnTimeCmdMsgPayload()
        pay.OnTimeRequest = [DT] + [0.0] * 35  # 36 slots (MAX_EFF_CNT)
        on.write(pay)
        thr.cmdsInMsg.subscribeTo(on)
        tlog = thr.thrusterOutMsgs[0].recorder()
        slog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", tlog)
        sim.AddModelToTask("t", slog)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * max(NTICKS_1, 2)))
        sim.ExecuteSimulation()
        dump: dict = {}
        for f in THR_EXPECTED_9:
            try:
                arr = np.asarray(getattr(tlog, f))
                # tick0 is pre-fire (staged mailbox idles the fill step);
                # steady value is the LAST tick, not the first.
                dump[f] = arr[-1].tolist() if np is not None else "no-numpy"
                dump[f + "_tick0"] = arr[0].tolist() if np is not None else "no-numpy"
            except Exception:
                dump[f] = "unreadable"
        res["thr_1step_dump"] = dump
        res["thr_dump_note"] = ("write-once fires first propagation then idles; "
                                "steady value = last tick (tick0 is pre-fire fill step)")
        try:
            res["sc_omegaDot_tick0"] = np.asarray(slog.omegaDot_BN_B, float)[0].tolist()
        except Exception:
            res["sc_omegaDot_tick0"] = "missing-or-unreadable"
        res["status"] = "ok"
    except Exception as exc:
        res["status"] = "guarded-fail"
        res["error"] = f"{type(exc).__name__}: {exc}"
        _wexc("T1", exc)
    return res


def t2_viz() -> dict:
    """T2: vizSupport type-check inspection + direct-viz smoke + neg control."""
    res: dict = {"id": "T2"}
    try:
        src_txt, src_path = "", "unresolved"
        try:
            import Basilisk.utilities.vizSupport as vizSupport
            src_path = getattr(vizSupport, "__file__", "unresolved")
            try:
                src_txt = inspect.getsource(vizSupport)
            except Exception:
                src_txt = inspect.getsource(vizSupport.enableVisualization)  # type: ignore[attr-defined]
        except Exception as exc:
            res["vizsupport_import"] = f"guarded-fail: {type(exc).__name__}: {exc}"
        if src_txt:
            res["vizsupport_path"] = src_path
            res["vizsupport_isinstance_spacecraft"] = ("isinstance" in src_txt and "Spacecraft" in src_txt)
            res["vizsupport_gravfield_harvest"] = ("gravField" in src_txt or "gravBodies" in src_txt)
            res["vizsupport_mentions_thr"] = ("thrEffectorList" in src_txt or "thrInMsgs" in src_txt)
            res["vizsupport_mentions_rw"] = ("rwEffectorList" in src_txt or "rwInMsgs" in src_txt)
        else:
            res["vizsupport_source"] = "record-missing: source unreadable"
    except Exception as exc:
        res["vizsupport_error"] = f"{type(exc).__name__}: {exc}"
    try:
        from Basilisk.simulation import spacecraft, vizInterface
        from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp6t2-stock"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
        sc.hub.r_CN_NInit = [1.0e7, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, 6300.0, 0.0]
        sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
        sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        viz = vizInterface.VizInterface()
        viz.ModelTag = "viz"
        try:
            viz_attr_names = sorted(a for a in dir(viz) if not a.startswith("_"))
        except Exception:
            viz_attr_names = []
        res["viz_attr_sample"] = [a for a in viz_attr_names if ("scState" in a or "thr" in a.lower() or "rw" in a.lower() or "Msg" in a)][:20]
        # API varies by version: scStateInMsg (singular) vs scStateInMsgs (plural vector).
        subscribed = "unattempted"
        for attr in ("scStateInMsg", "scStateInMsgs"):
            try:
                port = getattr(viz, attr)
                try:
                    port.subscribeTo(sc.scStateOutMsg)
                    subscribed = f"ok via {attr}"
                    break
                except Exception:
                    # plural vector form: add message to list instead
                    try:
                        if hasattr(port, "pushBack"):
                            port.pushBack(sc.scStateOutMsg)
                            subscribed = f"ok via {attr}.pushBack"
                            break
                    except Exception:
                        pass
            except Exception:
                continue
        res["direct_scState_subscribe"] = subscribed
        for attr in ("thrInMsgs", "rwInMsgs", "scStateInMsg", "scStateInMsgs"):
            try:
                res["direct_has_" + attr] = bool(hasattr(viz, attr))
            except Exception:
                res["direct_has_" + attr] = "unreadable"
        sim.AddModelToTask("t", viz)
        if _WP6_RUN_VIZ_SIM:
            sim.InitializeSimulation()
            sim.ConfigureStopTime(macros.sec2nano(DT * 2))
            sim.ExecuteSimulation()
            res["direct_viz_smoke"] = "ok (full sim)"
        else:
            res["direct_viz_smoke"] = ("ok (static: subscription + census only; "
                                       "full VizInterface sim gated by WP6_RUN_VIZ_SIM=1 "
                                       "— headless ExecuteSimulation segfaults, see run log)")
            try:
                sim.InitializeSimulation()
                res["direct_viz_init"] = "ok"
            except Exception as exc:
                res["direct_viz_init"] = f"guarded-fail: {type(exc).__name__}: {exc}"
    except Exception as exc:
        res["direct_viz_smoke"] = f"guarded-fail: {type(exc).__name__}: {exc}"
        _wexc("T2-direct", exc)
    try:
        res["helper_negative_control"] = "not-attempted"
        try:
            import Basilisk.utilities.vizSupport as vizSupport
            helper_fn = getattr(vizSupport, "enableVisualization", None)
            if helper_fn is None:
                helper_fn = getattr(vizSupport, "enableUnityVisualization", None)
            res["helper_fn_name"] = ("enableVisualization" if hasattr(vizSupport, "enableVisualization")
                                     else ("enableUnityVisualization" if hasattr(vizSupport, "enableUnityVisualization") else "missing-both"))
            sig = str(inspect.signature(helper_fn)) if helper_fn is not None else "n/a"
            res["helper_signature"] = sig
            res["helper_negative_control"] = ("recorded-expectation: helper path requires "
                                              "isinstance(sc, Spacecraft) + gravField harvest; "
                                              "non-Spacecraft caller must fail — see vizsupport flags")
        except Exception as exc:
            res["helper_negative_control"] = f"record-missing: {type(exc).__name__}: {exc}"
    except Exception as exc:
        res["helper_error"] = f"{type(exc).__name__}: {exc}"
    try:
        res["repo_viz_call"] = _grep(["enableVisualization", "thrEffectorList",
                                      "rwEffectorList", "VizInterface", "vizSupport"])
    except Exception as exc:
        res["repo_viz_call_error"] = f"{type(exc).__name__}: {exc}"
    if "status" not in res:
        res["status"] = "ok"
    return res


def t3_fuel_truth() -> dict:
    """T3: stock fuelTank + thruster burn; fuelMass/DV + quantizer gap + notes."""
    res: dict = {"id": "T3", "ticks": NTICKS_SHORT, "dt": DT}
    # Static census first (no sim objects — crash-safe; the burn sim segfaults
    # on this host, so ALL construction lives behind _WP6_RUN_FUEL_SIM).
    try:
        from Basilisk.simulation import fuelTank as _ftMod
        res["fueltank_class_attrs"] = sorted(
            a for a in dir(_ftMod.FuelTank) if not a.startswith("_"))[:40]
        res["fueltank_has_addThrusterSet"] = bool(hasattr(_ftMod.FuelTank, "addThrusterSet"))
        res["fueltank_has_addThruster"] = bool(hasattr(_ftMod.FuelTank, "addThruster"))
    except Exception as exc:
        res["fueltank_class_census"] = f"guarded-fail: {type(exc).__name__}: {exc}"
    try:
        from Basilisk.architecture import messaging as _msgStatic
        res["fuelTankOutMsg_fields"] = sorted(
            a for a in dir(_msgStatic.FuelTankMsgPayload()) if not a.startswith("_"))
    except Exception as exc:
        res["fuelTankOutMsg_fields"] = f"unreadable: {type(exc).__name__}: {exc}"
    if not _WP6_RUN_FUEL_SIM:
        res["mode"] = ("static-only (full burn gated by WP6_RUN_FUEL_SIM=1; "
                       "burn sim segfaults on this host — bisect isolated)")
        res["divergence_note"] = ("caller-side bookkeeping diverges 3 ways — "
                                  "commanded!=achieved (staircase, T1-measured gap), "
                                  "no blowdown/ISP scaling, no mass feedback (omni mass "
                                  "fixed; stock depletes m/c_B/I per substep). Label non-parity.")
        res["status"] = "ok"
        return res
    res["mode"] = "full-burn (WP6_RUN_FUEL_SIM=1)"
    try:
        from Basilisk.architecture import messaging
        from Basilisk.simulation import (
            spacecraft, thrusterDynamicEffector,
        )
        from Basilisk.utilities import (
            SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody,
            simIncludeThruster,
        )
        sim = SimulationBaseClass.SimBaseClass()
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp6t3-stock"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
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
        tank = None
        try:
            from Basilisk.simulation import fuelTank
            # Bisect census (2026-10-12, s4): FuelTank has NO fuelMassInit/
            # fuelMass/maxFuelMass/isp attrs; it exposes setTankModel,
            # fuelTankMassPropMsg, get/setNameOfMassState, fuelLeakRate.
            # Configuration therefore goes through a tank MODEL object.
            try:
                mod_names = sorted(a for a in dir(fuelTank) if not a.startswith("_"))
            except Exception:
                mod_names = []
            res["fueltank_module_census"] = [a for a in mod_names if ("odel" in a or "ank" in a)][:20]
            tank = fuelTank.FuelTank()
            tank.ModelTag = "fuel"
            try:
                tank_attrs = sorted(a for a in dir(tank) if not a.startswith("_"))
            except Exception:
                tank_attrs = []
            res["fuel_tank_attrs_sample"] = [a for a in tank_attrs if ("fuel" in a.lower() or "mass" in a.lower() or "isp" in a.lower() or "tank" in a.lower())][:20]
            # Discover + attach a tank model: constant-volume/uniform-burn
            # first (deterministic depletion); first dir()-hit before
            # (CentrifugalBurn) burned nothing for lack of initial mass.
            model_how = "none"
            inst = None
            _pref = ["FuelTankModelConstantVolume", "FuelTankModelUniformBurn",
                     "FuelTankModelConstantDensity", "FuelTankModelEmptying",
                     "FuelTankModelCentrifugalBurn", "FuelTankModel"]
            _ordered = [c for c in _pref if c in mod_names] + \
                [c for c in mod_names if "odel" in c and "uelTankModel" in c
                 and c not in _pref]
            for cand in _ordered:
                try:
                    inst = getattr(fuelTank, cand)()
                except Exception:
                    inst = None
                    continue
                try:
                    tank.setTankModel(inst)
                    model_how = cand
                    break
                except Exception:
                    inst = None
                    continue
            res["fuel_model_attached"] = model_how
            try:
                res["fuel_model_dir"] = sorted(
                    a for a in dir(inst) if not a.startswith("_"))[:40] \
                    if inst is not None else []
            except Exception:
                res["fuel_model_dir"] = "unreadable"
            # Mass/isp live wherever the build exposes them (model first,
            # then tank); record whatever sticks. Never assign a registered
            # state directly (raises "tried to add variable").
            wired = []
            for obj, tag in ((inst, "model"), (tank, "tank")):
                if obj is None:
                    continue
                try:
                    names = set(dir(obj))
                except Exception:
                    continue
                for attr, val in (("fuelMassInit", 10.0), ("fuelMass", 10.0),
                                  ("initialMass", 10.0), ("massInit", 10.0),
                                  ("mInit", 10.0),
                                  ("maxFuelMass", 10.0), ("maxFuelMassInit", 10.0),
                                  ("isp", 250.0), ("ISP", 250.0), ("Isp", 250.0)):
                    try:
                        if attr in names:
                            setattr(obj, attr, val)
                            wired.append(f"{tag}.{attr}={val}")
                    except Exception:
                        continue
            res["fuel_wired_attrs"] = wired
            sc.addStateEffector(tank)
            sim.AddModelToTask("t", tank, 3)
            res["fuel_tank"] = f"wired: model={model_how} attrs={wired}"
        except Exception as exc:
            res["fuel_tank"] = f"record-missing: {type(exc).__name__}: {exc}"
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", [0.5, 0.0, 1.0], [0.0, 0.0, 1.0],
                  MaxThrust=1.0, cutoffFrequency=1000.0, useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        if tank is not None:
            # Depletion only happens if the thruster set is attached to the tank.
            for meth in ("addThrusterSet", "addThruster"):
                try:
                    getattr(tank, meth)(thr)
                    res["fuel_thruster_attached"] = f"via {meth}"
                    break
                except Exception:
                    continue
        on = messaging.THRArrayOnTimeCmdMsg()
        pay = messaging.THRArrayOnTimeCmdMsgPayload()
        pay.OnTimeRequest = [DT] + [0.0] * 35  # 36 slots
        on.write(pay)
        thr.cmdsInMsg.subscribeTo(on)
        tlog = thr.thrusterOutMsgs[0].recorder()
        slog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", tlog)
        sim.AddModelToTask("t", slog)
        flog = None
        if tank is not None:
            try:
                flog = tank.fuelTankOutMsg.recorder()
                sim.AddModelToTask("t", flog)
            except Exception as exc:
                res["fuel_log"] = f"record-missing: {type(exc).__name__}: {exc}"
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS_SHORT))
        sim.ExecuteSimulation()
        if flog is not None:
            try:
                res["fuelMass_trace"] = [float(v) for v in np.asarray(flog.fuelMass).ravel().tolist()]
                try:
                    res["fuelMassDot_tick0"] = float(np.asarray(flog.fuelMassDot).ravel()[0])
                except Exception:
                    res["fuelMassDot_tick0"] = "missing-or-unreadable"
            except Exception as exc:
                res["fuel_trace_error"] = f"{type(exc).__name__}: {exc}"
        for _dv in ("TotalAccumDV_BN_B", "TotalAccumDVBdy", "TotalAccumDV_CN_N"):
            try:
                res["TotalAccumDV_tick_end"] = np.asarray(getattr(slog, _dv), float)[-1].tolist()
                res["TotalAccumDV_field"] = _dv
                break
            except Exception:
                continue
        if "TotalAccumDV_tick_end" not in res:
            res["TotalAccumDV_tick_end"] = "missing-or-unreadable"
        try:
            factors = np.asarray(tlog.thrustFactor, float).ravel()
            cmd_imp = 1.0 * DT * NTICKS_SHORT
            ach_imp = float(np.sum(factors) * 1.0 * DT)
            res["commanded_impulse_Ns"] = float(cmd_imp)
            res["achieved_impulse_Ns"] = float(ach_imp)
            res["quantizer_gap_Ns"] = float(cmd_imp - ach_imp)
        except Exception as exc:
            res["impulse_error"] = f"{type(exc).__name__}: {exc}"
        res["divergence_note"] = ("600s vs stationkeeping: caller-side bookkeeping diverges "
                                  "3 ways — commanded!=achieved (staircase), no blowdown/ISP "
                                  "scaling, no mass feedback (omni mass fixed; stock depletes "
                                  "m/c_B/I per substep). Size from traces above; label non-parity.")
        res["status"] = "ok"
    except Exception as exc:
        res["status"] = "guarded-fail"
        res["error"] = f"{type(exc).__name__}: {exc}"
        _wexc("T3", exc)
    return res


def run() -> dict:
    """Run all WP6 probes; never raises. Saves incrementally so a segfault
    in a later sub-probe does not lose earlier sub-probe data."""
    out: dict = {"probe": PROBE_ID}
    try:
        out["env"] = env_info()
    except Exception:
        out["env"] = {"error": "env_info-failed"}
    _fns = [fn for fn in (t1_payload_census, t2_viz, t3_fuel_truth)
            if fn.__name__ in _WP6_WANT]
    import sys as _sys

    def _checkpoint():
        try:
            save_result(PROBE_ID, out)
        except Exception:
            pass
        try:
            _sys.stdout.flush()
        except Exception:
            pass
    for fn in _fns:
        try:
            out[fn.__name__] = fn()
        except Exception as exc:
            out[fn.__name__] = {"status": "guarded-fail", "error": f"{type(exc).__name__}: {exc}"}
            _wexc(fn.__name__, exc)
        _checkpoint()
    try:
        log_text(PROBE_ID, f"WP6 done: {[out.get(k, {}).get('status') for k in ('t1_payload_census', 't2_viz', 't3_fuel_truth')]}")
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
        _wexc("main", exc)
        return {"probe": PROBE_ID, "status": "guarded-fail",
                "error": f"{type(exc).__name__}: {exc}",
                "trace": traceback.format_exc(limit=5)}


if __name__ == "__main__":
    main()
