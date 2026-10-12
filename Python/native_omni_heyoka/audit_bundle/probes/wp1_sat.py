#!/usr/bin/env python3.11
"""WP1 RW saturation diagnostic bundle (PLAN_ENVELOPE.md Rev 1).

Direction-aware cut, tick granularity, Omega_max setter-only,
uMax/Js/Gs via RWArrayConfigMsg, u_min/P_max, RPM vs rad/s units.
A: static source/msg introspection (always runs). B: live stock sims
(below/above/reverse/dt-ladder/3-wheel smoke, all guarded, <=60 ticks).
C: u_min/P_max/Omega_max=-1/custom-default edge cases (skip-with-reason).
Saves via probes.common.save_result('wp1_sat', ...). Never raises.
Stdlib+numpy+Basilisk only; results to audit_bundle/results/.
"""
from __future__ import annotations
import importlib, inspect, re, sys, traceback
from pathlib import Path

try:
    from probes.common import save_result, log_text, env_info, write_exception
except ImportError:
    try:
        from common import save_result, log_text, env_info, write_exception  # type: ignore
    except ImportError:
        import json
        def log_text(m: str) -> None: print(m, flush=True)  # type: ignore
        def env_info() -> dict: return {"python": sys.version, "stub": True}  # type: ignore
        def write_exception(e: BaseException) -> str:  # type: ignore
            return "".join(traceback.format_exception(type(e), e, e.__traceback__))
        def save_result(n: str, p: dict) -> Path:  # type: ignore
            o = Path(__file__).resolve().parents[1] / "results" / f"{n}.json"
            o.parent.mkdir(parents=True, exist_ok=True)
            o.write_text(json.dumps(p, indent=2, default=str)); return o

RPM_TO_RADS = 2 * 3.141592653589793 / 60.0
RADS_TO_RPM = 60.0 / (2 * 3.141592653589793)
CAP_RPM, MAX_TICKS = 6000.0, 60  # HR16 Omega_max; each sim <30 s
_PROBE = "wp1_sat"


def _log(msg: str) -> None:
    """2-arg/1-arg tolerant progress log (real common vs shim)."""
    try:
        log_text(_PROBE, msg)  # type: ignore
    except TypeError:
        try:
            log_text(msg)  # type: ignore
        except Exception:
            pass
    except Exception:
        pass


def _wexc(exc: BaseException) -> str:
    try:
        return write_exception(_PROBE, exc)  # type: ignore
    except TypeError:
        try:
            return write_exception(exc)  # type: ignore
        except Exception:
            return repr(exc)
    except Exception:
        return repr(exc)

def _import_rw_factory():
    """Return (rwFactory module, how) trying canonical Basilisk paths."""
    for dotted in (
        "Basilisk.utilities.simIncludeRW",
        "Basilisk.simulation.simIncludeRW",
        "simIncludeRW",
    ):
        try:
            return importlib.import_module(dotted), dotted
        except Exception:
            continue
    raise ModuleNotFoundError(
        "simIncludeRW not found (tried Basilisk.utilities.simIncludeRW, "
        "Basilisk.simulation.simIncludeRW, simIncludeRW)"
    )


# --- SECTION A: static (always runs, no sim) ---
def _rw_source() -> dict:
    d: dict = {"module": "simIncludeRW", "importable": False}
    try:
        m, how = _import_rw_factory()
        d["importable"] = True
        d["via"] = how
        try: src = inspect.getsource(m)
        except (OSError, TypeError):
            fp = getattr(m, "__file__", ""); d["file"] = fp
            src = Path(fp).read_text() if fp and Path(fp).exists() else ""
            d["source_len"] = len(src)
        else: d["file"] = getattr(m, "__file__", ""); d["source_len"] = len(src)
        g = re.search(r"Omega_max\s*=\s*([0-9.]+)", src)
        d["Omega_max_RPM"] = float(g.group(1)) if g else None
        d["HR16_6000_present"] = "6000" in src
        d["Js_formula"] = bool(re.search(r"Js\s*=\s*maxMomentum\s*/", src)
                               or re.search(r"maxMomentum\s*/\s*Omega", src))
        for kw in ("u_max", "Omega_max", "RWModel", "useRWfriction", "fCoulomb",
                   "fStatic", "betaStatic", "cViscous"):
            d[f"kwarg_{kw}"] = kw in src
        d["absent_fViscous"] = "fViscous" not in src  # Rev1: must hold
        d["absent_useRWVoltageIO"] = "useRWVoltageIO" not in src
        mu = re.search(r"u_min\s*=\s*([0-9.eE\-+]+)", src)
        mp = re.search(r"P_max\s*=\s*([0-9.eE\-+]+)", src)
        d["u_min_default"], d["P_max_default"] = (mu.group(1) if mu else None,
                                                 mp.group(1) if mp else None)
        d["minus1_off_mentioned"] = "-1" in src
        # Supported hardware-type strings: BSK calls exit() (not an exception)
        # on unknown types, so edge probes must only attempt discovered names.
        try:
            cands = re.findall(r'''(?:rwType|RWType)\s*==\s*['"]([^'"]+)['"]''', src)
            cands += re.findall(r'''["']((?:Honeywell|NanoAvionics|BCT|BlueCanyon|Custom|custom)\w*)["']''', src)
            seen, types = set(), []
            for c in cands:
                if c not in seen:
                    seen.add(c)
                    types.append(c)
            d["type_names_discovered"] = types[:20]
        except Exception:
            d["type_names_discovered"] = "unreadable"
    except Exception as e: d["error"] = f"{type(e).__name__}: {e}"
    return d

def _msg_fields() -> dict:
    o: dict = {}
    try:
        from Basilisk.architecture import messaging  # type: ignore
        for n in ("RWArrayConfigMsgPayload", "RWConfigLogMsgPayload"):
            try: o[n] = sorted(a for a in dir(getattr(messaging, n)()) if not a.startswith("_"))
            except Exception as e: o[n] = f"unavailable: {type(e).__name__}: {e}"
    except Exception as e:
        o["RWArrayConfigMsgPayload"] = "unavailable"
        o["RWConfigLogMsgPayload"] = "unavailable"; o["reason"] = f"{type(e).__name__}: {e}"
    o["GsMatrix_B_layout_question"] = ("GsMatrix_B flat layout row-vs-col unknown; "
        "round-trip against rwFactory allocator; Omega_max NOT in this message.")
    return o

def probe_static() -> dict:
    return {"simIncludeRW": _rw_source(), "messages": _msg_fields(),
            "units_note": "RPM kwarg (Omega_max) vs rad/s payload (Omega_max, wheelSpeeds)",
            "setter_only": "Omega_max setter-only; RWConfigLog.Omega_max output-only/circular."}

# --- SECTION B: live sims (each guarded, fresh timestamps, <=60 ticks) ---
def _one_wheel_sim(Omega_start_RPM: float, u_cmd: float, dt: float,
                   n_ticks: int = 60, gs_B=(1.0, 0.0, 0.0)) -> dict:
    """Stock Spacecraft + 1xHR16 at Omega_start_RPM, constant u rewritten each
    tick (fresh timestamps via Driver SysModel), per-tick log of wheelSpeeds +
    u_current/Omega from recorders."""
    n_ticks = min(int(n_ticks), MAX_TICKS)
    from Basilisk.simulation import spacecraft  # type: ignore
    simIncludeRW = _import_rw_factory()[0]
    from Basilisk.simulation import reactionWheelStateEffector as rwEff  # type: ignore
    from Basilisk.architecture import messaging, sysModel  # type: ignore
    from Basilisk.utilities import SimulationBaseClass, macros  # type: ignore
    sim = SimulationBaseClass.SimBaseClass(); sim.SetProgressBar(False)
    pr = sim.CreateNewProcess("p"); pr.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    sc = spacecraft.Spacecraft(); sc.ModelTag = "wp1_sat"
    # Real hub inertia is load-bearing: default (~zero) inertia + u=0.2 Nm
    # diverges to 1e27 rad/s^2 (host-observed). Demo-bus values.
    sc.hub.mHub = 750.0
    sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    sim.AddModelToTask("t", sc)
    fac = simIncludeRW.rwFactory()
    fac.create("Honeywell_HR16", [float(x) for x in gs_B], maxMomentum=50.0, Omega=Omega_start_RPM)
    rw = rwEff.ReactionWheelStateEffector()
    fac.addToSpacecraft("rw", rw, sc)
    sim.AddModelToTask("t", rw)

    class _Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(motorTorque=[0.0]))

        def UpdateState(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(motorTorque=[float(u_cmd)]),
                           t, self.moduleID)

    drv = _Driver(); sim.AddModelToTask("t", drv)
    rw.rwMotorCmdInMsg.subscribeTo(drv.out)
    spd_log = rw.rwSpeedOutMsg.recorder(); sim.AddModelToTask("t", spd_log)
    rw_log = rw.rwOutMsgs[0].recorder(); sim.AddModelToTask("t", rw_log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()
    import numpy as _np
    w_rad = _np.asarray(spd_log.wheelSpeeds, float).ravel()
    rpm = (w_rad * RADS_TO_RPM).tolist()
    try:
        uu = _np.asarray(rw_log.u_current, float).ravel().tolist()
    except Exception:
        uu = [float("nan")] * len(rpm)
    try:
        om = _np.asarray(rw_log.Omega, float).ravel().tolist()
    except Exception:
        om = w_rad.tolist()
    tail = rpm[-5:] if len(rpm) >= 5 else rpm
    slope = (tail[-1] - tail[0]) / ((len(tail) - 1) * dt) if len(tail) > 1 else 0.0
    return {"Omega_start_RPM": Omega_start_RPM, "u_cmd": u_cmd, "dt": dt,
            "n_ticks": n_ticks, "gs_B": list(gs_B), "wheel_RPM": rpm,
            "u_current": uu, "Omega_log": om,
            "overshoot_RPM": max(0.0, max(rpm) - CAP_RPM) if rpm else 0.0,
            "plateau_RPM": max(rpm) if rpm else float("nan"),
            "resume_slope_RPM_per_s": slope,
            "driver": "rewrite-every-tick (fresh); frozen variant would ZOH-stale torque"}

def _guarded(label: str, **kw) -> dict:
    try:
        r = _one_wheel_sim(**kw)
        r["spin"] = "pos" if kw.get("u_cmd", 0) >= 0 else "neg"
        return {"label": label, "ok": True, "result": r}
    except Exception as e: return {"label": label, "ok": False, "reason": f"skipped: {type(e).__name__}: {e}"}

def probe_sweeps() -> dict:
    cases = [("approach_below", dict(Omega_start_RPM=5950.0, u_cmd=0.2, dt=0.1, n_ticks=60)),
             ("approach_above", dict(Omega_start_RPM=6050.0, u_cmd=0.2, dt=0.1, n_ticks=40)),
             ("reverse_descent", dict(Omega_start_RPM=6050.0, u_cmd=-0.2, dt=0.1, n_ticks=60)),
             ("dt_0p02", dict(Omega_start_RPM=5950.0, u_cmd=0.2, dt=0.02, n_ticks=60))]
    out = [_guarded(l, **k) for l, k in cases]
    try:  # 3-wheel + tilted-axis smoke, short horizon (Driver rewrite, proven pattern)
        from Basilisk.simulation import spacecraft  # type: ignore
        simIncludeRW = _import_rw_factory()[0]
        from Basilisk.simulation import reactionWheelStateEffector as rwEff  # type: ignore
        from Basilisk.architecture import messaging, sysModel  # type: ignore
        from Basilisk.utilities import SimulationBaseClass, macros  # type: ignore
        dt, n = 0.1, 20
        sim = SimulationBaseClass.SimBaseClass(); sim.SetProgressBar(False)
        p = sim.CreateNewProcess("p"); p.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
        sc = spacecraft.Spacecraft(); sim.AddModelToTask("t", sc)
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
        fac = simIncludeRW.rwFactory()
        for gs in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            fac.create("Honeywell_HR16", gs, maxMomentum=50.0, Omega=5900.0)
        rw3 = rwEff.ReactionWheelStateEffector()
        fac.addToSpacecraft("rw", rw3, sc)
        sim.AddModelToTask("t", rw3)

        class _TriDriver(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.out = messaging.ArrayMotorTorqueMsg()

            def Reset(self, t):
                self.out.write(messaging.ArrayMotorTorqueMsgPayload(motorTorque=[0.0, 0.0, 0.0]))

            def UpdateState(self, t):
                self.out.write(messaging.ArrayMotorTorqueMsgPayload(motorTorque=[0.2, 0.2, 0.2]),
                               t, self.moduleID)

        _tri = _TriDriver(); sim.AddModelToTask("t", _tri)
        rw3.rwMotorCmdInMsg.subscribeTo(_tri.out)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(dt * n))
        sim.ExecuteSimulation()
        out.append({"label": "three_wheel_tilted_smoke", "ok": True,
                    "result": {"dt": dt, "n_ticks": n, "note": "3xHR16 spin-up smoke"}})
    except Exception as e: out.append({"label": "three_wheel_tilted_smoke", "ok": False,
                                       "reason": f"skipped: {type(e).__name__}: {e}"})
    return {"cases": out}

# --- SECTION C: edge cases (subprocess-isolated: BSK exit() on unknown
# types kills the child only, never this probe's JSON) ---
def _try_subprocess(key: str, note: str, snippet: str, timeout_s: int = 90) -> dict:
    """Run a create-snippet in a child python; record returncode + tail.

    Returns {"ok": True} iff the child exits 0 (create accepted).
    """
    import subprocess
    try:
        p = subprocess.run(
            [sys.executable, "-c", snippet],
            capture_output=True, text=True, timeout=timeout_s)
        tail = ((p.stderr or "") + "\n" + (p.stdout or "")).strip().splitlines()
        tail = "\n".join(tail[-6:])
        if p.returncode == 0:
            return {key: {"ok": True, "note": note, "child_tail": tail}}
        return {key: {"ok": False, "note": note,
                      "reason": f"child exit {p.returncode}", "child_tail": tail}}
    except Exception as e:
        return {key: {"ok": False, "note": note,
                      "reason": f"harness: {type(e).__name__}: {e}"}}


def _try(key: str, note: str, fn) -> dict:
    try: fn(); return {key: {"ok": True, "note": note}}
    except Exception as e: return {key: {"ok": False, "reason": f"skipped: {type(e).__name__}: {e}"}}


def probe_edge_cases(static_types=None) -> dict:
    o: dict = {}
    # Prefer discovered hardware-type spellings; fall back to HR16/Custom.
    custom_names = [t for t in (static_types or [])
                    if "ustom" in str(t)] or ["Custom", "custom", "CustomRW"]
    hr16_create = ("from Basilisk.utilities.simIncludeRW import rwFactory;"
                   "rwFactory().create('Honeywell_HR16',[1,0,0],"
                   "maxMomentum=50.0,Omega=0.0,%s)")
    custom_create = ("from Basilisk.utilities.simIncludeRW import rwFactory;"
                     "rwFactory().create('%s',[1,0,0],%s)")
    tried = []
    for cname in custom_names[:3]:
        snip = custom_create % (cname, "Js=0.1,u_max=0.2,Omega_max=-1.0")
        r = _try_subprocess("Omega_max_minus1_off",
                            f"{cname} Omega_max=-1 accepted (=off)", snip)
        tried.append((cname, r["Omega_max_minus1_off"].get("ok", False)))
        if tried[-1][1]:
            o.update(r)
            break
        o["_attempt_" + cname] = r["Omega_max_minus1_off"]
    else:
        o.update(r)
    o["custom_names_tried"] = [t for t, _ in tried]
    o.update(_try_subprocess("u_min_deadband", "u_min=1e-6 accepted",
                             hr16_create % "u_min=1e-6"))
    o.update(_try_subprocess("P_max_foldback", "P_max accepted",
                             hr16_create % "P_max=200.0"))
    # Custom without Omega_max must stay off (not inherit 6000 RPM): check the
    # created wheel's Omega_max in the child and echo it.
    cust_snip = ("from Basilisk.utilities.simIncludeRW import rwFactory;"
                 "w=rwFactory().create('%s',[1,0,0],Js=0.1,u_max=0.2);"
                 "print('OMEGA_MAX='+str(getattr(w,'Omega_max','unknown')))")
    for cname in custom_names[:3]:
        r = _try_subprocess("custom_no_default",
                            f"{cname} w/o Omega_max stays off",
                            cust_snip % cname)
        if r["custom_no_default"].get("ok", False):
            o.update(r)
            break
        o["_attempt_noDefault_" + cname] = r["custom_no_default"]
    else:
        o.update(r)
    return o

# --- main: never raises ---
def main() -> int:
    payload: dict = {}
    try:
        _log("[wp1_sat] SECTION A (static)")
        try: payload["env"] = env_info()
        except Exception: payload["env"] = {"env_info": "unavailable"}
        payload["static"] = probe_static()
        _log("[wp1_sat] SECTION B (live sims)")
        try: payload["sweeps"] = probe_sweeps()
        except Exception as e: payload["sweeps"] = {"error": _wexc(e)}
        try:  # interim checkpoint: edge section must never cost us A+B
            save_result("wp1_sat", dict(payload, edge={"status": "pending"}))
        except Exception:
            pass
        _log("[wp1_sat] SECTION C (edge cases, subprocess-isolated)")
        try: payload["edge"] = probe_edge_cases(
            payload.get("static", {}).get("simIncludeRW", {}).get("type_names_discovered") or [])
        except Exception as e: payload["edge"] = {"error": _wexc(e)}
        payload["units_note"] = "RPM kwarg vs rad/s payload"
        _log("[wp1_sat] saving wp1_sat bundle")
        save_result("wp1_sat", payload)
        return 0
    except Exception as e:
        try: payload["top_error"] = _wexc(e); save_result("wp1_sat", payload)
        except Exception: pass
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
