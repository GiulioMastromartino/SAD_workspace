"""Track D — fuel-burn segfault bisection (WP6 T3).

Segfault isolation for the T3 setup sequence in
``audit_bundle/probes/wp6_telem_viz_fuel.py::t3_fuel_truth`` (Spacecraft +
FuelTank + AddStateEffector + thrusterFactory + addToSpacecraft +
addThrusterSet + messages + recorders + AddModelToTask + Initialize +
ConfigureStopTime + Execute).

USE (from ``native_omni_heyoka/``)::

    ../../env/bin/python audit_bundle/probes/wp6_fuel_bisect.py

Design: EACH step runs in its OWN child process
(``sys.executable -c <snippet>``, timeout 120 s) so a segfault kills only
that child. The child replays ALL prior steps then adds exactly one new
operation (cumulative bisection). The parent records per-step
``{exit code, signal, tail}``, stops at the FIRST crashing step, prints the
bisect verdict (last-good / first-bad step) and saves everything via
``save_result('wp6_fuel_bisect', ...)``.

Cannot run on Linux here: Basilisk is macOS-only in this workspace, so on
a Linux host every child fails at import with ModuleNotFoundError and the
parent records that (still saving JSON) instead of raising. Run on the
macOS host with the Basilisk env for the real bisection.

Total host runtime: <5 min in practice (10 quick children; a global ~280 s
budget stops early if children hang). Per-child timeout is 120 s.
"""

from __future__ import annotations

import subprocess
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    from common import env_info, log_text, save_result
except Exception:  # tolerant: probes.common path or local fallback
    try:
        from probes.common import env_info, log_text, save_result  # type: ignore
    except Exception:
        def env_info() -> dict:  # type: ignore
            return {"fallback": True}

        def log_text(_name: str, _text: str) -> str:  # type: ignore
            return ""

        def save_result(_name: str, _payload: dict) -> str:  # type: ignore
            return ""

PROBE_ID = "wp6_fuel_bisect"
CHILD_TIMEOUT_S = 120
GLOBAL_BUDGET_S = 280  # keep total host runtime <5 min
TAIL_CHARS = 2000

# ---------------------------------------------------------------------------
# Cumulative setup fragments. FRAGS[i] adds exactly one operation; child N
# executes FRAGS[0..N-1]. Variable names are stable across fragments
# (sim/proc/sc/grav/earth/tank/thr/tlog/slog/flog/on/...).
# ---------------------------------------------------------------------------

_FRAG_S1 = '''
from Basilisk.utilities import SimulationBaseClass, macros
DT = 0.1
sim = SimulationBaseClass.SimBaseClass()
proc = sim.CreateNewProcess("p")
proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
print("s1: SimBaseClass + process + task wired", flush=True)
'''

_FRAG_S2 = '''
from Basilisk.simulation import spacecraft
from Basilisk.utilities import orbitalMotion
sc = spacecraft.Spacecraft()
sc.ModelTag = "wp6bisect-stock"
sc.hub.mHub = 750.0
sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
oe = orbitalMotion.ClassicElements()
oe.a = 1.0e7
oe.e = 0.01
oe.i = 33.3 * macros.D2R
oe.Omega = 48.2 * macros.D2R
oe.omega = 347.8 * macros.D2R
oe.f = 85.3 * macros.D2R
r0, v0 = orbitalMotion.elem2rv(3.986004418e14, oe)
sc.hub.r_CN_NInit = r0
sc.hub.v_CN_NInit = v0
sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
sim.AddModelToTask("t", sc, 1)
print("s2: Spacecraft + hub props + AddModelToTask(sc) done", flush=True)
'''

_FRAG_S3 = '''
from Basilisk.utilities import simIncludeGravBody
grav = simIncludeGravBody.gravBodyFactory()
earth = grav.createEarth()
earth.isCentralBody = True
grav.addBodiesTo(sc)
print("s3: earth central + addBodiesTo(sc) done", flush=True)
'''

_FRAG_S4 = '''
from Basilisk.simulation import fuelTank
try:
    _mod_names = sorted(a for a in dir(fuelTank) if not a.startswith("_"))
except Exception:
    _mod_names = []
print("s4: fueltank module census=" + str([a for a in _mod_names if ("odel" in a or "ank" in a)][:20]), flush=True)
tank = fuelTank.FuelTank()
tank.ModelTag = "fuel"
tank_attrs = sorted(a for a in dir(tank) if not a.startswith("_"))
_model_how = "none"
_model_inst = None
# Model preference: constant-volume/uniform-burn first (deterministic mass
# depletion). First dir()-hit before (CentrifugalBurn) burned nothing because
# no initial mass stuck.
_pref = ["FuelTankModelConstantVolume", "FuelTankModelUniformBurn",
         "FuelTankModelConstantDensity", "FuelTankModelEmptying",
         "FuelTankModelCentrifugalBurn", "FuelTankModel"]
_ordered = [c for c in _pref if c in _mod_names] + \
    [c for c in _mod_names if "odel" in c and "uelTankModel" in c and c not in _pref]
for _cand in _ordered:
    try:
        _inst = getattr(fuelTank, _cand)()
    except Exception:
        continue
    try:
        tank.setTankModel(_inst)
        _model_how = _cand
        _model_inst = _inst
        break
    except Exception:
        continue
print("s4: tank model attached via %s" % _model_how, flush=True)
try:
    _mdi = sorted(a for a in dir(_model_inst) if not a.startswith("_")) if _model_inst is not None else []
    print("s4: model dir()=" + str(_mdi[:40]), flush=True)
except Exception as _e:
    print("s4: model dir() unreadable: %s" % _e, flush=True)
_wired = []
for _obj, _tag in ((_model_inst, "model"), (tank, "tank")):
    if _obj is None:
        continue
    try:
        _names = set(dir(_obj))
    except Exception:
        continue
    for _attr, _val in (("fuelMassInit", 10.0), ("fuelMass", 10.0), ("initialMass", 10.0), ("massInit", 10.0), ("mInit", 10.0), ("maxFuelMass", 10.0), ("maxFuelMassInit", 10.0), ("isp", 250.0), ("ISP", 250.0), ("Isp", 250.0)):
        try:
            if _attr in _names:
                setattr(_obj, _attr, _val)
                _wired.append("%s.%s=%s" % (_tag, _attr, _val))
        except Exception:
            continue
print("s4: wired attrs=" + str(_wired), flush=True)
print("s4: tank dir() census=" + str([a for a in tank_attrs if ("fuel" in a.lower() or "mass" in a.lower() or "isp" in a.lower() or "tank" in a.lower())][:20]), flush=True)
try:
    print("s4: fuelMass readback=" + str(getattr(tank, "fuelMassInit", getattr(tank, "fuelMass", "unreadable"))), flush=True)
except Exception as _e:
    print("s4: fuelMass readback unreadable: %s" % _e, flush=True)
'''

_FRAG_S5 = '''
sc.addStateEffector(tank)
sim.AddModelToTask("t", tank, 3)
print("s5: sc.addStateEffector(tank) + AddModelToTask(tank) done", flush=True)
'''

_FRAG_S6 = '''
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.utilities import simIncludeThruster
tf = simIncludeThruster.thrusterFactory()
tf.create("TEST_Thruster", [0.5, 0.0, 1.0], [0.0, 0.0, 1.0], MaxThrust=1.0, cutoffFrequency=1000.0, useMinPulseTime=False)
thr = thrusterDynamicEffector.ThrusterDynamicEffector()
tf.addToSpacecraft("thr", thr, sc)
sim.AddModelToTask("t", thr, 2)
print("s6: thrusterFactory + addToSpacecraft + AddModelToTask(thr) done", flush=True)
print("s6: thrusterOutMsgs_len=%d" % len(thr.thrusterOutMsgs), flush=True)
'''

_FRAG_S7 = '''
attach_how = "none"
for _meth in ("addThrusterSet", "addThruster"):
    try:
        getattr(tank, _meth)(thr)
        attach_how = _meth
        break
    except Exception as _e:
        print("s7: %s failed: %s: %s" % (_meth, type(_e).__name__, _e), flush=True)
        continue
print("s7: tank-thruster attach via %s" % attach_how, flush=True)
if attach_how == "none":
    raise RuntimeError("neither addThrusterSet nor addThruster worked")
'''

_FRAG_S8 = '''
from Basilisk.architecture import messaging
on = messaging.THRArrayOnTimeCmdMsg()
pay = messaging.THRArrayOnTimeCmdMsgPayload()
pay.OnTimeRequest = [DT] + [0.0] * 35
on.write(pay)
thr.cmdsInMsg.subscribeTo(on)
tlog = thr.thrusterOutMsgs[0].recorder()
slog = sc.scStateOutMsg.recorder()
sim.AddModelToTask("t", tlog)
sim.AddModelToTask("t", slog)
flog = tank.fuelTankOutMsg.recorder()
sim.AddModelToTask("t", flog)
print("s8: THRArrayOnTimeCmdMsg write-once + subscribe + 3 recorders + AddModelToTask done", flush=True)
'''

_FRAG_S9 = '''
sim.InitializeSimulation()
print("s9: InitializeSimulation done", flush=True)
'''

_FRAG_S10 = '''
sim.ConfigureStopTime(macros.sec2nano(DT * 10))
sim.ExecuteSimulation()
print("s10: ConfigureStopTime + ExecuteSimulation(10 ticks dt=0.1) done", flush=True)
try:
    _fm = list(flog.fuelMass)
    print("s10: fuelMass_trace_len=%d first=%s last=%s" % (len(_fm), str(_fm[0]) if _fm else "empty", str(_fm[-1]) if _fm else "empty"), flush=True)
except Exception as _e:
    print("s10: fuelMass trace unreadable: %s: %s" % (type(_e).__name__, _e), flush=True)
'''

# Footer appended to EVERY child: guarded key-value readback + STEP_OK marker.
# STEP_OK presence (plus exit code 0) is the parent's success criterion.
_FOOTER_TMPL = '''
try:
    try:
        _rb = getattr(tank, "fuelMassInit", getattr(tank, "fuelMass", "n/a-pre-s4"))
    except Exception:
        _rb = "unreadable"
    print("KEY fuelMass_readback=%s" % str(_rb), flush=True)
except Exception:
    print("KEY fuelMass_readback=n/a (pre-s4)", flush=True)
try:
    print("KEY thrusterOutMsgs_len=%d" % len(thr.thrusterOutMsgs), flush=True)
except Exception:
    print("KEY thrusterOutMsgs_len=n/a (pre-s6)", flush=True)
print("STEP_OK {step}", flush=True)
'''

STEPS: list[tuple[str, str, str]] = [
    ("s1", "imports + SimBaseClass + process/task", _FRAG_S1),
    ("s2", "+ Spacecraft + hub props + AddModelToTask", _FRAG_S2),
    ("s3", "+ gravBodyFactory earth central + addBodiesTo", _FRAG_S3),
    ("s4", "+ FuelTank() + fuelMassInit/maxFuelMass/isp + dir() census", _FRAG_S4),
    ("s5", "+ sc.AddStateEffector(tank) + AddModelToTask(tank)", _FRAG_S5),
    ("s6", "+ thrusterFactory TEST_Thruster + ThrusterDynamicEffector + addToSpacecraft + AddModelToTask", _FRAG_S6),
    ("s7", "+ tank.addThrusterSet(thr) (fallback addThruster)", _FRAG_S7),
    ("s8", "+ THRArrayOnTimeCmdMsg write-once + subscribe + 3 recorders + AddModelToTask", _FRAG_S8),
    ("s9", "+ InitializeSimulation only", _FRAG_S9),
    ("s10", "+ ConfigureStopTime + ExecuteSimulation (10 ticks dt=0.1)", _FRAG_S10),
]


def _child_source(upto: int) -> str:
    """Cumulative child snippet through STEPS[upto] (0-based)."""
    body = "\n".join(frag for _, _, frag in STEPS[: upto + 1])
    footer = _FOOTER_TMPL.format(step=STEPS[upto][0])
    return body + "\n" + footer + "\n"


def _tail(text: str, n: int = TAIL_CHARS) -> str:
    if not text:
        return ""
    return text[-n:]


def _run_step(idx: int, timeout_s: float) -> dict:
    step_id, label, _ = STEPS[idx]
    src = _child_source(idx)
    t0 = time.time()
    timed_out = False
    try:
        cp = subprocess.run(
            [sys.executable, "-c", src],
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        rc = cp.returncode
        out, err = cp.stdout or "", cp.stderr or ""
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        rc = None
        out = (exc.stdout or "") if isinstance(getattr(exc, "stdout", None), str) else ""
        err = (exc.stderr or "") if isinstance(getattr(exc, "stderr", None), str) else ""
        if isinstance(getattr(exc, "stdout", None), bytes):
            try:
                out = exc.stdout.decode(errors="replace")  # type: ignore[union-attr]
            except Exception:
                out = ""
        if isinstance(getattr(exc, "stderr", None), bytes):
            try:
                err = exc.stderr.decode(errors="replace")  # type: ignore[union-attr]
            except Exception:
                err = ""
    except Exception as exc:  # parent-side spawn failure; never raises further
        rc = None
        out, err = "", f"parent-spawn-fail: {type(exc).__name__}: {exc}"
    dt = time.time() - t0
    sig = -rc if isinstance(rc, int) and rc < 0 else None
    marker = ("STEP_OK " + step_id) in out
    ok = (not timed_out) and rc == 0 and marker
    combined = (out + ("\n---STDERR---\n" + err if err else ""))
    return {
        "step": step_id,
        "label": label,
        "ok": bool(ok),
        "returncode": rc,
        "signal": sig,
        "timed_out": bool(timed_out),
        "step_ok_marker": bool(marker),
        "elapsed_s": round(dt, 2),
        "stdout_tail": _tail(out),
        "stderr_tail": _tail(err),
        "tail": _tail(combined),
    }


def main() -> dict:
    payload: dict = {"probe": PROBE_ID, "steps": []}
    try:
        payload["env"] = env_info()
    except Exception:
        payload["env"] = {"error": "env_info-failed"}
    payload["child_timeout_s"] = CHILD_TIMEOUT_S
    payload["global_budget_s"] = GLOBAL_BUDGET_S
    payload["note"] = (
        "Cumulative bisection of the T3 fuel-burn setup; each step replays all "
        "prior steps in a fresh child (sys.executable -c, 120 s timeout). "
        "Parent stops at the first crashing step. Basilisk is macOS-only: on "
        "Linux expect s1 to fail with ModuleNotFoundError (recorded, not raised)."
    )
    t_start = time.time()
    last_good: str | None = None
    first_bad: str | None = None
    for idx in range(len(STEPS)):
        remaining = GLOBAL_BUDGET_S - (time.time() - t_start)
        if remaining <= 0:
            payload.setdefault("stopped_early", []).append(
                {"reason": "global-budget-exhausted", "at_step": STEPS[idx][0]}
            )
            break
        rec = _run_step(idx, min(CHILD_TIMEOUT_S, max(5.0, remaining)))
        payload["steps"].append(rec)
        if rec["ok"]:
            last_good = rec["step"]
        else:
            first_bad = rec["step"]
            break  # stop at the first crashing step
    payload["last_good_step"] = last_good
    payload["first_bad_step"] = first_bad
    if first_bad is None and len(payload["steps"]) == len(STEPS):
        payload["verdict"] = "all 10 steps passed (no crash reproduced)"
        payload["status"] = "ok"
    elif first_bad is not None:
        tail0 = payload["steps"][-1].get("tail", "") if payload["steps"] else ""
        if first_bad == "s1" and ("ModuleNotFoundError" in tail0 or "No module named" in tail0):
            payload["verdict"] = (
                "s1 failed: Basilisk not importable on this host "
                "(expected on Linux; rerun on macOS Basilisk env)"
            )
            payload["status"] = "env-missing-basilisk"
        else:
            payload["verdict"] = (
                f"bisected: last-good={last_good} first-bad={first_bad} "
                f"(rc={payload['steps'][-1].get('returncode')} "
                f"sig={payload['steps'][-1].get('signal')})"
            )
            payload["status"] = "bisected"
    else:
        payload["verdict"] = "stopped early (global budget); bisection incomplete"
        payload["status"] = "incomplete"
    payload["elapsed_s"] = round(time.time() - t_start, 2)
    try:
        path = save_result(PROBE_ID, payload)
        payload["saved_to"] = path
    except Exception as exc:
        payload["save_error"] = f"{type(exc).__name__}: {exc}"
    try:
        log_text(PROBE_ID, f"bisect verdict: {payload.get('verdict')} "
                 f"[last-good={last_good} first-bad={first_bad}]")
    except Exception:
        pass
    print(f"[wp6_fuel_bisect] last-good={last_good} first-bad={first_bad}", flush=True)
    print(f"[wp6_fuel_bisect] verdict: {payload.get('verdict')}", flush=True)
    print(f"[wp6_fuel_bisect] saved: {payload.get('saved_to', payload.get('save_error'))}", flush=True)
    return payload


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        try:
            save_result(PROBE_ID, {"status": "error",
                                   "error": f"{type(exc).__name__}: {exc}",
                                   "trace": traceback.format_exc(limit=10)})
        except Exception:
            pass
        print(traceback.format_exc(), flush=True)
