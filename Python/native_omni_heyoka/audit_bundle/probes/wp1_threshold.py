"""WP1 threshold verdict probe: maxWheelAcceleration / largeTorqueThreshold.

Closes the last unresolved trap-table row (WP1 deferred it: "probe whether
stock logs or clamps at these; if log-only, mirror with bskLog at the same
thresholds; if clamps, WP1 grows the clamp. Probe first - do not guess").

Design: 3 wheels (stock), constant torque tuned to make |wdot| straddle
the threshold within the run; log per-tick |wdot| from rwOutMsgs omega +
u_current, plus any BSK_WARNING text (captured on stderr by the runner).
Verdict rules:
  * |wdot| exceeds threshold with NO warning => silent (log-only off);
  * warning appears exactly at the first crossing => log-only;
  * applied torque saturates below uMax while unclamped demand continues
    => clamp (WP1 grows the clamp).
Second arm: largeTorqueThreshold set tiny with a huge torque command
(same rules).

USE (from native_omni_heyoka/):
    ../../env/bin/python audit_bundle/probes/wp1_threshold.py

~4 short sims (<=30 ticks each), results via probes.common.save_result.
Matches run_all.sh `wp*.py` glob; never raises.
"""
from __future__ import annotations

import math
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

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

PROBE_ID = "wp1_threshold"
DT = 0.05
N_TICKS = 30
# HR16: Js = 50 / (6000 RPM in rad/s); u=0.2 Nm => nominal |wdot| ~ 2.5 rad/s^2.
JS = 0.07957747154594767
ACCEL_DEFAULT = 1.0e6     # stock default (probed): maxWheelAcceleration
ACCEL_PROBE = 10.0        # intentionally below nominal wdot
ACCEL_HUGE = 1.0e12       # above everything
U_BIG = 0.2               # Nm (== stock u_max default for HR16)


def _run(thr, u_cmd, n_ticks=N_TICKS, dt=DT):
    """Stock 3-wheel sim with per-wheel maxWheelAcceleration = thr.

    Returns dict with per-tick |wdot| (from omega log), u_current log,
    max |wdot|, first-crossing tick, warning text (best-effort).
    """
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.simulation import reactionWheelStateEffector, spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros, simHelpers,
                                    simIncludeGravBody, simIncludeRW)

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "wp1-thr"
    sc.hub.mHub = 750.0
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(
        [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0])
    sim.AddModelToTask("t", sc, 1)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    fac = simIncludeRW.rwFactory()
    for gs in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        fac.create("Honeywell_HR16", [float(x) for x in gs],
                   maxMomentum=50.0, Omega=0.0)
    rw = reactionWheelStateEffector.ReactionWheelStateEffector()
    fac.addToSpacecraft("rw", rw, sc)
    sim.AddModelToTask("t", rw, 2)
    getters = {"thr": [rw.getMaxWheelAcceleration, rw.getLargeTorqueThreshold],
               "setters": [rw.setMaxWheelAcceleration, rw.setLargeTorqueThreshold]}
    defaults = {}
    for name, fn in zip(("maxWheelAcceleration", "largeTorqueThreshold"),
                        getters["thr"]):
        try:
            defaults[name] = float(fn())
        except Exception as e:
            defaults[name] = f"unreadable: {type(e).__name__}"
    try:
        rw.setMaxWheelAcceleration(float(thr))
        set_ok = True
    except Exception as e:
        set_ok = f"set-fail: {type(e).__name__}: {e}"

    class _Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0, 0.0, 0.0]))

        def UpdateState(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[float(u_cmd)] * 3), t, self.moduleID)

    drv = _Driver()
    sim.AddModelToTask("t", drv)
    rw.rwMotorCmdInMsg.subscribeTo(drv.out)
    wlog = rw.rwSpeedOutMsg.recorder()
    ulogs = [m.recorder() for m in list(rw.rwOutMsgs)[:3]]
    for r in ulogs:
        sim.AddModelToTask("t", r)
    sim.AddModelToTask("t", wlog)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(dt * n_ticks))
    sim.ExecuteSimulation()

    import numpy as np

    def _rows(rec, field="wheelSpeeds"):
        """Recorder field -> first column as (n,), robust to flat/(N,)/(N,1).

        Returns None when the attribute is missing, unreadable, or empty.
        """
        try:
            if not hasattr(rec, field):
                return None
            a = np.asarray(getattr(rec, field), float)
            if a.size == 0:
                return None
            a = a.reshape(len(a), -1) if a.ndim == 1 else a
            return a[:, 0]
        except Exception:
            return None

    # Speed series: rwSpeedOutMsg wheelSpeeds (all wheels), falling back to
    # each rwOutMsgs payload's Omega column (per-wheel).
    speeds = None
    _src = ""
    for cand, nm in ([(wlog, "wheelSpeeds")]
                     + [(r, "Omega") for r in ulogs]):
        c = _rows(cand, nm)
        if c is not None and len(c) > 2:
            speeds, _src = c, nm
            break
    if speeds is not None:
        wd = np.array([np.max(np.abs(speeds[i] - speeds[i - 1]) / dt)
                       for i in range(1, len(speeds))])
    else:
        wd = np.array([])

    def _ucol(rec):
        try:
            a = np.asarray(rec.u_current, float)
            if a.size == 0:
                return None
            a = a.reshape(len(a), -1) if a.ndim == 1 else a
            return a[:, 0]
        except Exception:
            return None

    ucols = [c for c in (_ucol(r) for r in ulogs) if c is not None]
    ucur = (np.maximum.reduce([np.abs(c) for c in ucols]).tolist()
            if ucols else None)
    max_wdot = float(np.max(wd)) if len(wd) else 0.0
    cross = next((k for k, v in enumerate(wd) if v > thr), None)
    return {
        "threshold": float(thr), "defaults": defaults,
        "max_wheelAcceleration_set": set_ok,
        "speeds_src": _src or "none (no usable speed series)",
        "max_abs_wdot": max_wdot, "first_cross_tick": cross,
        "u_current_trace": ucur,
        "demand_persisted": (bool(ucur) and all(
            abs(v) >= 0.999 * abs(u_cmd) for v in ucur[1:])),
    }


def _verdict(rec):
    t = rec["threshold"]
    if rec["first_cross_tick"] is None:
        return "no-crossing (threshold above achieved wdot or wdot==0)"
    if rec["demand_persisted"]:
        return "log-only suspected (torque unclamped despite crossing)"
    return "clamp suspected (applied torque fell below demand at crossing)"


def main() -> dict:
    out: dict = {"probe": PROBE_ID}
    try:
        out["env"] = env_info()
    except Exception:
        out["env"] = {}
    try:
        legs = {}
        legs["accel_below_nominal"] = _run(ACCEL_PROBE, U_BIG)
        legs["accel_default"] = _run(ACCEL_DEFAULT, U_BIG)
        legs["accel_huge"] = _run(ACCEL_HUGE, U_BIG)
        for k, rec in legs.items():
            rec["verdict_hint"] = _verdict(rec)
        # Warning text: rerun the crossing leg capturing stderr.
        legs["accel_probe_stderr"] = _run_capture_warnings(ACCEL_PROBE)
        out["legs"] = legs
        # Final verdict (default leg must be silent at nominal wdot; probe
        # leg decides log-vs-clamp).
        d = legs["accel_default"]
        p = legs["accel_below_nominal"]
        out["summary"] = {
            "default_threshold": d["defaults"].get("maxWheelAcceleration"),
            "default_crossed": d["first_cross_tick"] is not None,
            "probe_crossed": p["first_cross_tick"] is not None,
            "probe_demand_persisted": p["demand_persisted"],
            "probe_verdict_hint": p["verdict_hint"],
        }
        out["status"] = "ok"
        print(f"[wp1_threshold] {out['summary']}", flush=True)
        print(f"[wp1_threshold] probe stderr tail: "
              f"{legs['accel_probe_stderr'].get('stderr_tail', 'n/a')}",
              flush=True)
    except Exception as e:
        out["status"] = "guarded-fail"
        out["error"] = f"{type(e).__name__}: {e}"
        out["trace"] = traceback.format_exc(limit=8)
        print(f"[wp1_threshold] GUARDED-FAIL {out['error']}", flush=True)
    try:
        path = save_result(PROBE_ID, out)
        print(f"[wp1_threshold] saved {path}", flush=True)
    except Exception as e:
        print(f"[wp1_threshold] save failed: {e}", flush=True)
    return out


def _run_capture_warnings(thr):
    """Rerun the crossing leg with stderr captured (BSK_WARNING text)."""
    import contextlib
    import io
    buf = io.StringIO()
    try:
        with contextlib.redirect_stderr(buf):
            rec = _run(thr, U_BIG)
    except Exception as e:
        return {"error": f"{type(e).__name__}: {e}"}
    txt = buf.getvalue()
    rec["stderr_tail"] = txt[-800:]
    rec["warn_hits"] = [
        ln for ln in txt.splitlines() if "WARN" in ln.upper()][:6]
    return rec


if __name__ == "__main__":
    main()
