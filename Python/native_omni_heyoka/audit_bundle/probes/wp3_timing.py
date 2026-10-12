"""WP3a/WP3b thruster timing diagnostics (PLAN Rev 1).
M0 mailbox matrix, MQ quantizer, M4 arc phase, M5 PWM, M1 ramps,
M2 cutoff ladder, M3 min-pulse, M6 blowdown/swirl/force-msg + static
THRConfig 4-field / THROutput 9-field checks. Results-only writes,
relative paths, stdlib+numpy+Basilisk. LONG_SWEEPS gates M4 ladder.
"""
from __future__ import annotations
import os
import traceback
import numpy as np
try:
    from probes.common import save_result, log_text, env_info, write_exception
except ImportError:
    import json
    def save_result(n, o):
        os.makedirs("results", exist_ok=True)
        json.dump(o, open(os.path.join("results", n + ".json"), "w"), indent=2, default=str)
    def log_text(n, t):
        os.makedirs("results", exist_ok=True)
        open(os.path.join("results", n + ".log"), "a").write(str(t) + "\n")
    def env_info():
        return {"fallback": True}
    def write_exception(n, e):
        log_text(n, "".join(traceback.format_exception(e)))
LONG_SWEEPS = os.environ.get("WP3_LONG", "0") == "1"
try:
    from cubesat_12u_sim.core import config as _C
    MU = float(_C.MU_EARTH); MASS = float(_C.MASS); I6 = np.asarray(_C.INERTIA, float)
except Exception:
    MU = 3.986004418e14; MASS = 12.0; I6 = np.diag([0.16, 0.16, 0.20])
DT_DEFAULT = 0.1
DUTY_Q = [0.0, 1e-12, 0.02, 0.49, 0.5, 0.9, 0.9999999, 1.0, 1.5, -0.1]
DT_LADDER = [0.1, 0.02, 0.01]
CUTOFF_LADDER = [10.0, 100.0, 1000.0]
try:
    import Basilisk  # noqa: F401
    HAS_BSK = True
except Exception:
    HAS_BSK = False
def _one_thruster_sim(duty_cmd, dt=DT_DEFAULT, writer="fresh", factory_kwargs=None, n_steps=3, driver_mode="normal"):
    """Copy of tests/test_thruster_stock.py::_one_thruster_sim (minimal)."""
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.simulation import spacecraft, thrusterDynamicEffector
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, simIncludeThruster
    fk = {"cutoffFrequency": 1000.0, "useMinPulseTime": False}
    if factory_kwargs:
        fk.update(factory_kwargs)
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt)))
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = 48.2 * macros.D2R, 347.8 * macros.D2R, 85.3 * macros.D2R
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[float(I6[i][j]) for j in range(3)] for i in range(3)]
    sc.hub.r_CN_NInit = list(r0); sc.hub.v_CN_NInit = list(v0)
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]; sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
    sim.AddModelToTask("t", sc, 1)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth(); earth.isCentralBody = True; grav.addBodiesTo(sc)
    tf = simIncludeThruster.thrusterFactory()
    tf.create("TEST_Thruster", [0.1, 0.1, 0.0], [-1, 0, 0], MaxThrust=1.0, **fk)
    thr = thrusterDynamicEffector.ThrusterDynamicEffector()
    tf.addToSpacecraft("thr", thr, sc)
    sim.AddModelToTask("t", thr, 2)
    if writer == "fresh":
        mode = driver_mode
        class Driver(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.out = messaging.THRArrayOnTimeCmdMsg(); self._prev = 0.0
            def Reset(self, t):
                p = messaging.THRArrayOnTimeCmdMsgPayload(); p.OnTimeRequest = [0.0] * 36
                self.out.write(p)
            def UpdateState(self, t):
                cur = float(duty_cmd) * dt
                if mode == "delayed":
                    send, self._prev = self._prev, cur
                elif mode == "frozen":
                    return
                else:
                    send = cur
                if mode == "double":
                    p2 = messaging.THRArrayOnTimeCmdMsgPayload(); o2 = [0.0] * 36
                    o2[0] = send * 0.5; p2.OnTimeRequest = o2
                    self.out.write(p2, t, self.moduleID)
                p = messaging.THRArrayOnTimeCmdMsgPayload(); ot = [0.0] * 36
                ot[0] = send; p.OnTimeRequest = ot
                self.out.write(p, t, self.moduleID)
        drv = Driver(); sim.AddModelToTask("t", drv); thr.cmdsInMsg.subscribeTo(drv.out)
    else:
        drv = messaging.THRArrayOnTimeCmdMsg(); pay = messaging.THRArrayOnTimeCmdMsgPayload()
        ot = [0.0] * 36; ot[0] = float(duty_cmd) * dt; pay.OnTimeRequest = ot
        drv.write(pay); thr.cmdsInMsg.subscribeTo(drv)
    log = sc.scStateOutMsg.recorder(); sim.AddModelToTask("t", log)
    tlog = None
    try:
        outs = getattr(thr, "thrusterOutMsgs", None)
        if outs is not None and len(outs) > 0:
            tlog = outs[0].recorder(); sim.AddModelToTask("t", tlog)
    except Exception:
        tlog = None
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(n_steps * dt)); sim.ExecuteSimulation()
    return np.asarray(log.v_CN_N, float), np.asarray(log.omega_BN_B, float), tlog
def _deltas(om):
    return [np.asarray(om[k] - om[k - 1], float) for k in range(1, len(om))]
def _norms(dl):
    return [float(np.linalg.norm(x)) for x in dl]
def _achieved(duty_cmd, dt=DT_DEFAULT, **kw):
    _, om, _ = _one_thruster_sim(duty_cmd, dt=dt, writer="fresh", **kw)
    dom = np.asarray(om[2] - om[1], float)
    return float(-dom[2] * I6[2, 2] / (0.1 * dt))
def _fire_class(dl, ref_idx=1, frac=0.05):
    ref = float(np.linalg.norm(dl[ref_idx])) if len(dl) > ref_idx else 0.0
    return ["fire" if (ref > 0 and float(np.linalg.norm(d)) >= frac * ref) else "idle" for d in dl]
def probe_M0():
    d = 5.0 / 6.0
    _, of, _ = _one_thruster_sim(d, writer="fresh", driver_mode="normal")
    _, od, _ = _one_thruster_sim(d, writer="fresh", driver_mode="double")
    _, oz, _ = _one_thruster_sim(d, writer="fresh", driver_mode="frozen")
    _, oo, _ = _one_thruster_sim(d, writer="once")
    cells = {}
    for n, om in (("fresh_same_FFF", of), ("two_writes_one_step", od), ("frozen_no_advance", oz), ("write_once", oo)):
        dl = _deltas(om)
        cells[n] = {"norms": _norms(dl), "fire_idle": _fire_class(dl),
                    "d1": [float(x) for x in np.asarray(dl[1]).ravel()]}
    return {"duty": d, "cells": cells}
def probe_MQ():
    rows = []
    for duty in DUTY_Q:
        got = _achieved(duty)
        try:
            from cubesat_12u_sim.core.actuators import quantize_duty_cycle
            exp = -float(quantize_duty_cycle(np.clip(np.array([duty]), 0.0, 1.0))[0])
        except Exception:
            c = min(max(float(duty), 0.0), 1.0)
            exp = 0.0 if c <= 0 else (-1.0 / 6.0 if c < 0.5 else (-5.0 / 6.0 if c < 1.0 else -1.0))
        rows.append({"cmd": float(duty), "achieved": float(got), "expected": float(exp), "err": float(got - exp)})
    return {"duties": rows}
def probe_M4():
    d = 5.0 / 6.0; dts = DT_LADDER if LONG_SWEEPS else [DT_DEFAULT]; out = {}
    for dt in dts:
        _, om, _ = _one_thruster_sim(d, dt=dt, writer="fresh", driver_mode="normal")
        _, omd, _ = _one_thruster_sim(d, dt=dt, writer="fresh", driver_mode="delayed")
        dl, dd = _deltas(om), _deltas(omd)
        out[str(dt)] = {"normal_norms": _norms(dl), "delayed_norms": _norms(dd),
                        "delta_matched_dt": [float(np.linalg.norm(a - b)) for a, b in zip(dl, dd)]}
    out["long_sweeps"] = bool(LONG_SWEEPS)
    return out
def probe_M5():
    dt = DT_DEFAULT; duty = 0.5
    _, oa, _ = _one_thruster_sim(duty, dt=dt, writer="fresh")
    _, ob, _ = _one_thruster_sim(duty, dt=dt, writer="fresh", driver_mode="double")
    return {"train_10Hz_norms": _norms(_deltas(oa)), "train_20Hz_proxy_norms": _norms(_deltas(ob)),
            "over_1.5_achieved": float(_achieved(1.5, dt=dt)), "neg_-0.1_achieved": float(_achieved(-0.1, dt=dt))}
def probe_M1():
    static = {}
    try:
        from Basilisk.simulation import thrusterDynamicEffector as _t
        cfg = _t.THRSimConfig()
        for k in ("ThrusterOnRamp", "ThrusterOffRamp", "cutoffFrequency", "MinOnTime", "thrBlowDownCoeff", "MaxSwirlTorque"):
            try:
                v = getattr(cfg, k, None)
                static[k] = len(v) if hasattr(v, "__len__") and not isinstance(v, (float, int)) else v
            except Exception as e:
                static[k] = "err:" + str(e)[:80]
        try:
            static["OnRamp_len"] = len(list(cfg.ThrusterOnRamp)); static["OffRamp_len"] = len(list(cfg.ThrusterOffRamp))
        except Exception:
            pass
    except Exception as e:
        static["error"] = str(e)[:200]
    step = {}
    try:
        from Basilisk.simulation import thrusterDynamicEffector as _t
        step["pair_type"] = "THRTimePair" if hasattr(_t, "THRTimePair") else "n/a-no-pair-type"
        _, om, _ = _one_thruster_sim(1.0, writer="fresh")
        step["step_norms"] = _norms(_deltas(om))
    except Exception as e:
        step["error"] = str(e)[:200]
    return {"static": static, "step": step}
def probe_M2():
    traces = {}
    for fc in CUTOFF_LADDER:
        try:
            _, om, tl = _one_thruster_sim(1.0, writer="fresh", factory_kwargs={"cutoffFrequency": float(fc)})
            cell = {"norms": _norms(_deltas(om))}
            if tl is not None:
                for f in ("thrustFactor", "thrustForce", "thrustBlowDownFactor"):
                    try:
                        cell[f] = [float(x) for x in np.asarray(getattr(tl, f)).ravel()[:8]]
                    except Exception:
                        pass
            else:
                cell["thrusterOutMsgs"] = "unavailable"
            traces[str(fc)] = cell
        except Exception as e:
            traces[str(fc)] = {"error": str(e)[:200]}
    return {"cutoff_ladder": traces}
def probe_M3():
    dt = 0.01; mn = 0.02; fk = {"useMinPulseTime": True, "cutoffFrequency": 1000.0, "MinOnTime": float(mn)}; out = {}
    for m in (0.2, 0.5, 1.0, 1.5):
        ot = m * mn; duty = ot / dt
        try:
            _, om, _ = _one_thruster_sim(duty, dt=dt, writer="fresh", factory_kwargs=fk, n_steps=3)
            out["x%.1f" % m] = {"ontime_cmd": float(ot), "duty_cmd": float(duty), "norms": _norms(_deltas(om))}
        except Exception as e:
            out["x%.1f" % m] = {"error": str(e)[:200]}
    try:
        _, omc, _ = _one_thruster_sim(0.5 * mn / dt, dt=dt, writer="fresh", factory_kwargs=fk, driver_mode="double", n_steps=4)
        out["chatter"] = {"norms": _norms(_deltas(omc))}
    except Exception as e:
        out["chatter"] = {"error": str(e)[:200]}
    out["MinOnTime"] = float(mn); out["dt"] = float(dt)
    return out
def probe_M6():
    out = {}
    try:
        _, om, _ = _one_thruster_sim(0.8, writer="fresh", factory_kwargs={"cutoffFrequency": 1000.0, "thrBlowDownCoeff": [0.1]})
        out["blowdown_smoke_norms"] = _norms(_deltas(om))
    except Exception as e:
        out["blowdown_smoke"] = "raise-or-ignored:" + str(e)[:160]
    try:
        from Basilisk.utilities import simIncludeThruster
        tfp = simIncludeThruster.thrusterFactory()
        try:
            tfp.create("TEST_Thruster", [0.1, 0.1, 0.0], [-1, 0, 0], MaxThrust=1.0, cutoffFrequency=1000.0, useMinPulseTime=False, MaxSwirlTorque=0.05)
            out["swirl_create"] = "accepted"
        except Exception as e:
            out["swirl_create"] = "raise-or-ignored:" + str(e)[:160]
    except Exception as e:
        out["swirl_create"] = "err:" + str(e)[:160]
    try:
        from Basilisk.architecture import messaging
        out["THRArrayCmdForceMsg_present"] = bool(hasattr(messaging, "THRArrayCmdForceMsg"))
        try:
            import omniheyoka.omni_heyoka as _oh  # type: ignore
            out["omni_force_subscribed"] = any("orce" in a for a in dir(_oh.OmniSpacecraft))
        except Exception:
            out["omni_force_subscribed"] = "omni-unavailable"
    except Exception as e:
        out["force_link_error"] = str(e)[:160]
    return out
def probe_static_msgs():
    out = {}
    try:
        from Basilisk.architecture import messaging
        try:
            cc = getattr(messaging, "THRConfigMsgPayload")
            out["THRConfig_4field"] = {f: (f in dir(cc)) for f in ("maxThrust", "rThrust_B", "swirlTorque", "tHatThrust_B")}
        except Exception as e:
            out["THRConfig_4field"] = {"error": str(e)[:160]}
        try:
            co = getattr(messaging, "THROutputMsgPayload")
            nm = [n for n in dir(co) if not n.startswith("_")]
            out["THROutput_9field"] = {k: (k in nm) for k in ("thrustFactor", "thrustForce", "thrustForce_B", "thrustTorquePntB_B", "thrustBlowDownFactor", "ispBlowDownFactor", "maxThrust", "thrusterLocation", "thrusterDirection")}
        except Exception as e:
            out["THROutput_9field"] = {"error": str(e)[:160]}
    except Exception as e:
        out["error"] = str(e)[:200]
    return out
def run():
    res: dict = {"env": {}, "probes": {}}
    try:
        res["env"] = dict(env_info())
    except Exception as e:
        res["env"] = {"error": str(e)[:200]}
    res["meta"] = {"long_sweeps": bool(LONG_SWEEPS), "has_basilisk": bool(HAS_BSK)}
    if not HAS_BSK:
        res["probes"] = {"static": "unavailable-no-basilisk"}
        save_result("wp3_timing", res)
        log_text("wp3_timing", "Basilisk missing; static-unavailable path saved.")
        return res
    for pid, fn in (("M0", probe_M0), ("MQ", probe_MQ), ("M4", probe_M4), ("M5", probe_M5), ("M1", probe_M1), ("M2", probe_M2), ("M3", probe_M3), ("M6", probe_M6), ("SMSG", probe_static_msgs)):
        try:
            res["probes"][pid] = fn(); log_text("wp3_timing", pid + " ok")
        except BaseException as exc:
            try:
                write_exception("wp3_timing", exc)
            except Exception:
                pass
            res["probes"][pid] = {"error": str(exc)[:300]}
            log_text("wp3_timing", pid + " ERROR " + str(exc)[:200])
    save_result("wp3_timing", res)
    return res
if __name__ == "__main__":
    try:
        run()
    except BaseException as exc:
        try:
            write_exception("wp3_timing", exc)
        except Exception:
            pass
        try:
            save_result("wp3_timing", {"status": "fatal", "error": str(exc)[:500]})
        except Exception:
            pass
        raise
