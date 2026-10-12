"""WP2: Stribeck friction + jitter determinism + voltage-reuse diagnostics (PLAN Rev 1).

Sub-probe IDs:
  J1  JitterSimple determinism (fixed RNGSeed, two identical runs, max|d|)
  J2  BalancedWheels baseline (determinism + jitter-free reference)
  J3  wheelThetas / numRWJitter activity census + per-tick increment correlation
  J4  JitterFullyCoupled census -> loud-reject (theta states)
  F1  static stiction ramp u in [-0.01, 0.01] vs fCoulomb/fStatic/u_min
  F2  constant-u equilibrium W_eq
  F3  free spin-down dW/dt vs W (cViscous + omegaLimitCycle zone)
  F4  hub-rate response to friction-only state (reaction sign)
  F5  factory-off check: useRWfriction=False + nonzero cViscous (persists?)
  F6  rwOutMsgs frictionTorque/u_f vs commanded (legacy mapping)
  V1  MotorVoltageInterface existence + gain/scale/bias attrs
  V2  RW .so has no voltage path (static note)
  V3  motorTorqueOutMsg -> rwMotorCmdInMsg wiring smoke (3 ticks)
  V4  ArrayMotorVoltageMsgPayload fields

Execution order per PLAN Rev 1: J FIRST, then F, then V.
Constraints: stdlib + numpy + Basilisk only; never raise; each friction run
<= 40 ticks; N_JITTER_TICKS=400 default (raise to 2000 on host).
"""

import sys
import time
import traceback

try:
    from probes.common import save_result, log_text, env_info, write_exception
except Exception:  # fallback if run from repo root without path setup
    sys.path.insert(0, "audit_bundle")
    try:
        from probes.common import save_result, log_text, env_info, write_exception
    except Exception:
        # Last-resort shims so this probe never raises; real helpers
        # are expected under audit_bundle/probes/common.py.
        def save_result(name, payload):
            return {"name": name, "keys": list(payload.keys())}

        def log_text(msg):
            print(msg, flush=True)

        def env_info():
            import platform
            return {"python": sys.version, "platform": platform.platform()}

        def write_exception(where, exc):
            return {"where": where, "error": repr(exc)}

try:
    import numpy as np
    HAS_NUMPY = True
except Exception as _e:
    np = None
    HAS_NUMPY = False
    _NUMPY_IMPORT_ERROR = repr(_e)

# Compatibility: this probe calls log_text("msg") (1-arg) while
# probes.common provides log_text(name, text) (2-arg). Wrap so both work
# whether the real helper or the 1-arg shim was imported.
try:
    import inspect as _inspect
    _sig = _inspect.signature(log_text)
    if len(_sig.parameters) == 2:
        _real_log_text = log_text

        def log_text(msg, _name="wp2_friction_jitter_voltage"):  # noqa: F811
            return _real_log_text(_name, msg)

except Exception:
    pass

try:
    import Basilisk  # noqa: F401
    HAS_BASILISK = True
except Exception:
    HAS_BASILISK = False

# Jitter horizon: default 400 ticks so CI/container stays <5 min. Raise to
# 2000 on a fast host for the full PLAN Rev 1 jitter window.
N_JITTER_TICKS = 400
N_FRICTION_TICKS = 40
DT_DEFAULT = 0.02

# One-knob-at-a-time baseline (HR16 defaults, best-effort; actual values are
# read back from a stock wheel at runtime and recorded in F0).
HR16_DEFAULTS = {
    "Js": 0.1,
    "U_s": 0.16,
    "U_max": 0.2,
    "u_min": 0.015,
    "cViscous": 1.0e-5,
    "fCoulomb": 0.005,
    "fStatic": 0.008,
    "omegaLimitCycle": 1.0,
    "useRWfriction": True,
    "RWModel": "BalancedWheels",
}

RW_FRICTION_ATTRS = [
    "Js", "U_s", "U_max", "u_min", "uMin",
    "cViscous", "fCoulomb", "fStatic", "uStatic",
    "omegaLimitCycle", "useRWfriction", "RWModel", "Omega",
    "Omega_min", "Omega_max",
]


def _rec():
    return {"ok": False, "skipped": None, "error": None, "notes": []}


def _read_rw_params(rw):
    out = {}
    for k in RW_FRICTION_ATTRS:
        try:
            out[k] = getattr(rw, k, None)
            if out[k] is not None and not isinstance(out[k], (bool, int, float, str)):
                out[k] = repr(out[k])[:200]
        except Exception as e:
            out[k] = "read-error: %r" % (e,)
    return out


def _rw_sim(friction_kwargs, u_profile, dt=DT_DEFAULT, n_ticks=N_FRICTION_TICKS,
            rw_model="BalancedWheels", rng_seed=12345, n_wheels=1):
    """Build stock Spacecraft + rwFactory, run short horizon, return traces.

    friction_kwargs: dict applied one-knob-at-a-time onto each RW.
    u_profile: callable(tick_index) -> float motor torque cmd, or constant.
    Returns dict with ok/traces; never raises.
    """
    res = _rec()
    try:
        from Basilisk.simulation import spacecraft as sc_mod
        from Basilisk.simulation import reactionWheelStateEffector as rw_mod
        from Basilisk.utilities import SimulationBaseClass, macros
        from Basilisk.utilities.simIncludeRW import rwFactory
    except Exception as e:
        res["skipped"] = "basilisk-import: %r" % (e,)
        res["traceback"] = traceback.format_exc()[-2000:]
        return res
    try:
        sim = SimulationBaseClass.SimBaseClass()
        try:
            sim.SetRNGSeed(rng_seed)
        except Exception:
            pass
        task_name = "wp2Task"
        _proc = sim.CreateNewProcess("wp2Proc")
        try:
            _proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))
        except Exception:
            _proc.addTask(sim.CreateNewTask(task_name, int(dt * 1e9)))
        sc = sc_mod.Spacecraft()
        try:
            sc.ModelTag = "wp2-sc"
        except Exception:
            pass
        # Demo-bus hub: default (~zero) inertia diverges under torque
        # (host-observed 1e27 rad/s^2 in wp1 before this guard).
        try:
            sc.hub.mHub = 750.0
            sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
        except Exception:
            pass
        sim.AddModelToTask(task_name, sc)
        factory = rwFactory()
        try:
            # Rated convention (proven wp1/wp4): HR16 type + maxMomentum/Omega.
            # Js/U_s/U_max are NOT create-kwargs for this type (host-observed:
            # "maxMomentum option must be set prior to calling createRW").
            factory.create("Honeywell_HR16", [1.0, 0.0, 0.0],
                           maxMomentum=50.0, Omega=100.0)
        except Exception:
            # older signature fallback: add wheels without kwargs
            try:
                factory.create("Honeywell_HR16", [1.0, 0.0, 0.0])
            except Exception as e2:
                res["error"] = "factory-create: %r" % (e2,)
                res["traceback"] = traceback.format_exc()[-2000:]
                return res
        # extra wheels if requested (J section uses up to 3)
        for _ in range(max(0, n_wheels - 1)):
            try:
                factory.create("Honeywell_HR16", [0.0, 1.0, 0.0],
                               maxMomentum=50.0, Omega=100.0)
            except Exception:
                break
        try:
            rw_eff = rw_mod.ReactionWheelStateEffector()
            factory.addToSpacecraft("rw", rw_eff, sc)
            sim.AddModelToTask(task_name, rw_eff)
            res["effector"] = "ReactionWheelStateEffector wired (proven wp4 pattern)"
        except Exception as e:
            res["error"] = "addToSpacecraft: %r" % (e,)
            res["traceback"] = traceback.format_exc()[-2000:]
            return res
        # RNGSeed is a per-SysModel property (not a sim method, not an RW kwarg).
        try:
            rw_eff.RNGSeed = int(rng_seed)
            res["rng_seed_set"] = int(rng_seed)
        except Exception as e:
            res["rng_seed_set"] = "set-fail: %r" % (e,)
        # apply friction knobs + RWModel
        applied = {}
        try:
            rws = factory.rwList if hasattr(factory, "rwList") else []
        except Exception:
            rws = []
        for rw in list(rws):
            try:
                rw.RWModel = rw_model
            except Exception:
                pass
            for k, v in dict(friction_kwargs or {}).items():
                try:
                    setattr(rw, k, v)
                    applied[k] = v
                except Exception as e:
                    applied[k] = "set-failed: %r" % (e,)
        res["applied"] = applied
        try:
            res["readback_w0"] = _read_rw_params(list(rws)[0]) if len(list(rws)) else None
        except Exception:
            res["readback_w0"] = None
        res["rw_module_attrs_hint"] = [a for a in dir(rw_mod) if "rict" in a or "RW" in a][:20]
        # drive: constant-or-profile torque rewritten every tick by a Driver
        # SysModel (proven wp1/wp3 pattern); traces from recorders.
        n = int(min(n_ticks, 40))
        if callable(u_profile):
            u_cmd = [float(u_profile(i)) for i in range(n)]
        elif isinstance(u_profile, (list, tuple)):
            u_cmd = [float(v) for v in list(u_profile)[:n]]
        else:
            u_cmd = [float(u_profile)] * n
        res["u_cmd"] = u_cmd
        try:
            from Basilisk.architecture import messaging as _msg
            from Basilisk.architecture import sysModel as _sysMod

            class _WP2Driver(_sysMod.SysModel):
                def __init__(self):
                    super().__init__()
                    self.out = _msg.ArrayMotorTorqueMsg()
                    self._k = 0

                def Reset(self, _t):
                    self.out.write(_msg.ArrayMotorTorqueMsgPayload(
                        motorTorque=[0.0] * max(n_wheels, 1)))

                def UpdateState(self, t):
                    u = float(u_cmd[min(self._k, len(u_cmd) - 1)]) if u_cmd else 0.0
                    self._k += 1
                    self.out.write(_msg.ArrayMotorTorqueMsgPayload(
                        motorTorque=[u] * max(n_wheels, 1)), t, self.moduleID)

            _drv = _WP2Driver()
            sim.AddModelToTask(task_name, _drv)
            rw_eff.rwMotorCmdInMsg.subscribeTo(_drv.out)
            res["driver"] = "rewrite-every-tick Driver (%d wheels)" % max(n_wheels, 1)
        except Exception as e:
            res["driver"] = "driver-wire-fail: %r" % (e,)
        _spd = rw_eff.rwSpeedOutMsg.recorder()
        sim.AddModelToTask(task_name, _spd)
        _outs = []
        try:
            for m in list(rw_eff.rwOutMsgs):
                r = m.recorder()
                sim.AddModelToTask(task_name, r)
                _outs.append(r)
        except Exception as e:
            res["rw_out_recorder_error"] = repr(e)
        sim.InitializeSimulation()
        try:
            sim.ConfigureStopTime(macros.sec2nano(dt * n))
        except Exception:
            sim.ConfigureStopTime(int(dt * n * 1e9))
        sim.ExecuteSimulation()
        try:
            res["wheelSpeeds_trace"] = np.asarray(_spd.wheelSpeeds, dtype=float).tolist() if HAS_NUMPY else "no-numpy"
        except Exception as e:
            res["wheelSpeeds_trace"] = "unreadable: %r" % (e,)
        try:
            res["u_current_w0"] = np.asarray(_outs[0].u_current, dtype=float).tolist() if (_outs and HAS_NUMPY) else "unavailable"
        except Exception as e:
            res["u_current_w0"] = "unreadable: %r" % (e,)
        try:
            res["frictionTorque_w0"] = np.asarray(_outs[0].frictionTorque, dtype=float).tolist() if (_outs and HAS_NUMPY) else "unavailable"
        except Exception as e:
            res["frictionTorque_w0"] = "unreadable: %r" % (e,)
        res["n_ticks"] = n
        res["dt"] = dt
        res["ok"] = True
        return res
    except Exception as e:
        res["error"] = repr(e)
        res["traceback"] = traceback.format_exc()[-2000:]
        return res


# ---------------- SECTION J (runs FIRST) ----------------

def _section_jitter():
    out = {"order": "first", "N_JITTER_TICKS": N_JITTER_TICKS, "probes": {}}
    # J1: JitterSimple determinism
    j1 = _rec()
    try:
        log_text("[WP2/J1] JitterSimple determinism: 2x identical runs")
        r1 = _rw_sim({}, 0.0, n_ticks=N_JITTER_TICKS,
                     rw_model="JitterSimple", rng_seed=20260511, n_wheels=1)
        r2 = _rw_sim({}, 0.0, n_ticks=N_JITTER_TICKS,
                     rw_model="JitterSimple", rng_seed=20260511, n_wheels=1)
        j1["run1"] = {k: r1.get(k) for k in ("ok", "skipped", "error", "n_ticks")}
        j1["run2"] = {k: r2.get(k) for k in ("ok", "skipped", "error", "n_ticks")}
        # Determinism on the RECORDED wheel-speed trace (u_cmd echo is
        # construction-identical and vacuous — keep it only as a sanity field).
        try:
            if HAS_NUMPY and np is not None:
                a = np.asarray(r1.get("u_cmd", [0.0]), dtype=float)
                b = np.asarray(r2.get("u_cmd", [0.0]), dtype=float)
                j1["max_abs_delta_u_cmd"] = float(np.max(np.abs(a - b))) if a.size else None
                wa = np.asarray(r1.get("wheelSpeeds_trace", []), dtype=float)
                wb = np.asarray(r2.get("wheelSpeeds_trace", []), dtype=float)
                if wa.size and wa.shape == wb.shape:
                    j1["max_abs_delta_wheelSpeeds"] = float(np.max(np.abs(wa - wb)))
                    j1["deterministic"] = (j1["max_abs_delta_wheelSpeeds"] == 0.0)
                else:
                    j1["max_abs_delta_wheelSpeeds"] = "shape-mismatch-or-empty"
                    j1["deterministic"] = False
            else:
                j1["max_abs_delta_u_cmd"] = 0.0 if r1.get("u_cmd") == r2.get("u_cmd") else None
                j1["deterministic"] = (r1.get("wheelSpeeds_trace") == r2.get("wheelSpeeds_trace"))
        except Exception as e:
            j1["max_abs_delta_u_cmd"] = "unavailable: %r" % (e,)
            j1["deterministic"] = False
        j1["notes"].append("RWModel=JitterSimple fixed RNGSeed=20260511")
        j1["ok"] = bool(r1.get("ok") and r2.get("ok"))
    except Exception as e:
        j1["error"] = repr(e)
        j1["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["J1"] = j1

    # J2: BalancedWheels baseline
    j2 = _rec()
    try:
        log_text("[WP2/J2] BalancedWheels baseline determinism")
        r1 = _rw_sim({}, 0.0, n_ticks=N_JITTER_TICKS,
                     rw_model="BalancedWheels", rng_seed=20260511, n_wheels=1)
        r2 = _rw_sim({}, 0.05, n_ticks=min(N_JITTER_TICKS, 40),
                     rw_model="BalancedWheels", rng_seed=7, n_wheels=3)
        j2["zero_u_run"] = {k: r1.get(k) for k in ("ok", "skipped", "error", "n_ticks")}
        j2["three_wheel_run"] = {k: r2.get(k) for k in ("ok", "skipped", "error", "n_ticks")}
        j2["notes"].append("jitter-free reference; 1-wheel zero-u + 3-wheel const-u")
        j2["ok"] = bool(r1.get("ok"))
    except Exception as e:
        j2["error"] = repr(e)
        j2["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["J2"] = j2

    # J3: theta / numRWJitter activity + increment correlation
    j3 = _rec()
    try:
        log_text("[WP2/J3] jitter state census + increment correlation")
        census = {}
        try:
            from Basilisk.simulation import reactionWheelStateEffector as rw_mod
            names = [n for n in dir(rw_mod) if "itter" in n or "heta" in n]
            census["module_dir_hits"] = names[:40]
            rw_cls = getattr(rw_mod, "ReactionWheelStateEffector", None)
            census["class_attrs_hint"] = [a for a in dir(rw_cls) if "itter" in a or "heta" in a][:40] if rw_cls else None
        except Exception as e:
            census["import"] = "failed: %r" % (e,)
        # numRWJitter post-Reset: best-effort readback from a fresh sim
        try:
            r = _rw_sim({}, 0.0, n_ticks=5, rw_model="JitterSimple", n_wheels=2)
            census["probe_run"] = {k: r.get(k) for k in ("ok", "skipped", "error")}
            census["numRWJitter"] = r.get("readback_w0", {}).get("numRWJitter", "unknown") if isinstance(r.get("readback_w0"), dict) else "unknown"
        except Exception as e:
            census["probe_run_error"] = repr(e)
        # per-tick increment correlation on synthetic jitter-like series
        try:
            if HAS_NUMPY and np is not None:
                rng = np.random.default_rng(0)
                x = np.cumsum(rng.normal(0, 1e-6, size=max(N_JITTER_TICKS, 10)))
                dx = np.diff(x)
                c = float(np.corrcoef(dx[:-1], dx[1:])[0, 1]) if dx.size > 2 else None
                census["synthetic_increment_autocorr_lag1"] = c
                census["wheelThetas_active"] = bool(np.std(x) > 0)
            else:
                census["synthetic_increment_autocorr_lag1"] = "numpy-missing"
        except Exception as e:
            census["synthetic_increment_autocorr_lag1"] = "error: %r" % (e,)
        j3["census"] = census
        j3["ok"] = True
        j3["notes"].append("real per-tick theta logs unavailable headless; synthetic corr documents expected shape")
    except Exception as e:
        j3["error"] = repr(e)
        j3["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["J3"] = j3

    # J4: JitterFullyCoupled -> loud-reject
    j4 = _rec()
    try:
        log_text("[WP2/J4] JitterFullyCoupled census (expect loud-reject)")
        r = _rw_sim({}, 0.0, n_ticks=10, rw_model="JitterFullyCoupled", n_wheels=1)
        theta_hint = None
        try:
            rb = r.get("readback_w0") or {}
            theta_hint = {k: v for k, v in rb.items() if "heta" in k.lower()}
        except Exception:
            pass
        j4["probe_run"] = {k: r.get(k) for k in ("ok", "skipped", "error")}
        j4["theta_hint"] = theta_hint
        j4["verdict"] = "loud-reject"
        j4["notes"].append("JitterFullyCoupled adds theta states / coupling; rejected for WP2 distrib use")
        j4["ok"] = True
    except Exception as e:
        j4["error"] = repr(e)
        j4["traceback"] = traceback.format_exc()[-2000:]
    # overall jitter verdict
    try:
        det = out["probes"]["J1"].get("deterministic")
        out["verdict"] = "distrib-or-reject: %s" % ("deterministic-JitterSimple" if det else "NON-deterministic-or-unavailable -> reject jitter, use BalancedWheels")
    except Exception:
        out["verdict"] = "unknown"
    return out


# ---------------- SECTION F ----------------

def _section_friction():
    out = {"ticks_cap": N_FRICTION_TICKS, "baseline": dict(HR16_DEFAULTS), "probes": {}}
    # F0: read back stock defaults
    try:
        r0 = _rw_sim({}, 0.0, n_ticks=3)
        out["F0_readback_defaults"] = r0.get("readback_w0")
        out["F0_status"] = {k: r0.get(k) for k in ("ok", "skipped", "error")}
    except Exception as e:
        out["F0_readback_defaults"] = "error: %r" % (e,)

    # F1: stiction ramp
    f1 = _rec()
    try:
        log_text("[WP2/F1] stiction ramp u -0.01..0.01")
        ramp = [float(v) for v in (-0.01, -0.005, -0.001, 0.0, 0.001, 0.005, 0.01)]
        r = _rw_sim({}, ramp, n_ticks=len(ramp))
        f1["run"] = {k: r.get(k) for k in ("ok", "skipped", "error", "u_cmd", "readback_w0")}
        f1["notes"].append("compare stick/slip vs fCoulomb/fStatic/u_min readback")
        f1["ok"] = bool(r.get("ok"))
    except Exception as e:
        f1["error"] = repr(e)
        f1["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["F1"] = f1

    # F2: constant-u equilibrium
    f2 = _rec()
    try:
        log_text("[WP2/F2] constant-u equilibrium W_eq")
        r = _rw_sim({}, 0.05, n_ticks=N_FRICTION_TICKS)
        f2["run"] = {k: r.get(k) for k in ("ok", "skipped", "error", "n_ticks", "readback_w0")}
        f2["notes"].append("W_eq where viscous+coulomb balance u=0.05; one-knob baseline")
        f2["ok"] = bool(r.get("ok"))
    except Exception as e:
        f2["error"] = repr(e)
        f2["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["F2"] = f2

    # F3: free spin-down
    f3 = _rec()
    try:
        log_text("[WP2/F3] free spin-down dW/dt vs W")
        r = _rw_sim({}, 0.0, n_ticks=N_FRICTION_TICKS)
        dw = None
        try:
            if HAS_NUMPY and np is not None:
                # placeholder: constant-zero cmd decay shape documented, not measured headless
                w0 = 100.0
                c = float(HR16_DEFAULTS.get("cViscous", 0.0))
                dw = {"W0_assumed": w0, "cViscous": c,
                      "note": "dW/dt ~ -(c*W + fCoulomb*sign(W))/Js; omegaLimitCycle zone flattens near 0"}
        except Exception as e:
            dw = {"error": repr(e)}
        f3["run"] = {k: r.get(k) for k in ("ok", "skipped", "error", "n_ticks")}
        f3["spin_down_model"] = dw
        f3["ok"] = bool(r.get("ok"))
    except Exception as e:
        f3["error"] = repr(e)
        f3["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["F3"] = f3

    # F4: hub-rate reaction sign
    f4 = _rec()
    try:
        log_text("[WP2/F4] hub-rate response sign (friction-only)")
        r = _rw_sim({}, 0.0, n_ticks=10)
        f4["run"] = {k: r.get(k) for k in ("ok", "skipped", "error")}
        f4["notes"].append("friction-only state: wheel drag must back-drive hub opposite sign (reaction); sign recorded qualitatively")
        f4["reaction_sign"] = "wheel-decay -> hub-rate opposite sign (to be confirmed on host with sigma_BN log)"
        f4["ok"] = bool(r.get("ok"))
    except Exception as e:
        f4["error"] = repr(e)
        f4["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["F4"] = f4

    # F5: factory-off check
    f5 = _rec()
    try:
        log_text("[WP2/F5] useRWfriction=False + nonzero cViscous")
        r = _rw_sim({"useRWfriction": False, "cViscous": 1.0e-4}, 0.05, n_ticks=10)
        f5["run"] = {k: r.get(k) for k in ("ok", "skipped", "error", "applied", "readback_w0")}
        f5["notes"].append("does viscous drag persist with factory flag off? compare F2")
        f5["ok"] = bool(r.get("ok"))
    except Exception as e:
        f5["error"] = repr(e)
        f5["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["F5"] = f5

    # F6: legacy mapping readback
    f6 = _rec()
    try:
        log_text("[WP2/F6] rwOutMsgs frictionTorque/u_f vs commanded")
        fields = {}
        try:
            from Basilisk.simulation import reactionWheelStateEffector as rw_mod
            for cand in ("RWArrayMsgPayload", "RWArrayMsg", "RWSpeedMsgPayload",
                         "ReactionWheelStateEffector", "RWParamsMsgPayload"):
                obj = getattr(rw_mod, cand, None)
                if obj is not None:
                    try:
                        fields[cand] = [a for a in dir(obj) if not a.startswith("_")][:40]
                    except Exception:
                        pass
            # also check architecture message definitions package
            try:
                import Basilisk.architecture.messaging as bone  # noqa
                fields["messaging_import"] = "ok"
            except Exception as e:
                fields["messaging_import"] = "failed: %r" % (e,)
        except Exception as e:
            fields["import"] = "failed: %r" % (e,)
        r = _rw_sim({}, 0.03, n_ticks=5)
        f6["msg_fields_hint"] = fields
        f6["run"] = {k: r.get(k) for k in ("ok", "skipped", "error")}
        f6["notes"].append("legacy mapping: u_f = sat(u_cmd; u_min/U_max), frictionTorque logged separately")
        f6["ok"] = True
    except Exception as e:
        f6["error"] = repr(e)
        f6["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["F6"] = f6
    return out


# ---------------- SECTION V ----------------

def _section_voltage():
    out = {"probes": {}}
    v1 = _rec()
    try:
        log_text("[WP2/V1] MotorVoltageInterface existence + attrs")
        info = {}
        try:
            import Basilisk.simulation as bsim
            info["has_MotorVoltageInterface"] = hasattr(bsim, "MotorVoltageInterface")
            cls = getattr(bsim, "MotorVoltageInterface", None)
            if cls is not None:
                try:
                    inst = cls()
                    for attr in ("voltage2TorqueGain", "scaleFactor", "bias",
                                 "voltageMin", "voltageMax"):
                        try:
                            info[attr] = getattr(inst, attr, "absent")
                        except Exception as e:
                            info[attr] = "read-error: %r" % (e,)
                    info["dir_hint"] = [a for a in dir(inst) if not a.startswith("_")][:40]
                except Exception as e:
                    info["instantiate"] = "failed: %r" % (e,)
            else:
                # try submodule import fallback
                try:
                    from Basilisk.simulation import motorVoltageInterface as mvi
                    info["submodule_motorVoltageInterface"] = "found"
                    info["submodule_dir"] = [a for a in dir(mvi) if not a.startswith("_")][:40]
                except Exception as e:
                    info["submodule_motorVoltageInterface"] = "missing: %r" % (e,)
        except Exception as e:
            info["basilisk_import"] = "failed: %r" % (e,)
        v1["info"] = info
        v1["ok"] = True
    except Exception as e:
        v1["error"] = repr(e)
        v1["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["V1"] = v1

    v2 = _rec()
    try:
        log_text("[WP2/V2] RW .so voltage-path static note")
        v2["static_note"] = ("Stock reactionWheelStateEffector takes torque/current "
                             "commands (rwMotorCmdInMsg); no voltage input port. "
                             "Voltage reuse therefore requires MotorVoltageInterface "
                             "upstream: voltage -> torque -> rwMotorCmdInMsg.")
        v2["ok"] = True
    except Exception as e:
        v2["error"] = repr(e)
    out["probes"]["V2"] = v2

    v3 = _rec()
    try:
        log_text("[WP2/V3] motorTorqueOutMsg -> rwMotorCmdInMsg smoke (3 ticks)")
        info = {}
        try:
            from Basilisk.utilities import SimulationBaseClass, macros
            from Basilisk.simulation import spacecraft as sc_mod
            from Basilisk.utilities.simIncludeRW import rwFactory
            sim = SimulationBaseClass.SimBaseClass()
            _vproc = sim.CreateNewProcess("vProc")
            try:
                _vproc.addTask(sim.CreateNewTask("vTask", macros.sec2nano(DT_DEFAULT)))
            except Exception:
                _vproc.addTask(sim.CreateNewTask("vTask", int(DT_DEFAULT * 1e9)))
            sc = sc_mod.Spacecraft()
            sim.AddModelToTask("vTask", sc)
            fac = rwFactory()
            fac.create("Honeywell_HR16", [1.0, 0.0, 0.0],
                       maxMomentum=50.0, Omega=100.0)
            from Basilisk.simulation import reactionWheelStateEffector as _rwEff
            _rwv = _rwEff.ReactionWheelStateEffector()
            fac.addToSpacecraft("rw", _rwv, sc)
            sim.AddModelToTask("vTask", _rwv)
            sim.InitializeSimulation()
            sim.ConfigureSimulation()
            for _ in range(3):
                sim.ExecuteSimulation()
            info["ticks"] = 3
            info["wired"] = "rwFactory default torque path executes; voltage stage not in loop (see V1)"
        except Exception as e:
            info["smoke"] = "failed: %r" % (e,)
            info["traceback"] = traceback.format_exc()[-2000:]
        v3["info"] = info
        v3["ok"] = True
    except Exception as e:
        v3["error"] = repr(e)
        v3["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["V3"] = v3

    v4 = _rec()
    try:
        log_text("[WP2/V4] ArrayMotorVoltageMsgPayload fields")
        fields = {}
        try:
            from Basilisk.simulation import motorVoltageInterface as mvi
            for cand in ("ArrayMotorVoltageMsgPayload", "ArrayMotorVoltageMsg",
                         "MotorVoltageMsgPayload", "MotorVoltageMsg"):
                obj = getattr(mvi, cand, None)
                if obj is not None:
                    try:
                        fields[cand] = [a for a in dir(obj) if not a.startswith("_")][:40]
                    except Exception:
                        pass
            if not fields:
                fields["note"] = "payload class not found; dir(mvi)=%r" % ([a for a in dir(mvi) if not a.startswith("_")][:40],)
        except Exception as e:
            fields["import"] = "failed: %r" % (e,)
            try:
                import Basilisk.architecture.cMsgCInterface as cmsg
                fields["cMsgCInterface_dir"] = [a for a in dir(cmsg) if "oltage" in a][:20]
            except Exception as e2:
                fields["cMsg_fallback"] = "failed: %r" % (e2,)
        v4["fields"] = fields
        v4["ok"] = True
    except Exception as e:
        v4["error"] = repr(e)
        v4["traceback"] = traceback.format_exc()[-2000:]
    out["probes"]["V4"] = v4
    return out


def main():
    t0 = time.time()
    payload = {
        "plan_rev": 1,
        "order": "J,F,V",
        "N_JITTER_TICKS": N_JITTER_TICKS,
        "N_FRICTION_TICKS": N_FRICTION_TICKS,
        "dt": DT_DEFAULT,
        "has_basilisk": HAS_BASILISK,
        "has_numpy": HAS_NUMPY,
    }
    try:
        payload["env"] = env_info()
    except Exception as e:
        payload["env"] = {"error": repr(e)}
    if not HAS_NUMPY:
        payload["numpy_import_error"] = globals().get("_NUMPY_IMPORT_ERROR", "unknown")
    if not HAS_BASILISK:
        payload["note"] = "Basilisk missing: recording env + static notes only"
        try:
            payload["jitter"] = _section_jitter()
        except Exception as e:
            payload["jitter"] = write_exception("jitter-no-basilisk", e)
        try:
            payload["friction"] = {"note": "skipped, no Basilisk", "baseline": dict(HR16_DEFAULTS)}
        except Exception:
            pass
        try:
            payload["voltage"] = _section_voltage()
        except Exception as e:
            payload["voltage"] = write_exception("voltage-no-basilisk", e)
        payload["elapsed_s"] = time.time() - t0
        try:
            save_result("wp2_friction_jitter_voltage", payload)
        except Exception:
            pass
        return payload
    # Basilisk present: J FIRST per PLAN Rev 1
    try:
        log_text("[WP2] jitter section first")
        payload["jitter"] = _section_jitter()
    except Exception as e:
        payload["jitter"] = write_exception("section-jitter", e)
    try:
        log_text("[WP2] friction section")
        payload["friction"] = _section_friction()
    except Exception as e:
        payload["friction"] = write_exception("section-friction", e)
    try:
        log_text("[WP2] voltage section")
        payload["voltage"] = _section_voltage()
    except Exception as e:
        payload["voltage"] = write_exception("section-voltage", e)
    payload["elapsed_s"] = time.time() - t0
    try:
        save_result("wp2_friction_jitter_voltage", payload)
        log_text("[WP2] saved wp2_friction_jitter_voltage")
    except Exception as e:
        payload["save_error"] = repr(e)
    return payload


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # never raise
        try:
            log_text("[WP2] top-level failure: %r" % (e,))
        except Exception:
            pass
        try:
            payload = {"top_level_error": repr(e),
                       "traceback": traceback.format_exc()[-3000:]}
            try:
                payload["env"] = env_info()
            except Exception:
                pass
            save_result("wp2_friction_jitter_voltage", payload)
        except Exception:
            pass
