#!/usr/bin/env python3.11
"""WP4+WP5 combo: hub-offset x env-force torque arms (stock-direct decision).

OPEN QUESTION (decided here, not by fiat): under hub offset c_B != 0, do the
stock drag / SRP torque arms shift by -c_B (like the measured thruster rule
(r-c)xF, P-WP4.4) or stay unshifted?

  PLAN_ENVELOPE.md Rev 1 WP4 (corrected): input-I about CoM, dynamics arms
  shift (r-c)xF, telemetry about-B unshifted.
  WP5: drag tau = comOffset x F, SRP tau = r_Cop x F, magnetic no arm.
  wp4_offset.json P-WP4.4 arm note: telemetry-about-B == unshifted r_thr x F.
  wp5_env.json: drag / SRP-faceted numbers (LEO ~400 km, 6-facet box).

What this probe does (STOCK ONLY, no omni, no analytic theory as verdict):
  (a) LEO drag fixture (~400 km) + dragDynamicEffector, hub offset
      [0.1,0,0] vs zero offset, same comOffset/CdA: per-tick torque delta
      dT = T_off - T_no, tested against -c x F to 5%.
  (b) Faceted-SRP decision facet (sun-facing, off-center r_Cop, so the arm
      is observable; the symmetric 6-facet box totals ~0 per the ledger and
      cannot decide): offset vs no-offset, same question for r_Cop arms.
  (c) Stock drag with comOffset=0 and c != 0: is |T| ~ 0 (unshifted) or
      ~ |-c x F| (shifted)?

Verdict rule (per leg, stock-direct only):
  shifted    if median |dT - (-c x F)| / max(|-c x F|) < 0.05,
  unshifted  if median |dT| / max(|-c x F|) < 0.05 (delta ~ 0),
  else ambiguous (both numbers recorded, no fiat).

Magnetic: no arm by construction (tau = m x B_B); static note only.
Saves via save_result('wp45_combo'). Every sim guarded try/except, horizons
<= 40 ticks, never raises. Stdlib+numpy+Basilisk(guarded) only.
"""
from __future__ import annotations

import importlib
import os
import sys
import traceback

import numpy as np

try:
    from probes.common import save_result, log_text, env_info, write_exception
except ImportError:  # running from inside probes/
    try:
        from common import save_result, log_text, env_info, write_exception  # type: ignore
    except ImportError:
        import json as _js
        from pathlib import Path as _P

        def save_result(n: str, p: dict) -> str:  # type: ignore
            o = _P(__file__).resolve().parents[1] / "results" / f"{n}.json"
            o.parent.mkdir(parents=True, exist_ok=True)
            o.write_text(_js.dumps({"name": n, "payload": p}, indent=2, default=str))
            return str(o)

        def log_text(n: str, t: str) -> str:  # type: ignore
            print(f"[{n}] {t}", flush=True)
            return ""

        def env_info() -> dict:  # type: ignore
            return {"python": sys.version, "stub": True}

        def write_exception(n: str, e: BaseException) -> str:  # type: ignore
            traceback.print_exc()
            return ""

PROBE_ID = "wp45_combo"
DT = 0.1
NTICKS = 10  # <= 40 hard cap


def _atmo_outbox(atmo):
    """Density output message of ExponentialAtmosphere (version-tolerant).

    Tries known spellings in order; returns (msg, attr_name). The 2.12
    build exposes NO singular outbox -- density flows through the
    ``envOutMsgs`` vector (first entry). Raises RuntimeError with a dir()
    dump if none exists (recorded by callers, never raised out).
    """
    for name in ("atmoDensOutMsg", "atmoPropsOutMsg", "envOutMsg", "atmoOutMsg"):
        try:
            msg = getattr(atmo, name)
        except Exception:
            continue
        if msg is not None:
            return msg, name
    try:
        vec = getattr(atmo, "envOutMsgs", None)
        if vec is not None and len(vec) > 0:
            return vec[0], "envOutMsgs[0]"
    except Exception:
        pass
    try:
        names = sorted(a for a in dir(atmo) if not a.startswith("_"))
    except Exception:
        names = []
    raise RuntimeError(
        "ExponentialAtmosphere exposes no known density outbox; dir=" + str(names))
C_OFF = np.array([0.1, 0.0, 0.0])
MU_EARTH = 3.986004418e14
R_EARTH = 6378137.0
LEO_R = R_EARTH + 400.0e3
LEO_V = float(np.sqrt(MU_EARTH / LEO_R))
AU = 1.495978707e11
I_DIAG = np.diag([900.0, 800.0, 600.0])
# Drag decision fixture (ledger-consistent): Cd=2.2, A=1, com=[0.05,0,0].
DRAG_CD, DRAG_A = 2.2, 1.0
DRAG_COM = np.array([0.05, 0.0, 0.0])
# SRP decision facet: sun-facing (+X), off-center arm so torque != 0.
SRP_NHAT = np.array([1.0, 0.0, 0.0])
SRP_RCOP = np.array([0.0, 0.5, 0.0])
SRP_CD, SRP_CS, SRP_AREA = 0.2, 0.3, 1.0


def _log(msg: str) -> None:
    try:
        log_text(PROBE_ID, msg)
    except Exception:
        pass


def _err(tag: str, exc: BaseException) -> dict:
    try:
        p = write_exception(PROBE_ID, exc)
    except Exception:
        p = ""
    return {"status": "error", "error": f"{type(exc).__name__}: {exc}",
            "exc_log": p, "tag": tag}


def _bsk_module(name: str):
    for dotted in (f"Basilisk.simulation.{name}", f"Basilisk.utilities.{name}",
                   f"Basilisk.{name}.{name}", f"Basilisk.{name}"):
        try:
            return importlib.import_module(dotted), ""
        except Exception:
            continue
    try:
        import Basilisk  # noqa
        return None, f"no submodule {name}"
    except Exception as e:
        return None, f"Basilisk unavailable: {type(e).__name__}: {e}"


def _recorders_for(eff, sim) -> dict:
    """Attach a recorder to every *OutMsg(s) attr; return {attr: recorder}."""
    out: dict = {}
    for attr in dir(eff):
        if "OutMsg" not in attr:
            continue
        try:
            msg = getattr(eff, attr)
            if isinstance(msg, (list, tuple)):
                for i, m in enumerate(msg):
                    try:
                        r = m.recorder()
                        sim.AddModelToTask("t", r)
                        out[f"{attr}[{i}]"] = r
                    except Exception:
                        continue
            else:
                try:
                    r = msg.recorder()
                    sim.AddModelToTask("t", r)
                    out[attr] = r
                except Exception:
                    continue
        except Exception:
            continue
    return out


def _col(rec, field: str):
    try:
        return np.asarray(getattr(rec, field), float)
    except Exception:
        return None


def _find_force_torque(recs: dict):
    """Search attached recorders for body force/torque columns."""
    F = T = None
    f_key = t_key = ""
    for key, rec in recs.items():
        for f in ("dragForce_B", "force_B", "srpForce_B", "Force_B",
                  "dragForce", "force"):
            arr = _col(rec, f)
            if arr is not None and arr.size:
                F, f_key = arr, f"{key}.{f}"
                break
        for f in ("dragTorque_B", "torque_B", "srpTorque_B", "Torque_B",
                  "dragTorque", "torque", "mtbNetTorque_B"):
            arr = _col(rec, f)
            if arr is not None and arr.size:
                T, t_key = arr, f"{key}.{f}"
                break
    return (F, f_key), (T, t_key)


def _verdict(dT: np.ndarray, Fref: np.ndarray, c: np.ndarray) -> dict:
    """Stock-direct shift decision for one leg (no theory as input)."""
    try:
        expect = np.array([-np.cross(c, F) for F in Fref])
        num = np.array([float(np.linalg.norm(d - e))
                        for d, e in zip(dT, expect)])
        den = np.array([float(np.linalg.norm(e)) for e in expect])
        den = np.where(den < 1e-30, 1e-30, den)
        rel = num / den
        zero_rel = np.array([float(np.linalg.norm(d)) for d in dT]) / den
        med_shift = float(np.median(rel))
        med_zero = float(np.median(zero_rel))
        if med_shift < 0.05:
            v = "shifted"
        elif med_zero < 0.05:
            v = "unshifted"
        else:
            v = "ambiguous"
        return {"verdict": v, "median_shift_rel": med_shift,
                "median_zero_rel": med_zero,
                "per_tick_shift_rel": [float(x) for x in rel],
                "expect_minus_cxF": [list(map(float, e)) for e in expect]}
    except Exception as e:
        return {"verdict": "error", "error": f"{type(e).__name__}: {e}"}


def _abs_fd(om) -> tuple:
    """Absolute FD torque per tick (I_DIAG * domega/dt) + its max norm.

    Firing proof: a leg whose absolute torque never clears 1e-9 is IDLE
    (mis-wired effector); its delta verdict would be vacuous zeros-vs-zeros.
    """
    try:
        arr = np.asarray(om, float)
        tabs = np.array([(I_DIAG @ (arr[i] - arr[i - 1]) / DT)
                         for i in range(1, len(arr))])
        mx = float(np.max([np.linalg.norm(t) for t in tabs])) if len(tabs) else 0.0
        return tabs, mx
    except Exception:
        return None, 0.0


def _gate_firing(out: dict, tag: str, max_abs: float, verdict: dict) -> dict:
    """Downgrade a verdict to untestable when the effector never fired."""
    out[f"{tag}_abs_max"] = max_abs
    out[f"{tag}_firing"] = bool(max_abs > 1e-9)
    if not out[f"{tag}_firing"]:
        verdict = dict(verdict)
        verdict["verdict"] = "untestable (effector idle)"
        verdict["note"] = ("absolute torque never cleared 1e-9: delta "
                           "verdict would compare zeros to zeros")
    return verdict


# ---------------------------------------------------------------------------
# (a) LEO drag: offset vs no-offset (same CdA/comOffset)
# ---------------------------------------------------------------------------
def leg_drag_offset() -> dict:
    out: dict = {"id": "drag-offset", "dt": DT, "ticks": NTICKS,
                 "hub_offset": list(map(float, C_OFF)),
                 "comOffset": list(map(float, DRAG_COM)),
                 "Cd": DRAG_CD, "A": DRAG_A,
                 "fixture": f"LEO R={LEO_R:.0f} m V={LEO_V:.2f} m/s"}
    try:
        from Basilisk.simulation import spacecraft
        from Basilisk.utilities import (SimulationBaseClass, macros,
                                        orbitalMotion, simIncludeGravBody)
        expMod, e1 = _bsk_module("exponentialAtmosphere")
        dragMod, e2 = _bsk_module("dragDynamicEffector")
        out["modules"] = {"exponentialAtmosphere": expMod is not None,
                          "dragDynamicEffector": dragMod is not None,
                          "err": [e1[:120], e2[:120]]}
        if expMod is None or dragMod is None:
            out["status"] = "guarded-fail"
            out["error"] = "stock drag modules missing on this host"
            return out
        try:
            out["drag_attrs"] = sorted(
                a for a in dir(getattr(dragMod, "DragDynamicEffector", object))
                if not a.startswith("_"))[:60]
        except Exception:
            out["drag_attrs"] = "unreadable"

        def run(hub_off) -> dict:
            sim = SimulationBaseClass.SimBaseClass()
            sim.SetProgressBar(False)
            proc = sim.CreateNewProcess("p")
            proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
            sc = spacecraft.Spacecraft()
            sc.ModelTag = "wp45-drag"
            sc.hub.mHub = 750.0
            sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
            try:
                sc.hub.r_BcB_B = list(map(float, hub_off))
            except Exception as e:
                return {"error": f"r_BcB_B unsettable: {e}"}
            sc.hub.r_CN_NInit = [LEO_R, 0.0, 0.0]
            sc.hub.v_CN_NInit = [0.0, LEO_V, 0.0]
            sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
            sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
            sim.AddModelToTask("t", sc, 1)
            grav = simIncludeGravBody.gravBodyFactory()
            earth = grav.createEarth()
            earth.isCentralBody = True
            grav.addBodiesTo(sc)
            # Atmosphere (record what sticks; attr names vary by build).
            atmo = getattr(expMod, "ExponentialAtmosphere")()
            wiring: dict = {}
            for k, v in (("baseDensity", 2.0e-12), ("scaleHeight", 88000.0),
                         ("planetRadius", R_EARTH)):
                try:
                    if hasattr(atmo, k):
                        setattr(atmo, k, v)
                        wiring[k] = "set"
                    else:
                        wiring[k] = "attr-missing"
                except Exception as e:
                    wiring[k] = f"set-fail: {type(e).__name__}"
            try:
                atmo.addSpacecraftToModel(sc.scStateOutMsg)
                wiring["atmo_sc"] = "addSpacecraftToModel"
            except Exception as e:
                wiring["atmo_sc"] = f"fail: {type(e).__name__}: {str(e)[:80]}"
            sim.AddModelToTask("t", atmo)
            # Drag effector config: BSK 2.12 keeps CdA/comOffset in the
            # `coreParams` struct, NOT on the module (direct attrs silently
            # miss). Discover + set on coreParams first, module direct second.
            drag = getattr(dragMod, "DragDynamicEffector")()
            obs: dict = {}
            core = getattr(drag, "coreParams", None)
            if core is not None:
                for k, v in (("projectedArea", DRAG_A), ("dragCoeff", DRAG_CD),
                             ("dragCoefficient", DRAG_CD),
                             ("comOffset", list(map(float, DRAG_COM))),
                             ("comOffset_B", list(map(float, DRAG_COM)))):
                    try:
                        if hasattr(core, k):
                            setattr(core, k, v)
                            obs["coreParams." + k] = "set"
                    except Exception as e:
                        obs["coreParams." + k] = f"fail: {type(e).__name__}"
            else:
                obs["coreParams"] = "absent"
            for k, v in (("dragCoeff", DRAG_CD), ("dragCoefficient", DRAG_CD),
                         ("area", DRAG_A), ("projArea", DRAG_A),
                         ("projectedArea", DRAG_A),
                         ("comOffset", list(map(float, DRAG_COM))),
                         ("comOffset_B", list(map(float, DRAG_COM)))):
                try:
                    if hasattr(drag, k):
                        setattr(drag, k, v)
                        obs[k] = "set"
                except Exception as e:
                    obs[k] = f"fail: {type(e).__name__}"
            try:
                dens_msg, dens_name = _atmo_outbox(atmo)
                drag.atmoDensInMsg.subscribeTo(dens_msg)
                wiring["dens_link"] = dens_name
            except Exception as e:
                wiring["dens_link"] = f"fail: {type(e).__name__}: {str(e)[:80]}"
            for meth in ("addSpacecraftToModel", "addDynamicEffector"):
                try:
                    if hasattr(drag, meth):
                        getattr(drag, meth)(sc.scStateOutMsg)
                        wiring[f"drag_{meth}"] = "ok"
                    elif meth == "addDynamicEffector" and hasattr(sc, meth):
                        getattr(sc, meth)(drag)
                        wiring[f"sc_{meth}"] = "ok"
                except Exception as e:
                    wiring[f"drag_{meth}"] = f"fail: {type(e).__name__}"
            sim.AddModelToTask("t", drag)
            slog = sc.scStateOutMsg.recorder()
            sim.AddModelToTask("t", slog)
            recs = _recorders_for(drag, sim)
            sim.InitializeSimulation()
            sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS))
            sim.ExecuteSimulation()
            return {"slog": slog, "recs": recs, "wiring": wiring,
                    "obs": obs}

        r_off = run(C_OFF)
        r_no = run(np.zeros(3))
        if "error" in r_off or "error" in r_no:
            out["status"] = "guarded-fail"
            out["error"] = f"off={r_off.get('error')} no={r_no.get('error')}"
            out["wiring"] = {k: v.get("wiring") for k, v in
                             (("off", r_off), ("no", r_no)) if "wiring" in v}
            return out
        out["wiring_off"], out["wiring_no"] = r_off["wiring"], r_no["wiring"]
        out["obs_off"], out["obs_no"] = r_off["obs"], r_no["obs"]
        (F_off, fk_off), (T_off, tk_off) = _find_force_torque(r_off["recs"])
        (F_no, fk_no), (T_no, tk_no) = _find_force_torque(r_no["recs"])
        out["force_keys"] = {"off": fk_off, "no": fk_no}
        out["torque_keys"] = {"off": tk_off, "no": tk_no}
        om_off = np.asarray(r_off["slog"].omega_BN_B, float)
        om_no = np.asarray(r_no["slog"].omega_BN_B, float)
        # Stock-direct torque channel: prefer effector msgs; else FD proxy
        # T_proxy = I * domega/dt (central gravity is torque-free).
        if T_off is not None and T_no is not None and \
                len(T_off) == len(T_no) and len(T_off) >= 3:
            n = min(len(T_off), len(T_no))
            dT, Fref = T_off[:n] - T_no[:n], \
                F_off[:n] if F_off is not None else F_no[:n]
            out["channel"] = f"effector {tk_off}"
        else:
            n = min(len(om_off), len(om_no))
            dT = np.array([(I_DIAG @ (om_off[i] - om_off[i - 1]) / DT)
                           - (I_DIAG @ (om_no[i] - om_no[i - 1]) / DT)
                           for i in range(1, n)])
            Fref = None
            out["channel"] = "FD-proxy I*domega/dt (effector msgs unreadable)"
        if Fref is None:
            # Fall back: reconstruct |F| from the ledger law for scale only;
            # verdict on the FD channel still reported, flagged approximate.
            rho, v = 2.0e-12, LEO_V
            fmag = 0.5 * rho * DRAG_CD * DRAG_A * v ** 2
            Fref = np.tile(np.array([0.0, -fmag, 0.0]), (len(dT), 1))
            out["F_note"] = "analytic scale fallback (msgs missing); delta is stock-direct"
        out["dT_per_tick"] = [list(map(float, d)) for d in dT]
        # Firing proof (absolute, per run): a silent effector makes any
        # delta verdict vacuous. Gravity is torque-free, so FD absolute
        # measures the effector torque scale in both runs.
        _, abs_off = _abs_fd(om_off)
        _, abs_no = _abs_fd(om_no)
        v = _verdict(np.asarray(dT), np.asarray(Fref), C_OFF)
        v = _gate_firing(out, "drag", max(abs_off, abs_no), v)
        out.update(v)
        out["decides_shift_5pct"] = (v.get("verdict") == "shifted")
        out["status"] = "ok"
        _log(f"[drag-offset] verdict={v.get('verdict')} "
             f"shift_rel={v.get('median_shift_rel')}")
    except Exception as e:
        out.update(_err("drag-offset", e))
    return out


# ---------------------------------------------------------------------------
# (b) Faceted SRP: offset vs no-offset (decision facet, observable arm)
# ---------------------------------------------------------------------------
def leg_srp_offset() -> dict:
    out: dict = {"id": "srp-offset", "dt": DT, "ticks": NTICKS,
                 "hub_offset": list(map(float, C_OFF)),
                 "facet": {"nHat_B": list(map(float, SRP_NHAT)),
                           "r_CopB_B": list(map(float, SRP_RCOP)),
                           "area": SRP_AREA, "c_diffuse": SRP_CD,
                           "c_specular": SRP_CS},
                 "sun_N": [AU, 0.0, 0.0],
                 "note": "single sun-facing off-center facet: the symmetric "
                         "6-facet box totals ~0 (ledger) and cannot decide"}
    try:
        from Basilisk.simulation import spacecraft
        from Basilisk.utilities import SimulationBaseClass, macros
        from Basilisk.architecture import messaging
        srpMod, e1 = _bsk_module("facetedSRPEffector")
        out["module"] = {"facetedSRPEffector": srpMod is not None,
                         "err": e1[:120]}
        if srpMod is None:
            out["status"] = "guarded-fail"
            out["error"] = "stock facetedSRPEffector missing on this host"
            return out
        cls = getattr(srpMod, "FacetedSRPEffector", None)
        try:
            out["srp_attrs"] = sorted(a for a in dir(cls) if not a.startswith("_"))[:60]
        except Exception:
            out["srp_attrs"] = "unreadable"

        def run(hub_off) -> dict:
            sim = SimulationBaseClass.SimBaseClass()
            sim.SetProgressBar(False)
            proc = sim.CreateNewProcess("p")
            proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
            sc = spacecraft.Spacecraft()
            sc.ModelTag = "wp45-srp"
            sc.hub.mHub = 750.0
            sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
            try:
                sc.hub.r_BcB_B = list(map(float, hub_off))
            except Exception as e:
                return {"error": f"r_BcB_B unsettable: {e}"}
            # High orbit: gravity torque-free, SRP isolated. Attitude fixed.
            sc.hub.r_CN_NInit = [LEO_R, 0.0, 0.0]
            sc.hub.v_CN_NInit = [0.0, LEO_V, 0.0]
            sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
            sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
            sim.AddModelToTask("t", sc, 1)
            srp = cls()
            wiring: dict = {}
            # Facet add is MESSAGE-driven in BSK 2.12 (method guesses raise
            # "tried to add this variable: numFacets"): publish a
            # FacetElementBodyMsg and subscribe the effector's
            # facetElementBodyInMsgs vector to it. Discover payload fields
            # from the class; record everything that sticks.
            facet_ok = False
            try:
                _fmsg_cls = getattr(messaging, "FacetElementBodyMsg", None)
                _fpay_cls = getattr(messaging, "FacetElementBodyMsgPayload",
                                    None)
                if _fmsg_cls is None or _fpay_cls is None:
                    raise AttributeError(
                        "no FacetElementBodyMsg(Payload) in messaging")
                try:
                    _pay_probe = _fpay_cls()
                    _pay_fields = sorted(
                        a for a in dir(_pay_probe) if not a.startswith("_"))
                except Exception:
                    _pay_fields = []
                wiring["facet_payload_fields"] = _pay_fields[:20]
                _pay = _fpay_cls()
                _stuck = {}
                for _k, _v in (("area", float(SRP_AREA)),
                               ("r_CopB_B", list(map(float, SRP_RCOP))),
                               ("rCop_B", list(map(float, SRP_RCOP))),
                               ("nHat_B", list(map(float, SRP_NHAT))),
                               ("nHatB", list(map(float, SRP_NHAT))),
                               ("c_diffuse", float(SRP_CD)),
                               ("cDiffuse", float(SRP_CD)),
                               ("c_specular", float(SRP_CS)),
                               ("cSpecular", float(SRP_CS))):
                    try:
                        if hasattr(_pay, _k):
                            setattr(_pay, _k, _v)
                            _stuck[_k] = "set"
                    except Exception as _e:
                        _stuck[_k] = f"fail: {type(_e).__name__}"
                wiring["facet_payload_set"] = _stuck
                _fmsg = _fmsg_cls()
                _fmsg.write(_pay)
                _linked = False
                _link_errs = {}
                # Size the InMsgs vector FIRST (setNumFacets exists on 2.12;
                # without it the vector is empty and every link below fails).
                try:
                    if hasattr(srp, "setNumFacets"):
                        getattr(srp, "setNumFacets")(1)
                        wiring["setNumFacets"] = "ok(1)"
                except Exception as _e:
                    wiring["setNumFacets"] = (
                        f"fail: {type(_e).__name__}: {str(_e)[:80]}")
                for _inm in ("facetElementBodyInMsgs", "facetInMsgs",
                             "facetElementBodyInMsg"):
                    try:
                        _port = getattr(srp, _inm, None)
                        if _port is None:
                            _link_errs[_inm] = "missing"
                            continue
                        if hasattr(_port, "pushBack"):
                            _port.pushBack(_fmsg)
                        elif isinstance(_port, list):
                            _port.append(_fmsg)
                        else:
                            _port.subscribeTo(_fmsg)
                        wiring["facet_link"] = _inm
                        _linked = True
                        break
                    except Exception as _e:
                        _link_errs[_inm] = f"{type(_e).__name__}: {str(_e)[:120]}"
                        continue
                if not _linked:
                    wiring["facet_link_errs"] = _link_errs
                    wiring["facet_link"] = (
                        "fail: no linkable facet InMsgs port; "
                        f"srp ports={[a for a in dir(srp) if 'acet' in a][:10]}")
                else:
                    facet_ok = True
                    wiring.setdefault("facet_add", "ok")
            except Exception as e:
                wiring["facet_add_try"] = f"{type(e).__name__}: {str(e)[:120]}"
            # Projected-area chain: the effector may REQUIRE
            # facetProjectedAreaInMsgs (occlusion). Discover the producer
            # module best-effort; record everything (missing => effector may
            # bskError at Initialize, which is itself a ledger datum).
            try:
                _paMod, _paErr = _bsk_module("facetedSpacecraftProjectedArea")
                wiring["pa_module"] = ("found" if _paMod is not None
                                       else f"missing: {_paErr[:80]}")
                if _paMod is not None:
                    _paCls = None
                    for _a in dir(_paMod):
                        if "roject" in _a and not _a.startswith("_"):
                            _o = getattr(_paMod, _a)
                            if isinstance(_o, type):
                                _paCls = _o
                                break
                    wiring["pa_class"] = (getattr(_paCls, "__name__", None)
                                          or "none-found")
                    if _paCls is not None:
                        _pa = _paCls()
                        # Heading source: the module raises "No valid heading
                        # source configured" at Initialize unless it can see a
                        # planet. Wire the discovered scState port + a static
                        # planet-position message (Earth-centered heading
                        # frame for this LEO fixture).
                        for _m in ("addSpacecraftToModel",):
                            try:
                                if hasattr(_pa, _m):
                                    getattr(_pa, _m)(sc.scStateOutMsg)
                                    wiring[f"pa_{_m}"] = "ok"
                            except Exception as _e:
                                wiring[f"pa_{_m}"] = (
                                    f"fail: {type(_e).__name__}")
                        try:
                            # Port census FIRST (unconditional): the heading
                            # error is thrown at Initialize, so a guess loop
                            # that fails tells us nothing. Dump every
                            # Msg-named port of the projected-area module.
                            wiring["pa_ports"] = [
                                a for a in dir(_pa) if "Msg" in a]
                        except Exception:
                            wiring["pa_ports"] = "unreadable"
                        try:
                            _pl = messaging.SpicePlanetStateMsgPayload()
                            _pl.PositionVector = [0.0, 0.0, 0.0]
                            _pl.VelocityVector = [0.0, 0.0, 0.0]
                            _plmsg = messaging.SpicePlanetStateMsg()
                            _plmsg.write(_pl)
                            for _pin in ("planetPosInMsg", "centralBodyInMsg",
                                         "planetInMsg", "centralBodyPosInMsg"):
                                try:
                                    _p = getattr(_pa, _pin, None)
                                    if _p is None:
                                        continue
                                    _p.subscribeTo(_plmsg)
                                    wiring["pa_heading"] = _pin
                                    break
                                except Exception:
                                    continue
                            if "pa_heading" not in wiring:
                                wiring["pa_heading"] = (
                                    "unwired; candidate ports="
                                    + str([a for a in dir(_pa)
                                           if "lanet" in a or "ody" in a][:8]))
                        except Exception as _e:
                            wiring["pa_heading"] = (
                                f"fail: {type(_e).__name__}: {str(_e)[:80]}")
                        sim.AddModelToTask("t", _pa)
                        _pa_out = None
                        for _a in dir(_pa):
                            if "utMsg" in _a and ("roject" in _a or "rea" in _a):
                                try:
                                    _pa_out = getattr(_pa, _a)
                                    wiring["pa_outmsg"] = _a
                                    break
                                except Exception:
                                    continue
                        if _pa_out is not None:
                            try:
                                _dst = getattr(
                                    srp, "facetProjectedAreaInMsgs", None)
                                if _dst is not None:
                                    if hasattr(_dst, "pushBack"):
                                        _dst.pushBack(_pa_out)
                                    elif isinstance(_dst, list):
                                        _dst.append(_pa_out)
                                    else:
                                        _dst.subscribeTo(_pa_out)
                                    wiring["pa_link"] = "facetProjectedAreaInMsgs"
                                else:
                                    wiring["pa_link"] = "fail: dst missing"
                            except Exception as _e:
                                wiring["pa_link"] = (
                                    f"fail: {type(_e).__name__}: "
                                    f"{str(_e)[:80]}")
                        else:
                            wiring["pa_outmsg"] = "none-found"
            except Exception as e:
                wiring["pa_chain"] = f"fail: {type(e).__name__}: {str(e)[:80]}"
            # Static sun state (full position for 1/r^2 scaling).
            try:
                sunMsg = messaging.SpicePlanetStateMsg()
                p = messaging.SpicePlanetStateMsgPayload()
                p.PositionVector = [AU, 0.0, 0.0]
                p.VelocityVector = [0.0, 0.0, 0.0]
                sunMsg.write(p)
                srp.sunStateInMsg.subscribeTo(sunMsg)
                wiring["sun_link"] = "static-SpicePlanetStateMsg"
            except Exception as e:
                wiring["sun_link"] = f"fail: {type(e).__name__}: {str(e)[:80]}"
            # No eclipse link -> stock defaults full-sun 1.0 (P1 truth table).
            wiring["eclipse"] = "unlinked->1.0 (full sun ON)"
            for meth in ("addSpacecraftToModel",):
                try:
                    if hasattr(srp, meth):
                        getattr(srp, meth)(sc.scStateOutMsg)
                        wiring[f"srp_{meth}"] = "ok"
                except Exception as e:
                    wiring[f"srp_{meth}"] = f"fail: {type(e).__name__}"
            sim.AddModelToTask("t", srp)
            slog = sc.scStateOutMsg.recorder()
            sim.AddModelToTask("t", slog)
            recs = _recorders_for(srp, sim)
            sim.InitializeSimulation()
            sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS))
            sim.ExecuteSimulation()
            return {"slog": slog, "recs": recs, "wiring": wiring,
                    "facet_ok": facet_ok}

        r_off = run(C_OFF)
        r_no = run(np.zeros(3))
        # Wiring first, ALWAYS (an Initialize-time bskError must not hide
        # the wiring state -- that's the ledger datum the verdict needs).
        for _k, _r in (("wiring_off", r_off), ("wiring_no", r_no)):
            if "wiring" in _r:
                out[_k] = _r["wiring"]
        if "error" in r_off or "error" in r_no:
            out["status"] = "guarded-fail"
            out["error"] = f"off={r_off.get('error')} no={r_no.get('error')}"
            return out
        (F_off, fk_off), (T_off, tk_off) = _find_force_torque(r_off["recs"])
        (F_no, fk_no), (T_no, tk_no) = _find_force_torque(r_no["recs"])
        out["force_keys"] = {"off": fk_off, "no": fk_no}
        out["torque_keys"] = {"off": tk_off, "no": tk_no}
        om_off = np.asarray(r_off["slog"].omega_BN_B, float)
        om_no = np.asarray(r_no["slog"].omega_BN_B, float)
        if T_off is not None and T_no is not None and \
                len(T_off) == len(T_no) and len(T_off) >= 3:
            n = min(len(T_off), len(T_no))
            dT, Fref = T_off[:n] - T_no[:n], \
                F_off[:n] if F_off is not None else F_no[:n]
            out["channel"] = f"effector {tk_off}"
        else:
            n = min(len(om_off), len(om_no))
            dT = np.array([(I_DIAG @ (om_off[i] - om_off[i - 1]) / DT)
                           - (I_DIAG @ (om_no[i] - om_no[i - 1]) / DT)
                           for i in range(1, n)])
            Fref = None
            out["channel"] = "FD-proxy I*domega/dt (effector msgs unreadable)"
        if Fref is None:
            P = 4.5398178907e-6
            Fref = np.tile(np.array([-P * SRP_AREA * 1.6, 0.0, 0.0]),
                           (len(dT), 1))
            out["F_note"] = "cannonball-scale fallback (msgs missing)"
        out["dT_per_tick"] = [list(map(float, d)) for d in dT]
        _, abs_off = _abs_fd(om_off)
        _, abs_no = _abs_fd(om_no)
        v = _verdict(np.asarray(dT), np.asarray(Fref), C_OFF)
        v = _gate_firing(out, "srp", max(abs_off, abs_no), v)
        out.update(v)
        out["decides_shift_5pct"] = (v.get("verdict") == "shifted")
        out["status"] = "ok"
        _log(f"[srp-offset] verdict={v.get('verdict')} "
             f"shift_rel={v.get('median_shift_rel')}")
    except Exception as e:
        out.update(_err("srp-offset", e))
    return out


# ---------------------------------------------------------------------------
# (c) comOffset=0 with c != 0: ~0 or ~-c x F?
# ---------------------------------------------------------------------------
def leg_drag_zero_com() -> dict:
    out: dict = {"id": "drag-zero-com", "dt": DT, "ticks": NTICKS,
                 "hub_offset": list(map(float, C_OFF)),
                 "comOffset": [0.0, 0.0, 0.0],
                 "question": "stock drag torque with comOffset=0 and c!=0: "
                             "~0 (unshifted) or ~-c x F (shifted)?"}
    try:
        from Basilisk.simulation import spacecraft
        from Basilisk.utilities import (SimulationBaseClass, macros,
                                        simIncludeGravBody)
        expMod, _ = _bsk_module("exponentialAtmosphere")
        dragMod, _ = _bsk_module("dragDynamicEffector")
        if expMod is None or dragMod is None:
            out["status"] = "guarded-fail"
            out["error"] = "stock drag modules missing on this host"
            return out
        sim = SimulationBaseClass.SimBaseClass()
        sim.SetProgressBar(False)
        proc = sim.CreateNewProcess("p")
        proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
        sc = spacecraft.Spacecraft()
        sc.ModelTag = "wp45-drag0"
        sc.hub.mHub = 750.0
        sc.hub.IHubPntBc_B = [[900.0, 0, 0], [0, 800.0, 0], [0, 0, 600.0]]
        sc.hub.r_BcB_B = list(map(float, C_OFF))
        sc.hub.r_CN_NInit = [LEO_R, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, LEO_V, 0.0]
        sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
        sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        sim.AddModelToTask("t", sc, 1)
        grav = simIncludeGravBody.gravBodyFactory()
        earth = grav.createEarth()
        earth.isCentralBody = True
        grav.addBodiesTo(sc)
        atmo = getattr(expMod, "ExponentialAtmosphere")()
        try:
            if hasattr(atmo, "baseDensity"):
                atmo.baseDensity = 2.0e-12
        except Exception:
            pass
        try:
            atmo.addSpacecraftToModel(sc.scStateOutMsg)
        except Exception:
            pass
        sim.AddModelToTask("t", atmo)
        drag = getattr(dragMod, "DragDynamicEffector")()
        out["coreParams"] = "absent"
        core = getattr(drag, "coreParams", None)
        if core is not None:
            for k, v in (("projectedArea", DRAG_A), ("dragCoeff", DRAG_CD),
                         ("dragCoefficient", DRAG_CD),
                         ("comOffset", [0.0, 0.0, 0.0]),
                         ("comOffset_B", [0.0, 0.0, 0.0])):
                try:
                    if hasattr(core, k):
                        setattr(core, k, v)
                        out["coreParams"] = "set:" + k
                except Exception:
                    pass
        for k, v in (("dragCoeff", DRAG_CD), ("area", DRAG_A),
                     ("comOffset", [0.0, 0.0, 0.0]),
                     ("comOffset_B", [0.0, 0.0, 0.0])):
            try:
                if hasattr(drag, k):
                    setattr(drag, k, v)
            except Exception:
                pass
        try:
            dens_msg, dens_name = _atmo_outbox(atmo)
            drag.atmoDensInMsg.subscribeTo(dens_msg)
            out["dens_link"] = dens_name
        except Exception as e:
            out["dens_link"] = f"fail: {type(e).__name__}"
        try:
            if hasattr(drag, "addSpacecraftToModel"):
                drag.addSpacecraftToModel(sc.scStateOutMsg)
        except Exception:
            pass
        sim.AddModelToTask("t", drag)
        slog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", slog)
        recs = _recorders_for(drag, sim)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(DT * NTICKS))
        sim.ExecuteSimulation()
        (_, _), (T, tk) = _find_force_torque(recs)
        om = np.asarray(slog.omega_BN_B, float)
        if T is not None and len(T) >= 2:
            mags = [float(np.linalg.norm(t)) for t in T]
            out["channel"] = f"effector {tk}"
        else:
            T = np.array([I_DIAG @ (om[i] - om[i - 1]) / DT
                          for i in range(1, len(om))])
            mags = [float(np.linalg.norm(t)) for t in T]
            out["channel"] = "FD-proxy I*domega/dt"
        rho, v = 2.0e-12, LEO_V
        fmag = 0.5 * rho * DRAG_CD * DRAG_A * v ** 2
        scale = float(np.linalg.norm(np.cross(C_OFF, [0.0, -fmag, 0.0])))
        med = float(np.median(mags)) if len(mags) else float("nan")
        out["torque_norm_per_tick"] = mags
        out["minus_cxF_scale"] = scale
        out["ratio_med_over_scale"] = (med / scale) if scale > 0 else None
        # Firing gate: an idle effector reads ~0 and would fake "unshifted".
        out["firing"] = bool(med > 1e-9)
        if not out["firing"]:
            out["verdict"] = "untestable (effector idle)"
        elif med < 0.05 * scale:
            out["verdict"] = "unshifted (~0)"
        elif abs(med - scale) / scale < 0.05:
            out["verdict"] = "shifted (~-c x F)"
        else:
            out["verdict"] = "ambiguous"
        out["status"] = "ok"
        _log(f"[drag-zero-com] verdict={out['verdict']} ratio={out['ratio_med_over_scale']}")
    except Exception as e:
        out.update(_err("drag-zero-com", e))
    return out


def main() -> int:
    payload: dict = {}
    try:
        try:
            payload["env"] = env_info()
        except Exception:
            payload["env"] = {"env_info": "unavailable"}
        payload.update(probe="wp45_combo", dt=DT, ticks_cap=40,
                       hub_offset=list(map(float, C_OFF)),
                       open_question="do stock drag/SRP torque arms shift by "
                       "-c_B like (r-c)xF, or stay unshifted? decided below "
                       "from stock-direct offset-vs-no-offset comparison")
        _log("LIVE drag-offset")
        try:
            payload["drag_offset"] = leg_drag_offset()
        except Exception as e:
            payload["drag_offset"] = _err("drag-offset", e)
        _log("LIVE srp-offset")
        try:
            payload["srp_offset"] = leg_srp_offset()
        except Exception as e:
            payload["srp_offset"] = _err("srp-offset", e)
        _log("LIVE drag-zero-com")
        try:
            payload["drag_zero_com"] = leg_drag_zero_com()
        except Exception as e:
            payload["drag_zero_com"] = _err("drag-zero-com", e)
        payload["magnetic_note"] = ("tau = m x B_B has NO moment arm: hub "
            "offset cannot shift it (WP5); gravity likewise (no arm). Only "
            "drag comOffset and SRP r_Cop arms are in question.")
        # M21/M22 in-JSON void discipline: the 2026-10-12 stock-direct
        # verdicts below are VOID (retained for archaeology only -- quote
        # the marker, never the verdict words at face value).
        payload["superseded"] = {
            "status": "superseded",
            "pointer": ("tests/test_wp45_env_offset.py::"
                        "test_stock_drag_arm_rule_rbcb_minus_com"),
            "void_reason": ("combo drag_offset '|dT| ~ 1e-17 => unshifted' "
                            "used a 100x-wrong density scale (flat rho=2e-12 "
                            "vs actual falloff rho~=2e-14 at 400 km / "
                            "scaleHeight 88000); combo drag_zero_com (~0 "
                            "torque) was an idle leg from an unwired "
                            "effector, refuted by a live com=0 run"),
            "applies_to": ["drag_offset", "drag_zero_com"],
            "rule": ("stock drag torque is T = F x (rBcB - comOffset); "
                     "see the pointer test for the 5-config firing-proven rows"),
        }
        save_result(PROBE_ID, payload)
        _log("done wp45_combo")
        return 0
    except Exception as e:
        try:
            payload["top_error"] = _err("main", e)
            save_result(PROBE_ID, payload)
        except Exception:
            pass
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
