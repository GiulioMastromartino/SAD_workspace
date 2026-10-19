"""Phase-3 Script-2 evidence probe: stock LEO-drag sim first host run.

Drives ``cubesat_12u_sim/backends/basilisk/leo_drag_sim.py`` (stock-only,
``run_simulation`` surface) unmodified and records the first-run evidence
list.md §B2 names:

  * firing gate value -- recorded ``neutralDensity`` max vs the script's
    ``DENSITY_FIRING_GATE`` (1e-13 kg/m^3 vacuity guard, envelope L1);
  * decay magnitude -- ``radius[0] - radius[-1]`` over the fixture leg
    (sign convention: > 0 == orbit decayed);
  * wind-link composition -- which links the script actually establishes
    (atmo ``envOutMsgs[-1]`` -> ``drag.atmoDensInMsg``; wind
    ``envOutMsgs[-1]`` -> ``drag.windVelInMsg``; the WindBase-side
    ``planetPosInMsg`` / ``epochInMsg`` and the ``v_air_N`` the corotation
    baseline actually produces).

The probe keeps a compact run history (``run_log``) so a fixing re-run does
not erase the first-run failure evidence.

Run:  PYTHONPATH=audit_bundle:probes:../.. ../../env/bin/python probes/leo_drag.py
Writes: audit_bundle/results/leo_drag.json + .log
Host-only (Basilisk 2.12.0; the sim is stock, so no wheel needed).
NOT in the run_all.sh glob: this is explicit Phase-3 script evidence, re-run
on demand only.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # Python/ for
# the sibling cubesat_12u_sim package (run_all.sh exports the same path).
from common import log_text, save_result, safe_import_basilisk  # noqa: E402

REPO_SIM = (Path(__file__).resolve().parents[3] / "cubesat_12u_sim"
            / "backends" / "basilisk" / "leo_drag_sim.py")


def _load_prior_log() -> list:
    """Compact history from an existing ledger (never grows unbounded)."""
    p = Path(__file__).resolve().parents[1] / "results" / "leo_drag.json"
    if not p.exists():
        return []
    try:
        doc = json.loads(p.read_text(encoding="utf-8"))
        return list(doc.get("payload", {}).get("run_log", []))
    except Exception:
        return []


def _run():
    """One host run of the stock sim. Returns the cell dict (or raises)."""
    import numpy as np  # noqa: F401
    from cubesat_12u_sim.backends.basilisk import leo_drag_sim as sim_mod

    res = sim_mod.run_simulation()
    r = np.asarray(res["truth_position"], float)
    rn = np.linalg.norm(r, axis=1)
    t = np.asarray(res["time"], float)
    v = np.asarray(res["truth_velocity"], float)
    # Wind-link composition: what the script actually established.
    links = {}
    try:
        import cubesat_12u_sim  # noqa: F401
        src = REPO_SIM.read_text(encoding="utf-8")
        links = {
            "drag_atmoDensInMsg_subscribeTo_atmo_envOutMsgs":
                "drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])" in src,
            "drag_windVelInMsg_subscribeTo_wind_envOutMsgs":
                "drag.windVelInMsg.subscribeTo(wind.envOutMsgs[-1])" in src,
            "wind_planetPosInMsg_linked":
                "wind.planetPosInMsg" in src,
            "wind_epochInMsg_linked": "wind.epochInMsg" in src,
            "wind_setUseSpiceOmegaFlag_False":
                "wind.setUseSpiceOmegaFlag(False)" in src,
            "wind_setPlanetOmega_N_called": "wind.setPlanetOmega_N" in src,
            "zero_wind_model": "ZeroWindModel" in src,
            "cannonball_drag": "drag.cannonballDrag()" in src,
        }
    except Exception as exc:  # never let introspection fail the ledger
        links = {"introspection_error": f"{type(exc).__name__}: {exc}"}

    # Corotation physics: |v_air| = |omega_E x r| at the fixture radius.
    r_leo = float(rn[0])
    v_coro = 7.2921159e-5 * r_leo  # sidereal rate, same constant as wp5_env
    return {
        "script": str(REPO_SIM),
        "samples": int(t.shape[0]),
        "t_end_s": float(t[-1]) if t.size else 0.0,
        "altitude_km": float(res["altitude"]) / 1e3,
        "density_max": float(res["density_max"]),
        "firing_gate": float(sim_mod.DENSITY_FIRING_GATE),
        "firing_gate_pass": bool(res["density_max"] > sim_mod.DENSITY_FIRING_GATE),
        "radius0_m": float(rn[0]),
        "radiusF_m": float(rn[-1]),
        "decay_m": float(rn[0] - rn[-1]),
        "decay_sign": "decayed (rho>0)" if rn[-1] < rn[0] else "grew",
        "altitude_decay_m": float(rn[0] - rn[-1]),
        "speed0_ms": float(np.linalg.norm(v[0])),
        "speedF_ms": float(np.linalg.norm(v[-1])),
        "dv_loss_ms": float(np.linalg.norm(v[0]) - np.linalg.norm(v[-1])),
        "corotation_airspeed_ms": v_coro,
        "projected_area_m2": float(res["projected_area"]),
        "drag_coeff": float(res["drag_coeff"]),
        "wind_links": links,
        "task_step_s": 1.0,
        "stock_only": not any(
            l.strip().startswith(("import omniheyoka", "from omniheyoka"))
            for l in REPO_SIM.read_text(encoding="utf-8").splitlines()),
    }


def main() -> int:
    import traceback as _tb

    if not safe_import_basilisk():
        save_result("leo_drag", {"status": "skipped-no-basilisk",
                                 "reason": "Basilisk not importable"})
        print("leo_drag: skipped (no Basilisk)")
        return 0
    run_log = _load_prior_log()
    try:
        cell = _run()
        payload: dict = {"status": "ok", "verdict": "stock LEO-drag run green",
                         "cells": {"leo_drag": cell}, "run_log": run_log}
        fails = []
        if not cell["firing_gate_pass"]:
            fails.append(f"density_max={cell['density_max']:.3e} <= gate "
                         f"{cell['firing_gate']:.0e} (vacuous drag)")
        if cell["decay_m"] <= 0.0:
            fails.append(f"no decay: {cell['decay_m']:.3f} m")
        payload["fails"] = fails
        if fails:
            payload["status"] = "fail"
            payload["verdict"] = f"STOCK RUN FAIL: {fails}"
    except Exception as exc:
        tb = _tb.format_exc()
        payload = {"status": "fail", "error": f"{type(exc).__name__}: {exc}",
                   "traceback": tb, "run_log": run_log,
                   "verdict": f"STOCK RUN RAISED: {type(exc).__name__}: {exc}"}
        Path(__file__).resolve().parents[1].joinpath(
            "results", "leo_drag.exception.log").write_text(tb, encoding="utf-8")

    run_log = run_log + [{
        "run": len(run_log) + 1,
        "status": payload["status"],
        "verdict": payload.get("verdict"),
        "error": payload.get("error"),
        "density_max": payload.get("cells", {}).get("leo_drag", {}).get("density_max"),
        "decay_m": payload.get("cells", {}).get("leo_drag", {}).get("decay_m"),
    }]
    payload["run_log"] = run_log
    out = save_result("leo_drag", payload)
    if payload["status"] == "ok":
        c = payload["cells"]["leo_drag"]
        log_text("leo_drag",
                 f"[script2] samples={c['samples']} t_end={c['t_end_s']:.1f}s "
                 f"alt={c['altitude_km']:.1f}km "
                 f"density_max={c['density_max']:.3e} (gate>{c['firing_gate']:.0e}) "
                 f"radius0={c['radius0_m']:.3f}m radiusF={c['radiusF_m']:.3f}m "
                 f"decay={c['decay_m']:.3f}m dv_loss={c['dv_loss_ms']:.3e}m/s "
                 f"stock_only={c['stock_only']}")
        log_text("leo_drag",
                 "[script2] wind-links " + " ".join(
                     f"{k}={v}" for k, v in c["wind_links"].items()))
    else:
        log_text("leo_drag", f"[script2] FAILED {payload['verdict']}")
        log_text("leo_drag", f"[script2] {payload.get('error')}")
    print(f"leo_drag: {payload['verdict']} -> {out}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
