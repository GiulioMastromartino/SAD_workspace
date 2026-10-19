"""Phase-3 Script-3 evidence probe: stock stationkeeping sim first host run.

Drives ``cubesat_12u_sim/missions/run_stationkeeping_stock.py`` unmodified
(``build_sim`` + the stock run block; the script's own ``main`` only adds a
matplotlib plot) and records the first-run evidence list.md §B3 names:

  * duty taper   -- the staircase-RCS OnTimeRequest profile per control tick
                    (mean/max duty, firing-tick count, prograde/retrograde
                    slot split, and the first-vs-last taper of the duty);
  * depletion    -- single ``FuelTank`` bookkeeping (prop0, burned, remaining,
                    and the burn-rate implied by the tank);
  * radius recovery -- r = |r_BN_N| - R_ORBIT over the leg (max |err|, rms,
                    final, and the injected 0.5 % dV sag it recovers from).

The probe keeps a compact run history (``run_log``) so a fixing re-run does
not erase the first-run failure evidence.

Run:  PYTHONPATH=audit_bundle:probes:../.. ../../env/bin/python probes/stationkeeping_stock.py
Writes: audit_bundle/results/stationkeeping_stock.json + .log
Host-only (Basilisk 2.12.0; the sim is stock, so no wheel needed).
NOT in the run_all.sh glob: explicit Phase-3 script evidence, re-run on
demand only.
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
            / "missions" / "run_stationkeeping_stock.py")

T_END = 7200.0
DV_FRAC = -0.005
PROP_KG = 2.0


def _prior_log() -> list:
    p = Path(__file__).resolve().parents[1] / "results" / "stationkeeping_stock.json"
    if not p.exists():
        return []
    try:
        return list(json.loads(p.read_text(encoding="utf-8"))
                    .get("payload", {}).get("run_log", []))
    except Exception:
        return []


def _run():
    """One host run of the stock stationkeeping sim. Returns the cell dict."""
    import numpy as np
    from Basilisk.utilities import macros
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.missions import run_stationkeeping_stock as sk

    sim, sc, h = sk.build_sim(dv_frac=DV_FRAC, prop_kg=PROP_KG,
                              tag="stock-sk-audit")
    # Loop recorder on the already-wired driver message (same .recorder() +
    # AddModelToTask pattern the audit-bundle loop probes use).
    ot_log = h["drv"].out.recorder()
    sim.AddModelToTask("t", ot_log)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T_END))
    sim.ExecuteSimulation()

    log = h["sc_log"]
    r = np.asarray(log.r_BN_N, float)
    t = np.asarray(log.times(), float) * 1e-9
    rerr = np.linalg.norm(r, axis=1) - float(C.R_ORBIT)
    fuel = float(np.asarray(
        sc.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
    fuel_reread = float(np.asarray(
        sc.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
    prop0 = float(h["prop0"])
    burned = prop0 - fuel

    ot = np.asarray(ot_log.OnTimeRequest, float)          # (n, 36)
    dt_ctrl = float(C.DT_CONTROL)
    duty = {s: ot[:, s] / dt_ctrl for s in (0, 1)}        # commanded duty/slot
    tot = duty[0] + duty[1]
    fire = tot > 0.0
    n_fire = int(fire.sum())
    # Taper: first vs last decile of the firing control ticks, plus the
    # saturation fraction (the P-law clips at a_max -> duty 1.0).
    taper = None
    idx = np.flatnonzero(fire)
    if idx.size:
        k = max(1, idx.size // 10)
        first = float(tot[idx[:k]].mean())
        last = float(tot[idx[-k:]].mean())
        taper = {"first_decile_duty": first, "last_decile_duty": last,
                 "taper_ratio": (last / first) if first > 0 else None,
                 "first_fire_tick": int(idx[0]), "last_fire_tick": int(idx[-1])}
    # Uncontrolled analytic reference: the injected dV error leaves the bus on
    # an ellipse with apoapsis at R_ORBIT, so the un-controlled sag is
    # 2*(R_ORBIT - a0). "Radius recovery" is measured against THIS, not
    # against the (exactly on-target) t=0 state.
    v_c = float(np.sqrt(C.MU_EARTH / float(C.R_ORBIT)))
    v0 = v_c * (1.0 + float(DV_FRAC))
    energy = 0.5 * v0 ** 2 - C.MU_EARTH / float(C.R_ORBIT)
    a0 = -C.MU_EARTH / (2.0 * energy)
    sag_uncontrolled = 2.0 * (float(C.R_ORBIT) - a0)
    return {
        "script": str(REPO_SIM),
        "t_end_s": T_END,
        "dv_frac": DV_FRAC,
        "prop0_kg": prop0,
        "fuel_remaining_kg": fuel,
        "fuel_burned_kg": burned,
        "burn_fraction": burned / prop0 if prop0 else 0.0,
        "depleted": bool(fuel <= 0.0),
        "control_hz": 1.0 / dt_ctrl,
        "control_ticks_recorded": int(ot.shape[0]),
        "duty_max": float(tot.max()),
        "duty_mean_firing": float(tot[fire].mean()) if n_fire else 0.0,
        "duty_saturated_fraction": float((tot > 0.999).mean()),
        "firing_ticks": n_fire,
        "firing_fraction": float(fire.mean()),
        "prograde_ticks": int((duty[0] > 0).sum()),
        "retrograde_ticks": int((duty[1] > 0).sum()),
        "duty_taper": taper,
        "duty_trace": [round(float(v), 6) for v in tot[::max(1, n_fire // 60)]],
        "r_err_initial_m": float(rerr[0]),
        "r_err_final_m": float(rerr[-1]),
        "r_err_max_abs_m": float(np.abs(rerr).max()),
        "r_err_rms_m": float(np.sqrt((rerr ** 2).mean())),
        "r_err_sag_uncontrolled_m": float(sag_uncontrolled),
        "r_err_recovery_vs_sag": (float(np.abs(rerr).max()) / sag_uncontrolled
                                  if sag_uncontrolled else None),
        "r_err_sag_closed_frac": (float((np.abs(rerr).min() - abs(rerr[-1]))
                                    / np.abs(rerr).min())
                                   if np.abs(rerr).min() > 0 else None),
        "r_orbit_m": float(C.R_ORBIT),
        "mass_kg": float(C.MASS),
        "tmax_jet_N": float(C.TMAX_JET),
        "isp_jet_s": float(C.ISP_JET),
        "burned_kg_per_N_s": (burned / (t[-1] - t[0]) * float(C.ISP_JET * 9.80665)
                              / float(C.TMAX_JET)) if t.size > 1 else None,
        "fuel_state_object_live": bool(fuel_reread == fuel),
        "stock_only": not any(
            l.strip().startswith(("import omniheyoka", "from omniheyoka"))
            for l in REPO_SIM.read_text(encoding="utf-8").splitlines()),
        "thruster_task_priority": ("2 (drives after the driver)"
                                   if 'AddModelToTask("t", thr, 2)'
                                   in REPO_SIM.read_text(encoding="utf-8")
                                   else "default (-1, before the driver)"),
    }


def main() -> int:
    if not safe_import_basilisk():
        save_result("stationkeeping_stock",
                    {"status": "skipped-no-basilisk",
                     "reason": "Basilisk not importable"})
        print("stationkeeping_stock: skipped (no Basilisk)")
        return 0
    run_log = _prior_log()
    payload: dict = {"status": "ok", "verdict": "stock stationkeeping run green",
                     "run_log": run_log}
    try:
        cell = _run()
        payload["cells"] = {"stationkeeping_stock": cell}
        fails = []
        if cell["firing_ticks"] <= 0:
            fails.append("RCS never fired (vacuous)")
        if cell["fuel_burned_kg"] <= 0.0:
            fails.append(f"no depletion: burned={cell['fuel_burned_kg']:.3e} kg")
        if cell["fuel_remaining_kg"] < 0.0:
            fails.append(f"negative fuel: {cell['fuel_remaining_kg']:.4f} kg")
        # Radius recovery is measured against the injected-error sag, NOT the
        # on-target t=0 state (the IC sits exactly at R_ORBIT, the apoapsis of
        # the sagging ellipse, so a t=0-relative metric is degenerate).
        if cell["r_err_recovery_vs_sag"] is None:
            fails.append("no uncontrolled sag reference")
        elif cell["r_err_recovery_vs_sag"] >= 1.0:
            fails.append(f"no radius recovery: max|err|="
                         f"{cell['r_err_max_abs_m']:.1f} m >= uncontrolled sag "
                         f"{cell['r_err_sag_uncontrolled_m']:.1f} m")
        if not cell["duty_taper"]:
            fails.append("no duty taper recorded")
        payload["fails"] = fails
        if fails:
            payload["status"] = "fail"
            payload["verdict"] = f"STOCK RUN FAIL: {fails}"
    except Exception as exc:
        import traceback as _tb
        tb = _tb.format_exc()
        payload = {"status": "fail", "error": f"{type(exc).__name__}: {exc}",
                   "traceback": tb, "run_log": run_log,
                   "verdict": f"STOCK RUN RAISED: {type(exc).__name__}: {exc}"}
        (Path(__file__).resolve().parents[1] / "results"
         / "stationkeeping_stock.exception.log").write_text(tb, encoding="utf-8")

    run_log = run_log + [{
        "run": len(run_log) + 1,
        "status": payload["status"],
        "verdict": payload.get("verdict"),
        "error": payload.get("error"),
        "firing_ticks": payload.get("cells", {}).get(
            "stationkeeping_stock", {}).get("firing_ticks"),
        "fuel_burned_kg": payload.get("cells", {}).get(
            "stationkeeping_stock", {}).get("fuel_burned_kg"),
        "r_err_final_m": payload.get("cells", {}).get(
            "stationkeeping_stock", {}).get("r_err_final_m"),
    }]
    payload["run_log"] = run_log
    out = save_result("stationkeeping_stock", payload)
    if payload["status"] == "ok":
        c = payload["cells"]["stationkeeping_stock"]
        log_text("stationkeeping_stock",
                 f"[script3] t_end={c['t_end_s']:.0f}s ctrl@{c['control_hz']:.2f}Hz "
                 f"ticks={c['control_ticks_recorded']} firing={c['firing_ticks']} "
                 f"duty_max={c['duty_max']:.4f} duty_mean={c['duty_mean_firing']:.4f} "
                 f"prograde/retro={c['prograde_ticks']}/{c['retrograde_ticks']} "
                 f"taper={c['duty_taper']}")
        log_text("stationkeeping_stock",
                 f"[script3] prop0={c['prop0_kg']:.4f}kg burned={c['fuel_burned_kg']:.6f}kg "
                 f"remaining={c['fuel_remaining_kg']:.6f}kg frac={c['burn_fraction']:.4f} "
                 f"depleted={c['depleted']}")
        log_text("stationkeeping_stock",
                 f"[script3] r_err initial={c['r_err_initial_m']:.1f}m "
                 f"final={c['r_err_final_m']:.1f}m max|.|={c['r_err_max_abs_m']:.1f}m "
                 f"rms={c['r_err_rms_m']:.1f}m recovery_vs_sag={c['r_err_recovery_vs_sag']} "
                 f"sag_closed_frac={c['r_err_sag_closed_frac']} "
                 f"(R_orbit={c['r_orbit_m']/1e3:.1f}km, stock_only={c['stock_only']})")
    else:
        log_text("stationkeeping_stock",
                 f"[script3] FAILED {payload['verdict']}")
    print(f"stationkeeping_stock: {payload['verdict']} -> {out}")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
