"""Tolerance-aligned stock baseline: what *should* the omni tol=1e-12 be compared against?

``bench_stock_vs_omni.py`` compares the omni arm (``OmniVecIntegrator``, Taylor
method, tol 1e-12) against the stock arm's *default* integrator -- whatever
``Spacecraft()`` installs, which is a fixed-step ``svIntegratorRK4``
(``swap_p0_ordering.py`` pins that; nothing in the bench sets
``integratorType``).  That is a matched-*step* comparison (same dt, same output
grid, same horizon), not a matched-*tolerance* one, and the two are not the
same claim: RK4 carries no error control, while the omni step is built to a
requested tolerance.

Stock Basilisk 2.12 ships error-controlled alternatives in
``Basilisk.simulation.svIntegrators``::

    svIntegratorRKF45         adaptive RK6    setRelativeTolerance/setAbsoluteTolerance
    svIntegratorRKF78         adaptive RK13   same API
    svIntegratorAdaptiveRungeKutta{1,2,3,4,6,7,9,13}
    svIntegratorRungeKutta(dyn, a, b, c)      free fixed-step Butcher table

and Basilisk's own demos use them where accuracy matters
(``scenarioAttitudeFeedback2T_stateEffTH`` -> RKF78, ``scenarioOrbitManeuverTH``
-> RKF45).  Both of those demos are in the mirror's reject set, so this bench
never compares against them.

Three measurements, all through ``bench_stock_vs_omni._arm_demo`` (the same
builders the ledger cells use), so a variant is never a per-surface rewrite:

``variants``   one demo, one dt, stock integrator swapped:
                 rk4    svIntegratorRK4                        (the bench basis)
                 rkf45  svIntegratorRKF45,  rel = abs = 1e-12  (aligned, RK6)
                 rkf78  svIntegratorRKF78,  rel = abs = 1e-12  (aligned, RK13)
                 ref    svIntegratorRKF78,  rel = abs = 1e-14  (reference)
                 omni   OmniVecIntegrator,  tol = 1e-12        (under test)
               Reported: wall time, and each arm's final state against ref.

``omni_tol_sweep``   the omni's own tol from 1e-4 to 1e-16, to price its 1e-12
               request rather than assume it.

``dt_ladder``  dt from 15 s down to 0.47 s for a gravity-only demo and the drag
               demo, giving the observed convergence order of each arm against
               the tight reference.  This is where a matched-tolerance baseline
               actually differs from a matched-step one.

Wall time is arm build + InitializeSimulation + ExecuteSimulation (the bench's
own convention), so both sides pay their own construction cost.  Metrology: the
shipped adaptive defaults are relTol=1e-4 / absTol=1e-8, so a 1e-12 request is
set explicitly and read back; the first heyoka build in a fresh process pays a
one-time LLVM cold start, so this probe performs the bench's own unrecorded JIT
warm-up first.  Ledger: ``results/integrator_align.json``.

Run: ``python audit_bundle/probes/integrator_align.py``
"""
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Repo root (Python/) on sys.path no matter the cwd -- identical to every
# other probe.  Do NOT insert parents[2] (native_omni_heyoka/): it carries a
# sourceless omniheyoka tree that would shadow the installed wheel.
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from common import log_text, save_result, safe_import_basilisk  # noqa: E402

import bench_stock_vs_omni as B  # noqa: E402

DT = 0.1
REPEATS = 3
TOL_REQUESTED = 1.0e-12
CASES = (("basicOrbit_LEO", 600.0),
         ("attFeedbackRW", 600.0),
         ("dragDeorbit", 1200.0))
ORDER = ("rk4", "rkf45", "rkf78", "ref", "omni")
TOL = {"rkf45": 1.0e-12, "rkf78": 1.0e-12, "ref": 1.0e-14}
FIELDS = ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B")

TOL_SWEEP = (1e-4, 1e-6, 1e-8, 1e-10, 1e-12, 1e-14, 1e-16)
TOL_DEMO, TOL_T = "basicOrbit_LEO", 600.0
LADDER = (15.0, 7.5, 3.75, 1.875, 0.9375, 0.46875)
LADDER_CASES = (("basicOrbit_LEO", 1200.0), ("dragDeorbit", 1200.0))


# --- shared arm builders ---------------------------------------------------
def _stock(name, dt):
    """The bench's stock arm, minus the (None) omni handle."""
    sim, sc, log, spd, _oi, aux = B._arm_demo(False, dt, name)
    return sim, sc, log, spd, aux


def _set_stock_integrator(sc, variant):
    """Swap the stock integrator.  RK4 (the default) is left alone."""
    if variant not in TOL:
        return
    from Basilisk.simulation import svIntegrators
    rk = (svIntegrators.svIntegratorRKF78(sc)
          if variant in ("rkf78", "ref")
          else svIntegrators.svIntegratorRKF45(sc))
    rk.relTol = float(TOL[variant])
    rk.absTol = float(TOL[variant])
    sc.setIntegrator(rk)


def _final(log, spd):
    import numpy as _np
    out = {f: _np.asarray(getattr(log, f))[-1].astype(float) for f in FIELDS}
    if spd is not None:
        out["wheels"] = _np.asarray(spd.wheelSpeeds)[-1].astype(float)
    return out


def _one(name, dt, variant, T, tol=None):
    """One arm: build + InitializeSimulation + ExecuteSimulation."""
    if variant == "omni":
        sim, sc, log, spd, oi, aux = B._arm_demo(True, dt, name)
        if tol is not None:
            oi.setTol(float(tol))
    else:
        sim, sc, log, spd, aux = _stock(name, dt)
        _set_stock_integrator(sc, variant)
    reset = aux.get("reset_fn")
    if reset is not None:
        B._run_two_phase(sim, T, float(aux.get("reset_at") or 10.0), reset)
    else:
        B._run(sim, T)
    extra = {}
    if variant == "omni":
        try:
            extra["steps"] = int(oi.getStepCountForObject(0))
            extra["build_ms"] = round(float(oi.getBuildSeconds()) * 1e3, 3)
        except Exception:
            pass
    return _final(log, spd), extra


# --- (1) stock-integrator variants at the bench dt ------------------------
def _variants(name, T):
    import numpy as _np
    walls, last = {}, {}
    for variant in ORDER:
        runs = []
        for _ in range(REPEATS):
            t0 = time.perf_counter()
            fin, _ = _one(name, DT, variant, T)
            runs.append(time.perf_counter() - t0)
            last[variant] = fin
        walls[variant] = runs
    ref = last["ref"]
    out = {}
    for variant in ORDER:
        runs = walls[variant]
        d = {f: float(_np.abs(last[variant][f] - ref[f]).max()) for f in FIELDS}
        if "wheels" in ref:
            d["wheels"] = float(
                _np.abs(last[variant]["wheels"] - ref["wheels"]).max())
        rec = {"wall_s": round(statistics.median(runs), 6),
               "wall_min_s": round(min(runs), 6),
               "walls_s": [round(w, 6) for w in runs],
               "vs_ref": {k: float(v) for k, v in d.items()}}
        if variant == "rk4":
            pair = {f: float(_np.abs(last["rk4"][f] - last["omni"][f]).max())
                    for f in FIELDS}
            if "wheels" in ref:
                pair["wheels"] = float(_np.abs(
                    last["rk4"]["wheels"] - last["omni"]["wheels"]).max())
            rec["rk4_vs_omni"] = pair
        out[variant] = rec
    omni = out["omni"]["wall_s"]
    out["omni"]["speedup_vs_omni"] = 1.0
    for variant in ("rk4", "rkf45", "rkf78", "ref"):
        out[variant]["speedup_vs_omni"] = round(out[variant]["wall_s"] / omni, 3)
        out[variant]["cost_vs_rk4"] = round(
            out[variant]["wall_s"] / out["rk4"]["wall_s"], 3)
    return out


# --- (2) the omni's own tolerance is a choice, not a constraint -----------
def _tol_sweep():
    import numpy as _np
    got = {}
    for tol in TOL_SWEEP:
        runs, fin = [], None
        for _ in range(REPEATS):
            t0 = time.perf_counter()
            fin, extra = _one(TOL_DEMO, DT, "omni", TOL_T, tol=tol)
            runs.append(time.perf_counter() - t0)
        got[tol] = {"wall_s": round(statistics.median(runs), 6),
                    "walls_s": [round(w, 6) for w in runs],
                    "steps": extra.get("steps"),
                    "build_ms": extra.get("build_ms"),
                    "final": fin}
    tight = got[min(TOL_SWEEP)]["final"]
    out = {}
    for tol in TOL_SWEEP:
        rec = {k: v for k, v in got[tol].items() if k != "final"}
        rec["dr_vs_tightest"] = float(
            _np.abs(got[tol]["final"]["r_BN_N"] - tight["r_BN_N"]).max())
        out["%g" % tol] = rec
    return out


# --- (3) convergence order: where a tolerance baseline differs ------------
def _ladder(name, T):
    import numpy as _np
    rows, prev = {}, None
    for dt in LADDER:
        r4, _ = _one(name, dt, "rk4", T)
        ro, _ex = _one(name, dt, "omni", T)
        rr, _ = _one(name, dt, "ref", T)
        e4 = float(_np.abs(r4["r_BN_N"] - rr["r_BN_N"]).max())
        eo = float(_np.abs(ro["r_BN_N"] - rr["r_BN_N"]).max())
        rec = {"rk4_vs_ref": e4, "omni_vs_ref": eo,
               "rk4_vs_omni": float(_np.abs(r4["r_BN_N"] - ro["r_BN_N"]).max()),
               "omni_steps": _ex.get("steps")}
        if prev is not None:
            prec = prev
            for key, cur in (("rk4", e4), ("omni", eo)):
                old = prec[key]
                rec["order_" + key] = (
                    round(math.log2(old / cur), 2)
                    if old > 0 and cur > 0 else None)
        rows["%g" % dt] = rec
        prev = {"rk4": e4, "omni": eo}
    return rows


def main():
    if not safe_import_basilisk():
        print("Basilisk unavailable -- skipping")
        return 1
    # JIT warm-up (metrology, not physics), same as the bench: the first heyoka
    # build in a fresh process pays a one-time LLVM cold start.
    warm, _, _, _, _, _ = B._arm(True, "hub", DT)
    B._run(warm, 2 * DT)

    payload = {"status": "ok", "dt": DT, "repeats": REPEATS,
               "tol_requested": TOL_REQUESTED, "variants": {},
               "omni_tol_sweep": {}, "dt_ladder": {}}
    for name, T in CASES:
        payload["variants"][name] = {"T": T, "by_variant": _variants(name, T)}
    payload["omni_tol_sweep"] = {"demo": TOL_DEMO, "T": TOL_T,
                                 "by_tol": _tol_sweep()}
    for name, T in LADDER_CASES:
        payload["dt_ladder"][name] = {"T": T, "by_dt": _ladder(name, T)}

    lines = []
    for name, T in CASES:
        v = payload["variants"][name]["by_variant"]
        lines.append("%s (T=%.0f s, dt=%.2f, median of %d)"
                     % (name, T, DT, REPEATS))
        lines.append("  %-5s %9s %9s %9s  %-13s %-13s %-13s"
                     % ("var", "wall_s", "x omni", "x rk4", "|dr| vs ref",
                        "|dv| vs ref", "|dr| rk4:omni"))
        for k in ORDER:
            r = v[k]
            lines.append("  %-5s %9.4f %9s %9s  %-13.4e %-13.4e %-13s"
                         % (k, r["wall_s"],
                            "%.2fx" % r["speedup_vs_omni"],
                            "%.2fx" % r.get("cost_vs_rk4", 1.0),
                            r["vs_ref"]["r_BN_N"], r["vs_ref"]["v_BN_N"],
                            ("%.4e" % r["rk4_vs_omni"]["r_BN_N"])
                            if "rk4_vs_omni" in r else "-"))
        lines.append("")

    sw = payload["omni_tol_sweep"]
    lines.append("omni tol sweep -- %s (T=%.0f s, dt=%.2f)"
                 % (sw["demo"], sw["T"], DT))
    lines.append("  %-8s %9s %12s %9s %9s  %s"
                 % ("tol", "wall_s", "build_ms", "steps", "x of 1e-4",
                    "|dr| vs 1e-16"))
    base = sw["by_tol"]["0.0001"]["wall_s"]
    for tol in TOL_SWEEP:
        r = sw["by_tol"]["%g" % tol]
        lines.append("  %-8g %9.4f %12s %9s %9.2f  %.4e"
                     % (tol, r["wall_s"], r["build_ms"], r["steps"],
                        r["wall_s"] / base, r["dr_vs_tightest"]))
    lines.append("")

    for name, T in LADDER_CASES:
        lines.append("dt ladder -- %s (T=%.0f s)" % (name, T))
        lines.append("  %-8s %-13s %-13s %-13s %-8s %-8s"
                     % ("dt", "|dr| rk4:ref", "|dr| omni:ref",
                        "|dr| rk4:omni", "ord rk4", "ord omni"))
        for dt in LADDER:
            r = payload["dt_ladder"][name]["by_dt"]["%g" % dt]
            lines.append("  %-8g %-13.4e %-13.4e %-13.4e %-8s %-8s"
                         % (dt, r["rk4_vs_ref"], r["omni_vs_ref"],
                            r["rk4_vs_omni"],
                            r.get("order_rk4", "-"), r.get("order_omni", "-")))
        lines.append("")

    text = "\n".join(lines)
    print(text)
    log_text("integrator_align", text)
    path = save_result("integrator_align", payload)
    print("ledger:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
