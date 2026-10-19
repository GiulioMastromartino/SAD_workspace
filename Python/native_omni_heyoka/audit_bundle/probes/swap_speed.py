"""Swap-path speedup table (Phase-3 item 4): honest cold-vs-warm/per-step split.

Replaces the stale Update-2/3 claim (4-5x + "~200 ms first JIT", predating the
measured ~18 s deterministic cold start) with a measured ledger on the swap
path. Fixture: hub + 3 RW + 1 staircase thruster (5 N, duty 0.5) + single
depleting tank (10 kg) + central gravity -- the WP-H config class
(nPars 21 / nStates 16), 600 ticks @ dt=0.1, open-loop (timing-identical with
or without drivers; no FSW in this probe).

3-RUN DESIGN (means + min/max, n_runs=3): stock x3 and warm x3 run in-process
(fresh sims each); cold x3 = 1 in-process + 2 fresh subprocesses
(`--cold-once` child mode: a second build in-process is warm BY DEFINITION,
so true cold repeats need fresh interpreters). A missing child marker fails
loudly -- never averaged silently.

Recorded (never gated here; tests/test_replacement_matrix.py adopts the
table once this ledger exists):
  cold_build_wall_s  mean wall of the 3 cold builds (graph staging; JIT is
                     lazy so this excludes the compile -- see first-run wall)
  *_min_s/*_max_s    spread per phase (contention visibility, with loadavg)
  warm_run_wall_s    mean 2nd..4th-run wall = steady-state stepping
  speedup_warm       mean(stock) / mean(warm run) at matched ticks

Ledger: audit_bundle/results/swap_speed.json. Host-gated: without Basilisk or
the built wheel it records status "skipped-no-basilisk", never errors.
"""
# Planned legs (comparable fixtures -- change constants only; timing logic,
# key names, step audits, and the --cold-once protocol stay fixed):
#   1. quiet leg: DUTY = 0.0 (same T_LEG/DT/TICKS) -- decides the segmentation
#      hypothesis (whether the warm speedup holds with no thruster firing).
#   2. long-warm leg: T_LEG = 600.0 (600 s warm, same DUTY) -- decides whether
#      the warm per-step cost stays flat over a 10x leg.
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Repo root (Python/) on sys.path no matter the cwd: omniheyoka.presets
# imports the sibling cubesat_12u_sim package (same path run_all.sh exports).
_REPO_ROOT = str(Path(__file__).resolve().parents[3])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from common import log_text, save_result, write_exception  # noqa: E402

DUTY = 0.5
DT = 0.1
T_LEG = 60.0
TICKS = int(T_LEG / DT)
MU = 398600441800000.0
MASS_DRY, PROP0 = 20.0, 10.0
I6 = [[10.0, 0.0, 0.0], [0.0, 11.0, 0.0], [0.0, 0.0, 12.0]]


def _loadavg():
    try:
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return []


def _base_sim(tag):
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                    simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = MASS_DRY
    sc.hub.IHubPntBc_B = [list(r) for r in I6]
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    # All three angles required: ClassicElements() leaves Omega/omega/f as
    # None (not 0.0) and elem2rv rejects None (same fixture as closed_loop).
    oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R, 347.8 * macros.D2R,
                                85.3 * macros.D2R)
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    return sim, sc


def _burn_driver(duty=DUTY):
    from Basilisk.architecture import messaging, sysModel

    class Burn(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = duty * DT
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)
    return Burn()


def _build(use_omni, duty=DUTY):
    """One arm of the speed leg. Returns (sim, oi or None)."""
    from Basilisk.simulation import (fuelTank, reactionWheelStateEffector,
                                      thrusterDynamicEffector)
    from Basilisk.utilities import macros, simIncludeRW, simIncludeThruster  # noqa: F401
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config

    sim, sc = _base_sim(("omni-" if use_omni else "rk-") + "speed")
    fac = simIncludeRW.rwFactory()
    for gs in ([1., 0, 0], [0, 1., 0], [0, 0, 1.]):
        fac.create("Honeywell_HR16", list(gs), maxMomentum=50.0, Omega=100.0)
    rw = reactionWheelStateEffector.ReactionWheelStateEffector()
    fac.addToSpacecraft("rw", rw, sc)
    sim.AddModelToTask("t", rw)
    tf = simIncludeThruster.thrusterFactory()
    tf.create("TEST_Thruster", [0.0, 0.0, 1.0], [0.0, 0.0, -1.0],
              MaxThrust=5.0, cutoffFrequency=1000.0, useMinPulseTime=False)
    thr = thrusterDynamicEffector.ThrusterDynamicEffector()
    tf.addToSpacecraft("thr", thr, sc)
    sim.AddModelToTask("t", thr, 2)
    burn = _burn_driver(duty)
    sim.AddModelToTask("t", burn)
    thr.cmdsInMsg.subscribeTo(burn.out)
    tank = fuelTank.FuelTank()
    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = PROP0
    tank.setTankModel(model)
    tank.setNameOfMassState("fuelMass")
    tank.setR_TB_B([0.0, 0.0, 0.0])
    tank.addThrusterSet(thr)
    sc.addStateEffector(tank)
    sim.AddModelToTask("t", tank)
    oi = None
    if use_omni:
        oi = OmniVecIntegrator(sc)
        mirror_stock_config(sim, sc, oi, thr_factory=tf, thr_cmd_msg=burn.out)
        sc.setIntegrator(oi)
    return sim, oi


def _run(sim, t_leg=T_LEG):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    t0 = time.perf_counter()
    sim.ConfigureStopTime(macros.sec2nano(t_leg))
    sim.ExecuteSimulation()
    return time.perf_counter() - t0


def _end_nanos(sim):
    """Simulated nanos reached (catches short-run artifacts: a 'fast' arm
    that stepped fewer ticks would show it here, not in the wall)."""
    return int(sim.TotalSim.CurrentNanos)


def _expected_nanos(t_leg=T_LEG):
    from Basilisk.utilities import macros
    return int(macros.sec2nano(t_leg))


def _cold_once() -> int:
    """Child mode (--cold-once): one cold build+run, marker JSON to stdout.

    No ledger write (parent owns it). Any failure -> traceback on stderr,
    nonzero exit, NO marker line: the parent treats a missing marker as a
    loud failure, never an averaged silence.
    """
    t0 = time.perf_counter()
    sim_o, oi = _build(True)
    build_wall = time.perf_counter() - t0
    build_s = float(oi.getBuildSeconds())
    run_wall = _run(sim_o)
    steps = int(oi.getStepCountForObject(0))
    print("SWAP_SPEED_COLD " + json.dumps({
        "build_wall_s": build_wall, "build_s": build_s,
        "run_wall_s": run_wall, "steps": steps,
        "end_nanos": _end_nanos(sim_o)}))
    return 0


def _leg_run(duty, t_leg, n=3):
    """One follow-up leg config: stock x n + omni x n, all in-process.

    Narrower than main(): no fresh-interpreter colds (cold-build stats live
    in the pinned swap_speed.json ledger). Run 1 of the omni arm is the
    first-run wall (lazy JIT if the cache misses this config); runs 2..n
    are the warm steady-state sample. Every wall carries its own loadavg
    snapshot so contention is attributable per run, not per leg.
    """
    ticks = int(round(t_leg / DT))
    expected_nanos = _expected_nanos(t_leg)
    stock_runs, stock_loads = [], []
    for _ in range(n):
        sim_s, _ = _build(False, duty=duty)
        stock_loads.append(_loadavg())
        stock_runs.append(_run(sim_s, t_leg))
        if _end_nanos(sim_s) != expected_nanos:
            raise RuntimeError(
                f"time audit failed (stock): end={_end_nanos(sim_s)}ns != "
                f"expected={expected_nanos}ns")
    omni_runs, omni_loads, omni_builds = [], [], []
    for _ in range(n):
        t0 = time.perf_counter()
        sim_o, oi = _build(True, duty=duty)
        omni_builds.append(time.perf_counter() - t0)
        omni_loads.append(_loadavg())
        omni_runs.append(_run(sim_o, t_leg))
        if int(oi.getStepCountForObject(0)) != ticks:
            raise RuntimeError(
                f"step audit failed: steps={oi.getStepCountForObject(0)} "
                f"!= ticks={ticks}")
        if _end_nanos(sim_o) != expected_nanos:
            raise RuntimeError(
                f"time audit failed: end={_end_nanos(sim_o)}ns != "
                f"expected={expected_nanos}ns")

    def _mean(xs):
        return sum(xs) / len(xs)

    def _r4(xs):
        return [round(x, 4) for x in xs]

    stock_mean = _mean(stock_runs)
    warm_runs = omni_runs[1:]
    warm_mean = _mean(warm_runs)
    return {
        "duty": duty,
        "t_leg": t_leg,
        "ticks": ticks,
        "dt": DT,
        "fixture_class": "firing" if duty > 0 else "quiet",
        "n_runs": n,
        "stock_wall_samples_s": _r4(stock_runs),
        "stock_wall_mean_s": stock_mean,
        "stock_loadavg": stock_loads,
        "omni_wall_samples_s": _r4(omni_runs),
        "omni_build_wall_samples_s": _r4(omni_builds),
        "omni_loadavg": omni_loads,
        "first_run_wall_s": omni_runs[0],
        "first_per_step_ms": round(omni_runs[0] / ticks * 1000.0, 4),
        "warm_run_wall_s": warm_mean,
        "warm_run_samples_s": _r4(warm_runs),
        "warm_per_step_ms": round(warm_mean / ticks * 1000.0, 4),
        "speedup_warm": round(stock_mean / warm_mean, 3) if warm_mean else None,
        "sim_end_nanos_ok": True,
        "sim_end_nanos_expected": expected_nanos,
    }


def _legs_main() -> int:
    """Follow-up legs mode (--legs): quiet + 600 s warm legs.

    Writes results/swap_speed_legs.json (+ .log). The pinned
    results/swap_speed.json is never touched here.
    """
    import traceback as _tb

    try:
        import Basilisk  # noqa: F401
    except Exception as exc:
        save_result("swap_speed_legs", {"status": "skipped-no-basilisk",
                                        "failed_import": "Basilisk",
                                        "error": f"{type(exc).__name__}: {exc}",
                                        "traceback": _tb.format_exc(limit=12)})
        print(f"swap_speed_legs: skipped (Basilisk unimportable: {exc})")
        return 0
    try:
        from omniheyoka.omni_heyoka import OmniVecIntegrator  # noqa: F401
    except Exception as exc:
        save_result("swap_speed_legs", {"status": "skipped-no-basilisk",
                                        "failed_import": "omniheyoka.omni_heyoka",
                                        "error": f"{type(exc).__name__}: {exc}",
                                        "traceback": _tb.format_exc(limit=12)})
        print(f"swap_speed_legs: skipped (omniheyoka unimportable: {exc})")
        return 0
    try:
        load0 = _loadavg()
        # Leg 1 (quiet): DUTY = 0.0, same 60 s / 600-tick fixture --
        # segmentation hypothesis (warm speedup with no thruster firing).
        quiet = _leg_run(0.0, 60.0)
        # Leg 2 (long-warm): same DUTY = 0.5, 600 s / 6000-tick leg --
        # warm per-step amortization over a 10x leg.
        long_warm = _leg_run(DUTY, 600.0)
        payload = {
            "status": "ok",
            "legs": {"quiet_duty0_60s": quiet,
                     "long_warm_duty05_600s": long_warm},
            "loadavg": load0,
            "loadavg_end": _loadavg(),
            "note": ("follow-up legs for the pinned swap_speed.json table "
                      "(untouched): quiet leg tests the segmentation "
                      "hypothesis at DUTY=0; long-warm leg tests per-step "
                      "amortization over 600 s. In-process runs only "
                      "(n=3/arm); cold-build stats live in swap_speed.json. "
                      "Per-run loadavg provenance on every sample."),
        }
        out = save_result("swap_speed_legs", payload)
        for name, leg in (("quiet", quiet), ("long_warm", long_warm)):
            log_text("swap_speed_legs",
                     f"[{name}] duty={leg['duty']} t_leg={leg['t_leg']}s "
                     f"ticks={leg['ticks']} "
                     f"stock_mean={leg['stock_wall_mean_s']:.4f}s "
                     f"{leg['stock_wall_samples_s']} "
                     f"first={leg['first_run_wall_s']:.4f}s "
                     f"warm={leg['warm_run_wall_s']:.4f}s "
                     f"{leg['warm_run_samples_s']} "
                     f"warm_per_step={leg['warm_per_step_ms']:.4f}ms "
                     f"speedup_warm={leg['speedup_warm']} "
                     f"load={leg['omni_loadavg']}")
        print(out)
        return 0
    except Exception as exc:
        write_exception("swap_speed_legs", exc)
        print(f"swap_speed_legs FAILED: {exc}")
        return 1


def main() -> int:
    import traceback as _tb

    if "--legs" in sys.argv:
        return _legs_main()

    if "--cold-once" in sys.argv:
        try:
            return _cold_once()
        except Exception:
            _tb.print_exc()
            return 1

    def _skip(which: str, exc: Exception) -> int:
        save_result("swap_speed", {"status": "skipped-no-basilisk",
                                   "failed_import": which,
                                   "error": f"{type(exc).__name__}: {exc}",
                                   "traceback": _tb.format_exc(limit=12)})
        print(f"swap_speed: skipped ({which} unimportable: {exc})")
        return 0

    try:
        import Basilisk  # noqa: F401
    except Exception as exc:
        return _skip("Basilisk", exc)
    try:
        from omniheyoka.omni_heyoka import OmniVecIntegrator  # noqa: F401
    except Exception as exc:
        return _skip("omniheyoka.omni_heyoka", exc)
    try:
        # 1. Stock x3 (fresh sim each -- re-Execute would continue the leg).
        stock_runs = []
        stock_nanos = []
        for _ in range(3):
            sim_s, _ = _build(False)
            stock_runs.append(_run(sim_s))
            stock_nanos.append(_end_nanos(sim_s))
        # 2. Cold x3 = 1 in-process (this also warms the process for phase 3)
        #    + 2 fresh subprocesses.
        def _steps_of(oi):
            return int(oi.getStepCountForObject(0))

        t0 = time.perf_counter()
        sim_o, oi = _build(True)
        colds = [{"build_wall_s": time.perf_counter() - t0,
                  "build_s": float(oi.getBuildSeconds()),
                  "run_wall_s": _run(sim_o),
                  "steps": _steps_of(oi),
                  "end_nanos": _end_nanos(sim_o)}]
        for _ in range(2):
            proc = subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), "--cold-once"],
                capture_output=True, text=True, timeout=600)
            for line in (proc.stdout or "").splitlines():
                if line.startswith("SWAP_SPEED_COLD "):
                    colds.append(json.loads(line[len("SWAP_SPEED_COLD "):]))
        if len(colds) != 3:
            raise RuntimeError(
                f"cold subprocess failed (got {len(colds) - 1}/2 markers); "
                "see child stderr in swap_speed log")
        # 3. Warm x3 in-process (post-cold: cache hot by construction).
        warms = []
        for _ in range(3):
            t0 = time.perf_counter()
            sim_w, oi2 = _build(True)
            warms.append({
                "build_wall_s": time.perf_counter() - t0,
                "build_s": float(oi2.getBuildSeconds()),
                "run_wall_s": _run(sim_w),
                "steps": _steps_of(oi2),
                "end_nanos": _end_nanos(sim_w)})
        # steps==ticks AND full 60 s simulated on every run, both arms --
        # a fast wall with short nanos is a short run, not a fast sim (loud).
        expected_nanos = _expected_nanos()
        for c in colds + warms:
            if c["steps"] != TICKS:
                raise RuntimeError(
                    f"step audit failed: steps={c['steps']} != ticks={TICKS}")
            if c["end_nanos"] != expected_nanos:
                raise RuntimeError(
                    f"time audit failed: end={c['end_nanos']}ns != "
                    f"expected={expected_nanos}ns")
        for n in stock_nanos:
            if n != expected_nanos:
                raise RuntimeError(
                    f"time audit failed (stock): end={n}ns != "
                    f"expected={expected_nanos}ns")
        load = _loadavg()

        def _mean(xs):
            return sum(xs) / len(xs)

        stock_mean = _mean(stock_runs)
        first_runs = [c["run_wall_s"] for c in colds]
        first_mean = _mean(first_runs)
        warm_runs = [w["run_wall_s"] for w in warms]
        warm_mean = _mean(warm_runs)

        def _r4(xs):
            return [round(x, 4) for x in xs]
        payload = {
            "status": "ok",
            "n_runs": 3,
            "fixture": ("hub+3RW+1thr(5N,duty0.5)+fuel(10kg,centered)+central, "
                        f"{TICKS} ticks @ dt={DT}, open-loop"),
            "ticks": TICKS,
            "steps": TICKS,
            "duty": DUTY,
            "fixture_class": "firing" if DUTY > 0 else "quiet",
            "cold_build_s": _mean([c["build_s"] for c in colds]),
            "cold_build_wall_s": _mean([c["build_wall_s"] for c in colds]),
            "cold_build_wall_min_s": min(c["build_wall_s"] for c in colds),
            "cold_build_wall_max_s": max(c["build_wall_s"] for c in colds),
            "cold_build_wall_samples_s": _r4([c["build_wall_s"] for c in colds]),
            "warm_build_s": _mean([w["build_s"] for w in warms]),
            "warm_build_wall_s": _mean([w["build_wall_s"] for w in warms]),
            "stock_wall_s": stock_mean,
            "stock_wall_min_s": min(stock_runs),
            "stock_wall_max_s": max(stock_runs),
            "stock_wall_samples_s": _r4(stock_runs),
            "omni_wall_s": first_mean,
            "first_run_min_s": min(first_runs),
            "first_run_max_s": max(first_runs),
            "first_run_samples_s": _r4(first_runs),
            "per_step_ms": round(first_mean / TICKS * 1000.0, 4),
            "warm_run_wall_s": warm_mean,
            "warm_run_min_s": min(warm_runs),
            "warm_run_max_s": max(warm_runs),
            "warm_run_samples_s": _r4(warm_runs),
            "warm_steps": TICKS,
            "sim_end_nanos_ok": True,
            "sim_end_nanos_expected": expected_nanos,
            "warm_per_step_ms": round(warm_mean / TICKS * 1000.0, 4),
            "speedup_warm": round(stock_mean / warm_mean, 3) if warm_mean else None,
            "loadavg": load,
            "note": ("means of 3 runs per phase (cold: 1 in-process + 2 fresh "
                     "interpreters); omni_wall_s/per_step_ms = FIRST omni run "
                     "(includes JIT if lazy); warm_run_wall_s/warm_per_step_ms "
                     "= post-cold runs (steady-state stepping iff cross-oi "
                     "JIT cache works); speedup_warm = mean(stock)/mean(warm "
                     "run); contention-inflated walls carry loadavg provenance."),
        }
        out = save_result("swap_speed", payload)
        log_text("swap_speed",
                 f"ticks={TICKS} n=3 "
                 f"stock={stock_mean:.4f}s [{min(stock_runs):.4f},{max(stock_runs):.4f}] "
                 f"first={first_mean:.3f}s "
                 f"warm_run={warm_mean:.4f}s "
                 f"[{min(w['run_wall_s'] for w in warms):.4f},"
                 f"{max(w['run_wall_s'] for w in warms):.4f}] "
                 f"warm_per_step={payload['warm_per_step_ms']:.4f}ms "
                 f"speedup_warm={payload['speedup_warm']} load={load}")
        print(out)
        return 0
    except Exception as exc:
        write_exception("swap_speed", exc)
        print(f"swap_speed FAILED: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
