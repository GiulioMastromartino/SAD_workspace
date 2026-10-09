"""One global CMA-ES over all NMPC+PID parameters, scored by 250-run Monte Carlo.

Replaces the three-stage PID/NMPC/joint sweep in :mod:`run_optimize` with a
single optimiser, a single population, and a cost function that sees a Monte
Carlo ensemble instead of one deterministic run.

    generation
      -> CMA-ES ask()            150 vectors from the 16-D hypercube
      -> for each vector          250 runs through the full C++ pipeline,
                                  against one frozen common-random-number
                                  design shared by every candidate
      -> reduce                 mean + weighted worst/best deciles + infeasible
      -> CMA-ES tell()

The horizon is one of the 16 coordinates (discrete), so it is tuned rather than
guessed. Both arms are simulated and both feed the cost.

Usage (from Python/)::

    ./env/bin/python -m cubesat_12u_sim.tuning.run_mc_optimize
    ./env/bin/python -m cubesat_12u_sim.tuning.run_mc_optimize \\
        --generations 20 --popsize 150 --workers 8
    ./env/bin/python -m cubesat_12u_sim.tuning.run_mc_optimize --resume

Wall time, measured on the reference machine at ``--days 0.5``:

===========  ==========  ==========================  ==============
horizon      ms/lane     150-candidate generation    pooled (8x)
===========  ==========  ==========================  ==============
H=2          28          18 min                      ~3 min
H=4          76          48 min                      ~7 min
H=6          158         1.6 h                       ~13 min
===========  ==========  ==========================  ==============

The searched horizon set is capped at H<=6 (``params.MC_HORIZONS``). Cost is
superlinear in H and a generation is otherwise bounded by its slowest member,
so letting H=12 into a population of 150 buys extra predictions at several
times the wall clock while stranding whichever worker drew one. The generator
prints the per-generation wall time for exactly this reason.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.optimization.mcdesign import MCConfig
from cubesat_12u_sim.optimization.mcdriver import run_sweep
from cubesat_12u_sim.optimization.mcobjective import (PENALTY_INFEASIBLE,
                                         TailWeights, est_nmpc_s)
from cubesat_12u_sim.optimization.params import (HYPER_SPACE, MC_HORIZONS,
                                   NMPC_SPACE, PID_SPACE)

OUTDIR = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                      "..", "..", "results_mc_optimize"))


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def _write(name: str, payload: dict) -> str:
    os.makedirs(OUTDIR, exist_ok=True)
    path = os.path.join(OUTDIR, name)
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True, default=float)
    return path


def resolve_backends() -> tuple[bool, list[str]]:
    """Fail loudly if the C++ pipeline this sweep depends on is missing.

    Unlike the staged sweep, there is no meaningful fallback here: the numpy
    path costs ~100 ms per run, so a 250-run ensemble would take half a minute
    per candidate and a single generation would run for days. A clear error at
    startup is far better than a sweep that appears to progress.
    """
    notes: list[str] = []
    try:
        import heyoka  # noqa: F401
        from cubesat_12u_sim.native.cxxdrive import _nmpcsolve, _pidbatch  # noqa: F401
    except Exception as exc:
        raise SystemExit(
            f"[fatal] the full C++ pipeline is unavailable ({type(exc).__name__}: "
            f"{exc}).\n"
            f"        This sweep scores every candidate with a 250-run Monte "
            f"Carlo; the numpy fallback is ~500x too slow for that. Build the "
            f"extensions first:\n"
            f"          cd Python && ./env/bin/pip install -e "
            f"./cubesat_12u_sim/native/cxxdrive")
    return True, notes


def seed_theta_from_results(which: str = "tuned") -> np.ndarray:
    """Build the CMA-ES seed mean from the committed staged-sweep winners.

    Starting the single global search at the *hand-tuned* defaults wastes the
    first generation: the PID defaults leave the station-keeping box on most
    injections, so a 150-candidate population would have to rediscover a
    working controller before any gradient information is available. The staged
    sweep already found a feasible point for each arm, so the global search
    starts from those and spends its budget improving them rather than finding
    feasibility.

    Falls back to the space defaults if the results files are missing, and says
    so — a silent fallback would make the run look like a continuation of the
    staged work when it is not.
    """
    theta = HYPER_SPACE.seed_theta()
    # The staged-sweep results sit next to this file's own OUTDIR, not inside
    # it: OUTDIR already points at results_mc_optimize/.
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    opt_dir = os.path.join(pkg_dir, "results_optimization")
    pid_path = os.path.join(opt_dir, "res_stage_pid.json")
    nmpc_path = os.path.join(opt_dir, "res_stage_nmpc.json")

    def _load(path: str) -> dict:
        with open(path) as fh:
            return json.load(fh)

    # The hypercube layout is NMPC-then-PID-then-horizon by construction, so the
    # slices are arithmetic rather than a name search.
    off = len(NMPC_SPACE)
    n_pid = len(PID_SPACE)
    try:
        pid = _load(pid_path)
        nmpc = _load(nmpc_path)
    except FileNotFoundError as exc:
        _log(f"  note: {exc.filename} not found; seeding from config defaults "
             f"instead of the staged-sweep winners")
        return HYPER_SPACE.seed_theta()

    theta[off:off + n_pid] = PID_SPACE.to_theta(pid["params"])

    # The staged sweep's NMPC parameters are NOT reusable. The cost function was
    # rewritten (q_pos/q_vel/r_fuel removed in favour of radius-error tracking
    # and the new fuel shape), so those keys are not in NMPC_SPACE and
    # NMPC_SPACE.to_theta would silently fill defaults for them anyway. The
    # NMPC arm therefore starts from its space defaults and is searched from
    # scratch -- there is no earlier tuned NMPC point to inherit.
    theta[:off] = NMPC_SPACE.seed_theta()
    _log("  seeded NMPC from space defaults: the cost function was rewritten, "
         "so the staged sweep's q_pos/q_vel/r_fuel no longer exist")

    # The staged sweep's horizon ranking is not transferable: it was chosen on a
    # single deterministic run, while here every candidate pays for its horizon
    # across the whole ensemble. Seed the value nearest the middle of the
    # searched set and let the Monte Carlo decide.
    h = int(nmpc.get("horizon", 6))
    seed_h = h if h in MC_HORIZONS else min(
        MC_HORIZONS, key=lambda c: (abs(math.log(c) - math.log(6)), c))
    theta[-1] = math.log(seed_h)
    _log(f"  seeded from staged-sweep winners (pid), horizon H={seed_h}"
         + ("" if seed_h == h else f" (staged H={h} is outside {MC_HORIZONS})"))
    return theta


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--generations", type=int, default=18,
                    help="CMA-ES generations (default: 20)")
    ap.add_argument("--popsize", type=int, default=150,
                    help="candidates per generation (default: 150)")
    ap.add_argument("--restarts", type=int, default=0,
                    help="extra CMA-ES runs, each seeded from the previous "
                         "run's best theta with a fresh step size. Not "
                         "IPOP-CMA: the population stays at --popsize, so a "
                         "restart only buys a better starting point. "
                         "(default: 0 = single run)")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 4) - 2),
                    help="process-pool workers (default: cpu_count - 2)")
    ap.add_argument("--seed", type=int, default=20261007,
                    help="base seed for both the design and CMA-ES")
    ap.add_argument("--sigma0", type=float, default=0.3,
                    help="initial CMA-ES step size in log space")
    ap.add_argument("--dampfac", type=float, default=0.3,
                    help="CMA-ES step-size damping multiplier (CSA_dampfac). "
                         "1.0 is pycma's conservative default and adapts sigma "
                         "slowly -- the symptom of 'sigma is becoming too slow' "
                         "and of a population that converges into a basin "
                         "while sigma is still large. 0.3 reaches the same "
                         "adaptive path ~3x faster (measured: 95.0%% -> 84.4%% "
                         "of sigma0 per generation on a 6-D test problem). "
                         "(default: 0.3)")
    ap.add_argument("--stagnation-window", type=int, default=8,
                    help="generations over which to measure improvement when "
                         "deciding whether a sweep is stuck in a basin")
    ap.add_argument("--stagnation-threshold", type=float, default=0.02,
                    help="relative improvement below which the sweep counts as "
                         "stagnant and forces a restart (default: 2%%)")
    ap.add_argument("--max-auto-restarts", type=int, default=4,
                    help="cap on restarts taken on stagnation, independent of "
                         "--restarts; each doubles the population (default: 4)")
    ap.add_argument("--restart-popsize-factor", type=float, default=2.0,
                    help="population multiplier applied at each restart. 2.0 is "
                         "the IPOP-CMA rule and is the actual local-basin "
                         "escape: a restart at the same popsize only re-descends "
                         "from a better start, while a larger population can "
                         "hold basins the smaller one folded out of. 1.0 "
                         "disables. (default: 2.0)")

    ap.add_argument("--runs", type=int, default=250,
                    help="Monte Carlo runs per candidate (default: 250)")
    ap.add_argument("--vehicles", type=int, default=25,
                    help="vehicle builds; runs are split evenly across them "
                         "(default: 25)")
    ap.add_argument("--days", type=float, default=0.5,
                    help="closed-loop horizon per run [days] (default: 0.5)")
    ap.add_argument("--spec-box", type=float, default=25e3,
                    help="station-keeping box half-width [m] (default: 25000)")
    ap.add_argument("--maxiter", type=int, default=50,
                    help="SLSQP iteration cap per bulk plan (default: 50)")
    ap.add_argument("--tol", type=float, default=C.HEYOKA_TOL,
                    help=f"Heyoka Taylor tolerance (default: {C.HEYOKA_TOL:g}). "
                         f"This is the biggest wall-time lever in the whole "
                         f"pipeline. Measured on the 250-run design at H=4: "
                         f"1e-12 (default) 18.9 s/candidate, 1e-8 13.2 s, "
                         f"1e-6 9.9 s. Mean dv moves <0.5% across that range "
                         f"but individual runs shift by up to ~4 m/s, which "
                         f"perturbs the worst-decile term the cost reads. "
                         f"Loosen only if you accept noisier runs; the default "
                         f"keeps results comparable with the committed sweep.")

    ap.add_argument("--w-hi", type=float, default=0.5,
                    help="weight on the worst-decile mean cost")
    ap.add_argument("--w-lo", type=float, default=0.5,
                    help="weight on the best-decile mean cost (subtracted)")
    ap.add_argument("--w-nmpc", type=float, default=0.5,
                    help="weight of the NMPC arm in the combined cost")
    ap.add_argument("--w-pid", type=float, default=0.5,
                    help="weight of the PID arm in the combined cost")
    ap.add_argument("--w-infeasible", type=float, default=1e3,
                    help="penalty per unit infeasible-run fraction")
    ap.add_argument("--tail-frac", type=float, default=0.1,
                    help="decile width for the tail statistics (default: 0.1)")
    ap.add_argument("--max-candidate-s", type=float, default=900.0,
                    help="reject a draw whose predicted closed-loop wall time "
                         "for all runs exceeds this (default: 900 s)")

    ap.add_argument("--seed-theta", default="tuned",
                    choices=["tuned", "defaults"],
                    help="'tuned' seeds from the committed staged-sweep "
                         "winners; 'defaults' seeds from config.py "
                         "(default: tuned)")
    ap.add_argument("--checkpoint", default="ckpt_mc.json",
                    help="checkpoint filename in OUTDIR ('' disables)")
    ap.add_argument("--resume", action="store_true",
                    help="resume from the checkpoint, if one exists")
    ap.add_argument("--no-plot", action="store_true", help="skip figures")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)


    _, backend_notes = resolve_backends()

    _log("=== NMPC+PID global hypercube sweep (CMA-ES, Monte Carlo in the loop) ===")
    _log(f"  hypercube: {len(HYPER_SPACE)} dimensions")
    for line in HYPER_SPACE.describe().splitlines():
        _log(line)
    _log(f"  popsize={args.popsize} generations={args.generations} "
         f"restarts={args.restarts} workers={args.workers} "
         f"seed={args.seed} sigma0={args.sigma0} "
         f"dampfac={args.dampfac}")
    _log(f"  MC: {args.runs} runs = {args.vehicles} vehicles x "
         f"{args.runs // args.vehicles} injections")
    _log(f"  horizon per run: {args.days} d "
         f"(outer {int(round(args.days * 86400 / 600))} x 600 s)")
    _log(f"  horizon searched over {MC_HORIZONS}; predicted NMPC cost per "
         f"candidate: "
         + ", ".join(f"H={h} {est_nmpc_s(h, args.runs):.0f}s"
                     for h in MC_HORIZONS))
    for n in backend_notes:
        _log(f"  warning: {n}")

    mc_cfg = MCConfig(n_runs=args.runs, n_vehicles=args.vehicles,
                      seed=args.seed)
    weights = TailWeights(w_hi=args.w_hi, w_lo=args.w_lo,
                          w_nmpc=args.w_nmpc, w_pid=args.w_pid,
                          w_infeasible=args.w_infeasible,
                          tail_frac=args.tail_frac)
    eval_cfg = {"dt_outer": 600.0, "dt_inner": 60.0,
                "t_end": args.days * 86400.0, "spec_box": args.spec_box,
                "w_viol": 1e3, "w_final": 0.05, "maxiter": args.maxiter,
                "pid_dt": C.DT_CONTROL, "tol": args.tol}

    seed_theta = (seed_theta_from_results() if args.seed_theta == "tuned"
                  else HYPER_SPACE.seed_theta())

    ckpt = os.path.join(OUTDIR, args.checkpoint) if args.checkpoint else None
    if ckpt:
        os.makedirs(OUTDIR, exist_ok=True)

    t0 = time.time()

    # ---- restart loop -------------------------------------------------
    # A restart re-runs the sweep seeded from the previous run's best theta with
    # a fresh step size. Deliberately NOT the IPOP-CMA restart (which would also
    # double the population): the population is fixed at args.popsize, so all a
    # restart buys is a second descent from a better starting point at the same
    # per-generation cost. Its value is largest when the first run was far from
    # converged -- which is what a deliberately large sigma0 is for.
    runs, tags = [], []
    cur_theta = seed_theta
    pop = int(round(args.popsize))     # IPOP grows this per restart
    r = 0                              # restart index = sweep index
    n_auto = 0                         # restarts taken on stagnation
    while True:
        ck = f"{ckpt}.r{r}" if ckpt else None
        tag = "seed" if r == 0 else f"restart #{r} (popsize {pop})"
        runs.append(None); tags.append(tag)
        _log(f"=== sweep {r + 1}: {tag} ===")
        rep = run_sweep(generations=args.generations, popsize=pop,
                        seed=args.seed + r, sigma0=args.sigma0,
                        dampfac=args.dampfac,
                        workers=args.workers, mc_cfg=mc_cfg, weights=weights,
                        eval_cfg=eval_cfg, seed_theta=cur_theta,
                        checkpoint=ck, resume=args.resume)
        runs[-1] = rep
        cur_theta = np.asarray(rep.best_theta, float)
        _log(f"  sweep {r + 1} best cost {rep.best_cost:.2f}  "
             f"({rep.wall_s / 60:.1f} min)")

        # Decide whether to restart at all. A FIXED --restarts budget always
        # runs out; stagnation is what makes a restart worth its doubling cost.
        stuck, why = _stagnant(rep.as_dict()["history"],
                               args.stagnation_window, args.stagnation_threshold)
        fixed_left = r < args.restarts
        auto_left = n_auto < args.max_auto_restarts
        if not (fixed_left or auto_left):
            break
        if fixed_left or stuck:
            if not fixed_left:
                n_auto += 1
                _log(f"  stagnant: {why}")
            r += 1
            pop = int(round(args.popsize
                            * args.restart_popsize_factor ** r))
            _log(f"  -> restart #{r} with popsize {pop}")
        else:
            break

    report = min(runs, key=lambda r: (
        0 if (r.best["nmpc"]["feasible_frac"] > 0
              and r.best["pid"]["feasible_frac"] > 0) else 1, r.best_cost))

    _summarise(report, args)

    payload = report.as_dict()
    payload["cli"] = vars(args)
    payload["restarts"] = [dict(x.as_dict(), reason=t)
                           for x, t in zip(runs, tags)]
    payload["best_sweep"] = runs.index(report)
    # The exact argv, so the exporter emits a runnable command line instead of
    # reconstructing one from the parsed namespace (which loses quoting and the
    # distinction between a defaulted and an explicitly-passed flag). The
    # interpreter is recorded as invoked; the exporter rewrites it to the
    # repo-relative ./env/bin/python so reproduce.sh is portable.
    payload["cli_cmd"] = [sys.executable, "-m",
                          "cubesat_12u_sim.tuning.run_mc_optimize"] + list(sys.argv[1:])
    path = _write("res_mc_optimize.json", payload)
    _log(f"\nwrote {path}  ({report.wall_s / 60:.1f} min)")

    if not args.no_plot:
        try:
            from cubesat_12u_sim.optimization.mcplots import make_figures
            for f in make_figures(payload, OUTDIR):
                _log(f"[info] wrote {f}")
        except Exception as exc:
            _log(f"  note: figures skipped ({type(exc).__name__}: {exc})")
    return 0


def _stagnant(hist: list, window: int, threshold: float) -> tuple[bool, str]:
    """Has the sweep stopped meaningfully improving?

    Compares the best cost over the final ``window`` generations against the
    best over everything before them, as a fraction of the earlier value. Still
    descending fast -> large ratio; settled into a basin -> collapses to zero.

    Both cost and feasibility are inspected. Cost alone is not enough here: a
    population scoring a flat penalty constant can look "improving" on a
    relative measure while every candidate in it is infeasible, which is the
    opposite of descending.

    Returns ``(stuck, reason)`` so the log can say *why* it restarted.
    """
    if len(hist) < 2 * window:
        return False, f"only {len(hist)} generations (< 2 x {window})"

    early = min(h["best_cost"] for h in hist[:-window])
    late = min(h["best_cost"] for h in hist[-window:])
    improved = (early - late) / max(abs(early), 1e-12)

    if not (np.isfinite(early) and np.isfinite(late)):
        return True, f"non-finite cost ({early:.3g} -> {late:.3g})"
    if improved >= threshold:
        return False, (f"improved {improved * 100:.1f}% over the last "
                       f"{window} generations")

    # Cost looks flat. Distinguish a converged FEASIBLE optimum from an
    # infeasible plateau: both are "no improvement", but only one is done.
    feas = max(h["best"]["nmpc"]["feasible_frac"] for h in hist[-window:])
    if feas <= 0.0:
        return True, "no feasible candidate in the last window"
    return True, (f"only {improved * 100:.2f}% improvement over the last "
                  f"{window} generations at {feas * 100:.0f}% feasible")


def _summarise(report, args) -> None:
    b = report.best
    n, p = b["nmpc"], b["pid"]
    _log(f"\n=== best over {len(report.history)} generation(s) ===")
    _log(f"  cost {report.best_cost:.3f}   horizon H={b['horizon']}")
    for tag, s in (("NMPC", n), ("PID ", p)):
        _log(f"  {tag}: dv mean={s['dv_mean']:8.3f} p50={s['dv_p50']:8.3f} "
             f"p90={s['dv_p90']:8.3f} max={s['dv_max']:9.3f} m/s")
        _log(f"        tail_hi={s['tail_hi']:9.3f}  tail_lo={s['tail_lo']:8.3f}"
             f"  feasible={s['feasible_frac'] * 100:5.1f}% "
             f"({s['n_infeasible']} infeasible)  box mean="
             f"{s['box_mean'] / 1e3:.2f} km worst={s['box_max'] / 1e3:.2f} km")
    _log("\n  parameters:")
    for k, v in sorted(b["params"].items()):
        _log(f"    {k:<18} {v:.6g}")
    hs = [h["best_cost"] for h in report.history]
    if len(hs) > 1:
        _log(f"\n  best-cost trace: {hs[0]:.1f} -> {min(hs):.1f} "
             f"({(hs[0] - min(hs)) / max(hs[0], 1e-9) * 100:+.1f}%)")


__all__ = ["main", "parse_args", "seed_theta_from_results", "OUTDIR"]


if __name__ == "__main__":
    sys.exit(main())