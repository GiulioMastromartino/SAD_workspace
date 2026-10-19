"""One global CMA-ES over the attitude controller, scored by Monte Carlo.

The three maneuvers -- detumble, slew, sun-pointing -- are the mission; the
controller under test is the MRP PD (``t = -K*e - P*fb``) with its rate/bias
observer. Every candidate runs the whole mission once per Monte Carlo run, in
a single ``run_chunk`` call whose batch lanes are the runs.

    generation
      -> CMA-ES ask()            50 vectors from the 5-D hypercube
      -> for each vector          50-run mission campaign through the full C++
                                  pipeline, against one frozen design shared
                                  by every candidate
      -> reduce                 mean + weighted worst/best deciles + infeasible
      -> CMA-ES tell()

Wall time, measured at full fidelity (50 Hz, 300 s mission, tol 1e-12):

    one candidate (50 runs)  ~4.7 s
    one 50-candidate generation, 8 workers   ~1.5-2 min

Usage (from Python/)::

    ./env/bin/python -m cubesat_12u_sim.tuning.run_att_optimize
    ./env/bin/python -m cubesat_12u_sim.tuning.run_att_optimize \\
        --generations 25 --popsize 50 --workers 8
    ./env/bin/python -m cubesat_12u_sim.tuning.run_att_optimize --resume
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.optimization.attdriver import run_sweep
from cubesat_12u_sim.optimization.attobjective import AttWeights
from cubesat_12u_sim.optimization.attparams import ATT_SPACE
from cubesat_12u_sim.uncertainty.monte_carlo import MonteCarloConfig

OUTDIR = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                      "..", "..", "results_att_optimize"))


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def _write(name: str, payload: dict) -> str:
    os.makedirs(OUTDIR, exist_ok=True)
    path = os.path.join(OUTDIR, name)
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True, default=float)
    return path


def resolve_backend() -> None:
    """Fail loudly if the C++ campaign driver is missing.

    There is no numpy fallback for this pipeline: the Python batch backend
    costs ~100 ms per run per maneuver against ~50-90 ms per whole mission in
    C++, so a sweep would run for days. A clear error at startup beats a sweep
    that appears to progress.
    """
    try:
        import heyoka  # noqa: F401
        from cubesat_12u_sim.native import cxxdrive as _cxx  # noqa: F401
        if not _cxx.available:
            raise ImportError("cxxdrive reports unavailable")
    except Exception as exc:
        raise SystemExit(
            f"[fatal] the full C++ campaign driver is unavailable "
            f"({type(exc).__name__}: {exc}). Build the extension first.")


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--generations", type=int, default=25)
    ap.add_argument("--popsize", type=int, default=50)
    ap.add_argument("--restarts", type=int, default=0,
                    help="extra CMA-ES runs, each seeded from the previous "
                         "run's best theta with a fresh step size")
    ap.add_argument("--restart-popsize-factor", type=float, default=2.0,
                    help="population multiplier per restart (IPOP rule)")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 4) - 2))
    ap.add_argument("--seed", type=int, default=20261007,
                    help="base seed for both the design and CMA-ES")
    ap.add_argument("--sigma0", type=float, default=0.3)
    ap.add_argument("--dampfac", type=float, default=0.3,
                    help="CMA-ES step-size damping multiplier")

    ap.add_argument("--runs", type=int, default=50,
                    help="Monte Carlo runs per candidate (default: 50)")
    ap.add_argument("--task-step", type=float, default=0.02,
                    help="control/plant step [s] (default: 0.02, the flown rate)")
    ap.add_argument("--tol", type=float, default=C.HEYOKA_TOL,
                    help="Heyoka Taylor tolerance for the campaign")

    ap.add_argument("--w-time", type=float, default=5.0,
                    help="cost of using a full phase to settle, in grams")
    ap.add_argument("--w-sun", type=float, default=50.0,
                    help="weight on leaving the sun keep-in cone")
    ap.add_argument("--w-hi", type=float, default=0.5)
    ap.add_argument("--w-lo", type=float, default=0.5)
    ap.add_argument("--w-infeasible", type=float, default=1e3)
    ap.add_argument("--tail-frac", type=float, default=0.1)

    ap.add_argument("--checkpoint", default="ckpt_att.json")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--no-plot", action="store_true", help="skip figures")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    resolve_backend()

    _log("=== attitude-loop global hypercube sweep (CMA-ES, MC in the loop) ===")
    _log(f"  hypercube: {len(ATT_SPACE)} dimensions")
    for line in ATT_SPACE.describe().splitlines():
        _log(line)
    mc = MonteCarloConfig()
    _log(f"  mission: detumble {mc.t_detumble:.0f}s -> slew {mc.t_slew:.0f}s "
         f"-> sun-point {mc.t_sunpoint:.0f}s (total {mc.t_total:.0f}s)")
    _log(f"  thresholds: rate {mc.rate_settle_thresh_deg_s} deg/s, slew "
         f"{mc.slew_settle_thresh_deg} deg, cone {mc.sun_cone_angle_deg} deg")
    _log(f"  popsize={args.popsize} generations={args.generations} "
         f"restarts={args.restarts} workers={args.workers} seed={args.seed}")

    weights = AttWeights(w_time=args.w_time, w_sun=args.w_sun,
                         w_hi=args.w_hi, w_lo=args.w_lo,
                         w_infeasible=args.w_infeasible,
                         tail_frac=args.tail_frac)
    # eval_cfg splats directly into evaluate_candidate's kwargs, so it must
    # contain exactly those (plus nothing else). The bundle is built from the
    # same two numbers, so resolution cannot drift between them.
    log_every = max(1, int(round(float(1.0) / args.task_step)))
    eval_cfg = {"dt": args.task_step, "log_every": log_every,
                "tol": args.tol, "quantize": True, "g0": C.G0}

    ckpt = os.path.join(OUTDIR, args.checkpoint) if args.checkpoint else None
    if ckpt:
        os.makedirs(OUTDIR, exist_ok=True)

    runs = []
    tags = []
    cur_theta = ATT_SPACE.seed_theta()
    pop = int(round(args.popsize))
    r = 0
    while True:
        ck = f"{ckpt}.r{r}" if ckpt else None
        tag = "seed" if r == 0 else f"restart #{r} (popsize {pop})"
        runs.append(None)
        tags.append(tag)
        _log(f"=== sweep {r + 1}: {tag} ===")
        rep = run_sweep(generations=args.generations, popsize=pop,
                        seed=args.seed + r, sigma0=args.sigma0,
                        dampfac=args.dampfac, workers=args.workers,
                        n_runs=args.runs, base_seed=args.seed,
                        mc_cfg=MonteCarloConfig(), weights=weights,
                        eval_cfg=eval_cfg, seed_theta=cur_theta,
                        checkpoint=ck, resume=args.resume)
        runs[-1] = rep
        cur_theta = np.asarray(rep.best_theta, float)
        _log(f"  sweep {r + 1} best cost {rep.best_cost:.2f}  "
             f"({rep.wall_s / 60:.1f} min)")
        if r >= args.restarts:
            break
        r += 1
        pop = int(round(args.popsize * args.restart_popsize_factor ** r))
        _log(f"  -> restart #{r} with popsize {pop}")

    def _feas(rep):
        return rep.best["stats"]["feasible_frac"] > 0.0

    report = min(runs, key=lambda x: (0 if _feas(x) else 1, x.best_cost))
    _summarise(report)

    payload = report.as_dict()
    payload["cli"] = vars(args)
    payload["restarts"] = [dict(x.as_dict(), reason=t)
                           for x, t in zip(runs, tags)]
    payload["cli_cmd"] = [sys.executable, "-m",
                          "cubesat_12u_sim.tuning.run_att_optimize"
                          ] + list(sys.argv[1:])
    path = _write("res_att_optimize.json", payload)
    _log(f"\nwrote {path}  ({report.wall_s / 60:.1f} min)")

    if not args.no_plot:
        try:
            from cubesat_12u_sim.optimization.attplots import make_figures
            for f in make_figures(payload, OUTDIR):
                _log(f"[info] wrote {f}")
        except Exception as exc:
            _log(f"  note: figures skipped ({type(exc).__name__}: {exc})")
    return 0


def _summarise(report) -> None:
    st = report.best["stats"]
    _log(f"\n=== best over {len(report.history)} generation(s) ===")
    _log(f"  cost {report.best_cost:.3f}")
    _log(f"  fuel mean={st['fuel_mean']:8.3f}g p50={st['fuel_p50']:8.3f} "
         f"p90={st['fuel_p90']:8.3f} max={st['fuel_max']:9.3f}g")
    _log(f"  tail_hi={st['tail_hi']:9.3f}  tail_lo={st['tail_lo']:8.3f}  "
         f"feasible={st['feasible_frac'] * 100:5.1f}% "
         f"({st['n_infeasible']} infeasible)")
    _log(f"  det settle={st['det_settle_mean']:6.1f}s  "
         f"slew settle={st['slew_settle_mean']:6.1f}s  "
         f"sun max={st['sun_max_mean']:6.2f} deg")
    _log("\n  parameters:")
    for k, v in sorted(report.best["params"].items()):
        _log(f"    {k:<18} {v:.6g}")


__all__ = ["main", "parse_args", "OUTDIR"]


if __name__ == "__main__":
    sys.exit(main())
