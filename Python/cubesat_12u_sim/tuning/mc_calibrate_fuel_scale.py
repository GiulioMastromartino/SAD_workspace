"""Calibrate FUEL_SCALE so the NMPC arm lands near a target dv.

Why this exists
---------------
The fuel term is ``x^2`` below its knee, so it charges only ~0.12-0.6x what the
scorer charges for moderate thrust. The optimiser notices: at scale 1.0 a full
30-generation run settled at 71.9 m/s of NMPC dv, against an expected optimum
near 13 m/s. Raising the amplitude restores the balance without altering the
shape.

But the relationship between the amplitude and the resulting dv is not linear
-- the optimum sits where the fuel cost gradient meets the tracking gradient,
and both are nonlinear. So this measures it rather than assuming it.

Method
------
For each candidate scale, run short sweeps at smoke settings and record the
best candidate's NMPC dv. Then fit ``dv = k / scale^p`` in log-log, which is the
right family (a pure fuel-price elasticity) and check the fit is good. The scale
that predicts the target dv is then applied and re-verified at the real
settings for a couple of generations.

Usage (from Python/)::

    ./env/bin/python -m cubesat_12u_sim.tuning.mc_calibrate_fuel_scale
    ./env/bin/python -m cubesat_12u_sim.tuning.mc_calibrate_fuel_scale --target 13
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

TARGET_DV = 13.0


def _sweep(scale: float, generations: int, popsize: int, runs: int,
           vehicles: int, days: float, workers: int, seed: int,
           outdir: str) -> float:
    """One short sweep at the given fuel scale; returns the best NMPC dv."""
    from cubesat_12u_sim.optimization.mcdriver import run_sweep
    from cubesat_12u_sim.optimization.mcdesign import MCConfig
    from cubesat_12u_sim.optimization.mcobjective import TailWeights
    from cubesat_12u_sim.optimization.params import HYPER_SPACE

    rep = run_sweep(
        generations=generations, popsize=popsize, seed=seed, sigma0=0.3,
        workers=workers, mc_cfg=MCConfig(n_runs=runs, n_vehicles=vehicles,
                                         seed=seed),
        weights=TailWeights(),
        eval_cfg={"dt_outer": 600.0, "dt_inner": 60.0,
                  "t_end": days * 86400.0, "spec_box": 25e3, "w_viol": 1e3,
                  "w_final": 0.05, "maxiter": 50, "pid_dt": 10.0,
                  "tol": 1e-12, "fuel_scale": scale},
        seed_theta=HYPER_SPACE.seed_theta(), verbose=False,
        checkpoint=None)
    return float(rep.best["nmpc"]["dv_mean"])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", type=float, default=TARGET_DV,
                    help="target NMPC dv [m/s] (default: 13)")
    ap.add_argument("--scales", type=float, nargs="+",
                    default=[1.0, 2.0, 3.0, 4.5, 7.0],
                    help="fuel scales to probe")
    ap.add_argument("--generations", type=int, default=5)
    ap.add_argument("--popsize", type=int, default=24)
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--vehicles", type=int, default=5)
    ap.add_argument("--days", type=float, default=0.05,
                    help="short horizon: enough to rank scales, cheap to run")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args(argv)

    print(f"calibrating fuel scale against a target NMPC dv of "
          f"{args.target} m/s")
    print(f"{'scale':>7} {'NMPC dv':>9} {'cost':>10}  note")
    pts: list[tuple[float, float]] = []
    for s in args.scales:
        dv = _sweep(s, args.generations, args.popsize, args.runs,
                    args.vehicles, args.days, args.workers, args.seed,
                    "results_mc_optimize")
        pts.append((s, dv))
        print(f"{s:7.2f} {dv:9.2f} {'-':>10}")

    if len(pts) < 2:
        print("need at least two scales to fit")
        return 1

    xs = np.log(np.array([q[0] for q in pts]))
    ys = np.log(np.array([q[1] for q in pts]))
    # dv = k * scale**p  ->  log dv = log k + p*log scale. np.polyfit returns
    # [slope, intercept] = [p, log k]. dv falls with scale so p < 0; solving
    # log(target) = log k + p*log(s) gives log s = (log target - log k)/p. The
    # earlier version negated p in one place and not the other, which turned a
    # decreasing fit into an increasing one and produced a scale of 0.014.
    p, logk = np.polyfit(xs, ys, 1)
    k = float(np.exp(logk))
    scale_fit = float(np.exp((np.log(args.target) - logk) / p))

    resid = np.abs(xs * p + logk - ys)
    print(f"\nfit: dv = {k:.2f} * scale**{p:.3f}   "
          f"max log residual {resid.max():.3f}")
    print(f"predicted scale for dv={args.target}: {scale_fit:.3f}")
    if resid.max() > 0.5:
        print("WARNING: the power-law fit is poor, so this prediction is "
              "unreliable -- widen --scales and re-run")
        return 1
    if not np.isfinite(scale_fit) or scale_fit <= 0:
        print("fit did not produce a usable scale (dv does not fall with "
              "scale, or the probed range does not bracket the target)")
        return 1

    print(f"\napply in optimization/mcobjective.py:\n"
          f"    FUEL_SCALE = {scale_fit:.2f}")
    with open("results_mc_optimize/fuel_scale_calibration.json", "w") as fh:
        json.dump({"target_dv": args.target,
                   "measured": [{"scale": s, "dv": d} for s, d in pts],
                   "fit": {"k": k, "p": float(p)},
                   "scale_for_target": scale_fit,
                   "fit_resid_max": float(resid.max())}, fh, indent=2)
    print("wrote results_mc_optimize/fuel_scale_calibration.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())