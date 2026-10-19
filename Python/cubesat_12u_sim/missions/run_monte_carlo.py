"""CLI runner script for the 12U CubeSat Whole Mission Monte Carlo simulation.

Usage:
  python -m cubesat_12u_sim.missions.run_monte_carlo
  python -m cubesat_12u_sim.missions.run_monte_carlo --runs 50 --save-dir results_mc
  python -m cubesat_12u_sim.missions.run_monte_carlo --t-detumble 100 --t-slew 100 --t-sunpoint 150
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from cubesat_12u_sim.uncertainty.monte_carlo import (
    MonteCarloConfig,
    plot_monte_carlo,
    run_monte_carlo,
    save_monte_carlo_results,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monte Carlo simulation of the 12U CubeSat mission (Detumble -> Slew -> Sunpoint) using thrusters only.",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=30,
        help="Number of Monte Carlo simulation runs (default: 30)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=100,
        help="Base random seed for reproducible dispersions (default: 100)",
    )
    parser.add_argument(
        "--t-detumble",
        type=float,
        default=80.0,
        help="Duration of Maneuver 1 (Detumble) [s] (default: 80.0)",
    )
    parser.add_argument(
        "--t-slew",
        type=float,
        default=80.0,
        help="Duration of Maneuver 2 (Slew) [s] (default: 80.0)",
    )
    parser.add_argument(
        "--t-sunpoint",
        type=float,
        default=140.0,
        help="Duration of Maneuver 3 (Sun-Pointing) [s] (default: 140.0)",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="results_mc",
        help="Directory to save summary JSON, NPZ, and generated plots (default: 'results_mc')",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable automatic plot generation.",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=1,
        help="Process-pool workers (default: 1; Basilisk sims are process-safe).",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    mc_cfg = MonteCarloConfig(
        t_detumble=args.t_detumble,
        t_slew=args.t_slew,
        t_sunpoint=args.t_sunpoint,
    )

    results = run_monte_carlo(
        num_runs=args.runs,
        base_seed=args.seed,
        mc_cfg=mc_cfg,
        verbose=True,
        n_jobs=args.jobs,
    )

    out_dir = Path(args.save_dir)
    save_monte_carlo_results(results, output_dir=out_dir)
    print(f"\n[info] Saved Monte Carlo data to: {out_dir.resolve()}")

    if not args.no_plot:
        plots = plot_monte_carlo(results, output_dir=out_dir)
        print(f"[info] Generated {len(plots)} publication figures:")
        for p in plots:
            print(f"  - {p}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
