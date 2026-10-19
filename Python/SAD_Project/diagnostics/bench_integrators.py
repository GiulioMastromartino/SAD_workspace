#!/usr/bin/env python3
"""Benchmark the RK4 and Heyoka backends of the SAD mission batch kernel.

Both backends run the same C++ mission loop, 12-thruster allocation, quaternion
PD, mission sequencer and dispersions.  Only the 13-state plant integration is
different:

    RK4     fixed step, 0.1 s detumble/slew and ``TRACK_DT`` track
    Heyoka  adaptive Taylor method, one ``propagate_for(dt)`` per control step

The first Heyoka call pays the JIT compilation cost; the script warms both
backends before timing so the numbers reflect steady-state throughput.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

# diagnostics -> SAD_Project -> Python
SAD_PROJECT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = SAD_PROJECT.parent
sys.path.insert(0, str(SAD_PROJECT / "python"))
sys.path.insert(0, str(PYTHON_ROOT))

import sad_py
from SAD_Project.optimization.params import DEFAULTS
from SAD_Project.optimization.objective import build_batch_params


def _summary(res):
    fuel = np.asarray(res["fuel_used"], float)
    rate = np.asarray(res["final_rate"], float)
    detumble = np.asarray(res["detumble_settle"], float)
    slew = np.asarray(res["slew_settle"], float)
    track_mean = np.asarray(res["track_ang_mean"], float)
    track_max = np.asarray(res["track_ang_max"], float)
    box = np.asarray(res["box_max"], float)
    return {
        "failures": int(np.sum(res["failed"])),
        "reached": int(np.sum(res["reached_track"])),
        "fuel_mean_kg": float(np.mean(fuel)),
        "rate_median_rad_s": float(np.nanmedian(rate)),
        "detumble_mean_s": float(np.mean(detumble)),
        "slew_mean_s": float(np.mean(slew)),
        "track_mean_deg": float(np.mean(track_mean)),
        "track_max_deg": float(np.max(track_max)),
        "box_max_km": float(np.max(box)),
        "box_mean_km": float(np.mean(box)),
    }


def _compare(rk_res, hy_res):
    """Largest pointwise KPI difference between the two backends."""
    keys = [
        "detumble_settle", "slew_settle", "final_rate", "fuel_used",
        "track_ang_mean", "track_ang_max", "orbit_err", "box_max",
    ]
    out = {}
    for key in keys:
        a = np.asarray(rk_res[key], float)
        b = np.asarray(hy_res[key], float)
        diff = np.abs(a - b)
        scale = np.maximum(np.abs(a), 1e-300)
        out[key] = {
            "max_abs": float(np.max(diff)),
            "max_rel": float(np.max(diff / scale)),
        }
    return out


def _run_backend(bp, use_heyoka, n, seed, repeats):
    bp.use_heyoka = bool(use_heyoka)
    times = []
    last = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        last = sad_py.run_mission_batch(n, seed, bp)
        times.append(time.perf_counter() - t0)
    return times, last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--N", type=int, default=300, help="mission runs per timing sample")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--tol", type=float, default=1e-12, help="Heyoka Taylor tolerance")
    ap.add_argument("--compact", action="store_true", help="Heyoka compact mode")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    if not sad_py.heyoka_available():
        raise SystemExit("sad_py was built without the Heyoka backend")

    bp, _ = build_batch_params(DEFAULTS, disperse=True, timing=False)
    bp.heyoka_tol = args.tol
    bp.heyoka_compact = args.compact

    # Warm-up: compile the Taylor system once and populate caches.
    for use in (False, True):
        bp.use_heyoka = use
        sad_py.run_mission_batch(1, args.seed, bp)

    rk_times, rk_res = _run_backend(bp, False, args.N, args.seed, args.repeats)
    hy_times, hy_res = _run_backend(bp, True, args.N, args.seed, args.repeats)

    rk = _summary(rk_res)
    hy = _summary(hy_res)
    cmp = _compare(rk_res, hy_res)
    rk_med = float(np.median(rk_times))
    hy_med = float(np.median(hy_times))

    rows = [
        ("wall time (s, median)", rk_med, hy_med, f"{hy_med / rk_med:.3f}x"),
        ("ms / run", 1000.0 * rk_med / args.N, 1000.0 * hy_med / args.N,
         f"{(hy_med / rk_med):.3f}x"),
        ("failures", rk["failures"], hy["failures"], ""),
        ("reached track", rk["reached"], hy["reached"], ""),
        ("fuel mean (kg)", rk["fuel_mean_kg"], hy["fuel_mean_kg"], ""),
        ("final rate median (rad/s)", rk["rate_median_rad_s"], hy["rate_median_rad_s"], ""),
        ("detumble mean (s)", rk["detumble_mean_s"], hy["detumble_mean_s"], ""),
        ("slew mean (s)", rk["slew_mean_s"], hy["slew_mean_s"], ""),
        ("track mean (deg)", rk["track_mean_deg"], hy["track_mean_deg"], ""),
        ("track max (deg)", rk["track_max_deg"], hy["track_max_deg"], ""),
        ("box mean (km)", rk["box_mean_km"], hy["box_mean_km"], ""),
        ("box max (km)", rk["box_max_km"], hy["box_max_km"], ""),
    ]

    print(f"N={args.N} runs, seed={args.seed}, repeats={args.repeats}, "
          f"Heyoka tol={args.tol:g}, compact={args.compact}")
    print(f"{'metric':28s} {'RK4':>14s} {'Heyoka':>14s} {'ratio':>9s}")
    print("-" * 70)
    for name, a, b, ratio in rows:
        print(f"{name:28s} {a:14.6g} {b:14.6g} {ratio:>9s}")

    print("\nKPI agreement (same seeds and dispersions):")
    print(f"{'metric':28s} {'max abs diff':>14s} {'max rel diff':>14s}")
    print("-" * 60)
    for key, value in cmp.items():
        print(f"{key:28s} {value['max_abs']:14.6g} {value['max_rel']:14.6g}")

    out = args.out
    if out is None:
        suffix = "_compact" if args.compact else ""
        out = Path(__file__).resolve().parents[1] / "results_bench" / (
            f"bench_heyoka_N{args.N}_tol{args.tol:g}{suffix}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "N": args.N,
        "seed": args.seed,
        "repeats": args.repeats,
        "heyoka_tol": args.tol,
        "heyoka_compact": bool(args.compact),
        "rk4": {"times_s": rk_times, "median_s": rk_med, "metrics": rk},
        "heyoka": {"times_s": hy_times, "median_s": hy_med, "metrics": hy},
        "comparison": cmp,
        "time_ratio_heyoka_over_rk4": hy_med / rk_med,
        "throughput_ratio_rk4_over_heyoka": rk_med / hy_med,
    }
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
