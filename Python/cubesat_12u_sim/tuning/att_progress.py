"""Progress monitor for a running attitude-loop sweep.

Reads the checkpoint the sweep writes after every generation and reports where
it is, how fast it is moving, and when it should finish.

Safe to run at any time: the checkpoint is written atomically via a temp file
and rename, so a read either sees the previous generation or the new one, never
a half-written file.

Usage (from Python/)::

    ./env/bin/python -m cubesat_12u_sim.tuning.att_progress --gens 25
    ./env/bin/python -m cubesat_12u_sim.tuning.att_progress --watch
    ./env/bin/python -m cubesat_12u_sim.tuning.att_progress \\
        --ckpt results_att_optimize/ckpt_att.json.r1 --gens 25 --trace 10
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

DEFAULT_CKPT = os.path.join(
    os.path.dirname(__file__), "..", "..", "results_att_optimize",
    "ckpt_att.json")


def _fmt_eta(seconds: float) -> str:
    if seconds <= 0 or seconds != seconds or seconds == float("inf"):
        return "unknown"
    m, s = divmod(int(seconds), 60)
    if m < 60:
        return f"{m}m {s:02d}s"
    h, m = divmod(m, 60)
    return f"{h}h {m:02d}m"


def report(path: str, total_gens: int | None = None, trace: int = 0) -> int:
    if not os.path.exists(path):
        print(f"no checkpoint at {path}\n"
              f"(the sweep writes it after the first generation completes)",
              file=sys.stderr)
        return 1

    try:
        with open(path) as fh:
            d = json.load(fh)
    except (json.JSONDecodeError, OSError) as exc:
        print(f"checkpoint unreadable ({type(exc).__name__}: {exc}); "
              f"the sweep may be writing it right now", file=sys.stderr)
        return 1

    hist = d.get("history", [])
    done = int(d.get("gen", len(hist)))
    if not hist:
        print(f"checkpoint has no completed generations yet "
              f"(gen counter {done}); first generation still running")
        return 0

    last = hist[-1]
    walls = [h["wall_s"] for h in hist]

    print(f"\n{'=' * 68}")
    print(f"  generations {done}/{total_gens or '?'}   "
          f"{'done' if total_gens and done >= total_gens else 'running'}")
    print(f"  evals {last['evals']}   last gen wall {last['wall_s']:.1f}s")
    print(f"{'=' * 68}")

    best = [h["best_cost"] for h in hist]
    mean = [h["mean_cost"] for h in hist]
    print(f"\n  best   {best[0]:9.1f} -> {min(best):9.1f}   "
          f"({(best[0] - min(best)) / max(best[0], 1e-9) * 100:+.1f}%)")
    print(f"  mean   {mean[0]:9.1f} -> {mean[-1]:9.1f}")
    if len(hist) > 1 and best[-1] > 0:
        print(f"  spread (last gen mean/best) "
              f"{mean[-1] / max(best[-1], 1e-12):.2f}x")

    if trace:
        k = max(1, len(hist) // trace)
        print("\n  gen   best      mean     feas   fuel[g]  wall/s")
        rows = hist[::k] + ([hist[-1]] if len(hist) % k else [])
        for h in rows:
            st = h.get("best", {}).get("stats", {})
            print(f"  {h['gen']:4d} {h['best_cost']:8.1f} {h['mean_cost']:8.1f} "
                  f"  {st.get('feasible_frac', float('nan')):4.2f}  "
                  f"{st.get('fuel_mean', float('nan')):7.3f}  "
                  f"{h['wall_s']:6.1f}")

    recent = walls[-min(5, len(walls)):]
    avg = sum(recent) / len(recent)
    if total_gens:
        left = (total_gens - done) * avg
        trend = ""
        if len(recent) >= 4 and recent[-1] > recent[0] * 1.5:
            trend = (f"  (rising: {recent[0]:.0f}s -> {recent[-1]:.0f}s)")
        print(f"\n  ~{avg:.0f}s per generation (mean of last {len(recent)})"
              f"{trend}")
        print(f"  ETA {_fmt_eta(left)}  ({total_gens - done} generations left)")

    infeas = [h.get("n_infeasible_pop", 0) for h in hist[-5:]]
    if any(infeas):
        print(f"\n  WARNING: {sum(infeas)} candidates with no successful run "
              f"in the last {len(infeas)} generation(s)")
    else:
        print(f"\n  healthy: every candidate in the last "
              f"{len(infeas)} generation(s) had at least one successful run")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ckpt", default=DEFAULT_CKPT, help="checkpoint path")
    ap.add_argument("--gens", type=int, default=None,
                    help="total generations, for the progress fraction and ETA")
    ap.add_argument("--trace", type=int, default=0,
                    help="print a trace of this many generations (0 = off)")
    ap.add_argument("--every", type=float, default=0.0,
                    help="repeat every N seconds instead of exiting once")
    ap.add_argument("--watch", action="store_true",
                    help="same as --every 60")
    args = ap.parse_args(argv)

    every = 60.0 if args.watch else args.every
    while True:
        report(args.ckpt, args.gens, args.trace)
        if every <= 0:
            return 0
        try:
            time.sleep(every)
        except KeyboardInterrupt:
            return 0


if __name__ == "__main__":
    sys.exit(main())
