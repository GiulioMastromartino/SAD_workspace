"""Progress monitor for a running MC hypercube sweep.

Reads the checkpoint the sweep writes after every generation
(``ckpt_mc.json`` in the results dir) and reports where it is, how fast it is
moving, and when it should finish.

Safe to run at any time: the checkpoint is written atomically via a temp file
and rename, so a read either sees the previous generation or the new one, never
a half-written file.

Usage (from Python/)::

    ./env/bin/python -m cubesat_12u_sim.tuning.mc_progress
    ./env/bin/python -m cubesat_12u_sim.tuning.mc_progress --every 60 --watch
    ./env/bin/python -m cubesat_12u_sim.tuning.mc_progress --trace 20
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

DEFAULT_CKPT = os.path.join(
    os.path.dirname(__file__), "..", "..", "results_mc_optimize", "ckpt_mc.json")


def dirname_of(p: str) -> str:
    return os.path.dirname(os.path.abspath(p))


def _n_runs_from(d: str) -> int | None:
    """Monte Carlo run count, from the sweep's own results JSON if it landed.

    The checkpoint does not carry the design's run count, and MS_PER_CANDIDATE
    is calibrated at 50 runs, so without this the "implied CPU work" line would
    be silently wrong for any --runs value other than 50.
    """
    res = os.path.join(d, "res_mc_optimize.json")
    if not os.path.exists(res):
        return None
    try:
        with open(res) as fh:
            return int(json.load(fh)["config"]["mc"]["n_runs"])
    except Exception:
        return None

# Measured seconds per candidate per horizon, for the reference configuration
# this monitor was calibrated against: 50 MC runs, 0.5 d/run, 4 workers.
#
# Only meaningful for THAT configuration. At the cheap smoke settings (10 runs,
# 0.02 d) a whole generation costs under a second and every figure below rounds
# to 0, so a non-zero total here is not evidence of a problem -- and, more
# importantly, a zero here is not evidence of a bug. The ETA is computed from
# the measured per-generation wall clock, which is correct at any settings;
# these constants are only used for the secondary "implied CPU work" line.
MS_PER_CANDIDATE = {2: 1.4, 4: 3.8, 6: 7.2}


def load(path: str) -> dict:
    with open(path) as fh:
        return json.load(fh)


def _fmt_eta(seconds: float) -> str:
    if seconds <= 0 or seconds != seconds or seconds == float("inf"):
        return "unknown"
    m, s = divmod(int(seconds), 60)
    if m < 60:
        return f"{m}m {s:02d}s"
    h, m = divmod(m, 60)
    return f"{h}h {m:02d}m"


def _int_keys(hc: dict) -> dict[int, int]:
    """Horizon counts keyed by int.

    JSON object keys are always strings, so a checkpoint read back from disk has
    {"2": 32} where the in-memory record had {2: 32}. Looking the horizon up
    without coercing silently yields 0 for every key, which is how this script
    first reported "implied CPU work 0s" and a 12-hour ETA on a run that was
    actually tracking ~45 minutes.
    """
    out: dict[int, int] = {}
    for k, v in (hc or {}).items():
        try:
            out[int(k)] = int(v)
        except (TypeError, ValueError):
            continue
    return out


def report(path: str, total_gens: int | None = None, trace: int = 0) -> int:
    if not os.path.exists(path):
        print(f"no checkpoint at {path}\n"
              f"(the sweep writes it after the first generation completes)",
              file=sys.stderr)
        return 1

    try:
        d = load(path)
    except (json.JSONDecodeError, OSError) as exc:
        # The sweep renames into place, so this should not happen; if the file
        # is genuinely mid-write, say so instead of reporting a false zero.
        print(f"checkpoint unreadable ({type(exc).__name__}: {exc}); "
              f"the sweep may be writing it right now", file=sys.stderr)
        return 1

    hist = d.get("history", [])
    done = int(d.get("gen", len(hist)))
    if not hist:
        print(f"checkpoint has no completed generations yet "
              f"(gen counter {done}); first generation still running")
        return 0
    for h in hist:
        h["horizon_counts"] = _int_keys(h.get("horizon_counts"))

    last = hist[-1]
    walls = [h["wall_s"] for h in hist]
    total = total_gens or _total_from_config(path)

    print(f"\n{'=' * 68}")
    print(f"  generations {done}/{total or '?'}   "
          f"{'done' if total and done >= total else 'running'}")
    print(f"  evals {last['evals']}   last gen wall {last['wall_s']:.1f}s")
    print(f"{'=' * 68}")

    # ---- convergence -------------------------------------------------
    best = [h["best_cost"] for h in hist]
    mean = [h["mean_cost"] for h in hist]
    print(f"\n  best   {best[0]:9.1f} -> {min(best):9.1f}   "
          f"({(best[0] - min(best)) / max(best[0], 1e-9) * 100:+.1f}%)")
    print(f"  mean   {mean[0]:9.1f} -> {mean[-1]:9.1f}")
    if len(hist) > 1 and best[0] > 0:
        print(f"  spread (last gen mean/best) "
              f"{mean[-1] / max(best[-1], 1e-12):.2f}x")

    if trace:
        k = max(1, len(hist) // trace)
        print("\n  gen   best      mean     H   wall/s   horizon mix")
        for h in hist[::k] + ([hist[-1]] if len(hist) % k else []):
            hc = h.get("horizon_counts", {})
            mix = " ".join(f"H{hh}:{n}" for hh, n in sorted(hc.items()))
            if not mix:
                mix = f"H{h['best_horizon']} (best only)"
            print(f"  {h['gen']:4d} {h['best_cost']:8.1f} {h['mean_cost']:8.1f} "
                  f"  {h['best_horizon']:2d} {h['wall_s']:6.1f}   {mix}")

    # ---- horizon mix: the thing that decides wall clock --------------
    hc = last.get("horizon_counts", {})
    if hc:
        tot = sum(hc.values())
        print(f"\n  horizon mix (last gen, {tot} candidates): "
              + "  ".join(f"H{h} {100 * n / tot:.0f}%" for h, n in sorted(hc.items())))
        known = {h: n for h, n in hc.items() if h in MS_PER_CANDIDATE}
        if known and sum(known.values()) == tot:
            # Per-candidate cost scales linearly with n_runs, so the table --
            # calibrated at 50 runs -- must be rescaled or the figure is wrong
            # for any other --runs setting.
            n_runs = _n_runs_from(dirname_of(path))
            scale = (n_runs / 50.0) if n_runs else 1.0
            cpu = (sum(MS_PER_CANDIDATE[h] * n for h, n in known.items())
                   / 1000 * scale)
            if cpu >= 1.0:
                print(f"  implied CPU work {cpu:.0f}s -> {cpu / 4:.0f}s on 4 "
                      f"workers balanced (measured {last['wall_s']:.0f}s: the "
                      f"difference is the straggler penalty)")
            else:
                print(f"  (run is far below the calibration settings for the "
                      f"per-candidate cost table; using the measured "
                      f"{last['wall_s']:.0f}s wall clock instead)")
        else:
            # A horizon outside the calibrated set would make the figure a
            # guess, so say that rather than printing a number built on a
            # default of zero.
            miss = sorted(set(hc) - set(MS_PER_CANDIDATE))
            print(f"  (per-candidate cost not calibrated for {miss}; "
                  f"using the measured wall clock instead)")

    # ---- ETA ---------------------------------------------------------
    # Ground truth is the measured wall clock of the generations actually run.
    # It is preferred over anything derived from MS_PER_CANDIDATE because it
    # already contains the real straggler penalty, which is why it comes in
    # above the perfectly-balanced figure. The mix is reported alongside so a
    # drift toward costlier horizons is visible as a rising trend rather than
    # silently baked into the estimate.
    recent = walls[-min(5, len(walls)):]
    avg = sum(recent) / len(recent)
    if total:
        left = (total - done) * avg
        trend = ""
        if len(recent) >= 4 and recent[-1] > recent[0] * 1.5:
            trend = (f"  (rising: {recent[0]:.0f}s -> {recent[-1]:.0f}s, "
                     f"likely the horizon mix shifting to H=4/H=6)")
        print(f"\n  ~{avg:.0f}s per generation (mean of last {len(recent)})"
              f"{trend}")
        print(f"  ETA {_fmt_eta(left)}  ({total - done} generations left)")

    # ---- health ------------------------------------------------------
    infeas = [h.get("n_infeasible_pop", 0) for h in hist[-5:]]
    if any(infeas):
        print(f"\n  WARNING: {sum(infeas)} candidates with no feasible run in "
              f"the last {len(infeas)} generation(s)")
    else:
        print(f"\n  healthy: every candidate in the last "
              f"{len(infeas)} generation(s) had at least one feasible run")
    return 0


def _total_from_config(path: str) -> int | None:
    """Total generations, from the run's own results JSON if it has landed."""
    res = os.path.join(os.path.dirname(path), "res_mc_optimize.json")
    if not os.path.exists(res):
        return None
    try:
        with open(res) as fh:
            return int(json.load(fh)["config"]["generations"])
    except Exception:
        return None


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