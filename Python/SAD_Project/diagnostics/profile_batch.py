#!/usr/bin/env python
"""Fine-grained profiler for the C++ mission-batch kernel.

Runs ``sad_py.run_mission_batch`` with the timing registry enabled and prints
where the wall time goes: per subsystem, per mission phase, and per MC run.
Also writes a JSON dump and a PNG bar chart.

Usage (from Python/):
    PYTHONPATH=$PWD/SAD_Project/python:$PWD \\
      env/bin/python -m SAD_Project.diagnostics.profile_batch --runs 50

Notes:
  * Timings are *inclusive* with respect to nesting; indented slot names are
    children of the slot above them (e.g. "  alloc_jacobiSVD" is inside
    "thruster_allocate"). Use the "exclusive ms" column to avoid double count.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import sad_py  # noqa: E402
from SAD_Project.core import config as C  # noqa: E402
from SAD_Project.optimization.params import DEFAULTS, theta_to_params  # noqa: E402

# ---- slot topology -------------------------------------------------------
# parent -> immediate children (used to compute exclusive time)
CHILDREN = {
    'thruster_allocate': ['  alloc_solve', '  alloc_clamp_scale'],
    'allocator_init': ['  alloc_build_A', '  alloc_jacobiSVD'],
    'dynamics_step': ['  dyn_vector_allocs', '  rhs_total(4 stages)'],
    '  rhs_total(4 stages)': ['    rhs_quat_to_dcm', '    rhs_gravity_grad', '    rhs_quat_deriv'],
    'sun_step': ['  sun_point_quat'],
    'thruster_step': ['  thr_dir_normalize'],
}
TOP_LEVEL = ['sun_step', 'startracker_step', 'mission_step', 'quatpd_step',
             'thruster_allocate', 'thruster_step', 'dynamics_step', 'kpi_bookkeeping']
PHASES = ['detumble', 'slew', 'track']


def make_bp(runs, seed, disperse=True, timing=True, track_dt=None, implicit_pd=None,
            implicit_track_only=None):
    p = theta_to_params(DEFAULTS)
    bp = sad_py.BatchParams()
    for k, v in p.items():
        setattr(bp, k, v)
    bp.R_ORBIT = C.R_ORBIT
    bp.MU = C.MU_EARTH
    bp.mass = C.MASS
    bp.disperse = disperse
    bp.timing = timing
    bp.track_dt = C.TRACK_DT if track_dt is None else float(track_dt)
    bp.implicit_pd = bool(getattr(C, 'IMPLICIT_PD', True)) if implicit_pd is None else bool(implicit_pd)
    bp.implicit_track_only = (bool(getattr(C, 'IMPLICIT_TRACK_ONLY', True))
                              if implicit_track_only is None else bool(implicit_track_only))
    return bp, p


def fmt_row(name, ms, pct, calls, ns_per_call):
    return f"  {name:<26s} {ms:>12.2f} {pct:>8.1f}% {calls:>10d} {ns_per_call:>11.1f}"


def exclusive_ms(name, ns_map):
    """Inclusive ms minus immediate-children ms."""
    inc = ns_map.get(name, 0.0) / 1e6
    kids = CHILDREN.get(name, [])
    return inc - sum(ns_map.get(c, 0.0) / 1e6 for c in kids)


def ascii_bar(fraction, width=40):
    n = int(round(max(0.0, min(1.0, fraction)) * width))
    return '#' * n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=50, help='number of MC runs')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--no-disperse', action='store_true')
    ap.add_argument('--no-timing', action='store_true', help='run without instrumentation')
    ap.add_argument('--track-dt', type=float, default=None,
                    help=f'track-phase step (s); default config TRACK_DT={C.TRACK_DT}')
    ap.add_argument('--legacy-pd', action='store_true',
                    help='use the legacy explicit PD (no implicit gain scaling)')
    ap.add_argument('--implicit-all', action='store_true',
                    help='scale detumble/slew too (default: track phase only)')
    sad_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    outdir = os.path.join(sad_dir, 'results_diagnostics')
    ap.add_argument('--json', default=os.path.join(outdir, 'batch_profile.json'))
    ap.add_argument('--png', default=os.path.join(outdir, 'batch_profile.png'))
    args = ap.parse_args()

    bp, p = make_bp(args.runs, args.seed, disperse=not args.no_disperse,
                    timing=not args.no_timing, track_dt=args.track_dt,
                    implicit_pd=not args.legacy_pd,
                    implicit_track_only=not args.implicit_all)

    t0 = time.perf_counter()
    res = sad_py.run_mission_batch(args.runs, args.seed, bp)
    wall_ms = (time.perf_counter() - t0) * 1e3

    T = res['timing']
    if not T.get('enabled'):
        print('timing disabled; rerun without --no-timing')
        return

    ns = {k: float(v) for k, v in T['ns'].items()}
    calls = {k: int(v) for k, v in T['calls'].items()}
    phase_ns = {k: [float(x) for x in v] for k, v in T['phase_ns'].items()}
    steps_total = int(T['steps'])
    phase_steps = [int(x) for x in T['phase_steps']]
    batch_ms = float(T['batch_ms'])
    run_ms = np.array(T['run_ms'], dtype=float)

    # ---------------- summary ----------------
    print('=' * 96)
    print(' SAD_Project mission-batch profile')
    print('=' * 96)
    print(f"  pybind module     : {sad_py.__file__}")
    print(f"  runs              : {args.runs}   seed={args.seed}   disperse={not args.no_disperse}")
    print(f"  batch wall (C++)  : {batch_ms:,.1f} ms   ({batch_ms/args.runs:.1f} ms/run)")
    print(f"  batch wall (py)   : {wall_ms:,.1f} ms   ({wall_ms/args.runs:.1f} ms/run)")
    print(f"  integration steps : {steps_total:,}   ({steps_total/args.runs:,.0f}/run)")
    tot_steps = max(1, sum(phase_steps))
    print(f"  steps by phase    : " + ' | '.join(
        f"{PHASES[i]} {phase_steps[i]:,} ({100*phase_steps[i]/tot_steps:.1f}%)" for i in range(3)))
    print(f"  us / step         : {1e3*batch_ms/tot_steps:.2f}")
    print(f"  reached/failed    : {T['reached_runs']}/{T['failed_runs']}")

    # ---------------- subsystem table ----------------
    print('-' * 96)
    print(' SUBSYSTEM BREAKDOWN  (inclusive; indented = child of the row above)')
    print('-' * 96)
    print(f"  {'slot':<26s} {'total ms':>12s} {'%batch':>9s} {'calls':>10s} {'ns/call':>11s}   excl.ms")
    order = ['sun_step', '  sun_point_quat', 'startracker_step', 'mission_step',
             'quatpd_step', 'thruster_allocate', '  alloc_solve', '  alloc_clamp_scale',
             'allocator_init', '  alloc_build_A', '  alloc_jacobiSVD',
             'thruster_step', '  thr_dir_normalize',
             'dynamics_step', '  dyn_vector_allocs', '  rhs_total(4 stages)',
             '    rhs_quat_to_dcm', '    rhs_gravity_grad', '    rhs_quat_deriv',
             'kpi_bookkeeping', 'dispersion', 'per_run_setup', 'loop_iteration_total']
    for name in order:
        ms = ns.get(name, 0.0) / 1e6
        nc = calls.get(name, 0)
        ns_pc = (ns.get(name, 0.0) / nc) if nc else 0.0
        pct = 100.0 * ms / batch_ms if batch_ms else 0.0
        print(fmt_row(name, ms, pct, nc, ns_pc) + f"  {exclusive_ms(name, ns):>9.2f}")

    # ---------------- phase table ----------------
    print('-' * 96)
    print(' PER-PHASE ATTRIBUTION  (top-level slots, inclusive ms)')
    print('-' * 96)
    for ph in range(3):
        ph_total = sum(phase_ns.get(s, [0, 0, 0])[ph] for s in TOP_LEVEL) / 1e6
        st = phase_steps[ph]
        print(f"  {PHASES[ph]:<9s} steps={st:>9,}  measured={ph_total:>10.1f} ms  "
              f"{1e3*ph_total/max(1,st):>8.2f} us/step")
        for s in TOP_LEVEL:
            v = phase_ns.get(s, [0, 0, 0])[ph] / 1e6
            if st and v > 0:
                print(f"      {s:<24s} {v:>10.1f} ms  ({100.0*v/max(1e-9,ph_total):>5.1f}% of phase)")

    # ---------------- per-run distribution ----------------
    if len(run_ms):
        print('-' * 96)
        print(' PER-RUN WALL TIME')
        print('-' * 96)
        print(f"  mean {run_ms.mean():.1f} ms | std {run_ms.std():.1f} ms | "
              f"min {run_ms.min():.1f} ms | p50 {np.percentile(run_ms,50):.1f} ms | "
              f"max {run_ms.max():.1f} ms")
        fast = np.sum(run_ms < 0.25 * np.median(run_ms))
        if fast:
            print(f"  note: {fast} run(s) finished in <25% of median -> early failure/abort")

    # ---------------- diagnosis ----------------
    print('-' * 96)
    print(' WHERE TO DEEP DIVE  (share of total batch time)')
    print('-' * 96)
    ranked = sorted(
        [(name, ns.get(name, 0.0) / 1e6) for name in order
         if not name.startswith(' ') and name not in
         ('loop_iteration_total', 'dispersion', 'per_run_setup')],
        key=lambda kv: -kv[1])
    for name, ms in ranked[:8]:
        pct = 100.0 * ms / batch_ms if batch_ms else 0.0
        print(f"  {name:<26s} {pct:>5.1f}%  {ascii_bar(pct/100.0)}")
    solve = ns.get('  alloc_solve', 0.0) / 1e6
    init = ns.get('allocator_init', 0.0) / 1e6
    alloc = ns.get('thruster_allocate', 0.0) / 1e6
    dyn = ns.get('dynamics_step', 0.0) / 1e6
    rhs = ns.get('  rhs_total(4 stages)', 0.0) / 1e6
    thr = ns.get('thruster_step', 0.0) / 1e6
    setup = ns.get('per_run_setup', 0.0) / 1e6
    top_sum = sum(ns.get(s, 0.0) / 1e6 for s in TOP_LEVEL)
    loop_tot = ns.get('loop_iteration_total', 0.0) / 1e6
    print()
    print(f"  allocator solve (pinv matvec): {100*solve/batch_ms:5.1f}% of batch "
          f"({100*solve/max(1e-9,alloc):.0f}% of allocation)")
    print(f"  allocator init (SVD, 1/run) : {100*init/batch_ms:5.1f}% of batch")
    print(f"  allocation total            : {100*alloc/batch_ms:5.1f}%")
    print(f"  dynamics total              : {100*dyn/batch_ms:5.1f}%  (rhs {100*rhs/batch_ms:.1f}%)")
    print(f"  thruster_step               : {100*thr/batch_ms:5.1f}%")
    print(f"  per-run setup (amortized)   : {100*setup/batch_ms:5.1f}%")
    print(f"  loop residual (unattributed): {100*max(0.0,loop_tot-top_sum)/batch_ms:5.1f}%")
    print()
    print('  Reading: any row >20% is a first-order optimization target;')
    print('  children marked with indentation are the levers inside that row.')

    # ---------------- save ----------------
    os.makedirs(os.path.dirname(args.json) or '.', exist_ok=True)
    payload = {
        'runs': args.runs, 'seed': args.seed, 'disperse': not args.no_disperse,
        'batch_ms': batch_ms, 'py_wall_ms': wall_ms,
        'track_dt': bp.track_dt,
        'implicit_pd': bool(bp.implicit_pd),
        'implicit_track_only': bool(bp.implicit_track_only),
        'steps': steps_total, 'phase_steps': phase_steps,
        'us_per_step': 1e3 * batch_ms / max(1, tot_steps),
        'ns': ns, 'calls': calls, 'phase_ns': phase_ns,
        'run_ms': list(run_ms),
        'params': p,
    }
    with open(args.json, 'w') as f:
        json.dump(payload, f, indent=2)
    print(f"\n  wrote {args.json}")

    # ---------------- png ----------------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        rows = [(n, ns.get(n, 0.0) / 1e6) for n in order if ns.get(n, 0.0) > 0]
        rows.sort(key=lambda kv: kv[1])
        labels = [r[0].strip() for r in rows]
        vals = [r[1] for r in rows]
        colors = ['#d95f02' if not r[0].startswith(' ') else '#9ecae1' for r in rows]
        fig, ax = plt.subplots(figsize=(10, 0.34 * len(rows) + 1.5))
        ax.barh(labels, vals, color=colors)
        ax.set_xlabel('inclusive time (ms)')
        ax.set_title(f'Mission-batch profile ({args.runs} runs, {batch_ms/1000:.1f}s total)')
        for i, v in enumerate(vals):
            ax.text(v, i, f'  {100*v/batch_ms:.1f}%', va='center', fontsize=8)
        ax.grid(True, axis='x', alpha=0.3)
        fig.tight_layout()
        os.makedirs(os.path.dirname(args.png) or '.', exist_ok=True)
        fig.savefig(args.png, dpi=150)
        plt.close(fig)
        print(f"  wrote {args.png}")
    except Exception as e:
        print(f"  png skipped: {e}")


if __name__ == '__main__':
    main()
