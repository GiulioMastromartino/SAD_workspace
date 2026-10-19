#!/usr/bin/env python
"""End-to-end profiler for the optimization pipeline.

Dissects:
  1. Python/module import cost
  2. one ``evaluate()`` (C++ batch kernel vs. Python-side overhead)
  3. a real CMA-ES generation with a process pool (ask / map / tell split,
     worker-sum vs. wall time -> parallel efficiency)
  4. projected wall time for a full run at different worker counts

Usage (from Python/):
    PYTHONPATH=$PWD/SAD_Project/python:$PWD \\
      env/bin/python -m SAD_Project.diagnostics.profile_pipeline --workers 8
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SAD_DIR = os.path.dirname(HERE)              # Python/SAD_Project
PY_DIR = os.path.dirname(SAD_DIR)            # Python/

sys.path.insert(0, os.path.join(SAD_DIR, 'python'))
sys.path.insert(0, SAD_DIR)
sys.path.insert(0, PY_DIR)


def section(title):
    print('\n' + '=' * 96)
    print(f' {title}')
    print('=' * 96)


def measure_import_cost():
    code = ("import time; t=time.perf_counter(); import sad_py; "
            "print((time.perf_counter()-t)*1e3)")
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join([os.path.join(SAD_DIR, 'python'),
                                         SAD_DIR, PY_DIR])
    try:
        out = subprocess.check_output([sys.executable, '-c', code], env=env,
                                      stderr=subprocess.DEVNULL, timeout=120)
        return float(out.strip())
    except Exception as e:
        return float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=50, help='MC runs per evaluation')
    ap.add_argument('--pop', type=int, default=15, help='CMA-ES population')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--skip-parallel', action='store_true')
    ap.add_argument('--cprofile', action='store_true')
    args = ap.parse_args()

    section('ENVIRONMENT')
    import sad_py
    from SAD_Project.core import config as C
    from SAD_Project.optimization.params import DEFAULTS
    from SAD_Project.optimization.objective import evaluate
    print(f"  python        : {sys.version.split()[0]}  ({sys.executable})")
    print(f"  sad_py        : {sad_py.__file__}")
    st = os.stat(sad_py.__file__)
    print(f"  sad_py mtime  : {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(st.st_mtime))}"
          f"  ({st.st_size/1e6:.2f} MB)")
    print(f"  params        : TS={DEFAULTS[-1]:.1f}s  R_ORBIT={C.R_ORBIT:.0f} m  mass={C.MASS} kg")

    section('IMPORT COST')
    imp_ms = measure_import_cost()
    print(f"  import sad_py : {imp_ms:.0f} ms  (one-time, per process)")

    # ---------------- serial evaluate scaling ----------------
    section('SINGLE evaluate() BREAKDOWN  (serial, no pool)')
    print(f"  {'N':>5s} {'eval ms':>10s} {'ms/run':>9s} {'C++ batch ms':>14s} {'py overhead ms':>15s}")
    serial = {}
    for N in sorted(set([1, max(1, args.runs // 4), max(1, args.runs // 2), args.runs])):
        t0 = time.perf_counter()
        cost, rd, timing = evaluate(DEFAULTS, N=N, seed=args.seed, return_timing=True)
        dt = (time.perf_counter() - t0) * 1e3
        cpp_ms = float(timing['batch_ms']) if timing else float('nan')
        py_ms = dt - cpp_ms
        serial[N] = dt
        print(f"  {N:>5d} {dt:>10.1f} {dt/N:>9.1f} {cpp_ms:>14.1f} {py_ms:>15.1f}")
    per_run_ms = serial[max(serial)] / max(serial)
    print(f"  -> steady-state: {per_run_ms:.1f} ms per MC run "
          f"(Python overhead ~{100*(serial[max(serial)] - 0)/serial[max(serial)]:.0f}% of eval incl. kernel)")

    # ---------------- parallel generation ----------------
    if not args.skip_parallel:
        section(f'CMA-ES GENERATION  (pop={args.pop}, N={args.runs}, workers={args.workers})')
        from SAD_Project.optimization.optimizer import main as opt_main
        print('  running 1 generation ...')
        t0 = time.perf_counter()
        opt_main(generations=1, popsize=args.pop, sigma0=0.6, seed=args.seed,
                 N=args.runs, workers=args.workers)
        total = time.perf_counter() - t0
        res_path = os.path.join(SAD_DIR, 'results_opt', 'opt_result.json')
        with open(res_path) as f:
            res = json.load(f)
        h = res['history'][-1]
        print('-' * 96)
        print(f"  generation wall   : {h['gen_s']:.1f} s")
        print(f"    ask             : {h['ask_ms']:.1f} ms")
        print(f"    map (dispatch)  : {h['map_s']:.1f} s")
        print(f"    tell            : {h['tell_ms']:.1f} ms")
        print(f"  worker eval sum   : {h['worker_eval_sum_s']:.1f} s")
        print(f"  parallel eff.     : {100*h['parallel_efficiency']:.0f}%  "
              f"({h['workers'] if 'workers' in h else args.workers} workers)")
        print(f"  throughput        : {h['runs_per_s']:.2f} runs/s  "
              f"({h['n_tasks']} tasks x {h['N']} runs = {h['n_tasks']*h['N']} runs)")
        print(f"  total incl. import: {total:.1f} s")

        # projection
        section('PROJECTION (50 generations, pop=15, N=50)')
        rps = h['runs_per_s']
        pop, N, gens = 15, 50, 50
        print(f"  {'workers':>7s} {'runs/s':>9s} {'s/gen':>9s} {'hours':>8s}")
        for w in [1, 2, 4, 8]:
            eff = h['parallel_efficiency'] if w == args.workers else max(0.6, h['parallel_efficiency'])
            wrps = rps / args.workers * w * (eff if w > 1 else 1.0)
            s_gen = pop * N / wrps
            print(f"  {w:>7d} {wrps:>9.2f} {s_gen:>9.1f} {s_gen*gens/3600:>8.2f}")
        print('  (eff. for non-measured worker counts assumed = measured)')

    # ---------------- cProfile ----------------
    if args.cprofile:
        section('cProfile: single evaluate()')
        import cProfile
        import pstats
        import io
        pr = cProfile.Profile()
        pr.enable()
        evaluate(DEFAULTS, N=args.runs, seed=args.seed)
        pr.disable()
        s = io.StringIO()
        pstats.Stats(pr, stream=s).sort_stats('cumulative').print_stats(20)
        print(s.getvalue())

    section('DONE')


if __name__ == '__main__':
    main()
