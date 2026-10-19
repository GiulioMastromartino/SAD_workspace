"""CMA-ES optimization of SAD_Project control parameters (cubesat_12u_sim pattern).

Instrumentation added:
  * per-generation wall-time split into ask / dispatch+map / tell
  * sum of worker evaluation time and the resulting parallel efficiency
  * optional C++ timing registry aggregation (``timing=True``)
  * everything dumped to results_opt/opt_result.json for later analysis

A single CMA-ES run is executed.  The MC evaluation seed is fixed by
generation (``seed + gen``) so every candidate in a generation is compared on
the same dispersion draws.
"""
import sys, os, json, time, math
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import cma
from concurrent.futures import ThreadPoolExecutor, as_completed
from SAD_Project.optimization.params import BOUNDS, DEFAULTS, PARAM_NAMES
from SAD_Project.core import config as C
from SAD_Project.optimization.objective import evaluate as _eval_single


def _init_worker():
    # ensure imports in worker
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def _eval_one(args):
    """Worker task. Returns cost, run_data, wall_seconds, cpp_timing, pid."""
    theta, seed, N, want_timing = args
    t0 = time.perf_counter()
    if want_timing:
        cost, run_data, timing = _eval_single(theta, N=N, seed=seed, return_timing=True)
    else:
        cost, run_data = _eval_single(theta, N=N, seed=seed)
        timing = None
    return cost, run_data, time.perf_counter() - t0, timing, os.getpid()


def _accumulate_timing(acc, timing):
    """Merge a C++ timing dict into a running accumulator."""
    if not timing or not timing.get('enabled'):
        return
    for k, v in timing['ns'].items():
        acc['ns'][k] = acc['ns'].get(k, 0.0) + float(v)
    for k, v in timing['calls'].items():
        acc['calls'][k] = acc['calls'].get(k, 0) + int(v)
    for k, vec in timing['phase_ns'].items():
        cur = acc['phase_ns'].setdefault(k, [0.0, 0.0, 0.0])
        for i in range(3):
            cur[i] += float(vec[i])
    acc['steps'] += int(timing.get('steps', 0))
    ps = acc['phase_steps']
    for i in range(3):
        ps[i] += int(timing.get('phase_steps', [0, 0, 0])[i])
    acc['batch_ms'] += float(timing.get('batch_ms', 0.0))
    acc['evals'] += 1


def _make_es(x0, sigma0, popsize, cma_seed, lb_log, ub_log):
    inopts = {
        "bounds": [list(lb_log), list(ub_log)],
        "popsize": int(popsize),
        "seed": int(cma_seed),
        "verbose": -9,
        "tolx": 1e-4,
        "tolfun": 1e-6,
    }
    return cma.CMAEvolutionStrategy(list(x0), float(sigma0), inopts=inopts)


def main(generations=8, popsize=12, sigma0=0.6, seed=42, N=5, workers=1,
         timing=False):
    t0 = time.perf_counter()
    lb = np.array([b[0] for b in BOUNDS])
    ub = np.array([b[1] for b in BOUNDS])
    lb_log = np.log(lb)
    ub_log = np.log(ub)
    theta0_log = np.log(np.clip(np.array(DEFAULTS, float), lb, ub))

    best_cost = 1e9
    best_theta = theta0_log.copy()
    all_history = []
    all_run_data = []
    timing_acc = {'ns': {}, 'calls': {}, 'phase_ns': {}, 'steps': 0,
                  'phase_steps': [0, 0, 0], 'batch_ms': 0.0, 'evals': 0}

    pool = ThreadPoolExecutor(max_workers=workers) if workers > 1 else None
    try:
        es = _make_es(theta0_log, sigma0, popsize, seed, lb_log, ub_log)
        gen = 0
        while gen < generations and not es.stop():
            t_gen = time.perf_counter()

            t_ask = time.perf_counter()
            xs = es.ask()
            thetas = [np.exp(np.clip(x, lb_log, ub_log)) for x in xs]
            # MC seed schedule is fixed by generation, so every candidate in a
            # generation is compared on the same dispersion draws.
            args_list = [(th, seed + gen, N, timing) for th in thetas]
            ask_s = time.perf_counter() - t_ask

            t_map = time.perf_counter()
            completion = []
            if pool is None:
                results = []
                for i, a in enumerate(args_list):
                    rr = _eval_one(a)
                    results.append(rr)
                    completion.append({'task': i, 'pid': rr[4], 'eval_s': rr[2],
                                       'done_at_s': time.perf_counter() - t_map,
                                       'cost': float(rr[0])})
            else:
                futs = {pool.submit(_eval_one, a): i for i, a in enumerate(args_list)}
                results = [None] * len(args_list)
                for fut in as_completed(futs):
                    i = futs[fut]
                    rr = fut.result()
                    results[i] = rr
                    completion.append({'task': i, 'pid': rr[4], 'eval_s': rr[2],
                                       'done_at_s': time.perf_counter() - t_map,
                                       'cost': float(rr[0])})
            map_s = time.perf_counter() - t_map

            # ---- per-worker (subprocess) accounting ----
            worker_stats = {}
            for c in completion:
                w = worker_stats.setdefault(c['pid'], {'tasks': 0, 'busy_s': 0.0,
                                                       'first_done_s': None, 'last_done_s': 0.0})
                w['tasks'] += 1
                w['busy_s'] += c['eval_s']
                if w['first_done_s'] is None:
                    w['first_done_s'] = c['done_at_s']
                w['last_done_s'] = max(w['last_done_s'], c['done_at_s'])
            for pid, w in worker_stats.items():
                w['idle_s'] = max(0.0, map_s - w['busy_s'])
                w['util_pct'] = 100.0 * w['busy_s'] / map_s if map_s > 0 else 0.0

            vals = [rr[0] for rr in results]
            worker_eval_s = float(sum(rr[2] for rr in results))
            for rr in results:
                all_run_data.extend(rr[1])
                _accumulate_timing(timing_acc, rr[3])
            for x, cost in zip(xs, vals):
                cost = float(cost)
                if cost < best_cost:
                    best_cost = cost
                    best_theta = x.copy()

            t_tell = time.perf_counter()
            es.tell(xs, vals)
            tell_s = time.perf_counter() - t_tell

            gen += 1
            gen_s = time.perf_counter() - t_gen
            # parallel efficiency: useful work / (wall * workers)
            parallel_eff = (worker_eval_s / (map_s * workers)) if (workers > 1 and map_s > 0) else 1.0
            rate = (len(xs) * N) / gen_s if gen_s > 0 else 0.0
            print(
                f"gen {gen}: best={min(vals):.4f} mean={np.mean(vals):.4f} "
                f"sigma={es.sigma:.4f} ({gen_s:.1f}s) "
                f"[ask {ask_s*1e3:.0f}ms | map {map_s:.1f}s | tell {tell_s*1e3:.0f}ms | "
                f"worker_sum {worker_eval_s:.1f}s | par_eff {parallel_eff*100:.0f}% | "
                f"{rate:.1f} runs/s]",
                flush=True,
            )
            if worker_stats:
                print(f"    subprocesses ({len(worker_stats)}): "
                      f"{'pid':>7s} {'tasks':>5s} {'busy s':>8s} {'idle s':>8s} {'util':>6s}")
                for pid, w in sorted(worker_stats.items(), key=lambda kv: -kv[1]['busy_s']):
                    print(f"    {'':>13s} {pid:>7d} {w['tasks']:>5d} {w['busy_s']:>8.2f} "
                          f"{w['idle_s']:>8.2f} {w['util_pct']:>5.0f}%")

            gt = {
                "cma_seed": int(seed), "popsize": int(popsize),
                "gen": gen, "global_gen": gen,
                "best": float(min(vals)), "mean": float(np.mean(vals)),
                "sigma": float(es.sigma), "gen_s": gen_s,
                "ask_ms": ask_s * 1e3, "map_s": map_s, "tell_ms": tell_s * 1e3,
                "worker_eval_sum_s": worker_eval_s, "parallel_efficiency": parallel_eff,
                "runs_per_s": rate, "n_tasks": len(xs), "N": N,
                "subprocesses": {str(pid): {"tasks": w["tasks"], "busy_s": w["busy_s"],
                                            "idle_s": w["idle_s"], "util_pct": w["util_pct"],
                                            "last_done_s": w["last_done_s"]}
                                 for pid, w in sorted(worker_stats.items())},
                "tasks": sorted(completion, key=lambda c: c["task"]),
            }
            all_history.append(gt)
    finally:
        if pool is not None:
            pool.shutdown(wait=True)

    total_generations = gen

    # ---------------- cost map ----------------
    os.makedirs(os.path.join(os.path.dirname(__file__), '..', 'results_cost_map'), exist_ok=True)
    fuels = np.array([d['fuel'] for d in all_run_data])
    times = np.array([d['time'] for d in all_run_data])
    costs = np.array([d['cost'] for d in all_run_data])
    np.savez(os.path.join(os.path.dirname(__file__), '..', 'results_cost_map', 'cost_map.npz'),
             fuel=fuels, time=times, cost=costs)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        n_pts = len(all_run_data)
        marker = 20 if n_pts <= 50000 else 6
        alpha = 0.6 if n_pts <= 50000 else 0.35
        fig, ax = plt.subplots(figsize=(8, 6))
        sc = ax.scatter(fuels, times, c=costs, cmap='viridis_r', s=marker,
                        edgecolors='none', alpha=alpha)
        plt.colorbar(sc, ax=ax, label='Cost')
        ax.set_xlabel('Fuel consumption (kg)')
        ax.set_ylabel('Total time detumble+slew (s)')
        ax.set_title(f'Cost surface (optimizer, {n_pts} points)')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(os.path.dirname(__file__), '..', 'results_cost_map', 'cost_map.png'), dpi=150)
        plt.close(fig)
        print('cost map saved:', n_pts, 'points')
    except Exception as e:
        print('cost map generation failed:', e)

    theta_opt = np.exp(np.clip(best_theta, lb_log, ub_log))
    out = {
        "params": dict(zip(PARAM_NAMES, [float(x) for x in theta_opt])),
        "cost": float(best_cost),
        "generations": total_generations,
        "time_s": time.perf_counter() - t0,
        "history": all_history,
        "timing": timing_acc if timing else None,
        "config": {"popsize": popsize, "sigma0": sigma0, "seed": seed,
                   "N": N, "workers": workers, "timing": timing,
                   "n_thrusters": len(C.THRUSTER_POS),
                   "tau_max": float(getattr(C, 'ACS_TAU_MAX', 0.1)),
                   "track_dt": float(C.TRACK_DT),
                   "implicit_pd": bool(getattr(C, 'IMPLICIT_PD', True)),
                   "implicit_track_only": bool(getattr(C, 'IMPLICIT_TRACK_ONLY', True)),
                   "box_km": float(getattr(C, 'ORBIT_BOX_KM', 5.0)),
                   "cone_deg": float(getattr(C, 'SUN_KEEP_IN_CONE_DEG', 15.0)),
                   "box_weight": float(getattr(C, 'COST_BOX_WEIGHT', 1.0)),
                   "cone_weight": float(getattr(C, 'COST_CONE_WEIGHT', 1.0)),
                   "cost_exp_rate": float(getattr(C, 'COST_EXP_RATE', 1.0)),
                   "disp_r_m": float(getattr(C, 'DISP_R_KM', 1.0)) * 1000.0,
                   "disp_v_mps": float(getattr(C, 'DISP_V_MPS', 0.5)),
                   "cost_mode": "max"},
    }
    os.makedirs(os.path.join(os.path.dirname(__file__), '..', 'results_opt'), exist_ok=True)
    with open(os.path.join(os.path.dirname(__file__), '..', 'results_opt', 'opt_result.json'), 'w') as f:
        json.dump(out, f, indent=2)
    print(json.dumps({k: out[k] for k in ('params', 'cost', 'generations', 'time_s', 'config')}, indent=2))


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='CMA-ES gain/TS optimizer (C++ batch kernel)')
    ap.add_argument('--generations', type=int, default=8)
    ap.add_argument('--popsize', type=int, default=12)
    ap.add_argument('--sigma0', type=float, default=0.6)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--N', type=int, default=5, help='MC runs per candidate')
    ap.add_argument('--workers', type=int, default=1)
    ap.add_argument('--timing', action='store_true',
                    help='collect the C++ timing registry (slow; diagnostics only)')
    args = ap.parse_args()
    main(generations=args.generations, popsize=args.popsize, sigma0=args.sigma0,
         seed=args.seed, N=args.N, workers=args.workers, timing=args.timing)
