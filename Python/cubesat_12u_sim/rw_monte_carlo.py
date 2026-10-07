"""Dispersed RW-demo Monte Carlo across backends (stock/py/native/mission).

Same 600 s PD loop as the timing ladder, but every run disperses mass,
inertia, attitude/rate tip-off, wheel speeds and orbit injection (seeded,
reproducible). FSW gains stay nominal: model error, as in `monte_carlo.py`.

Usage (from Python/):
    ./env/bin/python -m cubesat_12u_sim.rw_monte_carlo --runs 30 --backend native
    ./env/bin/python -m cubesat_12u_sim.rw_monte_carlo --runs 500 --backend mission --jobs 4
"""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import sys
import time
from pathlib import Path

import numpy as np


class RWMonteCarloConfig:
    """RW campaign setup + dispersions + success criteria."""

    duration: float = 600.0
    dt: float = 0.1
    K: float = 3.5
    P: float = 30.0

    # dispersions (std devs)
    mass_std_pct: float = 5.0
    inertia_std_pct: float = 5.0
    inertia_cross_std: float = 5.0  # abs [kg m^2] (principals ~600-900)
    sigma0_std: float = 0.02
    omega0_std_deg_s: float = 0.5
    wheel_speed_std_rpm: float = 5.0
    r0_std_m: float = 1000.0
    v0_std_m_s: float = 1.0

    # success criteria
    att_tol_deg: float = 1.0
    rate_tol_deg_s: float = 0.2
    settle_att_deg: float = 0.5

    # output
    quiet: bool = False  # BSK_WARNING log level (kills per-run INFO spam)

    # pool start method: spawn = safe default; fork inherits the parent's
    # warm imports + JIT'd LLVM state (validate: deterministic seeds must
    # reproduce serial stats exactly)
    start_method: str = "spawn"


def _apply_log_level(cfg) -> None:
    if getattr(cfg, "quiet", False):
        from Basilisk.architecture import bskLogging

        bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)


_NOMINAL_ORBIT = None  # per-process cache (workers compute once)


def _nominal_orbit():
    """Baseline orbit state (same elements as bsk_rw_stock), computed once."""
    global _NOMINAL_ORBIT
    if _NOMINAL_ORBIT is None:
        from Basilisk.utilities import macros, orbitalMotion
        from Basilisk.utilities.simIncludeGravBody import gravBodyFactory

        mu = gravBodyFactory().createEarth().mu
        oe = orbitalMotion.ClassicElements()
        oe.a = 10000000.0
        oe.e = 0.01
        oe.i = 33.3 * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        _NOMINAL_ORBIT = orbitalMotion.elem2rv(mu, oe)
    return _NOMINAL_ORBIT


def sample_rw_dispersions(seed: int, cfg: RWMonteCarloConfig) -> dict:
    """Seeded dispersed truth for one RW run (nominal FSW gains kept)."""
    rng = np.random.default_rng(int(seed))
    mass = float(750.0 * (1.0 + rng.normal(0.0, cfg.mass_std_pct / 100.0)))

    ixx = 900.0 * (1.0 + rng.normal(0.0, cfg.inertia_std_pct / 100.0))
    iyy = 800.0 * (1.0 + rng.normal(0.0, cfg.inertia_std_pct / 100.0))
    izz = 600.0 * (1.0 + rng.normal(0.0, cfg.inertia_std_pct / 100.0))
    ixy = rng.normal(0.0, cfg.inertia_cross_std)
    ixz = rng.normal(0.0, cfg.inertia_cross_std)
    iyz = rng.normal(0.0, cfg.inertia_cross_std)
    inertia = np.array([[ixx, ixy, ixz],
                        [ixy, iyy, iyz],
                        [ixz, iyz, izz]], float)
    evals, evecs = np.linalg.eigh(inertia)
    inertia = evecs @ np.diag(np.clip(evals, 1.0, 5000.0)) @ evecs.T

    sigma0 = np.array([0.1, 0.2, -0.3]) + rng.normal(0.0, cfg.sigma0_std, 3)
    omega0 = (np.array([0.001, -0.01, 0.03])
              + np.radians(rng.normal(0.0, cfg.omega0_std_deg_s, 3)))
    wheels = (np.array([100.0, 200.0, 300.0])
              + rng.normal(0.0, cfg.wheel_speed_std_rpm, 3))
    rN, vN = _nominal_orbit()
    r0 = np.asarray(rN, float) + rng.normal(0.0, cfg.r0_std_m, 3)
    v0 = np.asarray(vN, float) + rng.normal(0.0, cfg.v0_std_m_s, 3)
    return {"mass": mass, "inertia": inertia, "sigma0": sigma0,
            "omega0": omega0, "wheels_rpm": wheels, "r0": r0, "v0": v0}


def _att_series_deg(sigma):
    from .heyoka_attitude import mrp_diff_batch

    sg = np.asarray(sigma, float)
    e = mrp_diff_batch(sg, np.zeros_like(sg))  # ref = identity
    return np.degrees(4.0 * np.arctan(np.linalg.norm(e, axis=1)))


def _warm_backend(backend: str) -> None:
    """One throwaway 60 s run: warms imports + Heyoka LLVM state."""
    import heyoka  # noqa: F401

    cfg = RWMonteCarloConfig()
    cfg.duration = 60.0
    cfg.quiet = True
    if backend == "native":
        from .cpp_native import run_native_rw_pipeline

        run_native_rw_pipeline(cfg.duration, cfg.dt, cfg.K, cfg.P)
    elif backend == "stock":
        from .bsk_rw_stock import run_stock_rw

        run_stock_rw(cfg.duration, cfg.dt, cfg.K, cfg.P)
    elif backend == "mission":
        import numpy as np

        from .cpp_rw import run_cpp_rw_mission

        run_cpp_rw_mission(np.zeros(15), cfg.duration, cfg.dt, cfg.K, cfg.P)
    else:
        import numpy as np

        from .heyoka_rw import make_rw_integrator

        ta = make_rw_integrator(np.zeros(15))
        ta.propagate_for(0.1)


def _pool_init(backend: str) -> None:
    """Spawn-path worker warmup (fresh interpreters start cold).

    Under fork this is redundant (workers inherit the parent's warmed
    state — see the pre-pool warmup in run_rw_monte_carlo) but harmless.
    """
    _warm_backend(backend)


def _run_single_rw(args) -> dict:
    """Picklable per-run worker: (run_idx, seed, cfg, backend)."""
    import heyoka  # noqa: F401  # first: dynamic_lookup for rwdrive
    import time as _t

    run_idx, seed, cfg, backend = args
    _apply_log_level(cfg)
    from .heyoka_attitude import mrp_diff
    from .heyoka_rw import pack_rw_state

    t0 = _t.perf_counter()
    c0 = _t.process_time()  # CPU-only: wall-vs-CPU gap => idle (lock) vs busy
    d = sample_rw_dispersions(seed, cfg)
    t_disp = _t.perf_counter()
    RPM = 60.0 / (2.0 * np.pi)
    if backend == "stock":
        from .bsk_rw_stock import run_stock_rw

        r = run_stock_rw(cfg.duration, cfg.dt, cfg.K, cfg.P, d["mass"],
                         d["inertia"], d["r0"], d["v0"], d["sigma0"],
                         d["omega0"], d["wheels_rpm"])
        t, sig, om = r["t"], r["sigma_BN"], r["omega_BN_B"]
        w = np.asarray(r["wheel_speeds"], float)
    elif backend == "native":
        from .cpp_native import run_native_rw_pipeline

        r = run_native_rw_pipeline(cfg.duration, cfg.dt, cfg.K, cfg.P,
                                   d["mass"], d["inertia"], d["r0"], d["v0"],
                                   d["sigma0"], d["omega0"], d["wheels_rpm"])
        t, sig, om = r["t"], r["sigma_BN"], r["omega_BN_B"]
        w = np.asarray(r["wheel_speeds"], float)
    else:
        y0 = pack_rw_state(d["r0"], d["v0"], d["sigma0"], d["omega0"],
                           d["wheels_rpm"] * 2.0 * np.pi / 60.0)
        if backend == "py":
            from .heyoka_rw import run_rw_mirror

            h = run_rw_mirror(cfg.duration, cfg.dt, y0, cfg.K, cfg.P,
                              d["mass"], d["inertia"])
        elif backend == "mission":
            from .cpp_rw import run_cpp_rw_mission

            h = run_cpp_rw_mission(y0, cfg.duration, cfg.dt, cfg.K, cfg.P)
        else:
            raise ValueError(f"unknown backend {backend!r}")
        Y = h["Y"]
        t, sig, om = h["t"], Y[6:9, :].T, Y[9:12, :].T
        w = Y[12:15, :].T
    t_sim = _t.perf_counter()

    att = _att_series_deg(sig)
    rate = np.degrees(np.linalg.norm(np.asarray(om, float), axis=1))
    wrpm = np.abs(np.asarray(w, float)) * RPM
    settle = next((float(t[k]) for k in range(len(t)) if att[k] < cfg.settle_att_deg),
                 float(t[-1]))
    kpis = {"final_att_err_deg": float(att[-1]),
            "final_rate_deg_s": float(rate[-1]),
            "max_wheel_rpm": float(wrpm.max()),
            "settle_time_s": float(settle)}
    success = bool(kpis["final_att_err_deg"] < cfg.att_tol_deg
                   and kpis["final_rate_deg_s"] < cfg.rate_tol_deg_s)
    t_end = _t.perf_counter()
    return {"run_idx": run_idx, "seed": seed, "backend": backend,
            "kpis": kpis, "success": success,
            "phase_ms": {"disperse": (t_disp - t0) * 1e3,
                         "sim": (t_sim - t_disp) * 1e3,
                         "kpi": (t_end - t_sim) * 1e3,
                         "cpu": (_t.process_time() - c0) * 1e3}}


def compute_rw_statistics(runs: list[dict]) -> dict:
    keys = ["final_att_err_deg", "final_rate_deg_s", "max_wheel_rpm",
            "settle_time_s"]
    stats: dict = {"summary": {
        "n_runs": len(runs),
        "success_rate_pct": float(sum(r["success"] for r in runs) / len(runs) * 100.0)}}
    stats["kpis"] = {}
    for key in keys:
        vals = np.array([r["kpis"][key] for r in runs], float)
        stats["kpis"][key] = {m: float(f(vals)) for m, f in
                              (("mean", np.mean), ("std", np.std),
                               ("median", np.median), ("min", np.min),
                               ("max", np.max), ("p95", lambda v: np.percentile(v, 95.0)),
                               ("p99", lambda v: np.percentile(v, 99.0)))}
    return stats


def run_rw_monte_carlo(num_runs=30, base_seed=100, cfg=None, backend="native",
                       jobs=1, verbose=True, save_dir=None, no_plot=False) -> dict:
    """Dispersed RW campaign on one backend (serial or process pool)."""
    import time as _time

    if cfg is None:
        cfg = RWMonteCarloConfig()
    seeds = [base_seed + i for i in range(num_runs)]
    if int(jobs) > 1 and cfg.start_method == "fork":
        # warm the parent BEFORE forking AND before t0: workers inherit warm
        # imports + JIT'd LLVM state, and the one-time compile stays out of
        # the timed region (it is fixed cost, not per-run work)
        _warm_backend(backend)
    t0 = _time.time()
    if verbose:
        print(f"RW Monte Carlo: {num_runs} runs, backend={backend}, jobs={jobs}")
    runs: list[dict] = []
    if int(jobs) <= 1:
        for i, seed in enumerate(seeds):
            r = _run_single_rw((i + 1, seed, cfg, backend))
            runs.append(r)
            if verbose:
                k = r["kpis"]
                print(f'[Run {i + 1:3d}/{num_runs}] '
                      f'[{"PASS" if r["success"] else "FAIL"}] '
                      f'att={k["final_att_err_deg"]:.3f}deg '
                      f'rate={k["final_rate_deg_s"]:.4f}deg/s '
                      f'wmax={k["max_wheel_rpm"]:.0f}RPM')
    else:
        args = [(i + 1, seed, cfg, backend) for i, seed in enumerate(seeds)]
        # chunked dispatch: default chunksize=1 pays ~ms dispatch per task,
        # which dominates 6-70 ms sims over hundreds of runs
        chunksize = max(1, len(args) // (4 * max(1, int(jobs))))
        from functools import partial

        import multiprocessing as _mp

        try:
            mp_ctx = _mp.get_context(cfg.start_method)
        except Exception:
            mp_ctx = None
        pool_kw: dict = {"max_workers": int(jobs)}
        if cfg.start_method != "fork":
            # spawned workers start cold: initializer warms each of them
            pool_kw["initializer"] = partial(_pool_init, backend)
        if mp_ctx is not None:
            pool_kw["mp_context"] = mp_ctx
        with concurrent.futures.ProcessPoolExecutor(**pool_kw) as ex:
            for r in ex.map(_run_single_rw, args, chunksize=chunksize):
                runs.append(r)
                if verbose:
                    k = r["kpis"]
                    print(f'[Run {r["run_idx"]:3d}/{num_runs}] '
                          f'[{"PASS" if r["success"] else "FAIL"}] '
                          f'att={k["final_att_err_deg"]:.3f}deg '
                          f'rate={k["final_rate_deg_s"]:.4f}deg/s')
        runs.sort(key=lambda r: r["run_idx"])
    stats = compute_rw_statistics(runs)
    wall = _time.time() - t0
    if verbose:
        per = wall / max(1, len(runs)) * 1e3
        ph = {k: float(np.mean([r.get("phase_ms", {}).get(k, float("nan"))
                                for r in runs]))
              for k in ("disperse", "sim", "kpi", "cpu")}
        print("-" * 72)
        print(f"campaign wall: {wall:.2f}s for {len(runs)} runs "
              f"({per:.2f}ms/run, {len(runs) / max(wall, 1e-9):.1f} runs/s) "
              f"[backend={backend}, jobs={jobs}]")
        print(f"phase means [ms/run]: disperse {ph['disperse']:.2f} | "
              f"sim {ph['sim']:.2f} | kpi {ph['kpi']:.2f} | "
              f"cpu {ph['cpu']:.2f}")
        print(f"success: {stats['summary']['success_rate_pct']:.1f}% | "
              f"att p50/p95 {stats['kpis']['final_att_err_deg']['median']:.3f}/"
              f"{stats['kpis']['final_att_err_deg']['p95']:.3f} deg")
        print("-" * 72)
    out = {"runs": runs, "stats": stats, "wall_s": wall, "backend": backend}
    if save_dir is not None:
        save_rw_results(out, save_dir, cfg, no_plot=no_plot)
    return out


def save_rw_results(out: dict, save_dir, cfg, no_plot=False) -> None:
    d = Path(save_dir)
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "mc_summary.json", "w") as fh:
        json.dump({"backend": out["backend"], "wall_s": out["wall_s"],
                   "stats": out["stats"]}, fh, indent=2)
    np.savez_compressed(d / "mc_results.npz",
                        **{f"run_{r['run_idx']:04d}": np.array(
                            [r["kpis"]["final_att_err_deg"],
                             r["kpis"]["final_rate_deg_s"],
                             r["kpis"]["max_wheel_rpm"],
                             r["kpis"]["settle_time_s"]]) for r in out["runs"]})
    if not no_plot:
        plot_rw_monte_carlo(out, d)
    print(f"[info] saved {len(out['runs'])} runs -> {d}/")


def plot_rw_monte_carlo(out: dict, d: Path) -> None:
    import matplotlib.pyplot as plt

    runs = out["runs"]
    be = out["backend"]
    att = np.array([r["kpis"]["final_att_err_deg"] for r in runs])
    rate = np.array([r["kpis"]["final_rate_deg_s"] for r in runs])
    wmax = np.array([r["kpis"]["max_wheel_rpm"] for r in runs])
    stl = np.array([r["kpis"]["settle_time_s"] for r in runs])
    ok = np.array([r["success"] for r in runs])

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    axes[0, 0].hist(att, bins=20, color="steelblue")
    axes[0, 0].set_xlabel("final att error [deg]")
    axes[0, 1].hist(rate, bins=20, color="seagreen")
    axes[0, 1].set_xlabel("final rate [deg/s]")
    axes[1, 0].hist(wmax, bins=20, color="darkorange")
    axes[1, 0].set_xlabel("max wheel speed [RPM]")
    axes[1, 1].scatter(att, rate, c=np.where(ok, "g", "r"), s=12)
    axes[1, 1].set_xlabel("final att [deg]")
    axes[1, 1].set_ylabel("final rate [deg/s]")
    fig.suptitle(f"RW Monte Carlo ({be}): {len(runs)} runs, "
                 f"{ok.mean() * 100:.1f}% success, settle med {np.median(stl):.0f}s")
    fig.tight_layout()
    fig.savefig(d / "rwmc_hist.png", dpi=150)
    plt.close(fig)
    print(f"[info] wrote {d / 'rwmc_hist.png'}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--backend", default="native",
                    choices=["stock", "py", "native", "mission"])
    ap.add_argument("--save-dir", default="results_rwmc")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--quiet", action="store_true",
                    help="BSK_WARNING log level (silences per-run INFO spam)")
    ap.add_argument("--start-method", default="spawn", choices=["spawn", "fork"],
                    help="multiprocessing start method (fork reuses warm state)")
    ap.add_argument("--compare-backend", default="stock",
                    choices=["stock", "py", "native", "mission"],
                    help="rerun same seeds on a second backend + print comparison "
                         "(default stock; ignored when it equals --backend)")
    ap.add_argument("--no-compare", action="store_true",
                    help="skip the comparison rerun")
    a = ap.parse_args(argv)
    cfg = RWMonteCarloConfig()
    cfg.duration = float(a.duration)
    cfg.quiet = bool(a.quiet)
    cfg.start_method = str(a.start_method)
    _apply_log_level(cfg)
    out = run_rw_monte_carlo(a.runs, a.seed, cfg, a.backend, a.jobs, True,
                             a.save_dir, a.no_plot)
    if not a.no_compare and a.compare_backend != a.backend:
        cmp = run_rw_monte_carlo(a.runs, a.seed, cfg, a.compare_backend,
                                 a.jobs, False, None)
        _print_backend_comparison(out, cmp)
    return 0


def _print_backend_comparison(a: dict, b: dict) -> None:
    """Same-seed timing + KPI parity between two campaigns."""
    ra = {r["run_idx"]: r for r in a["runs"]}
    rb = {r["run_idx"]: r for r in b["runs"]}
    idx = sorted(set(ra) & set(rb))
    datt = max(abs(ra[k]["kpis"]["final_att_err_deg"]
                   - rb[k]["kpis"]["final_att_err_deg"]) for k in idx)
    drate = max(abs(ra[k]["kpis"]["final_rate_deg_s"]
                    - rb[k]["kpis"]["final_rate_deg_s"]) for k in idx)
    print("=" * 72)
    print(f"backend comparison ({len(idx)} runs, identical seeds):")
    for o in (a, b):
        n = len(o["runs"])
        print(f"  {o['backend']:<10} wall {o['wall_s']:8.2f}s  "
              f"{o['wall_s'] / n * 1e3:8.2f}ms/run  "
              f"success {o['stats']['summary']['success_rate_pct']:5.1f}%")
    print(f"  speedup ({a['backend']} vs {b['backend']}): "
          f"{b['wall_s'] / a['wall_s']:.2f}x  "
          f"({b['backend']} vs {a['backend']}: "
          f"{a['wall_s'] / b['wall_s']:.2f}x)")
    print(f"  cross-backend KPI parity: max |datt| {datt:.2e} deg, "
          f"max |drate| {drate:.2e} deg/s")
    worst = sorted(idx, key=lambda k: abs(ra[k]["kpis"]["final_att_err_deg"]
                                         - rb[k]["kpis"]["final_att_err_deg"]),
                   reverse=True)[:5]
    print(f"  worst run_idx: {worst} "
          f"(rerun: -m cubesat_12u_sim.run_rw_forensic --seeds {' '.join(map(str, [ra[k]['seed'] for k in worst]))})")
    print("=" * 72)


if __name__ == "__main__":
    sys.exit(main())
