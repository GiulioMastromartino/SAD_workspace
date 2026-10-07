"""Figures for the tuning sweep: optimizer performance + controller performance.

Reads the ``res_optimization.json`` dict (no re-running of the sweep) and
writes PNGs next to it:

* ``fig_opt_convergence.png`` — best-cost-so-far vs evaluations, every
  restart (optimizer convergence and restart spread).
* ``fig_opt_horizons.png`` — NMPC horizon sweep: best dv/box per horizon
  (structural choice, mirrors the reference N-sweep figure).
* ``fig_opt_validation.png`` — validation arms: dv, propellant, box, final
  energy error (controller head-to-head on the real simulator).
* ``fig_opt_trajectories.png`` — re-simulated winner trajectories at search
  resolution: radius error, energy error, cumulative dv (controller behavior,
  not just scalars).

``make_all_figures`` is called by ``run_optimize``; it also works standalone::

    ./env/bin/python -m cubesat_12u_sim.optimization.plots results_optimization/res_optimization.json
"""
from __future__ import annotations

import json
import os

import numpy as np


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _histories(stage: dict) -> list[tuple[list[float], int]]:
    """(history, evals-per-generation) per restart."""
    out = []
    restarts = stage.get("restarts", [])
    if restarts:
        src = [(r,) for r in restarts]
    else:
        src = [(r,) for h in stage.get("per_horizon", [])
               for r in h.get("restarts", [])]
    for (r,) in src:
        if r.get("history"):
            epg = max(1, int(r.get("evals", 0)) // max(1, int(r.get("generations", 1))))
            out.append(([float(h) for h in r["history"]], epg))
    return out


def fig_convergence(results: dict, path: str) -> str:
    plt = _mpl()
    fig, ax = plt.subplots(1, 3, figsize=(12, 4), sharey=False)
    for i, (key, title) in enumerate((("pid", "stage 1: PID"),
                                      ("nmpc", "stage 2: NMPC (all H)"),
                                      ("joint", "stage 3: joint"))):
        stage = results.get(key)
        if not stage:
            ax[i].set_title(f"{title}\n(no data)")
            continue
        hists = _histories(stage)
        for h, epg in hists:
            evals = (np.arange(len(h)) + 1) * epg
            ax[i].plot(evals, h, alpha=0.45, linewidth=0.9)
        best = min((min(h) for h, _ in hists), default=float("nan"))
        ax[i].axhline(best, color="k", linestyle="--", linewidth=1.0)
        ax[i].set_title(f"{title}\nbest J={best:.3f}")
        ax[i].set_xlabel("evaluations")
        ax[i].set_yscale("log")
        ax[i].grid(True, which="both", alpha=0.4)
    ax[0].set_ylabel("best cost so far J (log)")
    fig.suptitle("Optimizer convergence: every restart")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def fig_horizons(results: dict, path: str) -> str:
    plt = _mpl()
    nmpc = results.get("nmpc", {})
    rows = nmpc.get("per_horizon", [])
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    if not rows:
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return path
    hs = [r["horizon"] for r in rows]
    dvs = [r["result"]["dv"] for r in rows]
    box = [r["result"]["max_abs_r_err"] / 1e3 for r in rows]
    feas = [r["result"]["feasible"] for r in rows]
    colors = ["C0" if f else "C3" for f in feas]
    ax[0].bar([str(h) for h in hs], dvs, color=colors)
    ax[0].set_ylabel("best dv [m/s]")
    ax[0].set_xlabel("horizon H")
    ax[0].set_title(f"fuel vs lookahead (best H={nmpc.get('horizon')})")
    ax[1].bar([str(h) for h in hs], box, color=colors)
    ax[1].set_ylabel("best max |r err| [km]")
    ax[1].set_xlabel("horizon H")
    ax[1].set_title("box excursion vs lookahead")
    for a in ax:
        a.grid(True, axis="y", alpha=0.4)
    fig.suptitle("NMPC horizon sweep (green=feasible, red=infeasible)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def fig_validation(results: dict, path: str) -> str:
    plt = _mpl()
    arms = results.get("validation", {}).get("arms", [])
    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    if not arms:
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return path
    labels = [a.get("label", "?") for a in arms]
    x = np.arange(len(arms))
    ax[0, 0].bar(x, [a["dv"] for a in arms])
    ax[0, 0].set_ylabel("dv [m/s]")
    ax[0, 1].bar(x, [a["prop_g"] for a in arms])
    ax[0, 1].set_ylabel("propellant [g]")
    ax[1, 0].bar(x, [a["max_abs_r_err"] / 1e3 for a in arms])
    ax[1, 0].set_ylabel("max |r err| [km]")
    ax[1, 1].bar(x, [a["final_e_a"] / 1e3 for a in arms])
    ax[1, 1].set_ylabel("final e_a [km]")
    for a in ax.ravel():
        a.set_xticks(x)
        a.set_xticklabels(labels, rotation=18, ha="right", fontsize=8)
        a.grid(True, axis="y", alpha=0.4)
    t = results["validation"].get("t_end", 0)
    fig.suptitle(f"Validation on heyoka + thruster allocation (t={t / 3600:.1f} h)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def _resimulate(label: str, kind: str, theta, space, horizon: int,
                cfg) -> dict:
    """Closed-loop histories at search resolution for the winner plots."""
    from .. import config as C
    from ..nmpc import _rk4_step_orbit, semi_major_axis
    from .objective import initial_state, make_nmpc, make_pid

    if kind == "pid":
        ctrl = make_pid(space.from_theta(np.asarray(theta, float)))
        fn = ctrl.accel_command
    else:
        from .objective import EvalConfig
        ctrl = make_nmpc(space.from_theta(np.asarray(theta, float)), cfg,
                         horizon)
        fn = ctrl.accel_command
    dt = cfg.dt_inner
    n = int(round(cfg.t_end / dt))
    x = initial_state(cfg.v_frac)
    t = np.zeros(n + 1)
    r_err = np.zeros(n + 1)
    e_a = np.zeros(n + 1)
    dvh = np.zeros(n + 1)
    dv = 0.0
    r_err[0] = float(np.linalg.norm(x[0:3])) - C.R_ORBIT
    e_a[0] = semi_major_axis(x[0:3], x[3:6]) - C.R_ORBIT
    for k in range(1, n + 1):
        a = np.asarray(fn(x[0:3], x[3:6], dt), float) * cfg.alloc_eff
        dv += float(np.linalg.norm(a)) * dt
        x = _rk4_step_orbit(x, a, dt)
        t[k] = k * dt
        r_err[k] = float(np.linalg.norm(x[0:3])) - C.R_ORBIT
        e_a[k] = semi_major_axis(x[0:3], x[3:6]) - C.R_ORBIT
        dvh[k] = dv
    return {"label": label, "t": t, "r_err": r_err, "e_a": e_a, "dv": dvh,
            "dv_total": dv}


def fig_trajectories(results: dict, path: str, cfg=None) -> str:
    plt = _mpl()
    from .objective import EvalConfig, PID_SEARCH_CFG
    from .params import NMPC_SPACE, PID_SPACE
    pid = results.get("pid")
    nmpc = results.get("nmpc")
    cfgs = {"pid": PID_SEARCH_CFG,
            "nmpc": EvalConfig(t_days=results["config"]["days"],
                               spec_box=results["config"]["spec_box"],
                               alloc_eff=results["config"].get("alloc_eff", 0.5))}
    series = []
    if pid:
        series.append(_resimulate("PID (tuned)", "pid", pid["theta"],
                                  PID_SPACE, 0, cfgs["pid"]))
    if nmpc:
        series.append(_resimulate(f"NMPC H={nmpc['horizon']} (tuned)", "nmpc",
                                  nmpc["theta"], NMPC_SPACE,
                                  int(nmpc["horizon"]), cfgs["nmpc"]))
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for s in series:
        t_d = np.asarray(s["t"]) / 86400.0
        ax[0].plot(t_d, np.asarray(s["r_err"]) / 1e3, label=s["label"],
                   linewidth=0.9)
        ax[1].plot(t_d, np.asarray(s["e_a"]) / 1e3, label=s["label"],
                   linewidth=0.9)
        ax[2].plot(t_d, np.asarray(s["dv"]), label=s["label"], linewidth=1.0)
    ax[0].set_ylabel("radius err [km, symlog]")
    ax[0].set_yscale("symlog", linthresh=1e-2)
    ax[1].set_ylabel("energy e_a [km, symlog]")
    ax[1].set_yscale("symlog", linthresh=1e-2)
    ax[2].set_ylabel("cumul. dv [m/s]")
    ax[2].set_xlabel("time [d] (search horizon, surrogate dynamics)")
    for a in ax:
        a.legend(fontsize=8)
        a.grid(True, which="both", alpha=0.4)
    box = results["config"]["spec_box"] / 1e3
    ax[0].axhline(box, color="k", linestyle=":", linewidth=1.0)
    ax[0].axhline(-box, color="k", linestyle=":", linewidth=1.0)
    fig.suptitle("Winner trajectories at search resolution "
                 f"(box ±{box:.0f} km dotted)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def make_all_figures(results: dict, outdir: str) -> list[str]:
    os.makedirs(outdir, exist_ok=True)
    paths = []
    paths.append(fig_convergence(
        results, os.path.abspath(os.path.join(outdir, "fig_opt_convergence.png"))))
    paths.append(fig_horizons(
        results, os.path.abspath(os.path.join(outdir, "fig_opt_horizons.png"))))
    paths.append(fig_validation(
        results, os.path.abspath(os.path.join(outdir, "fig_opt_validation.png"))))
    paths.append(fig_trajectories(
        results, os.path.abspath(os.path.join(outdir, "fig_opt_trajectories.png"))))
    return paths


if __name__ == "__main__":
    import sys
    with open(sys.argv[1]) as fh:
        res = json.load(fh)
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(
        os.path.abspath(sys.argv[1]))
    for f in make_all_figures(res, outdir):
        print(f"wrote {f}")
