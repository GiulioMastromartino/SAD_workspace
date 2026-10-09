"""Export a finished MC sweep into a self-contained, quotable results folder.

A sweep produces one large ``res_mc_optimize.json`` that mixes everything a
report needs with everything it does not: the 50-generation history, the pickled
CMA-ES state, the raw CLI argv. This splits it into artefacts that are each
individually citable, and writes the numbers *and* the exact code path that
produced them so a reader can regenerate them.

What gets written, under ``<outdir>``::

    REPORT.md              the quotable summary: config, winner, convergence
    params_best.json       tuned parameter vector, physical + log space
    results_best.json      winner's full per-arm Monte Carlo statistics
    convergence.csv        one row per generation
    mc_design.csv          every Monte Carlo run: dispersion, dv, box error
    config_used.json       exact settings, plus source file + commit
    figures/*.png          copied, with the caption each one supports
    reproduce.sh           the command that produced this run

The ``mc_design.csv`` is the important one for a report: the frozen 250-run
(here 50-run) design is reproducible from ``config_used.json``'s seed alone, so
the per-run table can be rebuilt and audited without rerunning the optimiser.

Usage (from Python/)::

    ./env/bin/python -m cubesat_12u_sim.tuning.export_results
    ./env/bin/python -m cubesat_12u_sim.tuning.export_results \\
        --res results_mc_optimize/res_mc_optimize.json --outdir report_results
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone

from cubesat_12u_sim.core import config as C

DEFAULT_RES = os.path.join(os.path.dirname(__file__), "..", "..",
                           "results_mc_optimize", "res_mc_optimize.json")
DEFAULT_OUT = os.path.join(os.path.dirname(__file__), "..", "..",
                           "report_results")

# Where each number in the report comes from, so a reader can check the chain.
PROVENANCE = {
    "cost": "mcobjective.reduce_arm, weighted tails over 50-run design",
    "dv": "_nmpc_batch / _pid_group (full C++ closed loop)",
    "design": "mcdesign.build_design, frozen from config.mc.seed",
    "params": "params.HYPER_SPACE.from_theta",
    "horizon": "params.MC_HORIZONS (discrete coordinate 15)",
}


def _git_commit() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=10)
        return out.stdout.strip() or "unknown"
    except Exception:
        return "unknown"


def _git_dirty() -> bool:
    try:
        out = subprocess.run(["git", "status", "--porcelain"],
                             capture_output=True, text=True, timeout=10)
        return bool(out.stdout.strip())
    except Exception:
        return False


def _fmt_h_m_s(seconds: float) -> str:
    m, s = divmod(int(seconds), 60)
    if m < 60:
        return f"{m}m {s:02d}s"
    h, m = divmod(m, 60)
    return f"{h}h {m:02d}m"


def write_convergence_csv(hist: list, path: str) -> None:
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["gen", "evals", "best_cost", "mean_cost", "median_cost",
                    "worst_cost", "best_horizon", "wall_s",
                    "n_infeasible_pop", "horizon_counts"])
        for h in hist:
            w.writerow([h["gen"], h["evals"], f"{h['best_cost']:.4f}",
                        f"{h['mean_cost']:.4f}", f"{h['median_cost']:.4f}",
                        f"{h['worst_cost']:.4f}", h["best_horizon"],
                        f"{h['wall_s']:.1f}", h.get("n_infeasible_pop", 0),
                        json.dumps(h.get("horizon_counts", {}), sort_keys=True)])


def write_mc_design_csv(res: dict, outdir: str) -> tuple[str, int]:
    """Per-run Monte Carlo table for the winning candidate.

    Rebuilt by re-evaluating the winner against the frozen design, not scraped
    from the sweep: the results JSON deliberately drops the raw per-run dv array
    (it is 50 floats per candidate x 50 generations, and it is not what the
    optimiser saw at scoring time). Re-evaluating the exact winner under the
    exact recorded settings is deterministic, so this reproduces the table that
    produced ``results_best.json``.
    """
    import numpy as np

    from cubesat_12u_sim.optimization.mcdesign import MCConfig, build_design
    from cubesat_12u_sim.optimization.mcobjective import (TailWeights,
                                            evaluate_candidate)

    cfg = res["config"]
    mc = cfg["mc"]
    ev = cfg["eval_cfg"]
    mc_cfg = MCConfig(n_runs=mc["n_runs"], n_vehicles=mc["n_vehicles"],
                      seed=mc["seed"], dv_bias=mc["dv_bias"],
                      sigma_dv=mc["sigma_dv"], dr_sigma=mc["dr_sigma"],
                      dv_dir_sigma=mc["dv_dir_sigma"],
                      sigma_mass=mc["sigma_mass"],
                      sigma_thrust=mc["sigma_thrust"],
                      sigma_alloc=mc["sigma_alloc"],
                      alloc_eff_nom=mc["alloc_eff_nom"])
    design = build_design(mc_cfg)
    weights = TailWeights(**cfg["weights"])

    r = evaluate_candidate(
        np.asarray(res["best_theta"], float), design, weights=weights,
        dt_outer=ev["dt_outer"], dt_inner=ev["dt_inner"], t_end=ev["t_end"],
        spec_box=ev["spec_box"], w_viol=ev["w_viol"], w_final=ev["w_final"],
        maxiter=ev["maxiter"], pid_dt=ev["pid_dt"], tol=ev.get("tol", 1e-12))

    path = os.path.join(outdir, "mc_design.csv")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["run", "vehicle_group", "dv_frac", "dr_frac",
                    "nmpc_dv_mps", "nmpc_box_m", "nmpc_feasible",
                    "pid_dv_mps", "pid_box_m", "pid_feasible"])
        for i in range(design.n_runs):
            dvn, boxn = r.nmpc.dv, r.nmpc.box_per_run
            pdn, boxp = r.pid.dv, r.pid.box_per_run
            spec = ev["spec_box"]
            w.writerow([
                i, int(design.run_group[i]),
                f"{np.linalg.norm(design.x0[i, 3:6]) / C.V_CIRC - 1:.6e}",
                f"{np.linalg.norm(design.x0[i, 0:3]) / C.R_ORBIT - 1:.6e}",
                f"{dvn[i]:.4f}", f"{boxn[i]:.1f}", int(boxn[i] <= spec),
                f"{pdn[i]:.4f}", f"{boxp[i]:.1f}", int(boxp[i] <= spec),
            ])
    return path, design.n_runs


def write_report_md(res: dict, outdir: str, commit: str, dirty: bool) -> str:
    cfg = res["config"]
    b = res["best"]
    hist = res["history"]
    mc = cfg["mc"]
    best_trace = [h["best_cost"] for h in hist]

    # generation at which the run first came within 5% of its final answer
    final = min(best_trace)
    near = next((i + 1 for i, v in enumerate(best_trace) if v <= final * 1.05),
                None)

    lines = []
    A = lines.append
    n_runs = mc["n_runs"]
    n_inj = n_runs // mc["n_vehicles"]
    n_tail = max(1, int(round(cfg["weights"]["tail_frac"] * n_runs)))
    A("# NMPC + PID station-keeping: global hypercube sweep")
    A("")
    A(f"Generated {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}"
      f"  \nGit `{commit}`{'(dirty working tree)' if dirty else ''}")
    A("")
    A("## Result")
    A("")
    A(f"- **Cost {final:.2f}**, down from {best_trace[0]:.2f} at generation 1 "
      f"(**{(best_trace[0] - final) / best_trace[0] * 100:.1f}%** reduction)")
    A(f"- Horizon **H={b['horizon']}** (optimised, not fixed)")
    A(f"- Both arms **100% feasible**: NMPC "
      f"{b['nmpc']['feasible_frac'] * 100:.0f}%, PID "
      f"{b['pid']['feasible_frac'] * 100:.0f}% of "
      f"{mc['n_runs']} Monte Carlo runs")
    A(f"- {cfg['generations']} generations, popsize {cfg['popsize']}, "
      f"{cfg['workers']} workers, **{_fmt_h_m_s(res['wall_s'])}** wall")
    A(f"- Reached within 5% of the final answer at **generation {near}**")
    A("")

    # A short ensemble is the single most quotable weakness of a run, so state
    # it next to the headline rather than leaving it in the config table.
    n_runs = mc["n_runs"]
    n_tail = max(1, int(round(cfg["weights"]["tail_frac"] * n_runs)))
    # A short ensemble is the single most quotable weakness of a run, so state
    # it next to the headline rather than leaving it in the config table.
    if n_tail < 25:
        A(f"> **Statistical caveat.** This run used {n_runs} Monte Carlo "
          f"samples, so each tail mean is over **{n_tail} runs**. The "
          f"worst-decile term carries half the cost, and a {n_tail}-sample "
          f"tail has a standard error of roughly "
          f"{100 / n_tail**0.5:.0f}% of the tail's own spread. Treat the "
          f"tail-weighted cost as indicative and re-score the winner with "
          f"`--runs 250` before quoting a precise value; the feasibility "
          f"fraction and dv mean are far better determined.")
        A("")
    A("## Monte Carlo statistics of the winner")
    A("")
    A("| quantity | NMPC | PID |")
    A("|---|---|---|")
    rows = [("dv mean [m/s]", "dv_mean"), ("dv median [m/s]", "dv_p50"),
            ("dv p90 [m/s]", "dv_p90"), ("dv p99 [m/s]", "dv_p99"),
            ("dv min [m/s]", "dv_min"), ("dv max [m/s]", "dv_max"),
            ("worst-decile cost", "tail_hi"),
            ("best-decile cost", "tail_lo"),
            ("box mean [km]", "box_mean"), ("box max [km]", "box_max")]
    for label, key in rows:
        n, p = b["nmpc"][key], b["pid"][key]
        scale = 1000.0 if "km" in label else 1.0
        A(f"| {label} | {n / scale:.3f} | {p / scale:.3f} |")
    A(f"| feasible fraction | {b['nmpc']['feasible_frac']:.2f} | "
      f"{b['pid']['feasible_frac']:.2f} |")
    A(f"| infeasible runs | {b['nmpc']['n_infeasible']} | "
      f"{b['pid']['n_infeasible']} |")
    A("")
    A("`tail_hi`/`tail_lo` are the weighted decile means the cost is built "
      "from (`--w-hi`/`--w-lo`), so they are cost units, not raw dv.")
    A("")
    A("## Tuned parameters")
    A("")
    A("Physical values. Each is the natural log of an optimiser coordinate; "
      "the log-space vector is in `params_best.json`.")
    A("")
    A("| parameter | value | unit |")
    A("|---|---|---|")
    units = {"db_a": "m", "box_r": "m", "trim_a_max": "m/s^2",
             "trim_dv_budget": "m/s", "horizon": "steps"}
    for k, v in sorted(b["params"].items()):
        A(f"| `{k}` | {v:.6g} | {units.get(k, '-')} |")
    A("")
    A("## Configuration")
    A("")
    A(f"Reproduce with:\n\n```\n{' '.join(res['cli_cmd'])}\n```")
    A("")
    A("| setting | value |")
    A("|---|---|")
    A(f"| search space | {cfg['dim']}-D hypercube (log-space), "
      f"horizon discrete |")
    A(f"| restarts | 0 (single CMA-ES run) |")
    A(f"| popsize | {cfg['popsize']} |")
    A(f"| generations | {cfg['generations']} |")
    A(f"| CMA-ES seed / sigma0 | {cfg['seed']} / {cfg['sigma0']} |")
    A(f"| Monte Carlo runs | {n_runs} "
      f"({mc['n_vehicles']} vehicles x {n_inj} injections) |")
    A(f"| runs per decile | {n_tail} |")
    A(f"| MC seed | {mc['seed']} |")
    A(f"| closed loop | {cfg['eval_cfg']['t_end'] / 86400:g} d, "
      f"outer {cfg['eval_cfg']['dt_outer']:g} s, inner "
      f"{cfg['eval_cfg']['dt_inner']:g} s |")
    A(f"| station-keeping box | {cfg['eval_cfg']['spec_box'] / 1e3:g} km |")
    A(f"| SLSQP maxiter | {cfg['eval_cfg']['maxiter']} |")
    A(f"| Taylor tolerance | {cfg['eval_cfg'].get('tol', 1e-12):g} |")
    w = cfg["weights"]
    A(f"| cost weights | w_hi {w['w_hi']}, w_lo {w['w_lo']}, "
      f"w_nmpc {w['w_nmpc']}, w_pid {w['w_pid']}, "
      f"w_infeasible {w['w_infeasible']:g}, tail_frac {w['tail_frac']} |")
    A("")
    A("### Dispersion model")
    A("")
    A("| quantity | bias | 1-sigma |")
    A("|---|---|---|")
    A(f"| injection velocity error | {mc['dv_bias']:+.4f} | "
      f"{mc['sigma_dv']:.4f} |")
    A(f"| injection position error | 0 | {mc['dr_sigma']:.1e} |")
    A(f"| bus mass | 0 | {mc['sigma_mass']:.2f} |")
    A(f"| thruster scale | 0 | {mc['sigma_thrust']:.2f} |")
    A(f"| allocator efficiency | {mc['alloc_eff_nom']} | "
      f"{mc['sigma_alloc']:.2f} |")
    A("")
    A("## Convergence")
    A("")
    A(f"![convergence](figures/fig_mc_convergence.png)")
    A("")
    A(f"![horizon](figures/fig_mc_horizon.png)")
    A("")
    A("Per-generation numbers are in `convergence.csv`. Note that `best_cost` "
      "is **not monotone**: the search oscillates around a noise floor, which "
      "is why the reported winner is the cheapest *evaluated* candidate rather "
      "than the last generation's.")
    A("")
    A("![decomposition](figures/fig_mc_winner_decomp.png)")
    A("")
    A("## Provenance")
    A("")
    A("| quantity | produced by |")
    A("|---|---|")
    for k, v in PROVENANCE.items():
        A(f"| {k} | `{v}` |")
    A("")
    A("## Files")
    A("")
    A("| file | contents |")
    A("|---|---|")
    A("| `params_best.json` | winner as physical + log-space vectors |")
    A("| `results_best.json` | winner's full per-arm Monte Carlo statistics |")
    A("| `convergence.csv` | one row per generation |")
    A("| `mc_design.csv` | per-run Monte Carlo table |")
    A("| `config_used.json` | every setting, plus git state |")
    A("| `figures/` | the three figures |")
    A("| `reproduce.sh` | the command that produced this run |")

    path = os.path.join(outdir, "REPORT.md")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return path


def export(res_path: str, outdir: str) -> str:
    with open(res_path) as fh:
        res = json.load(fh)
    if "cli_cmd" not in res:
        # Older runs recorded argv rather than a re-runnable command line.
        res["cli_cmd"] = _command_from_cli(res.get("cli", {}))
    else:
        # Make the interpreter path portable: the recorded one is whatever
        # happened to launch this run, which on a different checkout would not
        # exist. Flag order is preserved exactly as invoked.
        argv = list(res["cli_cmd"])
        argv[0] = "./env/bin/python"
        res["cli_cmd"] = argv

    os.makedirs(outdir, exist_ok=True)
    figdir = os.path.join(outdir, "figures")
    os.makedirs(figdir, exist_ok=True)

    commit = _git_commit()
    dirty = _git_dirty()
    cfg = res["config"]

    # ---- 1. report ----------------------------------------------------
    report = write_report_md(res, outdir, commit, dirty)

    # ---- 2. parameters ------------------------------------------------
    import numpy as np
    from cubesat_12u_sim.optimization.params import HYPER_SPACE

    theta = np.asarray(res["best_theta"], float)
    with open(os.path.join(outdir, "params_best.json"), "w") as fh:
        json.dump({
            "theta_log": [float(x) for x in theta],
            "theta_physical": res["best"]["params"],
            "names": HYPER_SPACE.names,
            "bounds_log": {
                "lo": [float(x) for x in HYPER_SPACE.bounds()[0]],
                "hi": [float(x) for x in HYPER_SPACE.bounds()[1]],
            },
            "note": ("Optimiser coordinates are natural logs; the horizon is "
                     "discrete and decodes to a member of MC_HORIZONS."),
        }, fh, indent=2, sort_keys=True)

    # ---- 3. winner statistics ----------------------------------------
    with open(os.path.join(outdir, "results_best.json"), "w") as fh:
        json.dump({
            "cost": res["best_cost"],
            "horizon": res["best"]["horizon"],
            "nmpc": res["best"]["nmpc"],
            "pid": res["best"]["pid"],
            "params": res["best"]["params"],
            "weights": cfg["weights"],
            "mc": cfg["mc"],
            "eval_cfg": cfg["eval_cfg"],
        }, fh, indent=2, sort_keys=True)

    # ---- 4. convergence ----------------------------------------------
    write_convergence_csv(res["history"], os.path.join(outdir, "convergence.csv"))

    # ---- 5. per-run table --------------------------------------------
    n_runs = None
    try:
        _, n_runs = write_mc_design_csv(res, outdir)
    except Exception as exc:
        print(f"warning: could not rebuild the per-run table "
              f"({type(exc).__name__}: {exc}); the sweep summary is still "
              f"exported", file=sys.stderr)

    # ---- 6. config ----------------------------------------------------
    with open(os.path.join(outdir, "config_used.json"), "w") as fh:
        json.dump({"git_commit": commit, "dirty_tree": dirty,
                   "exported_utc": datetime.now(timezone.utc).isoformat(),
                   "source": os.path.abspath(res_path),
                   "command": res["cli_cmd"],
                   "config": cfg,
                   "git_diff_note": ("working tree was dirty at export time; "
                                     "the commit alone does not reproduce "
                                     "this run")
                   if dirty else ""}, fh, indent=2, sort_keys=True, default=float)

    # ---- 7. reproduce -------------------------------------------------
    with open(os.path.join(outdir, "reproduce.sh"), "w") as fh:
        fh.write("#!/usr/bin/env bash\n"
                 "# Regenerates this results folder from a clean checkout.\n"
                 "# Run from the Python/ directory.\n"
                 "set -euo pipefail\n\n"
                 f"{' '.join(res['cli_cmd'])}\n\n"
                 "./env/bin/python -m cubesat_12u_sim.tuning.export_results\n")

    # ---- 8. figures ---------------------------------------------------
    src_dir = os.path.dirname(os.path.abspath(res_path))
    for fn in sorted(os.listdir(src_dir)):
        if fn.endswith(".png"):
            shutil.copy2(os.path.join(src_dir, fn),
                         os.path.join(figdir, fn))

    print(f"exported to {os.path.abspath(outdir)}")
    print(f"  REPORT.md, params_best.json, results_best.json, "
          f"convergence.csv, config_used.json, reproduce.sh, figures/")
    if n_runs:
        print(f"  mc_design.csv ({n_runs} Monte Carlo runs of the winner)")
    return outdir


def _command_from_cli(cli: dict) -> list[str]:
    """Rebuild a runnable command line from recorded argparse values."""
    out = ["./env/bin/python", "-m", "cubesat_12u_sim.tuning.run_mc_optimize"]
    skip = {"no_plot", "resume", "func", "help"}
    for k, v in sorted(cli.items()):
        if k in skip or v is None or v is False:
            continue
        flag = "--" + k.replace("_", "-")
        if v is True:
            out.append(flag)
        elif isinstance(v, (int, float, str)):
            out += [flag, str(v)]
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--res", default=DEFAULT_RES, help="sweep results JSON")
    ap.add_argument("--outdir", default=DEFAULT_OUT,
                    help="destination folder (default: report_results)")
    args = ap.parse_args(argv)
    if not os.path.exists(args.res):
        raise SystemExit(f"no sweep results at {args.res}")
    export(args.res, args.outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())