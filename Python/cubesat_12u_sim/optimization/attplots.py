"""Figures for the attitude-loop hypercube sweep.

Three panels, each answering a question a reader of the sweep will ask:

1. **Convergence** -- best/mean/median cost per generation, plus the
   population spread (mean/best). A sweep that converges on a plateau with a
   wide spread has found a basin, not an optimum.
2. **Winner's cost decomposition** -- fuel mean vs worst-/best-decile, and
   feasibility. The tail weighting that produced the answer stays visible
   rather than buried in the scalar.
3. **Winner's phase table** -- settle times and fuel per maneuver, so the
   three phases can be read separately instead of as one number.
"""
from __future__ import annotations

import os

import numpy as np


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def make_figures(payload: dict, outdir: str) -> list[str]:
    plt = _mpl()
    os.makedirs(outdir, exist_ok=True)
    hist = payload.get("history") or []
    if not hist:
        return []
    written: list[str] = []

    gens = [h["gen"] for h in hist]
    best = np.array([h["best_cost"] for h in hist])
    mean = np.array([h["mean_cost"] for h in hist])
    med = np.array([h["median_cost"] for h in hist])

    # ---- 1. convergence -------------------------------------------------
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    ax[0].plot(gens, best, "o-", label="best", lw=1.8)
    ax[0].plot(gens, med, "s--", label="median", lw=1.2, alpha=0.8)
    ax[0].plot(gens, mean, ":", label="mean", lw=1.0, alpha=0.6)
    # Rejected draws sit at 2 * PENALTY_INFEASIBLE, far above any real cost. A
    # log axis would stretch to fit them and flatten every real difference, so
    # the axis is clipped to the feasible range instead.
    feas = best[best < 1e8]
    if len(feas):
        hi = float(feas.max())
        ax[0].set_ylim(hi * 0.5, hi * 4.0)
    ax[0].set_yscale("log")
    ax[0].set_xlabel("generation")
    ax[0].set_ylabel("cost J [g-equivalent]")
    ax[0].set_title("CMA-ES convergence (attitude loop)")
    ax[0].legend()
    ax[0].grid(True, alpha=0.3)

    ok = (best < 1e8) & (best > 0)
    if np.any(ok):
        ax[1].plot(np.array(gens)[ok], (mean / best)[ok], "o-",
                   color="tab:red")
    ax[1].set_xlabel("generation")
    ax[1].set_ylabel("population mean / best")
    ax[1].set_title("population spread")
    ax[1].grid(True, alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, "fig_att_convergence.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    written.append(p)

    # ---- 2. winner's decomposition --------------------------------------
    b = payload["best"]
    s = b["stats"]
    n_runs = payload.get("config", {}).get("n_runs", "?")
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.0))

    labels = ["fuel\nmean", "fuel\nworst 10%", "fuel\nbest 10%"]
    vals = [s["fuel_mean"], s["tail_hi"], s["tail_lo"]]
    xs = np.arange(len(vals))
    ax[0].bar(xs, vals, color="tab:blue", alpha=0.85)
    for x, v in zip(xs, vals):
        ax[0].text(x, v, f"{v:.2f}", ha="center", va="bottom", fontsize=8)
    ax[0].set_xticks(xs)
    ax[0].set_xticklabels(labels, fontsize=8)
    ax[0].set_ylabel("propellant [g]")
    ax[0].set_title(f"winner's fuel: mean vs tails (J={b['cost']:.1f})")
    ax[0].grid(True, axis="y", alpha=0.3)

    fr = s["feasible_frac"]
    ax[1].bar(["feasible"], [fr], color="tab:green", alpha=0.85)
    ax[1].text(0, fr + 0.02, f"{fr * 100:.0f}%", ha="center")
    ax[1].set_ylim(0, 1.15)
    ax[1].set_ylabel("fraction of runs succeeding")
    ax[1].set_title(f"Monte Carlo success ({n_runs} runs)")
    ax[1].grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, "fig_att_winner_decomp.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    written.append(p)

    # ---- 3. phase table --------------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.axis("off")
    rows = [
        ["detumble settle [s]", f"{s['det_settle_mean']:.1f}"],
        ["slew settle [s]", f"{s['slew_settle_mean']:.1f}"],
        ["sun worst angle [deg]", f"{s['sun_max_mean']:.2f}"],
        ["fuel mean [g]", f"{s['fuel_mean']:.3f}"],
        ["fuel p90 [g]", f"{s['fuel_p90']:.3f}"],
        ["infeasible runs", f"{s['n_infeasible']}"],
    ]
    tbl = ax.table(cellText=rows, colLabels=["quantity", "winner"],
                   loc="center", colWidths=[0.55, 0.3])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9)
    ax.set_title("winner by maneuver")
    fig.tight_layout()
    p = os.path.join(outdir, "fig_att_phases.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    written.append(p)

    return written


__all__ = ["make_figures"]
