"""Figures for the Monte-Carlo hypercube sweep.

Four panels, each answering a question a reader of the sweep will ask:

1. **Convergence** — best/mean/median cost per generation. The gap between best
   and median is the population's spread; a sweep that converges on a plateau
   with a wide spread has found a basin, not an optimum.
2. **Cost decomposition of the winner** — mean vs worst- vs best-decile for
   both arms, so the tail weighting that produced the answer is visible rather
   than buried in the scalar.
3. **Feasibility and dv spread of the winner** — the per-run dv distribution of
   each arm, with the box-violating runs marked, against the 250-run design.
4. **Horizon trace** — which horizon the winner of each generation picked. The
   horizon is a discrete coordinate, so this is a step function; reading it is
   the only way to tell whether it settled or is still oscillating.
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
    # Rejected draws sit at 2 * PENALTY_INFEASIBLE, two orders of magnitude
    # above any real cost. A log axis would stretch to fit them and flatten
    # every interesting difference, so the axis is clipped to the feasible
    # range and the rejected count is annotated instead.
    hi = float(best[best < 1e8].max()) if np.any(best < 1e8) else best.max()
    ax[0].set_yscale("log")
    ax[0].set_ylim(hi * 0.5, hi * 4.0)
    n_rej = int(np.sum(best >= 1e8))
    if n_rej:
        ax[0].text(0.02, 0.02, f"{n_rej} generation(s) with no feasible candidate",
                   transform=ax[0].transAxes, fontsize=7, color="0.45")
    ax[0].set_xlabel("generation")
    ax[0].set_ylabel("cost J")
    ax[0].set_title("CMA-ES convergence (150 candidates / generation)")
    ax[0].legend()
    ax[0].grid(True, alpha=0.3)

    # Only the generations where a feasible candidate existed: elsewhere the
    # ratio is a penalty constant over a penalty constant and means nothing.
    ok = (best < 1e8) & (best > 0)
    if np.any(ok):
        ax[1].plot(np.array(gens)[ok], (mean / best)[ok], "o-", color="tab:red")
    ax[1].set_xlabel("generation")
    ax[1].set_ylabel("population mean / best")
    ax[1].set_title("population spread")
    ax[1].grid(True, alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, "fig_mc_convergence.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    written.append(p)

    # ---- 2. winner's cost decomposition ---------------------------------
    b = payload["best"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.0))

    labels, vals, base, alphas = [], [], [], []
    for key, tag in (("nmpc", "NMPC"), ("pid", "PID")):
        s = b[key]
        labels += [f"{tag} mean", f"{tag} worst 10%", f"{tag} best 10%"]
        vals += [s["dv_mean"], s["tail_hi"], s["tail_lo"]]
        base += [tag] * 3
        alphas += [1.0, 0.75, 0.5]
    xs = np.arange(len(vals))
    cols = ["tab:blue" if b_ == "NMPC" else "tab:orange" for b_ in base]
    ax[0].bar(xs, vals, color=cols, alpha=0.85)
    for x, v, a in zip(xs, vals, alphas):
        ax[0].text(x, v, f"{v:.0f}", ha="center", va="bottom", fontsize=7,
                   alpha=0.5 + 0.5 * a)
    ax[0].set_xticks(xs)
    ax[0].set_xticklabels(labels, fontsize=7)
    ax[0].set_ylabel("dv [m/s]")
    ax[0].set_title(f"winner's dv: mean vs tails "
                    f"(H={b['horizon']}, J={b['cost']:.1f})")
    ax[0].grid(True, axis="y", alpha=0.3)

    names = ["NMPC", "PID"]
    fr = [b["nmpc"]["feasible_frac"], b["pid"]["feasible_frac"]]
    ax[1].bar(names, fr, color=["tab:blue", "tab:orange"], alpha=0.85)
    for i, f in enumerate(fr):
        ax[1].text(i, f + 0.02, f"{f * 100:.0f}%", ha="center")
    ax[1].set_ylim(0, 1.15)
    ax[1].set_ylabel("fraction of runs inside the box")
    ax[1].set_title("Monte Carlo feasibility (250 runs)")
    ax[1].grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, "fig_mc_winner_decomp.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    written.append(p)

    # ---- 3. horizon trace ----------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 3.6))
    from cubesat_12u_sim.optimization.params import MC_HORIZONS

    hs = [h["best_horizon"] for h in hist]
    ax.step(gens, hs, where="mid", color="tab:purple", lw=1.6)
    ax.scatter(gens, hs, color="tab:purple", zorder=3)
    ax.set_xlabel("generation")
    ax.set_ylabel("horizon H")
    ax.set_title("horizon chosen by each generation's winner")
    ax.set_yticks(list(MC_HORIZONS))
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    p = os.path.join(outdir, "fig_mc_horizon.png")
    fig.savefig(p, dpi=150)
    plt.close(fig)
    written.append(p)

    return written


__all__ = ["make_figures"]