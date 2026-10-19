"""Forensic reruns of RW MC seeds with full series (att/wheels/torque overlay).

Pinpoints WHERE two backends diverge: late + wheels pinned => saturation;
early transient => modeling (lag/jitter/friction) difference.

Usage (seeds printed by the --compare-backend worst list):
    ./env/bin/python -m cubesat_12u_sim.benchmarks.run_rw_forensic --seeds 123 456 --save-dir results_rwforensic
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def _full_series(seed, cfg, backend):
    """Run one seed on one backend; return (t_min, att_deg, w_rpm, u_norm)."""
    import heyoka  # noqa: F401

    from cubesat_12u_sim.uncertainty.rw_monte_carlo import _att_series_deg, sample_rw_dispersions

    d = sample_rw_dispersions(seed, cfg)
    if backend == "stock":
        from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw

        r = run_stock_rw(cfg.duration, cfg.dt, cfg.K, cfg.P, d["mass"],
                         d["inertia"], d["r0"], d["v0"], d["sigma0"],
                         d["omega0"], d["wheels_rpm"])
        t, sig = r["t"], r["sigma_BN"]
        w = np.asarray(r["wheel_speeds"], float)
        u = np.asarray(r["motor_torque"], float)
    elif backend == "native":
        from cubesat_12u_sim.backends.cpp.cpp_native import run_native_rw_pipeline

        r = run_native_rw_pipeline(cfg.duration, cfg.dt, cfg.K, cfg.P,
                                   d["mass"], d["inertia"], d["r0"], d["v0"],
                                   d["sigma0"], d["omega0"], d["wheels_rpm"])
        t, sig = r["t"], r["sigma_BN"]
        w = np.asarray(r["wheel_speeds"], float)
        u = np.asarray(r["motor_torque"], float)
    elif backend == "py":
        from cubesat_12u_sim.backends.heyoka.heyoka_rw import pack_rw_state, run_rw_mirror

        y0 = pack_rw_state(d["r0"], d["v0"], d["sigma0"], d["omega0"],
                           d["wheels_rpm"] * 2.0 * np.pi / 60.0)
        h = run_rw_mirror(cfg.duration, cfg.dt, y0, cfg.K, cfg.P,
                          d["mass"], d["inertia"])
        Y = h["Y"]
        t, sig = h["t"], Y[6:9, :].T
        w = Y[12:15, :].T
        u = h["u_motor"].T
    elif backend == "mission":
        from cubesat_12u_sim.backends.cpp.cpp_rw import run_cpp_rw_mission
        from cubesat_12u_sim.backends.heyoka.heyoka_rw import pack_rw_state

        y0 = pack_rw_state(d["r0"], d["v0"], d["sigma0"], d["omega0"],
                           d["wheels_rpm"] * 2.0 * np.pi / 60.0)
        h = run_cpp_rw_mission(y0, cfg.duration, cfg.dt, cfg.K, cfg.P)
        Y = h["Y"]
        t, sig = h["t"], Y[6:9, :].T
        w = Y[12:15, :].T
        u = h["u_motor"].T
    else:
        raise ValueError(backend)
    RPM = 60.0 / (2.0 * np.pi)
    return (np.asarray(t, float) / 60.0, _att_series_deg(sig),
            np.abs(np.asarray(w, float)) * RPM,
            np.linalg.norm(np.asarray(u, float), axis=1))


def main(argv=None) -> int:
    from cubesat_12u_sim.uncertainty.rw_monte_carlo import RWMonteCarloConfig

    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", required=True)
    ap.add_argument("--backends", nargs="+",
                    default=["stock", "mission"])
    ap.add_argument("--save-dir", default="results_rwforensic")
    ap.add_argument("--duration", type=float, default=600.0)
    a = ap.parse_args(argv)
    cfg = RWMonteCarloConfig()
    cfg.duration = float(a.duration)

    import matplotlib.pyplot as plt

    d = Path(a.save_dir)
    d.mkdir(parents=True, exist_ok=True)
    for seed in a.seeds:
        series = {b: _full_series(seed, cfg, b) for b in a.backends}
        b0, b1 = a.backends[0], a.backends[-1]
        t0, att0, _, _ = series[b0]
        t1, att1, _, _ = series[b1]
        n = min(len(att0), len(att1))
        datt = np.abs(att0[:n] - att1[:n])
        onset = next((float(t0[k]) for k in range(n) if datt[k] > 0.01), None)
        print(f"seed {seed}: final datt {datt[-1]:.3f} deg, "
              f"first 0.01-deg breach at "
              f"{f'{onset:.2f} min' if onset is not None else 'never'}")
        np.savez_compressed(d / f"seed_{seed}.npz",
                            **{f"{b}_att": series[b][1] for b in a.backends},
                            **{f"{b}_w": series[b][2] for b in a.backends},
                            **{f"{b}_u": series[b][3] for b in a.backends})
        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        styles = ["k", "r--", "b-.", "g:"]
        for j, b in enumerate(a.backends):
            t, att, w, u = series[b]
            axes[0].plot(t, att, styles[j % 4], lw=1.2, label=f"{b} att")
            for i, cc in enumerate("bgr"):  # solid=b0, dashed=others
                axes[1].plot(t, w[:, i], cc + ("-" if j == 0 else "--"),
                             lw=0.8, alpha=0.9 if j == 0 else 0.6)
            axes[2].plot(t, u, styles[j % 4], lw=1.0, label=f"{b} |u|")
        axes[0].set_ylabel("att err [deg]")
        axes[0].legend(fontsize=8)
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title(f"RW forensic seed {seed}: {b0} vs {b1}")
        axes[1].set_ylabel("|wheel| [RPM]")
        axes[1].grid(True, alpha=0.3)
        axes[2].set_ylabel("|motor torque| [Nm]")
        axes[2].set_xlabel("time [min]")
        axes[2].legend(fontsize=8)
        axes[2].grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(d / f"seed_{seed}.png", dpi=120)
        plt.close(fig)
        print(f"[info] wrote {d / f'seed_{seed}.png'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
