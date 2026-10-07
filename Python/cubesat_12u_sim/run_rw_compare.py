"""Stock vs Heyoka comparison on scenarioAttitudeFeedbackRW baseline.

Runs the stock Basilisk demo configuration and the Heyoka RW mirror from
identical ICs, prints divergence metrics, and writes overlay plots.
"""
from __future__ import annotations

import time

import numpy as np


def main() -> int:
    import matplotlib.pyplot as plt

    from .bsk_rw_stock import run_stock_rw
    from .heyoka_attitude import mrp_diff
    from .heyoka_rw import pack_rw_state, run_rw_mirror

    t0 = time.perf_counter()
    s = run_stock_rw(duration=600.0)
    t_stock = time.perf_counter() - t0
    y0 = pack_rw_state(s["r_CN_N"][0], s["v_CN_N"][0], s["sigma_BN"][0],
                       s["omega_BN_B"][0], s["wheel_speeds"][0])
    t0 = time.perf_counter()
    h = run_rw_mirror(duration=600.0, dt=0.1, y0=y0)
    t_hey = time.perf_counter() - t0

    Y = h["Y"]
    n = Y.shape[1] - 1
    att_err = float(np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(Y[6:9, n], s["sigma_BN"][-1])))))
    om_err = float(np.degrees(np.linalg.norm(Y[9:12, n] - s["omega_BN_B"][-1])))
    w_err = np.abs(Y[12:15, n] - s["wheel_speeds"][-1]) * 60 / (2 * np.pi)
    print(f"stock wall {t_stock:.1f}s, heyoka wall {t_hey:.1f}s")
    print(f"final attitude divergence: {att_err:.2e} deg")
    print(f"final rate divergence:     {om_err:.2e} deg/s")
    print(f"final wheel divergence:   {np.max(w_err):.2e} RPM")

    t_min = s["t"] / 60.0
    th = h["t"] / 60.0
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(t_min, np.degrees(np.linalg.norm(s["omega_BN_B"], axis=1)),
                 "k", lw=1.5, label="stock truth")
    axes[0].plot(th, np.degrees(np.linalg.norm(Y[9:12, :].T, axis=1)),
                 "r--", lw=1.0, label="heyoka mirror")
    axes[0].set_ylabel("body rate norm [deg/s]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("RW demo: stock Basilisk vs Heyoka mirror (600 s, K=3.5/P=30)")
    axes[1].plot(t_min, np.linalg.norm(s["sigma_BR"], axis=1), "k", lw=1.5)
    axes[1].plot(th, [np.linalg.norm(mrp_diff(Y[6:9, k], np.zeros(3)))
                       for k in range(n + 1)], "r--", lw=1.0)
    axes[1].set_ylabel("attitude error |MRP|")
    axes[1].grid(True, alpha=0.3)
    for i, c in enumerate("bgr"):
        axes[2].plot(t_min, s["wheel_speeds"][:, i] * 60 / (2 * np.pi),
                     c, lw=1.2, label=f"stock W{i + 1}")
        axes[2].plot(th, Y[12 + i, :] * 60 / (2 * np.pi), c + "--", lw=0.8)
    axes[2].set_ylabel("wheel speed [RPM]")
    axes[2].set_xlabel("time [min]")
    axes[2].legend(fontsize=8, ncol=3)
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    out = "rw_stock_vs_heyoka.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[info] wrote {out}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
