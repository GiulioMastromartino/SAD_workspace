"""Compare C++ Heyoka RW dynamics vs stock Basilisk vs Python Heyoka mirror.

Runs all three configurations from identical ICs and reports divergence metrics.
"""
from __future__ import annotations

import time

import numpy as np


def main() -> int:
    import matplotlib.pyplot as plt

    from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff
    from cubesat_12u_sim.backends.heyoka.heyoka_rw import pack_rw_state, run_rw_mirror
    from cubesat_12u_sim.backends.cpp.cpp_rw import run_cpp_rw_mission, run_cpp_rw_pipeline

    # --- Stock Basilisk ---
    t0 = time.perf_counter()
    s = run_stock_rw(duration=600.0)
    t_stock = time.perf_counter() - t0
    y0 = pack_rw_state(s["r_CN_N"][0], s["v_CN_N"][0], s["sigma_BN"][0],
                       s["omega_BN_B"][0], s["wheel_speeds"][0])

    # --- Python Heyoka mirror (6000 per-tick propagate_for calls) ---
    t0 = time.perf_counter()
    h = run_rw_mirror(duration=600.0, dt=0.1, y0=y0)
    t_pyhey = time.perf_counter() - t0

    # --- Fully C++ Heyoka mission (ONE pybind call, whole loop in C++) ---
    try:
        t0 = time.perf_counter()
        f = run_cpp_rw_mission(y0, duration=600.0, dt=0.1)
        t_fast = time.perf_counter() - t0
        fast_ok = True
    except Exception as e:
        print(f"[warn] fully-C++ mission unavailable ({e}); rebuild rwdrive first")
        f, t_fast, fast_ok = None, float("nan"), False

    # --- C++ Heyoka (Basilisk pipeline with C++ dynamics) ---
    t0 = time.perf_counter()
    c = run_cpp_rw_pipeline(duration=600.0, dt=0.1)
    t_cpphey = time.perf_counter() - t0

    # --- Native Heyoka (true compiled SysModel, stock FSW otherwise) ---
    try:
        from cubesat_12u_sim.backends.cpp.cpp_native import run_native_rw_pipeline

        t0 = time.perf_counter()
        cn = run_native_rw_pipeline(duration=600.0, dt=0.1)
        t_nat = time.perf_counter() - t0
        nat_ok = True
    except (ImportError, RuntimeError) as e:
        print(f"[warn] native pipeline unavailable ({e}); build native_rw first")
        cn, t_nat, nat_ok = None, float("nan"), False

    # --- Compare ---
    Y = h["Y"]
    n = Y.shape[1] - 1

    # C++ results
    sigma_cpp = c["sigma_BN"][-1]
    omega_cpp = c["omega_BN_B"][-1]

    # Python Heyoka results
    sigma_py = Y[6:9, n]
    omega_py = Y[9:12, n]

    # Stock results
    sigma_stock = s["sigma_BN"][-1]
    omega_stock = s["omega_BN_B"][-1]

    # Divergences
    att_err_py = float(np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(sigma_py, sigma_stock)))))
    om_err_py = float(np.degrees(np.linalg.norm(omega_py - omega_stock)))

    att_err_cpp = float(np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(sigma_cpp, sigma_stock)))))
    om_err_cpp = float(np.degrees(np.linalg.norm(omega_cpp - omega_stock)))

    att_err_cpp_vs_py = float(np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(sigma_cpp, sigma_py)))))
    om_err_cpp_vs_py = float(np.degrees(np.linalg.norm(omega_cpp - omega_py)))

    # Wheel speeds in RPM (wheels are pure torque integrators, so this is
    # the strictest parity channel: per-tick torque diffs accumulate here
    # even when hub attitude/rate look clean).
    RPM = 60.0 / (2.0 * np.pi)
    w_stock = np.asarray(s["wheel_speeds"], float)[:, :3] * RPM
    w_py = Y[12:15, :].T * RPM
    w_cpp = np.asarray(c["wheel_speeds"], float)[:, :3] * RPM
    m = min(len(w_stock), len(w_py), len(w_cpp))
    dw_py = np.abs(w_py[:m] - w_stock[:m])
    dw_cpp = np.abs(w_cpp[:m] - w_stock[:m])

    if nat_ok:
        att_err_nat = float(np.degrees(4 * np.arctan(np.linalg.norm(
            mrp_diff(cn["sigma_BN"][-1], sigma_stock)))))
        om_err_nat = float(np.degrees(np.linalg.norm(
            cn["omega_BN_B"][-1] - omega_stock)))
        w_nat = np.asarray(cn["wheel_speeds"], float)[:, :3] * RPM
        mn = min(m, len(w_nat))
        dw_nat = np.abs(w_nat[:mn] - w_stock[:mn])

    print(f"wall times: stock {t_stock:.2f}s, py-heyoka {t_pyhey:.2f}s, "
          f"cpp-pipeline {t_cpphey:.2f}s"
          + (f", native {t_nat:.2f}s" if nat_ok else ", native n/a")
          + (f", cpp-mission {t_fast:.2f}s" if fast_ok else ", cpp-mission n/a"))
    print(f"speedups vs stock: py {t_stock / t_pyhey:.2f}x, "
          f"cpp-pipeline {t_stock / t_cpphey:.2f}x"
          + (f", native {t_stock / t_nat:.2f}x" if nat_ok else "")
          + (f", cpp-mission {t_stock / t_fast:.2f}x" if fast_ok else ""))
    print()
    print("=== vs Stock Basilisk ===")
    print(f"Python Heyoka:  att {att_err_py:.2e} deg, rate {om_err_py:.2e} deg/s")
    print(f"C++ Heyoka:     att {att_err_cpp:.2e} deg, rate {om_err_cpp:.2e} deg/s")
    if nat_ok:
        print(f"Native Heyoka:  att {att_err_nat:.2e} deg, rate {om_err_nat:.2e} deg/s")
    print()
    print("=== wheel-speed residuals vs stock [RPM] (max over run / final) ===")
    for i in range(3):
        line = (f"W{i + 1}: py  max {dw_py[:, i].max():.3e}  final {dw_py[-1, i]:+.3e} | "
                f"cpp max {dw_cpp[:, i].max():.3e}  final {dw_cpp[-1, i]:+.3e}")
        if nat_ok:
            line += (f" | nat max {dw_nat[:, i].max():.3e} "
                     f" final {dw_nat[-1, i]:+.3e}")
        print(line)
    print()
    print("=== C++ Heyoka vs Python Heyoka ===")
    print(f"att {att_err_cpp_vs_py:.2e} deg, rate {om_err_cpp_vs_py:.2e} deg/s")
    if fast_ok:
        Yf, nf = f["Y"], f["Y"].shape[1] - 1
        att_fast = float(np.degrees(4 * np.arctan(np.linalg.norm(
            mrp_diff(Yf[6:9, nf], sigma_py)))))
        om_fast = float(np.degrees(np.linalg.norm(Yf[9:12, nf] - omega_py)))
        print()
        print("=== Fully-C++ mission vs Python mirror (must be ~1e-12) ===")
        print(f"att {att_fast:.2e} deg, rate {om_fast:.2e} deg/s")

    # --- Plot ---
    t_min = s["t"] / 60.0
    th = h["t"] / 60.0
    tc = c["t"] / 60.0

    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axes[0].plot(t_min, np.degrees(np.linalg.norm(s["omega_BN_B"], axis=1)),
                 "k", lw=1.5, label="stock truth")
    axes[0].plot(th, np.degrees(np.linalg.norm(Y[9:12, :].T, axis=1)),
                 "r--", lw=1.0, label="py-heyoka mirror")
    axes[0].plot(tc, np.degrees(np.linalg.norm(c["omega_BN_B"], axis=1)),
                 "b-.", lw=1.0, label="cpp-heyoka")
    axes[0].set_ylabel("body rate norm [deg/s]")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("RW demo: stock vs heyoka backends (600 s, K=3.5/P=30)")

    axes[1].plot(t_min, np.linalg.norm(s["sigma_BR"], axis=1), "k", lw=1.5)
    axes[1].plot(th, [np.linalg.norm(mrp_diff(Y[6:9, k], np.zeros(3)))
                        for k in range(n + 1)], "r--", lw=1.0)
    axes[1].plot(tc, [np.linalg.norm(mrp_diff(c["sigma_BN"][k], np.zeros(3)))
                        for k in range(len(c["t"]))], "b-.", lw=1.0)
    axes[1].set_ylabel("attitude error |MRP|")
    axes[1].grid(True, alpha=0.3)

    for i, c_col in enumerate("bgr"):
        axes[2].plot(t_min, s["wheel_speeds"][:, i] * 60 / (2 * np.pi),
                     c_col, lw=1.2, label=f"stock W{i + 1}")
        axes[2].plot(th, Y[12 + i, :] * 60 / (2 * np.pi), c_col + "--", lw=0.8)
        axes[2].plot(tc, c["wheel_speeds"][:, i] * 60 / (2 * np.pi),
                     c_col + "-.", lw=0.8, label=f"cpp W{i + 1}")
    axes[2].set_ylabel("wheel speed [RPM]")
    axes[2].legend(fontsize=8, ncol=3)
    axes[2].grid(True, alpha=0.3)

    for i, c_col in enumerate("bgr"):
        axes[3].plot(t_min[:m], dw_py[:, i], c_col + "--", lw=0.8)
        axes[3].plot(t_min[:m], dw_cpp[:, i], c_col + "-.", lw=0.8,
                     label=f"|dW{i + 1} cpp|")
        if nat_ok:
            axes[3].plot(t_min[:mn], dw_nat[:, i], c_col + ":",
                         lw=1.0, label=f"|dW{i + 1} nat|")
    axes[3].set_ylabel("wheel residual [RPM]")
    axes[3].set_xlabel("time [min]")
    axes[3].legend(fontsize=8, ncol=3)
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    out = "rw_cpp_vs_stock_vs_py.png"
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"\n[info] wrote {out}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
