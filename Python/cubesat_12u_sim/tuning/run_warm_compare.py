"""Warm-wall-time comparison: Basilisk stock vs Heyoka Python vs C++ cxxdrive.

All backends are warmed up first (JIT compiled, caches loaded), then timed
on the second run for a fair comparison.

Usage (from Python/):
    ./env/bin/python -m cubesat_12u_sim.tuning.run_warm_compare
"""
from __future__ import annotations

import sys
import time

import numpy as np


def _fmt(t: float) -> str:
    if t < 1e-3:
        return f"{t * 1e6:.1f} us"
    if t < 1.0:
        return f"{t * 1e3:.1f} ms"
    return f"{t:.2f} s"


# =====================================================================
# Basilisk stock (no JIT, just run twice to warm any caches)
# =====================================================================
def run_basilisk_warm(duration: float = 600.0) -> tuple[dict, float, float]:
    from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw

    # warm-up run
    t0 = time.perf_counter()
    run_stock_rw(duration=duration)
    t_warm = time.perf_counter() - t0

    # timed run
    t0 = time.perf_counter()
    result = run_stock_rw(duration=duration)
    t_timed = time.perf_counter() - t0
    return result, t_warm, t_timed


# =====================================================================
# Heyoka Python RW mirror (warm JIT, then time)
# =====================================================================
def run_heyoka_warm(duration: float = 600.0, y0: np.ndarray | None = None) -> tuple[dict, float, float]:
    from cubesat_12u_sim.backends.heyoka.heyoka_rw import run_rw_mirror

    # warm-up run (includes JIT)
    t0 = time.perf_counter()
    run_rw_mirror(duration=duration, y0=y0)
    t_warm = time.perf_counter() - t0

    # timed run (warm JIT)
    t0 = time.perf_counter()
    result = run_rw_mirror(duration=duration, y0=y0)
    t_timed = time.perf_counter() - t0
    return result, t_warm, t_timed


# =====================================================================
# C++ cxxdrive (warm JIT via disk cache, then time)
# =====================================================================
def run_cxx_warm(duration: float = 600.0, n_lanes: int = 8) -> tuple[dict, float, float]:
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.native import cxxdrive as _cxx
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import precompute_sun_refs, sun_direction_at, wrench_setup
    from cubesat_12u_sim.uncertainty.monte_carlo import sample_dispersions, MonteCarloConfig
    from cubesat_12u_sim.core.actuators import allocation_matrices

    if not _cxx.available:
        raise RuntimeError("cxxdrive extension unavailable")

    mc_cfg = MonteCarloConfig(
        task_step=0.1, log_step=1.0,
        t_detumble=duration * 0.4, t_slew=duration * 0.3,
        t_sunpoint=duration * 0.3,
        sigma_slew=(0.0, 0.0, 0.0),
        sun_roll_ref=(0.0, 0.0, 0.0),
        sun_pointing_axis_b=(0.0, 0.0, 1.0),
        rate_settle_thresh_deg_s=0.1,
        slew_settle_thresh_deg=1.0,
        sun_cone_angle_deg=10.0,
    )
    seeds = list(range(100, 100 + n_lanes))
    dt = float(mc_cfg.task_step)
    n_steps = int(round(mc_cfg.t_total / dt))
    log_every = max(1, int(round(float(mc_cfg.log_step) / dt)))
    disps = [sample_dispersions(int(s), mc_cfg) for s in seeds]
    sun_refs = precompute_sun_refs(mc_cfg, np.asarray(mc_cfg.sun_roll_ref, float), dt)

    sigma_init = np.array([np.asarray(d["sigma0"], float) for d in disps])
    k_det = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                float(mc_cfg.t_detumble), side="left"))
    k_slew = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                 float(mc_cfg.t_detumble + mc_cfg.t_slew),
                                 side="left"))
    REFS = np.empty((n_steps + 1, n_lanes, 3))
    REFS[:k_det] = sigma_init[None, :, :]
    REFS[k_det:k_slew] = np.asarray(mc_cfg.sigma_slew, float)[None, None, :]
    REFS[k_slew:] = np.asarray(sun_refs[k_slew:])[:, None, :]
    PH = np.zeros(n_steps + 1, dtype=np.int64)
    PH[k_det:k_slew] = 1
    PH[k_slew:] = 2

    ws_all = [wrench_setup(d) for d in disps]
    P_nom = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)

    def _call():
        return _cxx.run_chunk(
            np.array([d["r0"] for d in disps], float),
            np.array([d["v0"] for d in disps], float),
            np.array([d["sigma0"] for d in disps], float),
            np.array([d["omega0"] for d in disps], float),
            np.array([float(d["mass"]) for d in disps]),
            np.array([w["I6"] for w in ws_all]),
            np.array([w["invI6"] for w in ws_all]),
            np.stack([w["M_F"] for w in ws_all]),
            np.stack([w["M_T"] for w in ws_all]),
            np.stack([w["tmax"] for w in ws_all]),
            np.array([float(d["thruster_isp"]) for d in disps]),
            np.array([d["gyro_bias"] for d in disps], float),
            np.array([d["gyro_arw"] for d in disps], float),
            np.array([int(s) for s in seeds], dtype=np.uint64),
            P_nom, np.ascontiguousarray(REFS), PH,
            np.array([sun_direction_at(float((k) * dt))
                      for k in range(0, n_steps + 1, log_every)]),
            np.asarray(mc_cfg.sun_pointing_axis_b, float),
            float(C.ACS_K), float(C.ACS_P),
            float(C.ACS_OBSERVER_GAIN), float(C.ACS_OBSERVER_BIAS_GAIN),
            float(C.THRUSTER_CONTROL_SCALE), dt, n_steps, log_every,
            float(C.G0),
            float(mc_cfg.rate_settle_thresh_deg_s),
            float(mc_cfg.slew_settle_thresh_deg),
            float(mc_cfg.sun_cone_angle_deg),
            True, float(C.HEYOKA_TOL), False,
            float(C.MU_EARTH), bool(C.USE_GRAVITY_GRADIENT))

    # warm-up run
    t0 = time.perf_counter()
    _call()
    t_warm = time.perf_counter() - t0

    # timed run
    t0 = time.perf_counter()
    out = _call()
    t_timed = time.perf_counter() - t0

    result = {
        "omega_norm": np.asarray(out["omega_norm"], float),
        "sigma_err": np.asarray(out["sigma_err"], float),
        "sun_angle": np.asarray(out["sun_angle"], float),
        "on_time": np.asarray(out["on_time"], float),
        "phase_fuel": np.asarray(out["phase_fuel"], float),
    }
    return result, t_warm, t_timed


# =====================================================================
# Main
# =====================================================================
def main() -> int:
    duration = 600.0
    n_lanes = 8

    print("=" * 70)
    print("WARM WALL-TIME COMPARISON")
    print("=" * 70)
    print(f"Scenario: {duration:.0f} s, dt=0.1 s ({int(duration / 0.1)} steps)")
    print(f"C++ cxxdrive: {n_lanes} lanes")
    print()

    # --- Basilisk ---
    print("[1/3] Basilisk stock (warm)...")
    try:
        _, bsk_warm, bsk_timed = run_basilisk_warm(duration)
        print(f"  Warm-up: {_fmt(bsk_warm)}")
        print(f"  Timed:   {_fmt(bsk_timed)}")
    except Exception as e:
        print(f"  FAILED: {e}")
        bsk_timed = float("nan")

    # --- Heyoka Python ---
    print("[2/3] Heyoka Python (warm JIT)...")
    try:
        from cubesat_12u_sim.backends.heyoka.heyoka_rw import pack_rw_state
        # Need y0 from Basilisk for fair comparison
        from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw
        bsk_r = run_stock_rw(duration=duration)
        y0 = pack_rw_state(
            bsk_r["r_CN_N"][0], bsk_r["v_CN_N"][0],
            bsk_r["sigma_BN"][0], bsk_r["omega_BN_B"][0],
            bsk_r["wheel_speeds"][0])
        _, hey_warm, hey_timed = run_heyoka_warm(duration, y0=y0)
        print(f"  Warm-up: {_fmt(hey_warm)}")
        print(f"  Timed:   {_fmt(hey_timed)}")
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        hey_timed = float("nan")

    # --- C++ cxxdrive ---
    print(f"[3/3] C++ cxxdrive (warm, {n_lanes} lanes)...")
    try:
        _, cxx_warm, cxx_timed = run_cxx_warm(duration, n_lanes=n_lanes)
        print(f"  Warm-up: {_fmt(cxx_warm)}")
        print(f"  Timed:   {_fmt(cxx_timed)}")
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        cxx_timed = float("nan")

    # --- Summary ---
    print()
    print("=" * 70)
    print("WARM TIMING SUMMARY")
    print("=" * 70)
    print(f"{'Backend':<30} {'Warm-up':>12} {'Timed':>12} {'Speedup vs BSK':>15}")
    print("-" * 70)
    print(f"{'Basilisk stock (RW)':<30} {_fmt(bsk_warm):>12} {_fmt(bsk_timed):>12} {'1.00x':>15}")
    print(f"{'Heyoka Python (RW)':<30} {_fmt(hey_warm):>12} {_fmt(hey_timed):>12} {bsk_timed / hey_timed:>14.2f}x")
    print(f"{'C++ cxxdrive (thruster)':<30} {_fmt(cxx_warm):>12} {_fmt(cxx_timed):>12} {bsk_timed / cxx_timed:>14.2f}x")

    print()
    print("Per-lane compute (timed run / lanes):")
    print(f"  Basilisk stock:  {bsk_timed * 1000:9.2f} ms / 1 lane = {bsk_timed * 1000:9.2f} ms/lane")
    print(f"  Heyoka Python:   {hey_timed * 1000:9.2f} ms / 1 lane = {hey_timed * 1000:9.2f} ms/lane")
    print(f"  C++ cxxdrive:    {cxx_timed * 1000:9.2f} ms / {n_lanes} lanes = {cxx_timed * 1000 / n_lanes:9.2f} ms/lane")

    print()
    print("Note: Basilisk stock uses reaction wheels (15-state);")
    print("      C++ cxxdrive uses thrusters (12-state). Different demos,")
    print("      timing comparison is indicative only.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
