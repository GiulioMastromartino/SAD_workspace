"""Timing comparison: Basilisk stock vs Heyoka Python vs C++ cxxdrive.

Runs the same 600 s attitude control scenario on all three backends and
reports wall-clock times + speedups.

Usage (from Python/):
    ./env/bin/python -m cubesat_12u_sim.tuning.run_timing_compare
"""
from __future__ import annotations

import sys
import time

import numpy as np


def _fmt_time(t: float) -> str:
    if t < 1e-3:
        return f"{t * 1e6:.1f} us"
    if t < 1.0:
        return f"{t * 1e3:.1f} ms"
    return f"{t:.2f} s"


def run_basilisk_stock(duration: float = 600.0) -> tuple[dict, float]:
    """Run the stock Basilisk RW demo and return (result, wall_time)."""
    from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw

    t0 = time.perf_counter()
    result = run_stock_rw(duration=duration)
    wall = time.perf_counter() - t0
    return result, wall


def run_heyoka_rw(duration: float = 600.0, y0: np.ndarray | None = None) -> tuple[dict, float]:
    """Run the Heyoka Python RW mirror and return (result, wall_time)."""
    from cubesat_12u_sim.backends.heyoka.heyoka_rw import run_rw_mirror

    t0 = time.perf_counter()
    result = run_rw_mirror(duration=duration, y0=y0)
    wall = time.perf_counter() - t0
    return result, wall


def run_cxxdrive(duration: float = 600.0, n_lanes: int = 8) -> tuple[dict, float]:
    """Run the C++ cxxdrive thruster MC and return (result, wall_time)."""
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.native import cxxdrive as _cxx
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import precompute_sun_refs, sun_direction_at, wrench_setup
    from cubesat_12u_sim.uncertainty.monte_carlo import sample_dispersions, MonteCarloConfig

    if not _cxx.available:
        raise RuntimeError("cxxdrive extension unavailable")

    # Build a minimal MC config matching the Basilisk demo duration
    # t_total = t_detumble + t_slew + t_sunpoint (property, not a field)
    mc_cfg = MonteCarloConfig(
        task_step=0.1,
        log_step=1.0,
        t_detumble=duration * 0.4,
        t_slew=duration * 0.3,
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
    n_log = n_steps // log_every + 1

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

    from cubesat_12u_sim.core.actuators import allocation_matrices
    ws_all = [wrench_setup(d) for d in disps]
    P_nom = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)

    t0 = time.perf_counter()
    out = _cxx.run_chunk(
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
    wall = time.perf_counter() - t0

    result = {
        "omega_norm": np.asarray(out["omega_norm"], float),
        "sigma_err": np.asarray(out["sigma_err"], float),
        "sun_angle": np.asarray(out["sun_angle"], float),
        "on_time": np.asarray(out["on_time"], float),
        "phase_fuel": np.asarray(out["phase_fuel"], float),
        "n_lanes": n_lanes,
        "n_steps": n_steps,
    }
    return result, wall


def main() -> int:
    duration = 600.0
    n_lanes = 8

    print("=" * 70)
    print("TIMING COMPARISON: Basilisk stock vs Heyoka Python vs C++ cxxdrive")
    print("=" * 70)
    print(f"Scenario: {duration:.0f} s attitude control, dt=0.1 s")
    print()

    # --- Basilisk stock ---
    print("[1/3] Running Basilisk stock (RW demo)...")
    try:
        bsk_result, bsk_time = run_basilisk_stock(duration)
        print(f"      Wall time: {_fmt_time(bsk_time)}")
        print(f"      Steps: {len(bsk_result['t'])}")
        bsk_ok = True
    except Exception as e:
        print(f"      FAILED: {e}")
        bsk_time = float("nan")
        bsk_ok = False

    # --- Heyoka Python ---
    print("[2/3] Running Heyoka Python RW mirror...")
    try:
        from cubesat_12u_sim.backends.heyoka.heyoka_rw import pack_rw_state
        y0 = pack_rw_state(
            bsk_result["r_CN_N"][0], bsk_result["v_CN_N"][0],
            bsk_result["sigma_BN"][0], bsk_result["omega_BN_B"][0],
            bsk_result["wheel_speeds"][0]) if bsk_ok else None
        hey_result, hey_time = run_heyoka_rw(duration, y0=y0)
        print(f"      Wall time: {_fmt_time(hey_time)}")
        print(f"      Steps: {len(hey_result['t'])}")
        hey_ok = True
    except Exception as e:
        print(f"      FAILED: {e}")
        hey_time = float("nan")
        hey_ok = False

    # --- C++ cxxdrive ---
    print(f"[3/3] Running C++ cxxdrive (thruster MC, {n_lanes} lanes)...")
    try:
        cxx_result, cxx_time = run_cxxdrive(duration, n_lanes=n_lanes)
        print(f"      Wall time: {_fmt_time(cxx_time)}")
        print(f"      Steps: {cxx_result['n_steps']}, Lanes: {cxx_result['n_lanes']}")
        cxx_ok = True
    except Exception as e:
        print(f"      FAILED: {e}")
        cxx_time = float("nan")
        cxx_ok = False

    # --- Summary ---
    print()
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"{'Backend':<30} {'Wall time':>12} {'Speedup vs BSK':>15}")
    print("-" * 70)
    if bsk_ok:
        print(f"{'Basilisk stock (RW)':<30} {_fmt_time(bsk_time):>12} {'1.00x':>15}")
    if hey_ok:
        speedup = bsk_time / hey_time if bsk_ok else float("nan")
        print(f"{'Heyoka Python (RW)':<30} {_fmt_time(hey_time):>12} {speedup:>14.2f}x")
    if cxx_ok:
        speedup = bsk_time / cxx_time if bsk_ok else float("nan")
        print(f"{'C++ cxxdrive (thruster)':<30} {_fmt_time(cxx_time):>12} {speedup:>14.2f}x")

    if bsk_ok and hey_ok:
        print()
        print(f"Heyoka Python speedup vs Basilisk stock: {bsk_time / hey_time:.2f}x")
    if bsk_ok and cxx_ok:
        print(f"C++ cxxdrive speedup vs Basilisk stock:  {bsk_time / cxx_time:.2f}x")
    if hey_ok and cxx_ok:
        print(f"C++ cxxdrive speedup vs Heyoka Python:  {hey_time / cxx_time:.2f}x")

    print()
    print("Note: Basilisk stock uses reaction wheels (15-state);")
    print("      C++ cxxdrive uses thrusters (12-state). Different demos,")
    print("      timing comparison is indicative only.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
