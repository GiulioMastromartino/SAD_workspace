"""Timing comparison: stock vs Heyoka-Python vs native Heyoka pipeline.

Runs the same scenario on all backends and reports wall times + parity.

Usage (from Python/):
    ./env/bin/python -m cubesat_12u_sim.run_heyoka_pipeline_compare
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


def main() -> int:
    duration = 600.0
    seed = 1
    maneuver = "detumble"
    rate_feedback = "observer"
    task_step = 0.01

    print("=" * 70)
    print("BASILISK STOCK vs HEYOKA-POWERED BASILISK PIPELINE")
    print("=" * 70)
    print(f"Scenario: {maneuver}, {duration:.0f} s, dt={task_step} s")
    print()

    # --- Basilisk stock ---
    print("[1/2] Basilisk stock (native spacecraft dynamics)...")
    try:
        from .basilisk_sim import run_simulation as run_basilisk

        t0 = time.perf_counter()
        bsk_result = run_basilisk(
            duration=duration,
            seed=seed,
            maneuver=maneuver,
            rate_feedback=rate_feedback,
        )
        bsk_time = time.perf_counter() - t0
        print(f"  Wall time: {_fmt(bsk_time)}")
        print(f"  Samples:   {len(bsk_result['time'])}")
        bsk_ok = True
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        bsk_time = float("nan")
        bsk_ok = False

    # --- Heyoka-powered Basilisk ---
    print("[2/3] Heyoka-powered Basilisk pipeline...")
    try:
        from .heyoka_dynamics import run_heyoka_pipeline

        t0 = time.perf_counter()
        hey_result = run_heyoka_pipeline(
            duration=duration,
            seed=seed,
            maneuver=maneuver,
            rate_feedback=rate_feedback,
            task_step=task_step,
        )
        hey_time = time.perf_counter() - t0
        print(f"  Wall time: {_fmt(hey_time)}")
        print(f"  Samples:   {len(hey_result['time'])}")
        hey_ok = True
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        hey_time = float("nan")
        hey_ok = False

    # --- Native Heyoka (compiled SysModel, stock FSW otherwise) ---
    print("[3/3] Native Heyoka pipeline...")
    try:
        from .native_mc import run_native_mc_pipeline

        t0 = time.perf_counter()
        nat_result = run_native_mc_pipeline(
            duration=duration,
            seed=seed,
            maneuver=maneuver,
            rate_feedback=rate_feedback,
            task_step=task_step,
        )
        nat_time = time.perf_counter() - t0
        print(f"  Wall time: {_fmt(nat_time)}")
        print(f"  Samples:   {len(nat_result['time'])}")
        nat_ok = True
    except (ImportError, RuntimeError) as e:
        print(f"  SKIPPED (not built): {e}")
        nat_time = float("nan")
        nat_ok = False
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        nat_time = float("nan")
        nat_ok = False

    # --- Summary ---
    print()
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    if bsk_ok:
        print(f"  Basilisk stock:          {_fmt(bsk_time)}")
    if hey_ok:
        print(f"  Heyoka-powered Basilisk: {_fmt(hey_time)}")
    if nat_ok:
        print(f"  Native Heyoka:           {_fmt(nat_time)}")
    if bsk_ok and hey_ok:
        print(f"\n  Heyoka speedup: {bsk_time / hey_time:.2f}x")

        # Compare final states
        bsk_sigma = np.asarray(bsk_result["truth_attitude"][-1])
        hey_sigma = np.asarray(hey_result["truth_attitude"][-1])
        sigma_err = np.linalg.norm(bsk_sigma - hey_sigma)

        bsk_omega = np.asarray(bsk_result["truth_gyro"][-1])
        hey_omega = np.asarray(hey_result["truth_gyro"][-1])
        omega_err = np.linalg.norm(bsk_omega - hey_omega)

        print(f"\n  Final attitude divergence: {sigma_err:.2e} (MRP norm)")
        print(f"  Final rate divergence:     {omega_err:.2e} (rad/s)")
    if bsk_ok and nat_ok:
        print(f"\n  Native speedup: {bsk_time / nat_time:.2f}x")
        nat_sigma = np.asarray(nat_result["truth_attitude"][-1])
        nat_omega = np.asarray(nat_result["truth_gyro"][-1])
        print(f"\n  Native final attitude divergence: "
              f"{np.linalg.norm(bsk_sigma - nat_sigma):.2e} (MRP norm)")
        print(f"  Native final rate divergence:     "
              f"{np.linalg.norm(bsk_omega - nat_omega):.2e} (rad/s)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
