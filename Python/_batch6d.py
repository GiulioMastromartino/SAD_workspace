"""Throwaway: does "one big batched system" beat N separate 6-state systems?

The question: instead of 100 independent 6-state Kepler integrations, run one
taylor_adaptive_batch with B lanes -- which is layout-identical to a 6B-state
system, SIMD across lanes.

This is NOT hypothetical: _nmpcbatch.cost_batch already takes B lanes, while
nmpc_solve.cpp's plant step uses batch_template(1) inside a serial loop. So the
gradient is already batched and the plant is not, and the same measurement
answers what batching is worth at all.

Measures two regimes:
  homogeneous  -- B lanes from the same vehicle group (what we'd actually do)
  heterogeneous-- B lanes from different vehicle groups (worst case for a
                  shared Taylor order/step)
"""
import time

import numpy as np

import heyoka  # noqa: F401

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.native.cxxdrive import _nmpcbatch
from cubesat_12u_sim.optimization.mcdesign import MCConfig, build_design

MU, ROR = C.MU_EARTH, C.R_ORBIT
DT, N = 600.0, 4
ARGS = (DT, N, 1.0, 0.1, 0.5, 500.0, 0.05, 0.0, MU, ROR, 1e-12)

design = build_design(MCConfig(n_runs=250, n_vehicles=25, seed=1))


def batch_cost(x0s, Zs):
    X = np.ascontiguousarray(np.asarray(x0s, float))
    Z = np.ascontiguousarray(np.asarray(Zs, float))
    out = np.zeros(X.shape[0])
    _nmpcbatch.cost_batch(X, Z, *ARGS, out)
    return out


def timeit(x0s, Zs, reps):
    batch_cost(x0s, Zs)
    t = time.perf_counter()
    for _ in range(reps):
        batch_cost(x0s, Zs)
    return (time.perf_counter() - t) / reps


# homogeneous: lanes from one vehicle group (rows 0..9 share alloc_eff/a_max)
hom_x0 = [design.x0[i] for i in range(10)]
hom_z = [np.zeros(3 * N)] * 10
# heterogeneous: one lane from each of 10 different vehicle groups
het_x0 = [design.x0[g * 10] for g in range(10)]
het_z = [np.zeros(3 * N)] * 10

print("B-lane batch vs B separate 1-lane calls (same total work)")
print(f"{'B':>4} {'batched us':>11} {'serial us':>11} {'serial/batch':>13} "
      f"{'us/lane batch':>14}")
base = None
for B in (1, 2, 5, 10, 20, 50, 100):
    x0s, Zs = hom_x0[:1] * B, hom_z[:1] * B
    reps = max(3, 300 // B)
    tb = timeit(x0s, Zs, reps)
    ts = timeit(x0s[:1], Zs[:1], reps * B)
    s = base if base is None else base
    base = ts / B
    print(f"{B:4d} {tb*1e6:11.1f} {ts*1e6:11.1f} {ts/tb:12.1f}x "
          f"{tb/B*1e6:14.2f}")

print()
print("does a shared Taylor order/step hurt when lanes differ?")
print(f"{'B':>4} {'homogeneous us':>15} {'heterogeneous us':>18} "
      f"{'penalty':>9}")
for B in (10, 50, 100):
    xh = [design.x0[i % 10] for i in range(B)]
    xg = [design.x0[(i * 25) % 250] for i in range(B)]
    reps = max(3, 300 // B)
    th = timeit(xh, [np.zeros(3 * N)] * B, reps)
    tg = timeit(xg, [np.zeros(3 * N)] * B, reps)
    print(f"{B:4d} {th*1e6:15.1f} {tg*1e6:18.1f} {tg/th:8.2f}x")

print()
print("scaling reference: is it linear (no batching) or sublinear (SIMD)?")
for B in (1, 10, 100):
    x0s = [design.x0[i % 10] for i in range(B)]
    t = timeit(x0s, [np.zeros(3 * N)] * B, max(3, 300 // B))
    print(f"  B={B:4d}: {t/B*1e6:7.2f} us/lane")