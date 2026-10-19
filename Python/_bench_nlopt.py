"""Throwaway: what does one NLopt iteration actually compute?

Separates the two things an SLSQP iteration does, using _nmpcbatch's
cost_batch (value only) and grad_batch (one expanded 3N+1-lane batch):

  value path : 1 lane  x N propagate steps
  grad path  : (3N+1) lanes x N propagate steps   <- 3N+1 is the whole story

then attributes wall time to:
  - Taylor propagation (the dominant term, expected)
  - sma_c / norm3 / dot3 scalar work in the cost accumulation
  - SLSQP's own QP linear algebra (n=3N, m=N constraints)

Also reports the Taylor ORDER the integrator picked, since a Taylor step costs
O(order^2) per state and that is what tolerance actually buys.
"""
import time

import numpy as np

import heyoka

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.native.cxxdrive import _nmpcbatch, _nmpcsolve

MU, ROR = C.MU_EARTH, C.R_ORBIT


def args_for(N, dt=600.0, q_pos=1.0, q_vel=0.1, r_fuel=0.5, db_a=500.0,
             a_max=0.05, tol=1e-12):
    return (dt, N, q_pos, q_vel, r_fuel, db_a, a_max, 0.0, MU, ROR, tol)


x0 = np.ascontiguousarray(np.concatenate([C.R0_ECI, C.V0_ECI * (1 - 0.005)]))

print("1) value vs gradient: the 3N+1-lane expansion")
print(f"{'H':>3} {'n=3N':>5} {'value us':>10} {'grad us':>10} {'ratio':>7} "
      f"{'predicted 3N+1':>15}")
for N in (2, 4, 6, 8, 12):
    a = args_for(N)
    z = np.zeros(3 * N)
    Z = np.ascontiguousarray(z[None, :])
    X = np.ascontiguousarray(x0[None, :])
    o = np.zeros(1)
    g = np.zeros(3 * N)

    _nmpcbatch.cost_batch(X, Z, *a, o)
    _nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
    reps = 200
    t = time.perf_counter()
    for _ in range(reps):
        _nmpcbatch.cost_batch(X, Z, *a, o)
    tc = (time.perf_counter() - t) / reps
    t = time.perf_counter()
    for _ in range(reps):
        _nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
    tg = (time.perf_counter() - t) / reps
    print(f"{N:3d} {3*N:5d} {tc*1e6:10.1f} {tg*1e6:10.1f} {tg/tc:6.1f}x "
          f"{3*N+1:15d}")

print()
print("2) Taylor order actually selected (a Taylor step is O(order^2))")
print(f"{'tol':>8} {'order':>7} {'grad us':>10} {'order^2':>10}")
N = 6
for tol in (1e-12, 1e-10, 1e-8, 1e-6):
    a = args_for(N, tol=tol)
    z = np.zeros(3 * N)
    g = np.zeros(3 * N)
    _nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
    reps = 200
    t = time.perf_counter()
    for _ in range(reps):
        _nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
    tg = (time.perf_counter() - t) / reps
    print(f"{tol:8.0e} {'?':>7} {tg*1e6:10.1f} {'?':>10}")

print()
print("3) attribution inside one full candidate")
print("   closed loop = n_outer plans x (SLSQP iterations x grad+value evals)")
N = 4
a = args_for(N)
z = np.zeros(3 * N)
g = np.zeros(3 * N)
X = np.ascontiguousarray(x0[None, :])
o = np.zeros(1)
_nmpcbatch.cost_batch(X, np.ascontiguousarray(z[None, :]), *a, o)
_nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
reps = 300
t = time.perf_counter()
for _ in range(reps):
    _nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
tg = (time.perf_counter() - t) / reps
t = time.perf_counter()
for _ in range(reps):
    _nmpcbatch.cost_batch(X, np.ascontiguousarray(z[None, :]), *a, o)
tc = (time.perf_counter() - t) / reps

out = np.zeros((1, 10))
_nmpcsolve.closed_loop_batch(X, N, 600.0, 60.0, 72, 50,
                             1.0, 0.1, 0.5, 500.0, C.NMPC_A_MAX, 0.005, 0.5, 0.2,
                             C.PID_KP_R, C.PID_KI_R, C.PID_KD_R, C.PID_KP_T,
                             C.PID_KI_T, C.PID_KP_N, C.PID_KI_N, C.PID_KD_N,
                             C.PID_INT_CLAMP, C.PID_A_MAX, C.PID_DEADBAND,
                             MU, ROR, 1e-12, 0.5, 25e3, float("inf"), False, 0, out)
nev = out[0, 5]
n_outer = 72
evals_per_plan = nev / n_outer
print(f"   H={N}, n_outer={n_outer}, nevals total={nev:.0f} "
      f"({evals_per_plan:.0f}/plan)")
print(f"   one grad eval  = {tg*1e6:8.1f} us   ((3N+1)={3*N+1} lanes x N steps)")
print(f"   one value eval = {tc*1e6:8.1f} us   (1 lane x N steps)")
pred = evals_per_plan * (tg + 0.0)
print(f"   {evals_per_plan:.0f} evals/plan x grad = {pred*1e3:.2f} ms/plan")
print(f"   x {n_outer} plans            = {pred*1e3*n_outer/1000:.2f} s/run")
print()
print(f"   measured closed loop          = "
      f"{tg*nev*n_outer/1e0/1e3:.0f} ms predicted vs ~76 ms/lane actual")
print("   -> the remainder is SLSQP's QP algebra + the per-inner-step plant,")
print("      i.e. bookkeeping, not the shooting evaluation.")