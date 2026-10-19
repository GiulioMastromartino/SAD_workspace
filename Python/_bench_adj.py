"""Throwaway: adjoint vs forward-difference gradient, measured.

Corrects an earlier claim of mine. I asserted the adjoint would cut the
gradient from 3N+1 propagations to ~2, i.e. ~9x at H=6. That ignored SIMD: the
C++ grad_batch packs 3N+1 lanes into a Taylor batch that already amortises
across vector width. Measured grad/value was 3.6x at H=4 and 5.2x at H=6, NOT
19x. So the honest ceiling is (measured FD ratio) / (adjoint cost in value-
eval units), and it must be measured, not inferred.
"""
import time

import numpy as np

import heyoka  # noqa: F401

from cubesat_12u_sim.control import adjoint as A
from cubesat_12u_sim.control.nmpc import NMPCConfig, ScipyShootingBackend
from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.native.cxxdrive import _nmpcbatch

MU, ROR = C.MU_EARTH, C.R_ORBIT
x0 = np.concatenate([C.R0_ECI, C.V0_ECI * (1 - 0.005)])


def cpp_ratios(N, tol=1e-12):
    """Measured C++ cost of a value eval and of a grad eval, same lane logic."""
    a = (600.0, N, 1.0, 0.1, 0.5, 500.0, 0.05, 0.0, MU, ROR, tol)
    z = np.zeros(3 * N)
    X = np.ascontiguousarray(x0[None, :])
    Z = np.ascontiguousarray(z[None, :])
    o = np.zeros(1)
    g = np.zeros(3 * N)
    _nmpcbatch.cost_batch(X, Z, *a, o)
    _nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
    r = 300
    t = time.perf_counter()
    for _ in range(r):
        _nmpcbatch.cost_batch(X, Z, *a, o)
    tc = (time.perf_counter() - t) / r
    t = time.perf_counter()
    for _ in range(r):
        _nmpcbatch.grad_batch(x0, np.ascontiguousarray(z), *a, g)
    tg = (time.perf_counter() - t) / r
    return tc, tg


def numpy_fd(N):
    """Pure-numpy forward differences, the reference the adjoint is tested to."""
    z = np.zeros(3 * N)
    _, _, jac = ScipyShootingBackend.__new__(ScipyShootingBackend), None, None
    b = ScipyShootingBackend(use_cpp=False, grad="fd")
    cfg = NMPCConfig(horizon=N, dt_outer=600.0, q_pos=1.0, q_vel=0.1,
                     r_fuel=0.5, db_a=500.0, a_max=0.05)
    cost, _ = b._make_funcs(x0, cfg) if hasattr(b, "_make_funcs") else (None, None)
    return cost, jac


def adjoint_grad_us(N, reps=60):
    U = np.zeros((N, 3))
    U[0, 0] = 1e-6
    A.states_cost_grad  # touch
    t = time.perf_counter()
    for _ in range(reps):
        A.shooting_cost_and_grad(x0, U, dt=600.0, mu=MU, r_orbit=ROR,
                                 q_pos=1.0, q_vel=0.1, r_fuel=0.5, db_a=500.0,
                                 a_max=0.05)
    return (time.perf_counter() - t) / reps


def numpy_value_us(N, reps=60):
    U = np.zeros((N, 3))
    U[0, 0] = 1e-6
    t = time.perf_counter()
    for _ in range(reps):
        A.shooting_cost(x0, U, dt=600.0, mu=MU, r_orbit=ROR, q_pos=1.0,
                        q_vel=0.1, r_fuel=0.5, db_a=500.0, a_max=0.05)
    return (time.perf_counter() - t) / reps


print("C++ FD gradient already amortised across SIMD width:")
print(f"{'H':>3} {'value us':>9} {'grad us':>9} {'FD ratio':>9} {'3N+1 lanes':>11}")
for N in (2, 4, 6, 12):
    tc, tg = cpp_ratios(N)
    print(f"{N:3d} {tc*1e6:9.1f} {tg*1e6:9.1f} {tg/tc:8.1f}x {3*N+1:11d}")

print()
print("numpy adjoint cost (pure python, 1 forward + 1 backward):")
print(f"{'H':>3} {'value us':>9} {'adjoint us':>11} {'ratio':>7}")
for N in (2, 4, 6, 12):
    tv = numpy_value_us(N)
    ta = adjoint_grad_us(N)
    print(f"{N:3d} {tv*1e6:9.1f} {ta*1e6:11.1f} {ta/tv:6.1f}x")

print()
print("=> the adjoint's advantage over the C++ FD gradient, if ported to C++:")
print("   (a ported adjoint would cost ~the same shape as the numpy one but on")
print("    SIMD lanes; the honest bound is the numpy adjoint ratio vs FD ratio)")
for N in (2, 4, 6, 12):
    tc, tg = cpp_ratios(N)
    fd_ratio = tg / tc
    tv = numpy_value_us(N)
    adj_ratio = adjoint_grad_us(N) / tv
    print(f"   H={N:2d}: FD {fd_ratio:5.1f}x vs adjoint ~{adj_ratio:4.1f}x "
          f"-> upper bound {fd_ratio / adj_ratio:4.1f}x")