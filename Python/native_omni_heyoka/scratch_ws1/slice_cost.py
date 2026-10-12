"""Honest dynamics-slice cost: C++ bulk stepping, no Python per-step overhead.

Answers two questions:
  1. What does the runtime-COMPOSED symbolic system cost per step, vs the
     hand-written build_rw_system()?  (i.e. does composition cost anything?)
  2. What is the true dynamics slice of the stock pipeline?
"""
from __future__ import annotations

import ctypes
import sys
import time

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)
sys.path.insert(0, SRC + "/native_omni_heyoka")

from omniheyoka.ctypes_driver import _lib, OmniTaylor

_lib.omni_taylor_run.restype = ctypes.c_double
_lib.omni_taylor_run.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                 ctypes.c_double, ctypes.c_void_p]

DURATION, DT = 600.0, 0.1
MU, MASS = 3.986004418e14, 750.0
INERTIA = np.diag([900.0, 800.0, 600.0])
JS = np.full(3, 0.07957747154594767)
RPM0 = np.array([100.0, 200.0, 300.0])
S3 = 1.0 / np.sqrt(3.0)
ALIGNED = np.eye(3)
TILTED = np.array([[S3, S3, S3], [-S3, S3, S3], [S3, -S3, S3]])
N = int(DURATION / DT)


def bulk(axes, n=N, dt=DT):
    om = OmniTaylor(list(INERTIA.ravel()), MASS, [list(a) for a in axes],
                    list(JS), MU, False, 1e-13, False, True, RPM0,
                    [1e7, 2e6, -1.5e6], [-700., 5500., 3200.],
                    [0.1, 0.2, -0.3], [0.001, -0.01, 0.03])
    p = (ctypes.c_double * 3)(0.0, 0.0, 0.0)
    _lib.omni_taylor_run(om._h, n, dt, p)
    return om


def bench(fn, repeat=7, warmup=2):
    for _ in range(warmup):
        fn()
    ts = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        fn()
        ts.append(time.perf_counter() - t0)
    return min(ts)


if __name__ == "__main__":
    t_al = bench(lambda: bulk(ALIGNED))
    t_ti = bench(lambda: bulk(TILTED))
    print(f"C++ bulk Taylor stepping, {N} steps @ dt={DT} ({DURATION:g} s):")
    print(f"  composed system, ALIGNED wheels : {t_al*1e3:8.2f} ms "
          f"({t_al/N*1e6:.2f} us/step)")
    print(f"  composed system, TILTED wheels  : {t_ti*1e3:8.2f} ms "
          f"({t_ti/N*1e6:.2f} us/step)")
    print(f"  -> tilt costs {(t_ti/t_al-1)*100:.1f}% more per step (same solve,")
    print(f"     only the gs vectors differ -- as designed)")

    # reference: hand-written symbolic system, same integrator settings
    import heyoka as hy
    from cubesat_12u_sim.backends.heyoka.heyoka_rw import build_rw_system, pack_rw_pars
    P = pack_rw_pars(np.zeros(3), 750., np.diag([900., 800., 600.]), JS)
    y0 = np.concatenate([[1e7, 2e6, -1.5e6], [-700., 5500., 3200.],
                         [.1, .2, -.3], [.001, -.01, .03], RPM0 * 2 * np.pi / 60.])
    sys_ref = build_rw_system()

    def ref_run():
        ta = hy.taylor_adaptive(sys_ref, y0.tolist(), pars=list(map(float, P)),
                               tol=1e-13, compact_mode=False)
        for _ in range(N):
            ta.state[:] = y0  # keep the same work each step
            ta.propagate_for(DT)

    t_ref = bench(ref_run)
    print(f"\n  hand-written build_rw_system()   : {t_ref*1e3:8.2f} ms "
          f"({t_ref/N*1e6:.2f} us/step)")
    print(f"  -> composition overhead vs hand-writing: {t_al/t_ref:.2f}x per step")
    print()
    print("measured earlier for context:")
    print("  native_rw FULL pipeline (stock FSW + C++ dynamics) : 15.20 ms")
    print(f"  our composed dynamics slice (C++ only)            : {t_al*1e3:.1f} ms")
    print("  stock Basilisk FULL pipeline                       : ~110-162 ms")
