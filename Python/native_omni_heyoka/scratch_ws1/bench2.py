"""Tilted-wheel demo: stock Basilisk vs omni (C++-propagated), timed.

Generalisation test -- see bench_tilted.py's docstring for why this shape.
"""
from __future__ import annotations

import sys
import time

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)
sys.path.insert(0, SRC + "/native_omni_heyoka")

from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff
from omniheyoka.ctypes_driver import OmniTaylor

DURATION, DT = 600.0, 0.1
K, P = 3.5, 30.0
U_MAX = 0.2
S3 = 1.0 / np.sqrt(3.0)
AXES = np.array([[S3, S3, S3], [-S3, S3, S3], [S3, -S3, S3]])
MU, MASS = 3.986004418e14, 750.0
INERTIA = np.diag([900.0, 800.0, 600.0])
JS = np.full(3, 0.07957747154594767)
RPM0 = np.array([100.0, 200.0, 300.0])
ZERO = np.zeros(3)

from bench_tilted import run_tilted_stock, ics, tilted_u


def run_omni(duration=DURATION, dt=DT):
    r0, v0 = ics()
    om = OmniTaylor(
        inertia9=list(INERTIA.ravel()), mass=MASS,
        axes=[list(a) for a in AXES], Js=list(JS), mu=MU, use_gg=False,
        tol=1e-13, compact=False, use_rpm=True,
        wheel_speeds_rpm=RPM0, r0=r0, v0=v0,
        sigma0=[0.1, 0.2, -0.3], omega0=[0.001, -0.01, 0.03])
    n = int(round(duration / dt))
    out = np.zeros((n + 1, om.n_states))
    out[0] = om.state_np()
    for k in range(1, n + 1):
        st = out[k - 1]
        e = mrp_diff(st[6:9], ZERO)
        Lr = -K * e - P * st[9:12]
        om.set_pars(tilted_u(Lr))
        om.step(dt)
        sn = om.state_np()
        s2 = sn[6:9] @ sn[6:9]
        if s2 > 1.0:
            sn[6:9] = -sn[6:9] / s2
            om.state[6:9] = list(sn[6:9])
        out[k] = sn
    return out


def timeit(fn, repeat=3, warmup=1, **kw):
    for _ in range(warmup):
        fn(**kw)
    ts = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        r = fn(**kw)
        ts.append(time.perf_counter() - t0)
    return min(ts), r


if __name__ == "__main__":
    t_stock, stock = timeit(run_tilted_stock)
    t_omni, omni = timeit(run_omni)

    n = int(DURATION / DT)
    print(f"config : 3 TILTED wheels (pyre 1/sqrt(3)), diag(900,800,600), 750 kg")
    print(f"         {DURATION:g} s @ dt={DT} -> {n} ticks, PD K={K} P={P}")
    print()
    print(f"stock Basilisk (full pipeline, tilted) : {t_stock*1e3:9.2f} ms")
    print(f"omni (C++-propagated dynamics only)    : {t_omni*1e3:9.2f} ms")
    print(f"   -> dynamics slice is {t_stock/t_omni:6.2f}x cheaper than the whole"
          f" stock pipeline")
    print(f"   -> the slice is {100*t_omni/t_stock:5.1f}% of stock runtime"
          f" (the rest is FSW + nav + logging)")
    print()
    m = min(len(stock["sigma_BN"]), len(omni))
    att = np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(omni[-1, 6:9], stock["sigma_BN"][m - 1]))))
    rate = np.degrees(np.linalg.norm(omni[-1, 9:12] - stock["omega_BN_B"][m - 1]))
    print(f"parity vs stock Basilisk, tilted config:")
    print(f"   final attitude error : {att:.3e} deg")
    print(f"   final rate error     : {rate:.3e} deg/s")
    print(f"   |sigma| final        : omni {np.linalg.norm(omni[-1,6:9]):.6f}"
          f"  stock {np.linalg.norm(stock['sigma_BN'][m-1]):.6f}")
    print(f"   final r              : omni {omni[-1,0]:.3f}"
          f"  stock {stock['r_CN_N'][m-1,0]:.3f}")
