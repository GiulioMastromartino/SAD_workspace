"""Diagnose: does the ctypes driver reproduce the ALIGNED case vs run_stock_rw?

If aligned passes and tilted fails, the bug is tilt-specific.  If aligned also
fails, the bug is in the driver (control law, ICs, or MRP switching).
"""
from __future__ import annotations

import sys
import time

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)
sys.path.insert(0, SRC + "/native_omni_heyoka")

from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw
from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff
from omniheyoka.ctypes_driver import OmniTaylor

DURATION, DT = 120.0, 0.1
K, P = 3.5, 30.0
U_MAX = 0.2
MU, MASS = 3.986004418e14, 750.0
INERTIA = np.diag([900.0, 800.0, 600.0])
JS = np.full(3, 0.07957747154594767)
RPM0 = np.array([100.0, 200.0, 300.0])
ZERO = np.zeros(3)

from bench_tilted import ics


def omni_run(axes, duration=DURATION, dt=DT, verbose=False):
    r0, v0 = ics()
    om = OmniTaylor(list(INERTIA.ravel()), MASS, [list(a) for a in axes],
                    list(JS), MU, False, 1e-13, False, True,
                    RPM0, r0, v0, [0.1, 0.2, -0.3], [0.001, -0.01, 0.03])
    n = int(round(duration / dt))
    out = np.zeros((n + 1, om.n_states))
    out[0] = om.state_np()
    G = np.asarray(axes).T
    for k in range(1, n + 1):
        st = out[k - 1]
        e = mrp_diff(st[6:9], ZERO)
        Lr = -K * e - P * st[9:12]
        u = np.clip(np.linalg.solve(G, -Lr), -U_MAX, U_MAX)
        om.set_pars(u)
        om.step(dt)
        sn = om.state_np()
        s2 = sn[6:9] @ sn[6:9]
        if s2 > 1.0:
            sn[6:9] = -sn[6:9] / s2
            om.state[6:9] = list(sn[6:9])
        out[k] = sn
    return out


def compare(tag, stock, omni):
    n = min(len(stock["sigma_BN"]), len(omni))
    att = np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(omni[-1, 6:9], stock["sigma_BN"][n - 1]))))
    rate = np.degrees(np.linalg.norm(omni[-1, 9:12] - stock["omega_BN_B"][n - 1]))
    print(f"  {tag}: att={att:.3e} deg  rate={rate:.3e} deg/s  "
          f"|sig| {np.linalg.norm(omni[-1,6:9]):.6f} vs "
          f"{np.linalg.norm(stock['sigma_BN'][n-1]):.6f}")


if __name__ == "__main__":
    aligned = np.eye(3)
    S3 = 1.0 / np.sqrt(3.0)
    tilted = np.array([[S3, S3, S3], [-S3, S3, S3], [S3, -S3, S3]])

    print("ALIGNED (the validated native_rw case):")
    a = run_stock_rw(duration=DURATION, dt=DT)
    oa = omni_run(aligned)
    compare("omni vs stock", a, oa)

    print("TILTED:")
    t = run_tilted_stock(duration=DURATION, dt=DT) if False else None
    # reuse the stock runner but with tilted axes by direct construction
    from bench_tilted import run_tilted_stock as _rts
    t = _rts(duration=DURATION, dt=DT)
    ot = omni_run(tilted)
    compare("omni vs stock", t, ot)

    # --- timing breakdown: pure C++ stepping vs the Python driver loop -----
    print("\nTIMING (6000 steps @ dt=0.1):")
    om = OmniTaylor(list(INERTIA.ravel()), MASS, [list(a) for a in aligned],
                    list(JS), MU, False, 1e-13, False, True,
                    RPM0, [1e7, 2e6, -1.5e6], [-700., 5500., 3200.],
                    [0.1, 0.2, -0.3], [0.001, -0.01, 0.03])
    t0 = time.perf_counter()
    for _ in range(6000):
        om.step(0.1)
    t_step = time.perf_counter() - t0
    print(f"  ctypes step() only            : {t_step*1e3:8.2f} ms "
          f"({t_step/6000*1e6:.2f} us/step)")

    t0 = time.perf_counter()
    for _ in range(6000):
        om.set_pars([0.0, 0.0, 0.0])
    t_pars = time.perf_counter() - t0
    print(f"  set_pars() only               : {t_pars*1e3:8.2f} ms")

    # full python closed loop
    t0 = time.perf_counter()
    _ = omni_run(aligned, duration=600.0)
    t_full = time.perf_counter() - t0
    print(f"  full python closed loop       : {t_full*1e3:8.2f} ms")
    print(f"  -> python overhead is {(t_full - t_step - t_pars)/t_full*100:.0f}% of it")
