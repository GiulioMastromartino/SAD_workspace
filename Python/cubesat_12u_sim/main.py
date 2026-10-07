"""Demo: free-fall at 12000 km altitude, 12U CubeSat, Euler-angle attitude + gyro sampling + optional jet burn."""
from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from . import config as C
from .propagate import propagate
from .sensors import RateGyro
from .actuators import allocation_pinv

import sys


def main(with_burn: bool = False):
    # Optional: small station-keeping burn allocated through variable-thrust jets
    u = np.zeros(len(C.THRUSTER_TMAX))
    if with_burn:
        u = allocation_pinv(np.array([0.1, 0.0, 0.0]), np.zeros(3))
        print(f"[info] burn throttles (first 4): {u[:4]}")

    sol = propagate(t_end=min(C.T_END_DEFAULT, 7200.0), u_jet=u)
    r, v, euler, omega = sol.y[0:3], sol.y[3:6], sol.y[6:9], sol.y[9:12]

    # Sample gyros along true omega history
    gyro = RateGyro(seed=1)
    dt = C.DT_LOG
    w_meas = np.array([gyro.measure(w, dt) for w in omega.T]).T

    print(f"[info] R_orbit={C.R_ORBIT/1e3:.1f} km  V_circ={C.V_CIRC:.1f} m/s  T_orbit={C.T_ORBIT/3600:.2f} h")
    print(f"[info] Ix,Iy,Iz = {C.IX:.4f}, {C.IY:.4f}, {C.IZ:.4f} kg m^2")
    print(f"[info] final |r|={np.linalg.norm(r[:,-1])/1e3:.1f} km, euler[deg]={np.degrees(euler[:,-1])}")

    # Plots
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].plot(sol.t, np.degrees(euler.T))
    ax[0].legend(["roll phi", "pitch theta", "yaw psi"])
    ax[0].set_ylabel("Euler angle [deg]")
    ax[0].grid(True)
    ax[1].plot(sol.t, np.degrees(omega.T), alpha=0.6, label=["p true", "q true", "r true"])
    ax[1].plot(sol.t, np.degrees(w_meas.T), alpha=0.35, linestyle="--")
    ax[1].set_ylabel("body rate [deg/s]")
    ax[1].set_xlabel("time [s]")
    ax[1].grid(True)
    fig.suptitle(f"12U CubeSat free-fall @ {C.ALTITUDE/1e3:.0f} km alt — Euler attitude + gyro")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main(with_burn=("--burn" in sys.argv))
