"""Demo: PID station-keeping vs free-fall drift, integrated with heyoka.

Scenario: injection error leaves the 12U ~0.5% slow (elliptical, radius sags).
The RTN PID fires the variable-thrust jets to restore the 12000 km-altitude
circular orbit. Attitude starts detumbled (slow drift) to stay clear of the
Euler pitch singularity during the long Heyoka run.
"""
from __future__ import annotations
import numpy as np
from . import config as C
from .dynamics import pack_state
from .controller import OrbitPID
from .heyoka_propagate import propagate_heyoka, propagate_closedloop


def demo_state(dv_frac: float = -0.005):
    r0 = C.R0_ECI.copy()
    v0 = C.V0_ECI.copy() * (1.0 + dv_frac)
    euler0 = np.radians(np.array([5.0, 2.0, 0.0]))
    omega0 = np.radians(np.array([0.05, 0.02, -0.03]))
    return pack_state(r0, v0, euler0, omega0)


def radius_err(y: np.ndarray) -> np.ndarray:
    return np.linalg.norm(y[0:3, :], axis=0) - C.R_ORBIT


def main(t_end: float = 7200.0, plot: bool = True):
    y0 = demo_state()
    print(f"[info] injection error -0.5% dV; R_des={C.R_ORBIT/1e3:.1f} km, "
          f"ctrl@{1.0/C.DT_CONTROL:.2f} Hz, t_end={t_end/3600:.2f} h")

    open_log = propagate_heyoka(t_end=t_end, dt_log=C.DT_CONTROL, y0=y0)
    pid = OrbitPID()
    closed_log = propagate_closedloop(t_end=t_end, dt_control=C.DT_CONTROL, y0=y0, pid=pid)

    for tag, log in (("open-loop ", open_log), ("PID-closed", closed_log)):
        e = radius_err(log["y"])
        print(f"[{tag}] max|r-err|={np.abs(e).max()/1e3:.2f} km  "
              f"rms={np.sqrt((e**2).mean())/1e3:.2f} km  final={e[-1]/1e3:.2f} km")
    print(f"[PID-closed] fuel dv={closed_log['dv']:.2f} m/s, "
          f"mean throttle={closed_log['u'].mean():.4f}")

    if plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax[0].plot(open_log["t"] / 3600, radius_err(open_log["y"]) / 1e3, label="free-fall drift")
        ax[0].plot(closed_log["t"] / 3600, radius_err(closed_log["y"]) / 1e3, label="PID jets")
        ax[0].set_ylabel("radius error [km]")
        ax[0].legend()
        ax[0].grid(True)
        ax[1].plot(closed_log["t"] / 3600, closed_log["u"].T, alpha=0.6, linewidth=0.8)
        ax[1].set_ylabel("throttle u [0..1]")
        ax[1].set_xlabel("time [h]")
        ax[1].grid(True)
        fig.suptitle("12U station-keeping @ 12000 km alt — heyoka + RTN PID")
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
