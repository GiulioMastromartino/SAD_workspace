"""3-year station-keeping with heyoka (orbit-only 6-state, RTN PID).

Scenario: -0.5% injection-velocity error on the 12000 km-altitude circular
orbit; the PID fires the variable-thrust jets to recover and hold the orbit.
Open-loop drift shown for contrast.

Outputs (results_3yr/): radius plot, thruster-energy plots, results.npz.
Thruster energy bookkeeping: cumulative dv, total impulse (m*dv), propellant
via Tsiolkovsky with config ISP_JET. Allocation assumed ideal in the 6-state
loop; max commanded accel is checked a posteriori against jet capability.
"""
from __future__ import annotations
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from . import config as C
from .controller import SemiMajorPID
from .heyoka_propagate import propagate_closedloop_orbit, propagate_heyoka

YEAR = 365.25 * 86400.0
OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results_3yr")


def main(t_years: float = 3.0, dt_control: float = 600.0, tol: float = 1e-10):
    os.makedirs(OUTDIR, exist_ok=True)
    t_end = t_years * YEAR
    n_expect = int(round(t_end / dt_control))
    print(f"[info] {t_years:.2f} yr = {t_end/86400:.0f} d, dt_ctrl={dt_control:.0f} s "
          f"-> {n_expect} ticks, tol={tol:g}")

    r0 = C.R0_ECI.copy()
    v0 = C.V0_ECI.copy() * 0.995  # -0.5% injection error

    # --- open-loop drift (daily samples, cheap) ---
    y0_full = np.concatenate([r0, v0,
                              np.radians([5.0, 2.0, 0.0]),
                              np.radians([0.05, 0.02, -0.03])])
    open_log = propagate_heyoka(t_end=t_end, dt_log=86400.0, y0=y0_full)
    r_open = np.linalg.norm(open_log["y"][0:3, :], axis=0) - C.R_ORBIT

    # --- closed-loop PID (heyoka, orbit-only): semi-major-axis feedback converges
    # once and then coasts (zero steady burn in pure Keplerian free-fall) ---
    closed = propagate_closedloop_orbit(t_end=t_end, dt_control=dt_control,
                                        r0=r0, v0=v0, pid=SemiMajorPID(),
                                        tol=tol)
    assert np.all(np.isfinite(closed["y"])), "non-finite state in closed loop!"
    t_yr = closed["t"] / YEAR
    r_err = np.linalg.norm(closed["y"][0:3, :], axis=0) - C.R_ORBIT
    a_mag = np.linalg.norm(closed["a_eci"], axis=0)

    # a posteriori jet-capability check (PID caps at PID_A_MAX per axis)
    a_cap = 2.0 * C.TMAX_JET / C.MASS  # ~2 jets per axis available
    print(f"[info] max commanded |a|={a_mag.max():.4f} m/s^2 "
          f"(per-axis cap {C.PID_A_MAX:.3f}, ~capability {a_cap:.3f})")

    dv, imp, prop = closed["dv"], closed["impulse"], closed["propellant"]
    print(f"[open]   max|r-err|={np.abs(r_open).max()/1e3:.1f} km")
    print(f"[closed] max|r-err|={np.abs(r_err).max()/1e3:.3f} km  "
          f"final={r_err[-1]/1e3:.3f} km")
    print(f"[closed] dv_total={dv[-1]:.2f} m/s  impulse={imp[-1]:.1f} N s  "
          f"propellant~{prop[-1]*1e3:.1f} g (Isp={C.ISP_JET:.0f} s)")

    np.savez(os.path.join(OUTDIR, "results.npz"),
             t_s=closed["t"], r_err_m=r_err, a_eci=closed["a_eci"],
             dv=dv, impulse=imp, propellant=prop,
             t_open_s=open_log["t"], r_open_m=r_open)

    # --- Fig 1: radius over time ---
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(open_log["t"] / YEAR, r_open / 1e3, label="free-fall drift", alpha=0.8)
    ax.plot(t_yr, r_err / 1e3, label="PID jets (heyoka)", alpha=0.8)
    ax.set_xlabel("time [yr]")
    ax.set_ylabel("radius error [km]")
    ax.grid(True)
    ax.legend()
    from mpl_toolkits.axes_grid1.inset_locator import inset_axes
    axins = inset_axes(ax, width="38%", height="42%", loc="lower left")
    axins.plot(t_yr, r_err, linewidth=0.5)
    axins.set_xlim(0.05, float(t_years))  # skip day-1 recovery transient
    axins.set_ylim(-150.0, 150.0)
    axins.set_ylabel("PID residual [m]", fontsize=8)
    axins.set_xlabel("time [yr]", fontsize=8)
    axins.tick_params(labelsize=7)
    axins.grid(True)
    fig.suptitle(f"12U @ {C.ALTITUDE/1e3:.0f} km alt — 3-yr radius: drift vs PID station-keeping")
    fig.subplots_adjust(top=0.9, right=0.97)
    fig.savefig(os.path.join(OUTDIR, "radius_3yr.png"), dpi=120)

    # --- Fig 2: thruster energy consumption (top: recovery-burn profile in hours,
    # --- middle/bottom: 3-yr cumulative dv / impulse / propellant) ---
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
    m = closed["t"] <= 5 * 86400.0
    ax[0].plot(closed["t"][m] / 3600.0, a_mag[m], linewidth=0.8)
    ax[0].set_ylabel("|a_thrust| [m/s²]")
    ax[0].set_xlabel("time [h since start]")
    ax[0].set_title("recovery burn (zero thrust afterwards)", fontsize=9)
    ax[0].grid(True)
    ax[1].plot(t_yr, dv, label="cumul. dv")
    ax[1].set_ylabel("cumul. dv [m/s]")
    ax[1].grid(True)
    ax2 = ax[1].twinx()
    ax2.plot(t_yr, imp, color="tab:orange", alpha=0.7, label="impulse")
    ax2.set_ylabel("cumul. impulse [N s]")
    ax[2].plot(t_yr, prop * 1e3)
    ax[2].set_ylabel("propellant [g]")
    ax[2].set_xlabel("time [yr]")
    ax[2].grid(True)
    fig.suptitle(f"Thruster energy use — total {dv[-1]:.1f} m/s, "
                 f"{imp[-1]:.0f} N s, ~{prop[-1]*1e3:.0f} g @ Isp {C.ISP_JET:.0f}s")
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, "energy_3yr.png"), dpi=120)
    print(f"[info] wrote {OUTDIR}/radius_3yr.png, energy_3yr.png, results.npz")


if __name__ == "__main__":
    main()
