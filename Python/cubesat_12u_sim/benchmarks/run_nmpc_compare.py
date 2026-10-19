"""Fuel comparison: legacy PID vs hierarchical NMPC + fast-PID trim.

Short (2 h, 12-state Euler/Heyoka, -0.5% injection):
  PID @10 s  vs  NMPC bulk @10 s + trim @1 s (TRIM_A_MAX + TRIM_DV_BUDGET).

Long (orbit-only 6-state, 30-day recovery + coast extrapolation to 3 yr):
  SemiMajorPID @600 s vs NMPC @600 s + trim @60 s. After recovery (K
  consecutive zero windows) the loop coasts ballistically — same physics as
  run_3year (zero steady burn in pure Kepler), so dv is recovery-only.

Usage (from Python/):
  ./env/bin/python -m cubesat_12u_sim.benchmarks.run_nmpc_compare --mode both --no-plot
  ./env/bin/python -m cubesat_12u_sim.benchmarks.run_nmpc_compare --mode short
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.core.dynamics import pack_state
from cubesat_12u_sim.control.controller import OrbitPID, SemiMajorPID
from cubesat_12u_sim.control.hierarchical import HierarchicalController, FastTrimPID
from cubesat_12u_sim.control.nmpc import NMPCConfig

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results_nmpc")


def _run_short(t_end: float = 7200.0, dt_outer: float = 10.0, dt_inner: float = 1.0) -> dict:
    from cubesat_12u_sim.backends.heyoka.heyoka_propagate import make_integrator

    r0 = C.R0_ECI.copy()
    v0 = C.V0_ECI.copy() * 0.995
    euler0 = np.radians(np.array([5.0, 2.0, 0.0]))
    omega0 = np.radians(np.array([0.05, 0.02, -0.03]))
    y0 = pack_state(r0, v0, euler0, omega0)

    out = {}
    for tag, make_ctrl in (
        ("pid", lambda: OrbitPID()),
        ("nmpc", lambda: HierarchicalController(
            NMPCConfig(dt_outer=dt_outer, horizon=8), FastTrimPID())),
    ):
        ctrl = make_ctrl()
        ta = make_integrator(y0, np.zeros(len(C.THRUSTER_TMAX)))
        n_outer = int(round(t_end / dt_outer))
        sub = int(round(dt_outer / dt_inner))
        n_thr = len(C.THRUSTER_TMAX)
        t = np.zeros(n_outer * sub + 1)
        y = np.zeros((12, n_outer * sub + 1))
        u_hist = np.zeros((n_thr, n_outer * sub + 1))
        y[:, 0] = y0
        dv = 0.0
        k = 0
        for _ in range(n_outer):
            for _ in range(sub):
                r, v, euler = y[0:3, k], y[3:6, k], y[6:9, k]
                om = y[9:12, k]
                # dt must match the step the dynamics actually take (dt_inner),
                # not the outer replan period: OrbitPID's integrator does
                # int += e*dt, so passing dt_outer here over-weights the I term
                # by dt_outer/dt_inner and hands the PID baseline a worse
                # controller than it would be deployed with.
                if tag == "pid":
                    u, F_b, tau_b = ctrl.throttle_command(r, v, euler, dt_inner)
                else:
                    u, F_b, tau_b = ctrl.throttle_command(r, v, euler, dt_inner, om)
                for i, val in enumerate(np.concatenate([F_b, tau_b])):
                    ta.pars[i] = float(val)
                ta.propagate_for(dt_inner)
                k += 1
                t[k] = ta.time
                y[:, k] = ta.state
                u_hist[:, k] = u
                dv += float(np.linalg.norm(F_b) / C.MASS * dt_inner)
        r_err = np.linalg.norm(y[0:3, :], axis=0) - C.R_ORBIT
        e_a = np.array([_energy_err(y[0:3, i], y[3:6, i]) for i in range(y.shape[1])])
        info: dict = {"t": t, "y": y, "u": u_hist, "dv": dv, "r_err": r_err, "e_a": e_a}
        if tag == "nmpc":
            info["fuel_split"] = ctrl.fuel_split
        out[tag] = info
        print(f"[short/{tag}] max|r-err|={np.abs(r_err).max()/1e3:.2f} km "
              f"final_r={r_err[-1]/1e3:.2f} km final_e_a={e_a[-1]/1e3:.2f} km "
              f"dv={dv:.2f} m/s mean_u={u_hist.mean():.4f}"
              + (f" trim_frac={info['fuel_split']['trim_frac']:.2%}"
                 f" plans={info['fuel_split']['plans']}"
                 f" fallbacks={info['fuel_split']['fallbacks']}" if tag == "nmpc" else ""))
    return out


def _energy_err(r: np.ndarray, v: np.ndarray) -> float:
    from cubesat_12u_sim.control.nmpc import semi_major_axis
    return semi_major_axis(r, v) - C.R_ORBIT


def _run_long(t_days: float = 30.0, dt_outer: float = 600.0, dt_inner: float = 60.0) -> dict:
    """30-day recovery with early-coast exit; dv extrapolated flat to 3 yr.

    PID path steps at dt_outer (600 s, like run_3year). NMPC path steps the
    dynamics at dt_inner (60 s) with bulk replanned at dt_outer and trim
    DISABLED (authority 0: long-horizon trim on radius would reintroduce the
    epicycle fight; bulk energy NMPC does all the work, like SemiMajorPID).
    Coast detection uses bulk accel only. Energy error e_a is the primary KPI
    (radius oscillates naturally; e_a does not).
    """
    import heyoka as hy
    from cubesat_12u_sim.backends.heyoka.heyoka_propagate import build_orbit_system

    t_end = t_days * 86400.0
    r0 = C.R0_ECI.copy()
    v0 = C.V0_ECI.copy() * 0.995
    out = {}

    # ---- PID path (own clean loop at dt_outer) ----
    pid = SemiMajorPID()
    system = build_orbit_system()
    ta = hy.taylor_adaptive(system, list(map(float, np.concatenate([r0, v0]))),
                            pars=[0.0, 0.0, 0.0], tol=1e-10,
                            high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY))
    n_pid = int(round(t_end / dt_outer))
    tp = np.zeros(n_pid + 64)  # +margin for sampled ballistic coast tail
    yp = np.zeros((6, n_pid + 64))
    dvp_hist = np.zeros(n_pid + 64)
    yp[:, 0] = np.concatenate([r0, v0])
    dv = 0.0
    kp = 0
    zero_w = 0
    coast_at = None
    for _ in range(n_pid):
        r, v = yp[0:3, kp], yp[3:6, kp]
        a = pid.accel_command(r, v, dt_outer)
        ta.pars[0], ta.pars[1], ta.pars[2] = map(float, a)
        ta.propagate_for(dt_outer)
        kp += 1
        tp[kp] = ta.time
        yp[:, kp] = ta.state
        dv += float(np.linalg.norm(a) * dt_outer)
        dvp_hist[kp] = dv
        zero_w = zero_w + 1 if float(np.linalg.norm(a)) < C.PID_DEADBAND else 0
        if zero_w >= 20:
            coast_at = tp[kp]
            # sampled ballistic coast (daily): shows the true epicycle
            # oscillation instead of a fake straight line to day 30, and
            # keeps dv_hist flat (dv bug: the old single-jump point left
            # dv_hist at its initialized 0, drawing dv decaying to zero).
            ta.pars[0] = ta.pars[1] = ta.pars[2] = 0.0
            t_rem = t_end - tp[kp]
            ns = max(2, int(round(t_rem / 86400.0)) + 1)
            for _ in range(ns):
                ta.propagate_for(t_rem / ns)
                kp += 1
                tp[kp] = ta.time
                yp[:, kp] = ta.state
                dvp_hist[kp] = dv
            tp, yp, dvp_hist = tp[:kp + 1], yp[:, :kp + 1], dvp_hist[:kp + 1]
            break
    else:
        tp, yp, dvp_hist = tp[:kp + 1], yp[:, :kp + 1], dvp_hist[:kp + 1]
    # ballistic coast extension to a full 3 yr (one heyoka jump, zero fuel:
    # pure Kepler needs no station-keeping once energy is restored)
    T_3YR = 3.0 * 365.25 * 86400.0
    ta.pars[0] = ta.pars[1] = ta.pars[2] = 0.0
    ta.propagate_for(T_3YR - tp[kp])
    r3, v3 = np.array(ta.state[0:3]), np.array(ta.state[3:6])
    r_err = np.linalg.norm(yp[0:3, :], axis=0) - C.R_ORBIT
    e_a = np.array([_energy_err(yp[0:3, i], yp[3:6, i]) for i in range(yp.shape[1])])
    prop = float(C.MASS * (1.0 - np.exp(-dv / (C.ISP_JET * C.G0))))
    out["pid"] = {"t": tp, "y": yp, "dv": dv, "dv_3yr": dv, "dv_hist": dvp_hist,
                  "prop_3yr_g": prop * 1e3, "r_err": r_err, "e_a": e_a,
                  "r_3yr": r3, "v_3yr": v3,
                  "e_a_3yr": _energy_err(r3, v3),
                  "rerr_3yr": float(np.linalg.norm(r3)) - C.R_ORBIT,
                  "coast_at_d": (coast_at / 86400.0) if coast_at else None}
    extra = f" coast@{coast_at/86400:.1f}d" if coast_at else " no-coast"
    print(f"[long/pid] dv_rec={dv:.2f} m/s dv_3yr={dv:.2f} m/s prop~{prop*1e3:.0f} g "
          f"e_a30d={e_a[-1]/1e3:.2f} km e_a3yr={out['pid']['e_a_3yr']/1e3:.2f} km "
          f"rerr3yr={out['pid']['rerr_3yr']/1e3:.2f} km{extra}")

    # ---- NMPC path (dynamics at dt_inner, bulk replan at dt_outer, trim off) ----
    ctrl = HierarchicalController(
        NMPCConfig(dt_outer=dt_outer, horizon=6),
        FastTrimPID(authority=0.0, dv_budget=0.0))  # trim disabled for long horizon
    ta2 = hy.taylor_adaptive(system, list(map(float, np.concatenate([r0, v0]))),
                             pars=[0.0, 0.0, 0.0], tol=1e-10,
                             high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY))
    n_in = int(round(t_end / dt_inner))
    tn = np.zeros(n_in + 64)  # +margin for sampled ballistic coast tail
    yn = np.zeros((6, n_in + 64))
    dvn_hist = np.zeros(n_in + 64)
    yn[:, 0] = np.concatenate([r0, v0])
    dvn = 0.0
    kn = 0
    zero_w = 0
    coast_at = None
    for _ in range(n_in):
        r, v = yn[0:3, kn], yn[3:6, kn]
        a = ctrl.accel_command(r, v, dt_inner)
        bulk = float(np.linalg.norm(ctrl._bulk_a))
        ta2.pars[0], ta2.pars[1], ta2.pars[2] = map(float, a)
        ta2.propagate_for(dt_inner)
        kn += 1
        tn[kn] = ta2.time
        yn[:, kn] = ta2.state
        dvn += float(np.linalg.norm(a) * dt_inner)
        dvn_hist[kn] = dvn
        zero_w = zero_w + 1 if bulk < C.PID_DEADBAND else 0
        if zero_w >= 200:  # 200 x 60 s = 3.3 h of zero bulk
            coast_at = tn[kn]
            ta2.pars[0] = ta2.pars[1] = ta2.pars[2] = 0.0
            t_rem = t_end - tn[kn]
            ns = max(2, int(round(t_rem / 86400.0)) + 1)
            for _ in range(ns):
                ta2.propagate_for(t_rem / ns)
                kn += 1
                tn[kn] = ta2.time
                yn[:, kn] = ta2.state
                dvn_hist[kn] = dvn
            tn, yn, dvn_hist = tn[:kn + 1], yn[:, :kn + 1], dvn_hist[:kn + 1]
            break
    else:
        tn, yn, dvn_hist = tn[:kn + 1], yn[:, :kn + 1], dvn_hist[:kn + 1]
    ta2.pars[0] = ta2.pars[1] = ta2.pars[2] = 0.0
    ta2.propagate_for(T_3YR - tn[kn])
    r3n, v3n = np.array(ta2.state[0:3]), np.array(ta2.state[3:6])
    r_errn = np.linalg.norm(yn[0:3, :], axis=0) - C.R_ORBIT
    e_an = np.array([_energy_err(yn[0:3, i], yn[3:6, i]) for i in range(yn.shape[1])])
    propn = float(C.MASS * (1.0 - np.exp(-dvn / (C.ISP_JET * C.G0))))
    out["nmpc"] = {"t": tn, "y": yn, "dv": dvn, "dv_3yr": dvn, "dv_hist": dvn_hist,
                   "prop_3yr_g": propn * 1e3, "r_err": r_errn, "e_a": e_an,
                   "r_3yr": r3n, "v_3yr": v3n,
                   "e_a_3yr": _energy_err(r3n, v3n),
                   "rerr_3yr": float(np.linalg.norm(r3n)) - C.R_ORBIT,
                   "coast_at_d": (coast_at / 86400.0) if coast_at else None}
    extra = f" coast@{coast_at/86400:.1f}d" if coast_at else " no-coast"
    print(f"[long/nmpc] dv_rec={dvn:.2f} m/s dv_3yr={dvn:.2f} m/s prop~{propn*1e3:.0f} g "
          f"e_a30d={e_an[-1]/1e3:.2f} km e_a3yr={out['nmpc']['e_a_3yr']/1e3:.2f} km "
          f"rerr3yr={out['nmpc']['rerr_3yr']/1e3:.2f} km{extra}")
    print(f"           trim_frac={ctrl.fuel_split['trim_frac']:.2%} "
          f"plans={ctrl.fuel_split['plans']} fallbacks={ctrl.fuel_split['fallbacks']}")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="both", choices=["short", "long", "both"])
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    os.makedirs(OUTDIR, exist_ok=True)
    res: dict = {}
    if args.mode in ("short", "both"):
        res["short"] = _run_short()
    if args.mode in ("long", "both"):
        res["long"] = _run_long()
    if not args.no_plot and "short" in res:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        s = res["short"]
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax[0].plot(s["pid"]["t"] / 3600, s["pid"]["r_err"] / 1e3, label="PID")
        ax[0].plot(s["nmpc"]["t"] / 3600, s["nmpc"]["r_err"] / 1e3, label="NMPC+trim")
        ax[0].set_ylabel("radius err [km, symlog]")
        ax[0].set_yscale("symlog", linthresh=1e-2)
        ax[0].legend()
        ax[0].grid(True, which="both")
        ax[1].plot(s["nmpc"]["t"] / 3600, s["nmpc"]["u"].T, alpha=0.5, linewidth=0.7)
        ax[1].set_ylabel("NMPC throttle")
        ax[1].set_xlabel("time [h]")
        ax[1].grid(True)
        fig.suptitle("2h station-keeping: PID vs NMPC+trim")
        fig.tight_layout()
        fig.savefig(os.path.join(OUTDIR, "nmpc_2h.png"), dpi=120)
        print(f"[info] wrote {OUTDIR}/nmpc_2h.png")
    if not args.no_plot and "long" in res:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        lg = res["long"]
        from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
        fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=False)
        for tag, ls in (("pid", "-"), ("nmpc", "--")):
            d = lg[tag]
            t_d = np.asarray(d["t"]) / 86400.0
            ax[0].plot(t_d, np.asarray(d["r_err"]) / 1e3, ls, label=tag, linewidth=0.8)
            ax[1].plot(t_d, np.asarray(d["e_a"]) / 1e3, ls, label=tag, linewidth=0.8)
            ax[2].plot(t_d, np.asarray(d["dv_hist"]), ls, label=tag, linewidth=1.0)
            if d["coast_at_d"] is not None:  # 'o' = control off, rest is ballistic
                ci = int(np.searchsorted(t_d, d["coast_at_d"]))
                ax[0].plot(t_d[ci], np.asarray(d["r_err"])[ci] / 1e3, "o", ms=4)
                ax[1].plot(t_d[ci], np.asarray(d["e_a"])[ci] / 1e3, "o", ms=4)
                ax[2].plot(t_d[ci], np.asarray(d["dv_hist"])[ci], "o", ms=4)
        ax[0].set_ylabel("radius err [km, symlog]")
        ax[0].set_yscale("symlog", linthresh=1e-2)
        ax[0].legend()
        ax[0].grid(True, which="both")
        ax[1].set_ylabel("energy e_a [km, symlog]")
        ax[1].set_yscale("symlog", linthresh=1e-2)
        ax[1].legend()
        ax[1].grid(True, which="both")
        # insets: the recovery transient (days 0-2) is the whole story;
        # the 28 d tail is flat coast. Same curves, linear scale, xlim 2 d.
        for i, key in ((0, "r_err"), (1, "e_a")):
            ins = inset_axes(ax[i], width="42%", height="48%", loc="upper right")
            for tag, ls in (("pid", "-"), ("nmpc", "--")):
                d = lg[tag]
                t_d = np.asarray(d["t"]) / 86400.0
                m = t_d <= 2.0
                ins.plot(t_d[m], np.asarray(d[key])[m] / 1e3, ls, linewidth=0.9)
            ins.set_xlim(0, 2.0)
            ins.set_title("first 2 d", fontsize=8)
            ins.tick_params(labelsize=7)
            ins.grid(True)
            mark_inset(ax[i], ins, loc1=1, loc2=2, fc="none", ec="0.5")
        ax[2].set_ylabel("cumul. dv [m/s] (flat = coasting)")
        ax[2].set_xlabel("time [d since start (30d recovery)]")
        ax[2].legend()
        ax[2].grid(True)
        p, n = lg["pid"], lg["nmpc"]
        fig.suptitle(f"30d recovery + ballistic coast to 3yr: PID {p['dv_3yr']:.1f} m/s "
                     f"(e_a3yr {p['e_a_3yr']/1e3:.2f} km) vs NMPC {n['dv_3yr']:.1f} m/s "
                     f"(e_a3yr {n['e_a_3yr']/1e3:.2f} km)")
        fig.tight_layout()
        fig.savefig(os.path.join(OUTDIR, "nmpc_3yr.png"), dpi=120)
        print(f"[info] wrote {OUTDIR}/nmpc_3yr.png")


if __name__ == "__main__":
    main()
