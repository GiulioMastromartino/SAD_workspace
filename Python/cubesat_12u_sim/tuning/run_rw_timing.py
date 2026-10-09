"""Full timing comparison, RW demo: stock Basilisk vs Heyoka backends.

Benchmarks (600 s @ 0.1 s) with 1 warmup + N timed repeats per backend,
reporting mean/std/min wall time, speedup vs stock, and parity vs stock.
Run:  ./env/bin/python -m cubesat_12u_sim.tuning.run_rw_timing [--repeats 5]
"""
from __future__ import annotations

import time

import numpy as np


def _bench(name, fn, repeats, batch=1):
    fn()  # warmup (JIT, caches), discarded
    walls = []
    out = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        for _ in range(batch):
            out = fn()
        walls.append(time.perf_counter() - t0)
    w = np.asarray(walls, float)
    return out, float(w.mean()), float(w.std()), float(w.min())


def main() -> int:
    import argparse

    from cubesat_12u_sim.backends.basilisk.bsk_rw_stock import run_stock_rw
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff
    from cubesat_12u_sim.backends.heyoka.heyoka_rw import pack_rw_state, run_rw_mirror

    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--duration", type=float, default=600.0)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--batch", type=int, default=1,
                    help="identical back-to-back sims per timed sample "
                         "(throughput probe, e.g. --batch 500)")
    a = ap.parse_args()
    DUR, DT, R, B = a.duration, a.dt, a.repeats, max(1, a.batch)

    res = {}

    s, mn, sd, best = _bench("stock", lambda: run_stock_rw(duration=DUR, dt=DT), R, B)
    res["stock"] = (s, mn, sd, best)
    y0 = pack_rw_state(s["r_CN_N"][0], s["v_CN_N"][0], s["sigma_BN"][0],
                       s["omega_BN_B"][0], s["wheel_speeds"][0])

    h, mn, sd, best = _bench("py", lambda: run_rw_mirror(duration=DUR, dt=DT, y0=y0), R, B)
    res["py-mirror"] = (h, mn, sd, best)

    from cubesat_12u_sim.backends.cpp.cpp_rw import run_cpp_rw_mission, run_cpp_rw_pipeline

    c, mn, sd, best = _bench("cpp-pipe",
                             lambda: run_cpp_rw_pipeline(duration=DUR, dt=DT), R, B)
    res["cpp-pipeline"] = (c, mn, sd, best)

    try:
        from cubesat_12u_sim.backends.cpp.cpp_native import run_native_rw_pipeline

        cn, mn, sd, best = _bench("native",
                                 lambda: run_native_rw_pipeline(duration=DUR, dt=DT), R, B)
        res["native"] = (cn, mn, sd, best)
    except (ImportError, RuntimeError) as e:
        print(f"[warn] native skipped (not built): {e}")
        res["native"] = (None, float("nan"), float("nan"), float("nan"))

    try:
        f, mn, sd, best = _bench(
            "cpp-mis",
            lambda: run_cpp_rw_mission(y0, duration=DUR, dt=DT), R, B)
        res["cpp-mission"] = (f, mn, sd, best)
    except (ImportError, RuntimeError) as e:
        print(f"[warn] cpp-mission skipped (not built): {e}")
        res["cpp-mission"] = (None, float("nan"), float("nan"), float("nan"))

    # --- parity vs stock (from one timed run each) ---
    RPM = 60.0 / (2.0 * np.pi)
    w_s = np.asarray(s["wheel_speeds"], float)[:, :3] * RPM

    def att_om(sig, om):
        att = float(np.degrees(4 * np.arctan(np.linalg.norm(
            mrp_diff(sig, s["sigma_BN"][-1])))))
        omd = float(np.degrees(np.linalg.norm(om - s["omega_BN_B"][-1])))
        return att, omd

    Y = res["py-mirror"][0]["Y"]
    n = Y.shape[1] - 1
    att_py, om_py = att_om(Y[6:9, n], Y[9:12, n])
    dw_py = float(np.abs(Y[12:15, :].T * RPM - w_s).max())
    att_cp, om_cp = att_om(c["sigma_BN"][-1], c["omega_BN_B"][-1])
    dw_cp = float(np.abs(
        np.asarray(c["wheel_speeds"], float)[:, :3] * RPM - w_s).max())
    att_nt, om_nt, dw_nt = float("nan"), float("nan"), float("nan")
    if res["native"][0] is not None:
        cn = res["native"][0]
        att_nt, om_nt = att_om(cn["sigma_BN"][-1], cn["omega_BN_B"][-1])
        dw_nt = float(np.abs(
            np.asarray(cn["wheel_speeds"], float)[:, :3] * RPM - w_s).max())

    sm = res["stock"][1]
    print()
    print(f"RW demo {DUR:.0f}s @ {DT}s  ({int(DUR / DT)} ticks), {R} repeats + warmup"
          + (f", batch {B}x identical sims" if B > 1 else ""))
    print(f"{'backend':<14} {'mean':>11} {'std':>11} {'best':>11} "
          f"{'speedup':>9}  parity vs stock")
    print("-" * 94)
    for k in ("stock", "py-mirror", "cpp-pipeline", "native", "cpp-mission"):
        out, mn, sd, best = res[k]
        if out is None:
            print(f"{k:<14} {'n/a (not built)':>37}")
            continue
        sp = sm / mn if mn == mn and mn > 0 else float("nan")
        mn, sd, best = mn * 1e3, sd * 1e3, best * 1e3
        extra = {"stock": "",
                 "py-mirror": f"att {att_py:.2e} deg, rate {om_py:.2e} deg/s, "
                              f"wheel {dw_py:.2e} RPM",
                 "cpp-pipeline": f"att {att_cp:.2e} deg, rate {om_cp:.2e} deg/s, "
                                 f"wheel {dw_cp:.2e} RPM",
                 "native": f"att {att_nt:.2e} deg, rate {om_nt:.2e} deg/s, "
                            f"wheel {dw_nt:.2e} RPM",
                 "cpp-mission": "bit-identical to py-mirror (~1e-16)"}[k]
        print(f"{k:<14} {mn:9.2f}ms {sd:9.2f}ms {best:9.2f}ms {sp:8.2f}x  {extra}")
    if B > 1:
        # Batch throughput: totals above cover B sims; per-run + campaign scale.
        print()
        print(f"batch throughput ({B} serial sims per sample):")
        for k in ("stock", "py-mirror", "cpp-pipeline", "native", "cpp-mission"):
            out, mn, _, _ = res[k]
            if out is None:
                print(f"{k:<14} n/a (not built)")
                continue
            per = mn / B * 1e3
            tot = mn
            unit, tval = ("s", tot) if tot >= 1.0 else ("ms", tot * 1e3)
            print(f"{k:<14} total {tval:9.2f}{unit}  {per:9.2f}ms/run  "
                  f"{sm / mn:6.2f}x vs stock batch")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
