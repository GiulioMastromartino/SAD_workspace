import sys
from pathlib import Path
import time
import numpy as np

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / "python"))
sys.path.insert(0, str(HERE.parents[2]))
import sad_py
from SAD_Project.optimization.objective import build_batch_params
from SAD_Project.optimization.params import DEFAULTS

N = 100
SEED = 42
MODES = {0: "current", 1: "direct", 2: "smooth-pair"}
INTS = {False: "RK4", True: "Heyoka"}

rows = []
for use_heyoka in (False, True):
    for mode in (0, 1, 2):
        bp, _ = build_batch_params(DEFAULTS, disperse=True, timing=True)
        bp.use_heyoka = use_heyoka
        bp.thruster_mode = mode
        bp.track_dt = 1.0
        if mode in (1, 2):
            for ax in ("detumble", "slew", "track"):
                setattr(bp, ax + "_K", getattr(bp, ax + "_K") * 0.5)
                setattr(bp, ax + "_P", getattr(bp, ax + "_P") * 0.5)
                setattr(bp, ax + "_Ki", getattr(bp, ax + "_Ki") * 0.5)
        if use_heyoka:
            bp.timing = False
            _ = sad_py.run_mission_batch(1, SEED, bp)
        bp.timing = True
        t0 = time.perf_counter()
        res = sad_py.run_mission_batch(N, SEED, bp)
        wall = time.perf_counter() - t0
        failed = int(np.sum(res["failed"]))
        reached = int(np.sum(res["reached_track"]))
        trackmax = float(np.nanmean(res["track_ang_max"]))
        trackmean = float(np.nanmean(res["track_ang_mean"]))
        finalrate = float(np.nanmean(res["final_rate"]))
        fuel = float(np.nanmean(res["fuel_used"]))
        boxmax = float(np.nanmean(res["box_max"]))
        orbiterr = float(np.nanmean(res["orbit_err"]))
        trackmax_std = float(np.nanstd(res["track_ang_max"]))
        fuel_std = float(np.nanstd(res["fuel_used"]))
        boxmax_std = float(np.nanstd(res["box_max"]))
        batch_ms = float(res["timing"]["batch_ms"])
        steps = int(res["timing"]["steps"])
        rows.append((INTS[use_heyoka], MODES[mode], wall, batch_ms, steps,
                     failed, reached, trackmean, trackmax, trackmax_std, finalrate,
                     fuel, fuel_std, boxmax, boxmax_std, orbiterr))
        print(f"{INTS[use_heyoka]:6} mode {mode} ({MODES[mode]:11}) wall={wall:.3f}s batch_ms={batch_ms:.1f} steps={steps} failed={failed} reached={reached}")

print("\nSummary (N=%d, seed=%d):" % (N, SEED))
hdr = ["integrator","mode","wall_s","batch_ms","steps","failed","reached","track_mean_deg","track_max_deg","track_max_std","final_rate","fuel_kg","fuel_std","box_max_km","box_max_std","orbit_err_km"]
print(" | ".join(hdr))
for r in rows:
    print(" | ".join(str(x) if not isinstance(x,float) else f"{x:.6g}" for x in r))
