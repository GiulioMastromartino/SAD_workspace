import sys, time
from pathlib import Path
sys.path.insert(0, str(Path("/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/SAD_Project/python").resolve()))
sys.path.insert(0, str(Path("/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/SAD_Project").resolve().parent))
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import sad_py
from SAD_Project.optimization.objective import build_batch_params
from SAD_Project.optimization.params import DEFAULTS

def run_chunk(args):
    emb, seed, n = args
    bp,_ = build_batch_params(DEFAULTS, disperse=True, timing=False)
    bp.track_embedded = emb
    bp.track_sample_dt = 10.0
    bp.use_heyoka = False
    r = sad_py.run_mission_batch(n, seed, bp)
    return emb, np.array(r["track_ang_max"]), np.array(r["track_ang_mean"]), np.array(r["final_rate"]), np.array(r["fuel_used"]), np.array(r["box_max"]), np.array(r["orbit_err"]), np.array(r["failed"]), np.array(r["reached_track"])

def main():
    N=1000
    WORKERS=8
    CHUNK=N//WORKERS
    for emb in (False, True):
        t0=time.perf_counter()
        args=[(emb, 42+i, CHUNK) for i in range(WORKERS)]
        with ProcessPoolExecutor(max_workers=WORKERS) as pool:
            res=list(pool.map(run_chunk, args))
        wall=time.perf_counter()-t0
        trackmax=np.concatenate([x[1] for x in res])
        trackmean=np.concatenate([x[2] for x in res])
        finalrate=np.concatenate([x[3] for x in res])
        fuel=np.concatenate([x[4] for x in res])
        box=np.concatenate([x[5] for x in res])
        orb=np.concatenate([x[6] for x in res])
        failed=np.concatenate([x[7] for x in res])
        reached=np.concatenate([x[8] for x in res])
        print(f"embedded={emb} N={N} workers={WORKERS} wall={wall:.2f}s")
        print(f"  failed={int(failed.sum())} reached={int(reached.sum())}")
        print(f"  track_max_mean={trackmax.mean():.4f} (std {trackmax.std():.4f})")
        print(f"  track_mean_mean={trackmean.mean():.4f}")
        print(f"  final_rate={finalrate.mean():.3e} (std {finalrate.std():.3e})")
        print(f"  fuel={fuel.mean():.6f} (std {fuel.std():.6f})")
        print(f"  box_max={box.mean():.3f} (std {box.std():.3f}) km")
        print(f"  orbit_err={orb.mean():.4f} km")
        if emb:
            # not available from chunks aggregated; run one timing probe
            pass

if __name__ == "__main__":
    main()
