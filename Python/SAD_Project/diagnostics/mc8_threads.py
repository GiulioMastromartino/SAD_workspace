import sys, time
from pathlib import Path
sys.path.insert(0, str(Path("/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/SAD_Project/python").resolve()))
sys.path.insert(0, str(Path("/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/SAD_Project").resolve().parent))
import numpy as np
from concurrent.futures import ThreadPoolExecutor
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
    N=1000; WORKERS=8; CHUNK=N//WORKERS
    for emb in (False, True):
        # Warm cache in main thread.
        bp,_ = build_batch_params(DEFAULTS, disperse=True, timing=False)
        bp.track_embedded = emb
        bp.track_sample_dt = 10.0
        bp.use_heyoka = False
        _ = sad_py.run_mission_batch(1, 42, bp)
        t0=time.perf_counter()
        args=[(emb, 42+i, CHUNK) for i in range(WORKERS)]
        with ThreadPoolExecutor(max_workers=WORKERS) as pool:
            res=list(pool.map(run_chunk, args))
        wall=time.perf_counter()-t0
        trackmax=np.concatenate([x[1] for x in res])
        print(f"embedded={emb} N={N} threads={WORKERS} wall={wall:.2f}s track_max_mean={trackmax.mean():.4f} (std {trackmax.std():.4f}) failed={int(np.concatenate([x[7] for x in res]).sum())} reached={int(np.concatenate([x[8] for x in res]).sum())}")

if __name__ == "__main__":
    main()
