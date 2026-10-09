"""One global CMA-ES over the whole hypercube, Monte Carlo in the loop.

The pipeline this replaces was three staged searches (PID, then NMPC swept over
horizons, then a joint refinement), each with its own restarts, each scoring a
*single* deterministic closed-loop run. This module collapses that into one
optimiser, one population, one cost function:

* **one hypercube** — :data:`~cubesat_12u_sim.optimization.params.HYPER_SPACE`,
  all 6 NMPC + 9 PID coordinates and the horizon (discrete) in a single 16-D
  box, so no coordinate is frozen at a value another stage happened to pick.
* **no restarts** — one CMA-ES run. The staged version's restarts existed to
  escape bad basins in a cheap, noisy landscape; with 150 candidates per
  generation and a 250-run common-random-number ensemble behind every one of
  them, the population does that exploration instead.
* **Monte Carlo inside the cost** — every candidate is scored on the same
  frozen 250-run design (:mod:`.mcdesign`), reduced by mean-plus-weighted-tails
  (:mod:`.mcobjective`).

Parallelism is across candidates, not across restarts: CMA-ES generations are
sequential, so the only real unit of work is one candidate's 250-run ensemble.
Each worker primes its own C++ template cache once and stays warm for the whole
sweep.

**Why the whole design is shipped to every worker.** ``MCDesign`` is built once
in the parent and pickled into the pool. Rebuilding it per worker would give
each worker its own injection errors, which would make candidate costs depend
on which worker drew them — noise the optimiser would chase. The design is
therefore part of the worker's identity, not something it derives.
"""
from __future__ import annotations

import os
import pickle
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field

import numpy as np

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.optimization.mcdesign import MCConfig, MCDesign, build_design
from cubesat_12u_sim.optimization.mcobjective import (CandidateResult,
                                         TailWeights, evaluate_candidate)
from cubesat_12u_sim.optimization.params import HYPER_SPACE

# Populated by _init_worker; module-level because a pool worker is a fresh
# interpreter and per-call payloads would be re-unpickled on every submission.
_W: dict = {}


def _bounds():
    lb, ub = HYPER_SPACE.bounds()
    # cma wants its bounds strictly outside the domain so it can still propose
    # an out-of-range point and have its own resampling handle it; the evaluator
    # clips before decoding.
    return lb + 1e-12, ub - 1e-12


def _init_worker(design: MCDesign, cfg: dict, weights: TailWeights) -> None:
    """Prime a pool worker as an evaluator.

    ``import heyoka`` happens here rather than per candidate: the extensions
    need libheyoka's symbols in the process before they can be dlopened, and
    the import is the expensive part of the first call.
    """
    import heyoka  # noqa: F401
    import cma  # noqa: F401

    _W.clear()
    _W.update(design=design, cfg=cfg, weights=weights)
    _warm(design, cfg)


def _warm(design: MCDesign, cfg: dict) -> None:
    """Build the C++ template caches once, on a single lane.

    JIT dominates the first closed-loop run by an order of magnitude, and the
    Taylor templates are keyed by (lanes, tol, mu) — so warming every lane
    count the sweep might use would cost more than it saves. One lane at the
    default horizon is enough to pay the import and the compile; the per-horizon
    templates still get built lazily by whichever candidate needs them, and the
    cache is per-process so it is paid at most once per worker either way.
    """
    from cubesat_12u_sim.optimization.mcobjective import _nmpc_group, _pid_group
    from cubesat_12u_sim.optimization.params import (NMPC_SPACE, PID_SPACE)

    vg = design.vehicles[0]
    x0 = design.x0[:1]
    p = HYPER_SPACE.from_theta(HYPER_SPACE.seed_theta())
    n_outer = int(round(cfg.get("t_end", 43200.0) / cfg.get("dt_outer", 600.0)))
    n_steps = int(round(cfg.get("t_end", 43200.0) / cfg.get("pid_dt", 10.0)))
    try:
        _nmpc_group(x0, {k: p[k] for k in NMPC_SPACE.names},
                    int(round(p["horizon"])), vg,
                    cfg.get("dt_outer", 600.0), cfg.get("dt_inner", 60.0),
                    n_outer, cfg.get("maxiter", 50),
                    cfg.get("spec_box", 25e3), cfg.get("tol", C.HEYOKA_TOL))
        _pid_group(x0, {k: p[k] for k in PID_SPACE.names}, vg,
                   cfg.get("pid_dt", 10.0), n_steps,
                   cfg.get("spec_box", 25e3), cfg.get("w_viol", 1e3),
                   cfg.get("w_final", 0.05))
    except Exception:
        # A warm-up failure is not fatal: the real evaluation has its own
        # try/except and will mark the candidate infeasible. Swallowing it here
        # keeps pool construction from failing outright.
        pass


def _eval_one(theta) -> dict:
    """Evaluate one candidate and return a picklable payload."""
    import time as _time

    t0 = _time.time()
    res = evaluate_candidate(theta, _W["design"], weights=_W["weights"],
                             **_W["cfg"])
    payload = res.as_dict()
    payload["wall_s"] = _time.time() - t0
    return payload


def _eval_many(thetas: list) -> list:
    return [_eval_one(t) for t in thetas]


@dataclass
class GenerationRecord:
    """What one CMA-ES generation produced."""

    gen: int
    evals: int
    best_cost: float
    mean_cost: float
    median_cost: float
    worst_cost: float
    best_theta: list
    best_horizon: int
    best: dict
    wall_s: float
    n_infeasible_pop: int = 0
    # {horizon: count} across the whole population. Recorded because the
    # horizon is a free coordinate whose cost varies ~5x across its range, so
    # it -- not the population mean -- decides a generation's wall clock, and
    # an ETA computed without it is a guess.
    horizon_counts: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"gen": self.gen, "evals": self.evals,
                "best_cost": self.best_cost, "mean_cost": self.mean_cost,
                "median_cost": self.median_cost, "worst_cost": self.worst_cost,
                "best_theta": self.best_theta,
                "best_horizon": self.best_horizon, "best": self.best,
                "wall_s": self.wall_s,
                "n_infeasible_pop": self.n_infeasible_pop,
                "horizon_counts": self.horizon_counts}


@dataclass
class SweepReport:
    """Everything the sweep produced, ready to serialise."""

    best: dict
    best_theta: list
    best_cost: float
    history: list = field(default_factory=list)
    design: dict = field(default_factory=dict)
    config: dict = field(default_factory=dict)
    wall_s: float = 0.0

    def as_dict(self) -> dict:
        return {"best": self.best, "best_theta": self.best_theta,
                "best_cost": self.best_cost, "history": self.history,
                "design": self.design, "config": self.config,
                "wall_s": self.wall_s}


def _rank_key(payload: dict) -> tuple:
    """Feasible candidates outrank infeasible ones, then lower cost wins."""
    n = payload["nmpc"]
    p = payload["pid"]
    feas = (n["feasible_frac"] > 0.0) and (p["feasible_frac"] > 0.0)
    return (0 if feas else 1, payload["cost"])


def run_sweep(*, generations: int, popsize: int = 150, seed: int = 20261007,
              sigma0: float = 0.3, dampfac: float = 1.0, workers: int = 1,
              mc_cfg: MCConfig = MCConfig(), weights: TailWeights = TailWeights(),
              eval_cfg: dict | None = None, seed_theta=None,
              checkpoint: str | None = None, resume: bool = False,
              verbose: bool = True) -> SweepReport:
    """Run the single global CMA-ES sweep and return its report.

    ``eval_cfg`` carries the closed-loop resolution (``dt_outer``, ``dt_inner``,
    ``t_end``, ``spec_box``, ``w_viol``, ``w_final``, ``maxiter``) handed to
    :func:`evaluate_candidate`.

    ``checkpoint``: one JSON line per generation, plus a final ``{"es": ...}``
    line holding the pickled CMA-ES state, so ``--resume`` can pick the search
    back up where it stopped instead of re-running the whole thing.
    """
    import cma

    eval_cfg = eval_cfg or {}
    t_start = time.time()

    design = build_design(mc_cfg)
    if verbose:
        print(f"[design] {design.n_runs} runs = "
              f"{len(design.vehicles)} vehicles x {mc_cfg.n_injections} injections",
              flush=True)
        _s = design.summary()
        print(f"[design] dv_frac {_s['dv_frac']['mean']:+.5f} +- "
              f"{_s['dv_frac']['std']:.5f}   alloc_eff "
              f"{_s['alloc_eff']['mean']:.3f} +- {_s['alloc_eff']['std']:.3f}   "
              f"a_max {_s['a_max']['mean']:.4f} +- {_s['a_max']['std']:.4f}",
              flush=True)

    lb, ub = _bounds()
    n = len(HYPER_SPACE)
    theta0 = np.clip(np.asarray(
        seed_theta if seed_theta is not None else HYPER_SPACE.seed_theta(),
        float), lb, ub)

    # CSA_dampfac is the multiplier on CMA-ES's step-size damping. The default
    # (1) adapts sigma conservatively, which on this landscape reads as "sigma
    # is becoming too slow": the population converges to a basin long before
    # sigma has moved anywhere. Measured on a 6-D test problem, one generation
    # at CSA_dampfac=1 shrinks sigma to 95.0% of sigma0, at 0.3 to 84.4% -- the
    # same adaptive path reached three times faster. Passed through rather than
    # hardcoded so the trade-off is a CLI flag, not a silent default.
    inopts = {"bounds": [list(lb), list(ub)], "popsize": int(popsize),
              "seed": int(seed), "verbose": -9, "tolx": 1e-4, "tolfun": 1e-6,
              "CSA_dampfac": float(dampfac)}
    es = cma.CMAEvolutionStrategy(list(theta0), sigma0, inopts=inopts)

    history: list[GenerationRecord] = []
    gen = 0

    # ---- resume --------------------------------------------------------
    if resume and checkpoint and os.path.exists(checkpoint):
        gen, history, es = _load_checkpoint(checkpoint, inopts, theta0, sigma0)
        if verbose and history:
            print(f"[resume] {len(history)} generation(s) restored, "
                  f"resuming at gen {gen}", flush=True)

    pool = None
    if workers > 1:
        pool = ProcessPoolExecutor(
            max_workers=workers, initializer=_init_worker,
            initargs=(design, eval_cfg, weights))
    else:
        _init_worker(design, eval_cfg, weights)

    try:
        while gen < generations and not es.stop():
            t_gen = time.time()
            xs = es.ask()
            if pool is None:
                payloads = [_eval_one(x) for x in xs]
            else:
                # shard the population one chunk per worker: the C++ ensemble is
                # the unit of work, so a per-candidate submission would be 150
                # IPC round trips a generation for no benefit
                shards = [list(s) for s in np.array_split(np.asarray(xs, float),
                                                         workers)]
                futs = [pool.submit(_eval_many, sh) for sh in shards if len(sh)]
                payloads = []
                for f in futs:
                    payloads.extend(f.result())

            ys = [float(p["cost"]) for p in payloads]
            es.tell(xs, ys)

            gen += 1
            # min(), not np.argmin. argmin over a list of 2-tuples makes an
            # (n, 2) array and returns the index into its FLAT form, which can
            # exceed n -- that crashed, and whenever the two runs it did not
            # crash on it silently returned payloads[0] instead of the winner.
            best_p = min(payloads, key=_rank_key)
            costs = np.array(ys)
            hs = np.array([int(p["horizon"]) for p in payloads])
            hcnt = {int(h): int((hs == h).sum()) for h in sorted(set(hs.tolist()))}
            rec = GenerationRecord(
                gen=gen, evals=int(es.result.evaluations),
                best_cost=float(best_p["cost"]),
                mean_cost=float(costs.mean()),
                median_cost=float(np.median(costs)),
                worst_cost=float(costs.max()),
                best_theta=list(best_p["theta"]),
                best_horizon=int(best_p["horizon"]),
                best=best_p,
                wall_s=time.time() - t_gen,
                n_infeasible_pop=int(sum(
                    1 for p in payloads
                    if p["nmpc"]["feasible_frac"] <= 0.0
                    or p["pid"]["feasible_frac"] <= 0.0)),
                horizon_counts=hcnt)
            history.append(rec)

            if verbose:
                print(f"  gen {gen:3d}  best={rec.best_cost:10.2f} "
                      f"mean={rec.mean_cost:10.2f} "
                      f"median={rec.median_cost:10.2f} "
                      f"H={best_p['horizon']:2d} "
                      f"feas={best_p['nmpc']['feasible_frac']:.2f}/"
                      f"{best_p['pid']['feasible_frac']:.2f} "
                      f"({rec.wall_s / 60:.1f} min)", flush=True)

            if checkpoint:
                _write_checkpoint(checkpoint, gen, history, es)
    finally:
        if pool is not None:
            pool.shutdown(wait=True, cancel_futures=True)

    # rank across the whole history, not just the last generation: CMA-ES's
    # internal best can be a point it later abandoned, and the reported winner
    # should be the cheapest *feasible* point actually evaluated.
    all_best = min((r.best for r in history),
                   key=_rank_key, default=None)
    if all_best is None:
        raise RuntimeError("CMA-ES produced no generations")

    return SweepReport(
        best=all_best,
        best_theta=all_best["theta"],
        best_cost=float(all_best["cost"]),
        history=[r.as_dict() for r in history],
        design=design.summary(),
        config={"generations": generations, "popsize": popsize, "seed": seed,
                "sigma0": sigma0, "workers": workers,
                "mc": mc_cfg.__dict__.copy(),
                "weights": weights.as_dict(),
                "eval_cfg": eval_cfg,
                "dim": n},
        wall_s=time.time() - t_start)


def _write_checkpoint(path: str, gen: int, history: list, es) -> None:
    """Rewrite the checkpoint atomically.

    The ES pickle can be tens of kB, so it is written to a temp file and
    renamed: a kill during the write must not leave a half-written state that
    ``--resume`` would silently mis-load.
    """
    import json

    recs = [r.as_dict() for r in history]
    blob = pickle.dumps(es)
    payload = {"gen": gen, "history": recs,
               "es_pickle_b64": _b64(blob)}
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(payload, fh)
    os.replace(tmp, path)


def _b64(b: bytes) -> str:
    import base64
    return base64.b64encode(b).decode()


def _unb64(s: str) -> bytes:
    import base64
    return base64.b64decode(s.encode())


def _load_checkpoint(path: str, inopts: dict, theta0, sigma0):
    """Restore ``(gen, history, es)`` from a checkpoint file.

    If the pickled CMA-ES state cannot be read (a cma version change, a
    truncated file), fall back to restarting the search from the best theta
    seen so far. That loses the covariance state and the evaluation history is
    kept only for the log, so it is a real degradation — it is logged rather
    than hidden, and the caller decides whether to accept it.
    """
    import json
    import cma

    with open(path) as fh:
        d = json.load(fh)
    recs = d.get("history", [])
    gen = int(d.get("gen", len(recs)))
    history = [GenerationRecord(**_rec_from_dict(r)) for r in recs]

    es = None
    try:
        es = pickle.loads(_unb64(d["es_pickle_b64"]))
    except Exception as exc:
        print(f"[resume] WARNING: could not restore CMA-ES state "
              f"({type(exc).__name__}); restarting search from the best theta "
              f"so far", flush=True)
    if es is None:
        es = cma.CMAEvolutionStrategy(
            list(theta0), sigma0, inopts=inopts)
    return gen, history, es


def _rec_from_dict(r: dict) -> dict:
    return {"gen": r["gen"], "evals": r["evals"],
            "best_cost": r["best_cost"], "mean_cost": r["mean_cost"],
            "median_cost": r["median_cost"], "worst_cost": r["worst_cost"],
            "best_theta": r["best_theta"],
            "best_horizon": r["best_horizon"], "best": r["best"],
            "wall_s": r["wall_s"],
            "n_infeasible_pop": r.get("n_infeasible_pop", 0),
            "horizon_counts": r.get("horizon_counts", {})}


__all__ = ["run_sweep", "SweepReport", "GenerationRecord"]