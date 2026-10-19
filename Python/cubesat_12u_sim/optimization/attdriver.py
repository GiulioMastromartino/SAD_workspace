"""One global CMA-ES over the attitude controller, Monte Carlo in the loop.

Same architecture as the orbit pipeline (:mod:`.mcdriver`): one optimiser, one
population, no restarts, every candidate scored on the same frozen ensemble,
parallelism across candidates. Deliberately a separate module rather than a
generalisation, for the reason documented there -- the two pipelines have
genuinely different shapes (no horizon coordinate here, a different design
object, a different rank key), and sharing code would couple two committed
results together.

The only shared pieces are the checkpoint read/write helpers, imported from
:mod:`.mcdriver`: they are generic over history dicts plus a pickled CMA-ES
state, and duplicating the atomic-write logic would be worse than the import.
"""
from __future__ import annotations

import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field

import numpy as np

from cubesat_12u_sim.optimization.attdesign import AttBundle, build_bundle
from cubesat_12u_sim.optimization.attobjective import (
    AttWeights, evaluate_candidate)
from cubesat_12u_sim.optimization.attparams import ATT_SPACE
from cubesat_12u_sim.optimization.mcdriver import _write_checkpoint
from cubesat_12u_sim.uncertainty.monte_carlo import MonteCarloConfig

# Populated by _init_worker; module-level because a pool worker is a fresh
# interpreter and per-call payloads would be re-unpickled on every submission.
_W: dict = {}


def _bounds():
    lb, ub = ATT_SPACE.bounds()
    # cma wants its bounds strictly outside the domain so it can still propose
    # an out-of-range point and have its own resampling handle it; the evaluator
    # clips before decoding.
    return lb + 1e-12, ub - 1e-12


def _init_worker(bundle: AttBundle, mc_cfg: MonteCarloConfig, cfg: dict,
                 weights: AttWeights) -> None:
    """Prime a pool worker as an evaluator.

    ``import heyoka`` first: the extension needs libheyoka's symbols before it
    can be dlopened, and the import is the expensive part of the first call.
    The warm call also builds this worker's Taylor template for the sweep's
    (B, tol) key -- ~3.5 min at B=50, paid once per worker, in parallel.
    """
    import heyoka  # noqa: F401
    import cma  # noqa: F401

    _W.clear()
    _W.update(bundle=bundle, mc_cfg=mc_cfg, cfg=cfg, weights=weights)
    try:
        _eval_one(ATT_SPACE.seed_theta())
    except Exception:
        # A warm-up failure is not fatal: the real evaluation has its own
        # try/except and will mark the candidate infeasible.
        pass


def _eval_one(theta) -> dict:
    """Evaluate one candidate and return a picklable payload."""
    import time as _time

    t0 = _time.time()
    res = evaluate_candidate(theta, _W["bundle"], _W["mc_cfg"],
                             weights=_W["weights"], **_W["cfg"])
    payload = res.as_dict()
    payload["wall_s"] = _time.time() - t0
    return payload


def _eval_many(thetas: list) -> list:
    return [_eval_one(t) for t in thetas]


@dataclass
class AttGenerationRecord:
    gen: int
    evals: int
    best_cost: float
    mean_cost: float
    median_cost: float
    worst_cost: float
    best_theta: list
    best: dict
    wall_s: float
    n_infeasible_pop: int = 0

    def as_dict(self) -> dict:
        return {"gen": self.gen, "evals": self.evals,
                "best_cost": self.best_cost, "mean_cost": self.mean_cost,
                "median_cost": self.median_cost, "worst_cost": self.worst_cost,
                "best_theta": self.best_theta, "best": self.best,
                "wall_s": self.wall_s,
                "n_infeasible_pop": self.n_infeasible_pop}


@dataclass
class AttSweepReport:
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
    feas = payload["stats"]["feasible_frac"] > 0.0
    return (0 if feas else 1, payload["cost"])


def _rec_from_dict(r: dict) -> dict:
    return {"gen": r["gen"], "evals": r["evals"],
            "best_cost": r["best_cost"], "mean_cost": r["mean_cost"],
            "median_cost": r["median_cost"], "worst_cost": r["worst_cost"],
            "best_theta": r["best_theta"], "best": r["best"],
            "wall_s": r["wall_s"],
            "n_infeasible_pop": r.get("n_infeasible_pop", 0)}


def _load_att_checkpoint(path: str, inopts: dict, theta0, sigma0):
    """Restore ``(gen, history, es)`` from an attitude checkpoint file.

    Same atomic-write/reset contract as the orbit loader, but constructs
    attitude records. Kept separate rather than parameterising the orbit one:
    the two record shapes differ and neither pipeline should be able to break
    the other's resume path.
    """
    import json
    import pickle
    import cma

    from cubesat_12u_sim.optimization.mcdriver import _unb64

    with open(path) as fh:
        d = json.load(fh)
    recs = d.get("history", [])
    gen = int(d.get("gen", len(recs)))
    history = [AttGenerationRecord(**_rec_from_dict(r)) for r in recs]

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


def run_sweep(*, generations: int, popsize: int = 50, seed: int = 20261007,
              sigma0: float = 0.3, dampfac: float = 0.3, workers: int = 1,
              n_runs: int = 50, base_seed: int = 20261007,
              mc_cfg: MonteCarloConfig | None = None,
              weights: AttWeights = AttWeights(),
              eval_cfg: dict | None = None, seed_theta=None,
              checkpoint: str | None = None, resume: bool = False,
              verbose: bool = True) -> AttSweepReport:
    """Run the single global CMA-ES sweep and return its report.

    ``eval_cfg`` carries the campaign resolution (``task_step``, ``log_step``,
    ``tol``, ``quantize``, ``g0``) handed to :func:`evaluate_candidate`. The
    mission itself -- durations, targets, thresholds, dispersions -- lives in
    ``mc_cfg`` and is fixed: the controller must fit inside the mission, not
    resize it.
    """
    import cma

    mc_cfg = mc_cfg or MonteCarloConfig()
    eval_cfg = eval_cfg or {}
    t_start = time.time()

    bundle = build_bundle(n_runs=n_runs, base_seed=base_seed, mc_cfg=mc_cfg,
                          task_step=eval_cfg.get("dt", 0.02),
                          log_step=eval_cfg.get("log_every", 50)
                          * eval_cfg.get("dt", 0.02))
    if verbose:
        print(f"[design] {bundle.n_runs} runs, {bundle.n_steps} steps each, "
              f"seeds {bundle.seeds[0]}..{bundle.seeds[-1]}", flush=True)

    lb, ub = _bounds()
    n = len(ATT_SPACE)
    theta0 = np.clip(np.asarray(
        seed_theta if seed_theta is not None else ATT_SPACE.seed_theta(),
        float), lb, ub)

    inopts = {"bounds": [list(lb), list(ub)], "popsize": int(popsize),
              "seed": int(seed), "verbose": -9, "tolx": 1e-4, "tolfun": 1e-6,
              "CSA_dampfac": float(dampfac)}
    es = cma.CMAEvolutionStrategy(list(theta0), sigma0, inopts=inopts)

    history: list[AttGenerationRecord] = []
    gen = 0

    if resume and checkpoint:
        import os as _os
        if _os.path.exists(checkpoint):
            gen, history, es = _load_att_checkpoint(checkpoint, inopts,
                                                    theta0, sigma0)
            if verbose and history:
                print(f"[resume] {len(history)} generation(s) restored, "
                      f"resuming at gen {gen}", flush=True)

    pool = None
    if workers > 1:
        pool = ProcessPoolExecutor(
            max_workers=workers, initializer=_init_worker,
            initargs=(bundle, mc_cfg, eval_cfg, weights))
    else:
        _init_worker(bundle, mc_cfg, eval_cfg, weights)

    try:
        while gen < generations and not es.stop():
            t_gen = time.time()
            xs = es.ask()
            if pool is None:
                payloads = [_eval_one(x) for x in xs]
            else:
                # shard the population one chunk per worker: one candidate is
                # one run_chunk call, the unit of work, so per-candidate
                # submission would be pure IPC overhead
                shards = [list(s) for s in np.array_split(np.asarray(xs, float),
                                                         workers)]
                futs = [pool.submit(_eval_many, sh) for sh in shards if len(sh)]
                payloads = []
                for f in futs:
                    payloads.extend(f.result())

            ys = [float(p["cost"]) for p in payloads]
            es.tell(xs, ys)

            gen += 1
            # min(), not np.argmin: argmin over key tuples flattens and can
            # return an out-of-range index (found the hard way in mcdriver).
            best_p = min(payloads, key=_rank_key)
            costs = np.array(ys)
            rec = AttGenerationRecord(
                gen=gen, evals=int(es.result.evaluations),
                best_cost=float(best_p["cost"]),
                mean_cost=float(costs.mean()),
                median_cost=float(np.median(costs)),
                worst_cost=float(costs.max()),
                best_theta=list(best_p["theta"]),
                best=best_p,
                wall_s=time.time() - t_gen,
                n_infeasible_pop=int(sum(
                    1 for p in payloads
                    if p["stats"]["feasible_frac"] <= 0.0)))
            history.append(rec)

            if verbose:
                st = best_p["stats"]
                print(f"  gen {gen:3d}  best={rec.best_cost:10.2f} "
                      f"mean={rec.mean_cost:10.2f} "
                      f"median={rec.median_cost:10.2f} "
                      f"feas={st['feasible_frac']:.2f} "
                      f"fuel={st['fuel_mean']:7.3f}g "
                      f"({rec.wall_s / 60:.1f} min)", flush=True)

            if checkpoint:
                _write_checkpoint(checkpoint, gen, history, es)
    finally:
        if pool is not None:
            pool.shutdown(wait=True, cancel_futures=True)

    # rank across the whole history: the reported winner is the cheapest
    # *feasible* point actually evaluated, not CMA-ES's internal best.
    all_best = min((r.best for r in history),
                   key=_rank_key, default=None)
    if all_best is None:
        raise RuntimeError("CMA-ES produced no generations")

    return AttSweepReport(
        best=all_best,
        best_theta=all_best["theta"],
        best_cost=float(all_best["cost"]),
        history=[r.as_dict() for r in history],
        design=bundle.summary(),
        config={"generations": generations, "popsize": popsize, "seed": seed,
                "sigma0": sigma0, "dampfac": dampfac, "workers": workers,
                "n_runs": n_runs, "base_seed": base_seed,
                "weights": weights.as_dict(), "eval_cfg": eval_cfg,
                "dim": n},
        wall_s=time.time() - t_start)


__all__ = ["run_sweep", "AttSweepReport", "AttGenerationRecord"]
