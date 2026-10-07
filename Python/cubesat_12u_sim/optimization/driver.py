"""CMA-ES driver with bounds, restarts, an evaluation cache and a worker pool.

Mirrors the reference tuning scripts: several independent restarts per problem,
best run kept, and parallelism across *restarts*. (Parallelism across CMA-ES
generations would need a synchronisation point every generation, which costs
more than it saves at these population sizes; CMA-ES generations are inherently
sequential.)

Per-worker initialisation matters: each process builds its own evaluation cache
and imports cma/scipy once, so the fixed cost is not paid per candidate.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .objective import Result


@dataclass
class RunReport:
    """Outcome of one CMA-ES restart."""

    theta: np.ndarray
    result: Result
    evals: int
    generations: int
    peer: Result | None = None  # joint runs: the PID arm at the same theta

    def as_dict(self) -> dict:
        d = {"theta": [float(x) for x in self.theta],
             "cost": self.result.cost, "dv": self.result.dv,
             "max_abs_r_err": self.result.max_abs_r_err,
             "final_e_a": self.result.final_e_a,
             "feasible": self.result.feasible,
             "evals": self.evals, "generations": self.generations}
        if self.peer is not None:
            d["peer_pid"] = self.peer.as_dict()
        return d


def rank_key(res: Result) -> tuple:
    """Feasible candidates always outrank infeasible ones, then lower cost."""
    return (0 if res.feasible else 1, res.cost)


def _cost(payload: dict, kind: str) -> float:
    return float(payload["nmpc"]["cost"] if kind == "joint" else payload["cost"])


def _as_result(payload: dict, kind: str) -> tuple[Result, Result | None]:
    if kind == "joint":
        return Result(**payload["nmpc"]), Result(**payload["pid"])
    return Result(**payload), None


# ---------------------------------------------------------------------------
# worker pool
# ---------------------------------------------------------------------------

_W: dict = {}


def _init_worker(payload: dict) -> None:
    import cma  # noqa: F401  (once per process, not per candidate)

    _W.clear()
    _W.update(payload)
    _W["cache"] = {}


def _run_batch(thetas) -> list[dict]:
    """Evaluate a batch of candidates; returns plain dicts (picklable)."""
    from .objective import evaluate_joint, evaluate_nmpc, evaluate_pid

    kind, cfg, horizon = _W["kind"], _W["cfg"], _W["horizon"]
    lb, ub, cache = _W["lb"], _W["ub"], _W["cache"]
    out = []
    for th in thetas:
        th = np.clip(np.asarray(th, float), lb, ub)
        if kind == "pid":
            out.append(evaluate_pid(th, cfg, cache).as_dict())
        elif kind == "nmpc":
            out.append(evaluate_nmpc(th, cfg, horizon, cache).as_dict())
        else:
            res_n, res_p = evaluate_joint(th, cfg, horizon, cache)
            out.append({"nmpc": res_n.as_dict(), "pid": res_p.as_dict()})
    return out


# ---------------------------------------------------------------------------
# single restart
# ---------------------------------------------------------------------------

def run_restart(theta0, space, kind: str, cfg, horizon: int, *, seed: int,
                maxiter: int, popsize: int | None = None,
                workers: int = 1, sigma0: float = 0.3) -> RunReport:
    """One CMA-ES run from ``theta0``. Never raises: caller handles failure."""
    import cma

    lb, ub = space.bounds()
    lb = lb + 1e-12          # cma wants bounds strictly outside the domain
    ub = ub - 1e-12
    n = len(space)

    payload = {"kind": kind, "cfg": cfg, "horizon": horizon,
               "lb": lb, "ub": ub}
    # cma >= 3 takes bounds/popsize/seed through the options dict, not as
    # positional args.
    inopts = {"bounds": [list(lb), list(ub)],
              "popsize": popsize or (4 + int(3 * np.log(n))),
              "maxiter": maxiter, "verbose": -9, "seed": int(seed),
              "tolx": 1e-4, "tolfun": 1e-6}

    es = cma.CMAEvolutionStrategy(list(np.clip(theta0, lb, ub)), sigma0,
                                  inopts=inopts)

    pool = None
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        pool = ProcessPoolExecutor(max_workers=workers,
                                   initializer=_init_worker,
                                   initargs=(payload,))
    else:
        _init_worker(payload)

    try:
        while not es.stop():
            xs = es.ask()
            if pool is None:
                payloads = _run_batch(xs)
            else:
                futures = [pool.submit(_run_batch, [x]) for x in xs]
                payloads = [f.result()[0] for f in futures]
            es.tell(xs, [_cost(p, kind) for p in payloads])
    finally:
        if pool is not None:
            pool.shutdown(wait=True, cancel_futures=True)

    # Re-evaluate the reported best so its Result is a fresh evaluation of the
    # exact theta being reported, rather than whatever happened to sit in the
    # final generation. _W is pool-local, so prime it here.
    _init_worker(payload)
    final_payloads = _run_batch([es.result.xbest])
    res, peer = _as_result(final_payloads[0], kind)
    return RunReport(theta=np.asarray(es.result.xbest, float), result=res,
                     peer=peer, evals=int(es.result.evaluations),
                     generations=int(es.result.iterations))


# ---------------------------------------------------------------------------
# multistart
# ---------------------------------------------------------------------------

def multistart(space, kind: str, cfg, horizon: int, *, restarts: int, seed: int,
               maxiter: int, workers: int, popsize: int | None = None,
               theta_seed=None, verbose: bool = True) -> tuple[RunReport, list]:
    """Run ``restarts`` CMA-ES runs; restart 0 seeds from ``theta_seed``.

    Returns ``(best_report, all_reports)``.
    """
    reports: list[RunReport] = []
    for i in range(restarts):
        rng = np.random.default_rng(seed + 1000 * i)
        if i == 0 and theta_seed is not None:
            t0 = np.clip(np.asarray(theta_seed, float), *space.bounds())
        elif i == 0:
            t0 = space.seed_theta()
        else:
            t0 = space.random_theta(rng)
        try:
            rep = run_restart(t0, space, kind, cfg, horizon, seed=seed + i,
                              maxiter=maxiter, popsize=popsize, workers=workers)
        except Exception as exc:  # one broken restart must not sink the sweep
            if verbose:
                print(f"    restart {i}: FAILED ({type(exc).__name__}: {exc})")
            continue
        reports.append(rep)
        if verbose:
            r = rep.result
            extra = ""
            if rep.peer is not None:
                extra = f" | PID dv={rep.peer.dv:7.3f} box={rep.peer.max_abs_r_err / 1e3:6.2f} km"
            print(f"    restart {i}: cost={r.cost:9.3f} dv={r.dv:7.3f} m/s "
                  f"box={r.max_abs_r_err / 1e3:6.2f} km "
                  f"e_a={r.final_e_a / 1e3:8.3f} km "
                  f"{'FEASIBLE' if r.feasible else 'INFEASIBLE'}{extra} "
                  f"({rep.evals} evals)")
    if not reports:
        raise RuntimeError("every CMA-ES restart failed")
    best = min(reports, key=lambda r: rank_key(r.result))
    return best, reports
