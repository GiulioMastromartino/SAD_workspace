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

from dataclasses import dataclass, field

import numpy as np

from cubesat_12u_sim.tuning.profiling import profiler as _prof
from .objective import Result


@dataclass
class RunReport:
    """Outcome of one CMA-ES restart."""

    theta: np.ndarray
    result: Result
    evals: int
    generations: int
    peer: Result | None = None  # joint runs: the PID arm at the same theta
    history: list = field(default_factory=list)  # best-cost-so-far per generation

    def as_dict(self) -> dict:
        d = {"theta": [float(x) for x in self.theta],
             "cost": self.result.cost, "dv": self.result.dv,
             "max_abs_r_err": self.result.max_abs_r_err,
             "final_e_a": self.result.final_e_a,
             "feasible": self.result.feasible,
             "evals": self.evals, "generations": self.generations,
             "history": [float(h) for h in self.history]}
        if self.peer is not None:
            d["peer_pid"] = self.peer.as_dict()
        return d


def rank_key(res: Result) -> tuple:
    """Feasible candidates always outrank infeasible ones, then lower cost."""
    return (0 if res.feasible else 1, res.cost)


def _bounds(space):
    lb, ub = space.bounds()
    return lb + 1e-12, ub - 1e-12  # cma wants bounds strictly outside the domain


def _cost(payload: dict, kind: str) -> float:
    return float(payload["nmpc"]["cost"] if kind == "joint" else payload["cost"])


def _bounds(space):
    lb, ub = space.bounds()
    return lb + 1e-12, ub - 1e-12  # cma wants bounds strictly outside the domain


def _make_payload(kind, cfg, horizon, lb, ub, use_cpp_pid, nmpc_backend,
                  shared_cache=None):
    return {"kind": kind, "cfg": cfg, "horizon": horizon,
            "lb": lb, "ub": ub, "use_cpp_pid": bool(use_cpp_pid),
            "nmpc_backend": nmpc_backend, "shared_cache": shared_cache}


def _as_result(payload: dict, kind: str) -> tuple[Result, Result | None]:
    if kind == "joint":
        return Result(**payload["nmpc"]), Result(**payload["pid"])
    return Result(**payload), None


# ---------------------------------------------------------------------------
# worker pool
# ---------------------------------------------------------------------------

_W: dict = {}
# The SyncManager backing the shared evaluation cache. Held at module scope
# because dropping the manager shuts its process down and invalidates the proxy
# that live workers are still using.
_MANAGER = None


def _init_worker(payload: dict, reset_timing: bool = True) -> None:
    """Prime this process as an evaluator.

    ``reset_timing`` must be True in a pool worker and False when the parent
    primes itself in-process. A forked worker inherits a *copy* of the parent's
    profiler, already partially populated with the parent's sections
    (``pool_setup``, ``restart0``, a half-open ``dispatch``). Reporting that
    copy back would double-count the parent's own work and nest it under a
    random generation, so a worker starts from an empty tree.

    The parent passes False on purpose: priming for the final re-evaluation
    must not erase the tree it is building.
    """
    import cma  # noqa: F401  (once per process, not per candidate)

    _W.clear()
    _W.update(payload)
    cache = payload.get("shared_cache")
    if cache is None:
        cache = {}
    _W["cache"] = cache
    p = _prof()
    if reset_timing:
        p.reset()
    _W["prof"] = p


def _make_shared_cache():
    """A cache dict visible to every pool worker, or ``None`` if unavailable.

    Per-worker caches are correct but make the hit rate a function of
    ``--workers``: the same candidate is simulated once in each worker that
    draws it, so ``evaluations`` — the denominator of every per-horizon rate in
    the report — shifts with the worker count and two runs become incomparable.

    Evaluation is deterministic, so sharing is safe. A ``Manager`` dict costs one
    IPC round trip per get and per put; at these rates that is a few hundred
    microseconds per generation against recomputing whole candidates, so it is
    worth it. Returns ``None`` on any failure and the caller degrades to
    per-worker caches rather than failing the sweep.
    """
    global _MANAGER
    try:
        from multiprocessing import Manager
        mgr = Manager()
        _MANAGER = mgr  # keep alive: losing it shuts the proxy's process down
        return mgr.dict()
    except Exception:
        return None


def _worker_prof():
    """This process's profiler, created on first use.

    Each pool worker records into its own profiler and the parent folds the
    serialised subtrees back in, so parent-side totals are *CPU* time summed
    over workers. Comparing that against the wall time of the enclosing section
    is how you confirm the pool is actually parallelising.
    """
    p = _W.get("prof")
    if p is None:
        p = _W["prof"] = _prof()
    return p


def _merge_shot(a: dict, b: dict) -> dict:
    """Additively merge two serialised timing subtrees (shards of one batch).

    The sharded dispatch paths receive one subtree per worker; summing them
    before folding into the parent keeps the merge cost independent of worker
    count. Derived fields are recomputed by the destination's ``to_dict``, so
    only totals, calls and counts are carried.
    """
    a["total_s"] = float(a.get("total_s", 0.0)) + float(b.get("total_s", 0.0))
    a["calls"] = int(a.get("calls", 0)) + int(b.get("calls", 0))
    ac = a.setdefault("counts", {})
    for k, v in (b.get("counts") or {}).items():
        ac[k] = ac.get(k, 0.0) + float(v)
    ach = a.setdefault("children", {})
    for name, cd in (b.get("children") or {}).items():
        if name in ach:
            _merge_shot(ach[name], cd)
        else:
            ach[name] = cd
    return a


def _drain_worker_timing() -> dict | None:
    """Snapshot this worker's timing subtree and reset it for the next batch.

    Draining (rather than reading a cumulative total) is what keeps per-batch
    attribution honest: each batch contributes exactly once, with no double
    counting when a worker process is reused across many generations.

    Only valid inside a pool worker. In-process evaluation shares the parent's
    profiler, so resetting it there would erase the parent's own sections — that
    path passes ``drain=False`` and lets the caller's ``isolated()`` scope it
    instead.
    """
    p = _worker_prof()
    if not p.enabled:
        return None
    snap = p.to_dict()
    p.reset()
    return snap


def _run_cxx(thetas, j_star: float, has_incumbent: bool,
             drain: bool = True) -> tuple[list[dict], dict | None]:
    """Full-C++ closed-loop sims for a shard of candidates (NMPC cxx backend).

    Scoring stays in Python (single source: :func:`objective.score`); the
    incumbent (``j_star``, ``has_incumbent``) enables safe early abort inside
    the C++ loop. Never raises: a broken candidate ranks last, like every
    other backend.

    Returns ``(payloads, timing)``; ``timing`` is this batch's drained subtree,
    ``None`` when timing is disabled or when ``drain`` is False (in-process
    evaluation, whose timing already sits in the caller's tree).
    """
    from .objective import PENALTY_INFEASIBLE, Result, score
    from .params import NMPC_SPACE
    from . import nmpcsolve as NS

    kind, cfg, horizon = _W["kind"], _W["cfg"], _W["horizon"]
    lb, ub = _W["lb"], _W["ub"]
    assert kind == "nmpc"
    p = _worker_prof()
    out = []
    for th in thetas:
        th = np.clip(np.asarray(th, float), lb, ub)
        try:
            # The whole closed loop happens inside one crossing, so this single
            # section is the only thing that can be attributed here. Everything
            # finer-grained lives inside the C++ kernel and is not visible from
            # Python; that limit is the reason eval.cxx is reported as its own
            # section rather than being folded into eval.sim.
            with p.section("cxx"):
                dv, mx, ea, info = NS.closed_loop(th, cfg, horizon, NMPC_SPACE,
                                                  j_star=j_star,
                                                  has_incumbent=has_incumbent)
            with p.section("score"):
                res = score(dv, mx, ea, cfg)
            p.count("aborted" if info.get("aborted") else "exact", 1)
            p.count("cxx_nevals", int(info.get("nevals", 0)))
        except Exception:
            res = Result(PENALTY_INFEASIBLE, float("inf"), float("inf"),
                         float("inf"), False, "cxx-exception")
            p.count("cxx_exception", 1)
        out.append(res.as_dict())
    return out, (_drain_worker_timing() if drain else None)


def _run_batch(thetas, drain: bool = True) -> tuple[list[dict], dict | None]:
    """Evaluate a batch of candidates.

    Returns ``(payloads, timing)`` where the payloads are plain picklable dicts
    and ``timing`` is this batch's drained timing subtree (``None`` when timing
    is off, or when ``drain`` is False). The two-value return is what lets the
    parent attribute per-generation cost without a second IPC channel.
    """
    from .objective import evaluate_joint, evaluate_nmpc, evaluate_pid

    kind, cfg, horizon = _W["kind"], _W["cfg"], _W["horizon"]
    lb, ub, cache = _W["lb"], _W["ub"], _W["cache"]
    nmpc_backend = _W.get("nmpc_backend", "numpy")
    p = _worker_prof()
    if kind == "pid" and _W.get("use_cpp_pid"):
        from .pidbatch import available as _cpp_ok, evaluate_pid_batch
        from .params import PID_SPACE
        if _cpp_ok:
            clipped = [np.clip(np.asarray(th, float), lb, ub) for th in thetas]
            with p.section("pid_cpp"):
                res = evaluate_pid_batch(clipped, cfg, PID_SPACE)
            return ([r.as_dict() for r in res],
                    _drain_worker_timing() if drain else None)
    out = []
    for th in thetas:
        th = np.clip(np.asarray(th, float), lb, ub)
        if kind == "pid":
            out.append(evaluate_pid(th, cfg, cache).as_dict())
        elif kind == "nmpc":
            out.append(evaluate_nmpc(th, cfg, horizon, cache,
                                     backend=nmpc_backend).as_dict())
        else:
            if nmpc_backend == "cxx":
                # joint on the full-C++ path: NMPC arm in C++, PID arm numpy.
                # No incumbent abort here (exact sims; the incumbent lives in
                # the restart loop and only reaches workers through _run_cxx).
                from . import nmpcsolve as _NS
                from .objective import (PENALTY_INFEASIBLE, Result,
                                        evaluate_pid, score)
                from .params import NMPC_SPACE as _NSP
                n = len(_NSP)
                with p.section("cxx"):
                    try:
                        dv, mx, ea, _info = _NS.closed_loop(
                            np.asarray(th, float)[:n], cfg, horizon, _NSP)
                        failed = False
                    except Exception:
                        dv = mx = ea = float("inf")
                        failed = True
                if failed:
                    res_n = Result(PENALTY_INFEASIBLE, float("inf"),
                                   float("inf"), float("inf"), False,
                                   "cxx-exception")
                    p.count("cxx_exception", 1)
                else:
                    with p.section("score"):
                        res_n = score(dv, mx, ea, cfg)
                res_p = evaluate_pid(np.asarray(th, float)[n:], cfg, cache)
            else:
                res_n, res_p = evaluate_joint(th, cfg, horizon, cache,
                                              nmpc_backend=nmpc_backend)
            out.append({"nmpc": res_n.as_dict(), "pid": res_p.as_dict()})
    return out, (_drain_worker_timing() if drain else None)


# ---------------------------------------------------------------------------
# single restart
# ---------------------------------------------------------------------------

def run_restart(theta0, space, kind: str, cfg, horizon: int, *, seed: int,
maxiter: int, popsize: int | None = None,
                 workers: int = 1, sigma0: float = 0.3,
                 use_cpp_pid: bool = False,
                 nmpc_backend: str = "numpy",
                 shared_cache=None,
                 _executor=None) -> RunReport:
    """One CMA-ES run from ``theta0``. Never raises: caller handles failure.

    ``_executor``: shared pool from :func:`multistart` (kept warm across
    restarts so worker JIT is paid once per stage, not once per restart).
    When omitted and ``workers > 1`` a private pool is created instead.
    """
    import cma

    lb, ub = _bounds(space)
    n = len(space)

    payload = _make_payload(kind, cfg, horizon, lb, ub, use_cpp_pid,
                            nmpc_backend, shared_cache=shared_cache)
    # cma >= 3 takes bounds/popsize/seed through the options dict, not as
    # positional args.
    inopts = {"bounds": [list(lb), list(ub)],
              "popsize": popsize or (4 + int(3 * np.log(n))),
              "maxiter": maxiter, "verbose": -9, "seed": int(seed),
              "tolx": 1e-4, "tolfun": 1e-6}

    es = cma.CMAEvolutionStrategy(list(np.clip(theta0, lb, ub)), sigma0,
                                  inopts=inopts)

    _pf = _prof()

    pool = _executor
    own_pool = False
    if pool is None:
        if workers > 1:
            from concurrent.futures import ProcessPoolExecutor
            pool = ProcessPoolExecutor(max_workers=workers,
                                       initializer=_init_worker,
                                       initargs=(payload,))
            own_pool = True
        else:
            _init_worker(payload, reset_timing=False)

    # Whole-population single future when the evaluator is a fused batch
    # kernel (pid-cpp): 1 IPC round-trip per generation instead of popsize.
    fused = kind == "pid" and use_cpp_pid
    # Full-C++ sims (nmpc cxx): shard the population into one future per
    # worker; each future runs whole sims in C++ with safe early abort
    # against the incumbent best cost.
    cxx = kind == "nmpc" and nmpc_backend == "cxx"
    best_j = float("inf")
    feas_seen = False
    best_so_far = float("inf")
    history: list[float] = []

    try:
        while not es.stop():
            # One generation, timed as a section. Its children split the cost
            # three ways that matter when a sweep is slow: ask/tell (CMA-ES
            # bookkeeping), dispatch (IPC and waiting), and the worker CPU that
            # comes back folded in from the pool.
            with _pf.section("generation"):
                with _pf.section("ask"):
                    xs = es.ask()
                with _pf.section("dispatch"):
                    _pf.count("mode.pool" if pool is not None else "mode.inline")
                    if pool is None:
                        # In-process: the evaluator records straight into this
                        # profiler, so no merge is needed (draining would reset
                        # the parent's tree, and an isolated() wrapper would
                        # add a redundant level above the "eval" section the
                        # evaluator already opens). The subtree shape therefore
                        # matches the pooled case.
                        if cxx:
                            payloads, timing = _run_cxx(xs, best_j, feas_seen,
                                                       drain=False)
                        else:
                            payloads, timing = _run_batch(xs, drain=False)
                    elif fused:
                        payloads, timing = pool.submit(
                            _run_batch, list(xs)).result()
                        if timing is not None:
                            _pf.merge_all(timing)
                    elif cxx:
                        shards = [list(s)
                                  for s in np.array_split(list(xs), workers)]
                        futs = [pool.submit(_run_cxx, sh, best_j, feas_seen)
                                for sh in shards if len(sh)]
                        payloads, timing = [], None
                        for f in futs:
                            part, t = f.result()
                            payloads.extend(part)
                            if t is not None:
                                timing = t if timing is None else _merge_shot(
                                    timing, t)
                        if timing is not None:
                            _pf.merge_all(timing)
                    else:
                        futures = [pool.submit(_run_batch, [x]) for x in xs]
                        payloads, timing = [], None
                        for f in futures:
                            part, t = f.result()
                            payloads.append(part[0])
                            if t is not None:
                                timing = t if timing is None else _merge_shot(
                                    timing, t)
                        if timing is not None:
                            _pf.merge_all(timing)
                    ys = [_cost(p, kind) for p in payloads]
                with _pf.section("tell"):
                    es.tell(xs, ys)
                for pl in payloads:
                    c = float(pl["cost"] if kind != "joint"
                              else pl["nmpc"]["cost"])
                    f = bool(pl["feasible"] if kind != "joint"
                             else pl["nmpc"]["feasible"])
                    if c < best_j:
                        best_j = c
                    feas_seen = feas_seen or f
                best_so_far = min(best_so_far, min(ys))
                history.append(best_so_far)
    finally:
        if own_pool and pool is not None:
            pool.shutdown(wait=True, cancel_futures=True)

    # Re-evaluate the reported best so its Result is a fresh evaluation of the
    # exact theta being reported, rather than whatever happened to sit in the
    # final generation. _W is pool-local, so prime it here. No incumbent abort
    # (j_star=inf): the reported number must be the exact cost, never an
    # abort-truncated bound.
    _init_worker(payload, reset_timing=False)
    # The cache is bypassed for this one call. Now that the cache is shared it
    # would otherwise very often contain this exact theta, and the re-evaluation
    # would silently become a lookup — which would quietly remove the guarantee
    # this line exists to provide (a value that is always a full, untruncated
    # cost). Values are bit-identical either way, so this is about the guarantee
    # staying real rather than about the number changing.
    _shared = _W.get("cache")
    _W["cache"] = {}
    try:
        with _pf.section("final_eval"):
            if cxx:
                final_payloads, timing = _run_cxx([es.result.xbest],
                                                  float("inf"), False,
                                                  drain=False)
            else:
                final_payloads, timing = _run_batch([es.result.xbest],
                                                    drain=False)
    finally:
        _W["cache"] = _shared
    res, peer = _as_result(final_payloads[0], kind)
    return RunReport(theta=np.asarray(es.result.xbest, float), result=res,
                     peer=peer, evals=int(es.result.evaluations),
                     generations=int(es.result.iterations),
                     history=history)


# ---------------------------------------------------------------------------
# multistart
# ---------------------------------------------------------------------------

def multistart(space, kind: str, cfg, horizon: int, *, restarts: int, seed: int,
               maxiter: int, workers: int, popsize: int | None = None,
               theta_seed=None, verbose: bool = True,
               checkpoint_path: str | None = None,
               skip_completed: int = 0,
               use_cpp_pid: bool = False,
               nmpc_backend: str = "numpy") -> tuple[RunReport, list]:
    """Run ``restarts`` CMA-ES runs; restart 0 seeds from ``theta_seed``.

    Returns ``(best_report, all_reports)``.

    ``checkpoint_path``: when given, each finished restart is appended as one
    JSON line (theta + result + restart index), so killing the sweep loses at
    most one restart. ``skip_completed``: resume mode — skip this many leading
    restarts (their reports are reloaded from the checkpoint file when it
    exists and matches).
    """
    import json as _json
    import os as _os

    _pf = _prof()
    reports: list[RunReport] = []
    # Resumed restarts cost no wall time but still belong in the tree: without
    # this section a resumed sweep would look artificially cheap next to a
    # from-scratch one.
    with _pf.section("resume"):
        if skip_completed and checkpoint_path and _os.path.exists(checkpoint_path):
            with open(checkpoint_path) as _fh:
                for _line in _fh:
                    _line = _line.strip()
                    if not _line:
                        continue
                    _d = _json.loads(_line)
                    if len(reports) >= skip_completed:
                        break
                    _res = Result(cost=float(_d["cost"]), dv=float(_d["dv"]),
                                  max_abs_r_err=float(_d["max_abs_r_err"]),
                                  final_e_a=float(_d["final_e_a"]),
                                  feasible=bool(_d["feasible"]),
                                  notes=str(_d.get("notes", "")))
                    _peer = None
                    if _d.get("peer_pid") is not None:
                        _pd = _d["peer_pid"]
                        _peer = Result(**{k: _pd[k] for k in
                                          ("cost", "dv", "max_abs_r_err",
                                           "final_e_a", "feasible", "notes")})
                    reports.append(RunReport(
                        theta=np.asarray(_d["theta"], float), result=_res,
                        evals=int(_d.get("evals", 0)),
                        generations=int(_d.get("generations", 0)), peer=_peer))
                    _pf.count("restarts", 1)
            if verbose and reports:
                print(f"    resumed {len(reports)} restart(s) "
                      f"from {checkpoint_path}")

    start_at = len(reports) if skip_completed else 0
    # One pool for all restarts in this sweep: workers stay warm (C++ JIT
    # templates, imports) across restarts instead of paying startup per
    # restart. Payload is identical for every restart (only theta0/seed vary).
    pool = None
    try:
        with _pf.section("pool_setup"):
            shared_cache = None
            if pool is None and workers > 1 and start_at < restarts:
                from concurrent.futures import ProcessPoolExecutor
                lb, ub = _bounds(space)
                shared_cache = _make_shared_cache()
                _pf.count("cache.shared", 1 if shared_cache is not None else 0)
                pool = ProcessPoolExecutor(
                    max_workers=workers, initializer=_init_worker,
                    initargs=(_make_payload(kind, cfg, horizon, lb, ub,
                                            use_cpp_pid, nmpc_backend,
                                            shared_cache=shared_cache),))
            _pf.count("workers", workers if pool is not None else 1)
        for i in range(start_at, restarts):
            rng = np.random.default_rng(seed + 1000 * i)
            if i == 0 and theta_seed is not None:
                t0 = np.clip(np.asarray(theta_seed, float), *space.bounds())
            elif i == 0:
                t0 = space.seed_theta()
            else:
                t0 = space.random_theta(rng)
            # Per-restart section: this is the level at which "which restart was
            # expensive" is answerable, and at which the CPU-vs-wall comparison
            # for the pool is meaningful.
            with _pf.section(f"restart{i}") as rsec:
                try:
                    rep = run_restart(t0, space, kind, cfg, horizon, seed=seed + i,
                                      maxiter=maxiter, popsize=popsize,
                                      workers=workers,
                                      use_cpp_pid=use_cpp_pid,
                                      nmpc_backend=nmpc_backend,
                                      shared_cache=shared_cache,
                                      _executor=pool)
                except Exception as exc:  # one broken restart must not sink the sweep
                    if verbose:
                        print(f"    restart {i}: FAILED "
                              f"({type(exc).__name__}: {exc})")
                    if rsec is not None:
                        rsec.counts["failed"] = 1
                    continue
            reports.append(rep)
            if checkpoint_path:
                _d = rep.as_dict()
                _d["restart"] = i
                _d["kind"] = kind
                _d["horizon"] = horizon
                with _pf.section("checkpoint_write"):
                    with open(checkpoint_path, "a") as _fh:
                        _fh.write(_json.dumps(_d, default=float) + "\n")
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
    finally:
        if pool is not None:
            with _pf.section("pool_shutdown"):
                pool.shutdown(wait=True, cancel_futures=True)
    with _pf.section("rank"):
        if not reports:
            raise RuntimeError("every CMA-ES restart failed")
        best = min(reports, key=lambda r: rank_key(r.result))
        _pf.count("reports", len(reports))
    return best, reports
