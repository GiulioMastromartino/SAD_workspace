"""Tune the PID and NMPC controllers for minimum dv inside a station-keeping box.

Three stages, each feeding the next (run ``--stage all`` for the full sweep):

  1. **pid**   — CMA-ES over OrbitPID gains + ``box_r``.
  2. **nmpc**  — CMA-ES over NMPC cost weights + energy deadband + trim
     authority/budget, swept over the *structural* parameter (horizon). Each
     horizon is an independent problem because it changes the decision-vector
     size, not just a gain, so they are ranked against each other afterwards.
  3. **joint** — one vector holding both arms, seeded from stages 1 and 2, so
     the final head-to-head is tuned-vs-tuned rather than tuned-vs-default.
     The NMPC arm is the objective; the PID arm is carried along and reported.

Everything runs on a coarse RK4 rollout during the search (see
``optimization.objective``) and the winners are re-checked through the real
heyoka/thruster path by ``optimization.validate``.

Usage (from Python/)::

    ./env/bin/python -m cubesat_12u_sim.tuning.run_optimize --stage all
    ./env/bin/python -m cubesat_12u_sim.tuning.run_optimize --stage nmpc --restarts 3
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.tuning import profiling
from cubesat_12u_sim.optimization.driver import multistart, rank_key
from cubesat_12u_sim.optimization.objective import (PID_SEARCH_CFG, SEARCH_CFG, VALIDATE_CFG,
                                     EvalConfig, evaluate_nmpc, evaluate_pid,
                                     measure_alloc_eff)
from cubesat_12u_sim.optimization.params import (JOINT_SPACE, NMPC_HORIZONS, NMPC_SPACE,
                                  PID_SPACE)
from cubesat_12u_sim.optimization.validate import propellant_g, validate_nmpc, validate_pid

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "..", "..", "results_optimization")


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def _write(name: str, payload: dict) -> str:
    os.makedirs(OUTDIR, exist_ok=True)
    path = os.path.abspath(os.path.join(OUTDIR, name))
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True, default=float)
    return path


def _ckpt_path(args, name: str) -> str | None:
    if not getattr(args, "checkpoint", True):
        return None
    os.makedirs(OUTDIR, exist_ok=True)
    return os.path.abspath(os.path.join(OUTDIR, f"ckpt_{name}.jsonl"))


def _resume_count(args, path: str | None, restarts: int) -> int:
    """How many leading restarts already checkpointed (resume mode only)."""
    if not getattr(args, "resume", False) or not path:
        return 0
    try:
        with open(path) as fh:
            n = sum(1 for line in fh if line.strip())
    except FileNotFoundError:
        return 0
    return min(n, restarts)


def _fmt(res) -> str:
    tag = "FEASIBLE" if res.feasible else "INFEASIBLE"
    return (f"dv={res.dv:8.3f} m/s  box={res.max_abs_r_err / 1e3:7.2f} km  "
            f"e_a={res.final_e_a / 1e3:9.3f} km  J={res.cost:9.3f}  {tag}")


# ---------------------------------------------------------------------------
# stage 1: PID
# ---------------------------------------------------------------------------

def resolve_backends(args) -> tuple[str, str, list[str]]:
    """Decide the *effective* PID and NMPC backends once, up front.

    Every accelerated backend is an optional compiled extension. A run where
    ``_nmpcbatch`` fails to load silently produces numpy-RK4 numbers while the
    results JSON can still say ``"nmpc_backend": "cpp"`` — which is worse than
    no record at all, because it is confidently wrong. Resolving once, here,
    means the value that is reported, the value the stages use, and the value
    validation uses are the same string, and every downgrade is logged and
    persisted instead of being a local rebinding that disappears at the end of
    a function.

    ``cxx`` needs both libheyoka and NLopt; if only the first is present it
    degrades to ``cpp`` (batch cost/gradient, same prediction family) rather
    than all the way to numpy, since ``cpp`` is the closer substitute.
    """
    from cubesat_12u_sim.optimization.nmpcbatch import available as cpp_ok
    from cubesat_12u_sim.optimization.nmpcsolve import available as cxx_ok
    from cubesat_12u_sim.optimization.pidbatch import available as pid_cpp_ok

    notes: list[str] = []

    pid_eff = args.pid_backend
    if pid_eff == "cpp" and not pid_cpp_ok:
        pid_eff = "numpy"
        notes.append("pid: _pidbatch unavailable, fell back to numpy")

    nmpc_eff = args.nmpc_backend
    if nmpc_eff == "cpp" and not cpp_ok:
        nmpc_eff = "numpy"
        notes.append("nmpc: _nmpcbatch unavailable, fell back to numpy")
    elif nmpc_eff == "cxx" and not cxx_ok:
        nmpc_eff = "cpp" if cpp_ok else "numpy"
        notes.append(f"nmpc: _nmpcsolve unavailable (needs libheyoka + NLopt), "
                     f"fell back to {nmpc_eff}")

    for note in notes:
        _log(f"  warning: {note}")
    _log(f"  pid  backend (effective): {_backend_label(pid_eff)}")
    _log(f"  nmpc backend (effective): {_backend_label(nmpc_eff)}")
    return pid_eff, nmpc_eff, notes


def _backend_label(name: str) -> str:
    return {"cpp": "heyoka-cpp batch (C++ cost/grad, Taylor)",
            "cxx": "full-cxx closed loop (C++ + embedded NLopt-SLSQP)",
            "numpy": "numpy-rk4 (Python reference)"}.get(name, name)


def _validation_backend(nmpc_backend: str) -> str:
    """Map a search backend to the one validation can actually honour.

    ``cxx`` collapses the entire closed loop into one C++ crossing; validation
    cannot use it, because validation runs the real 12-state heyoka dynamics
    stepped from Python. The nearest prediction family is ``cpp``, the Heyoka
    Taylor cost/gradient — so ``cxx`` maps to ``cpp`` rather than silently
    dropping to the numpy closures.
    """
    return "numpy" if nmpc_backend == "numpy" else "cpp"


def stage_pid(args, cfg: EvalConfig, use_cpp: bool) -> dict:
    _p = profiling.profiler()
    _log("\n=== stage 1/3: PID gains (CMA-ES, %d restarts) ===" % args.restarts)
    _log(PID_SPACE.describe())
    with _p.section("baseline"):
        baseline = evaluate_pid(PID_SPACE.seed_theta(), cfg)
    _log(f"  config defaults: {_fmt(baseline)}")
    ckpt = _ckpt_path(args, "pid")
    skip = _resume_count(args, ckpt, args.restarts)
    _log(f"  pid backend: {'cpp-batch' if use_cpp else 'numpy'}")
    with _p.section("sweep"):
        best, reports = multistart(
            PID_SPACE, "pid", cfg, horizon=0, restarts=args.restarts,
            seed=args.seed, maxiter=args.maxiter, workers=args.workers,
            popsize=args.popsize, checkpoint_path=ckpt, skip_completed=skip,
            use_cpp_pid=use_cpp)
    _log(f"  best -> {_fmt(best.result)}")
    if best.result.dv < baseline.dv:
        _log(f"  gain vs config defaults: "
             f"{100 * (1 - best.result.dv / baseline.dv):.1f}% less dv")
    return {
        "stage": "pid",
        "space": PID_SPACE.names,
        "params": PID_SPACE.from_theta(best.theta),
        "theta": [float(x) for x in best.theta],
        "result": best.result.as_dict(),
        "restarts": [r.as_dict() for r in reports],
        "baseline_result": baseline.as_dict(),
    }


# ---------------------------------------------------------------------------
# stage 2: NMPC, with a horizon sweep
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# per-horizon timing
# ---------------------------------------------------------------------------
#
# The horizon sweep is a *structural* comparison: H changes the size of the
# decision vector, so it changes what the solver has to do, not merely how it
# is weighted. That makes cost-per-horizon a result in its own right, and one
# that cannot be recovered from the aggregate: an H=2 and an H=12 arm together
# average to something neither of them cost.
#
# These helpers lift each horizon's subtree out of the shared tree and reduce it
# to the handful of numbers that actually vary with H. Everything reported is
# an accumulated mean over the whole horizon's CMA-ES run (all restarts, all
# generations, all solves), never a single measurement.

def _find_all(node: dict, name: str, acc: list | None = None) -> list:
    """Every node named ``name`` anywhere in a serialised subtree.

    Aggregating matters: a horizon's solves are spread over one node per restart
    (and per generation path), so returning only the first match would pair a
    single restart's call count with iteration counts summed across all of them
    — inflating the per-solve rate by roughly the restart count.
    """
    if acc is None:
        acc = []
    if not isinstance(node, dict):
        return acc
    if name in (node.get("children") or {}):
        acc.append(node["children"][name])
    for child in (node.get("children") or {}).values():
        _find_all(child, name, acc)
    return acc


def _find(node: dict, name: str) -> dict | None:
    """First node named ``name``; use :func:`_find_all` when rates matter."""
    found = _find_all(node, name, [])
    return found[0] if found else None


def _agg(found: list) -> tuple[float, int]:
    """``(total_s, calls)`` summed over the nodes from :func:`_find_all`."""
    return (sum(n.get("total_s", 0.0) for n in found),
            sum(int(n.get("calls", 0)) for n in found))


def _count_sum(node: dict, name: str) -> float:
    """Total of a dimensionless counter across a serialised subtree."""
    if not isinstance(node, dict):
        return 0.0
    total = float((node.get("counts") or {}).get(name, 0.0))
    for child in (node.get("children") or {}).values():
        total += _count_sum(child, name)
    return total


def _horizon_timing(prof, section: str) -> dict:
    """Reduce one horizon's subtree to the numbers that vary with H.

    Kept flat and small on purpose: this lands in ``per_horizon`` alongside
    ``dv`` and feasibility, so a horizon can be compared on cost and effort in
    one glance. The full subtree stays available in the top-level ``timing``
    tree for anything not anticipated here.

    Every rate is normalised by evaluations rather than reported raw, because
    the horizons in a sweep are not guaranteed to have run the same amount of
    work: a restart that fails early, or one skipped on resume, changes the
    denominators. A raw millisecond total would then compare effort rather than
    cost per unit of effort.
    """
    sub = prof.subtree(section)
    if not sub:
        return {}
    evals = _count_sum(sub, "cache_miss")
    # Cost/gradient work is not comparable across backends without splitting it.
    # numpy differentiates by finite differences, so it pays ~3N+1 cost() calls
    # per iteration. cpp hands over a batched gradient, so it pays 1 crossing
    # that evaluates 3N+1 lanes inside C++. Reporting one combined number made
    # the cpp path look like it did no work at all.
    obj_evals = _count_sum(sub, "cost_evals")
    grad_calls = _count_sum(sub, "grad_evals")
    grad_lanes = _count_sum(sub, "grad_lanes")
    solve_total, solve_calls = _agg(_find_all(sub, "solve"))
    min_total, min_calls = _agg(_find_all(sub, "minimize"))
    generations = 0
    for name, child in (sub.get("children") or {}).items():
        if name.startswith("restart"):
            _, g = _agg(_find_all(child, "generation"))
            generations += g
    out = {
        "wall_s": sub.get("total_s", 0.0),
        "evaluations": evals,
        "generations": generations,
    }
    if evals:
        out["wall_s_per_eval"] = sub.get("total_s", 0.0) / evals
    if solve_calls:
        out["solve_ms"] = solve_total / solve_calls * 1e3
        out["solve_calls"] = solve_calls
        if evals:
            # actual controller replans per evaluated candidate
            out["plans_per_eval"] = solve_calls / evals
        nit = _count_sum(sub, "nit")
        if nit:
            out["nit_per_solve"] = nit / solve_calls
            # Per-iteration cost isolates decision-vector size from iteration
            # count: this is what shows a bigger horizon paying more per
            # iteration rather than simply iterating more.
            out["ms_per_nit"] = solve_total / nit * 1e3
        if solve_calls:
            # ~3N+1 on the numpy backend (finite differences); ~1 crossing over
            # 3N+1 batched lanes on cpp. Both are reported so the two backends
            # are comparable on the same footing.
            out["objective_evals_per_solve"] = obj_evals / solve_calls
            if grad_calls:
                out["grad_calls_per_solve"] = grad_calls / solve_calls
                out["lanes_per_grad"] = grad_lanes / grad_calls
    if min_calls:
        out["minimize_ms"] = min_total / min_calls * 1e3
    jac_total, jac_calls = _agg(_find_all(sub, "jac_setup"))
    if jac_calls:
        # Carries the cold Heyoka JIT on the cpp backend (~7 s first call, then
        # ~0.17 s warm from disk), so it is reported apart from the solve.
        out["jac_setup_ms"] = jac_total / jac_calls * 1e3
    fallbacks = _count_sum(sub, "failure") + _count_sum(sub, "plan_exception")
    if fallbacks:
        out["solver_failures"] = fallbacks
    return out


def _horizon_timing_table(per_horizon: list) -> list:
    """Flat per-horizon cost/effort rows, in sweep order."""
    rows = []
    for entry in per_horizon:
        t = entry.get("timing") or {}
        if not t:
            continue
        rows.append({
            "horizon": entry["horizon"],
            "wall_s": t.get("wall_s"),
            "evaluations": t.get("evaluations"),
            "wall_s_per_eval": t.get("wall_s_per_eval"),
            "plans_per_eval": t.get("plans_per_eval"),
            "objective_evals_per_solve": t.get("objective_evals_per_solve"),
            "grad_calls_per_solve": t.get("grad_calls_per_solve"),
            "lanes_per_grad": t.get("lanes_per_grad"),
            "nit_per_solve": t.get("nit_per_solve"),
            "ms_per_nit": t.get("ms_per_nit"),
            "solve_ms": t.get("solve_ms"),
            "minimize_ms": t.get("minimize_ms"),
        })
    return rows


def _fmt_h(v, spec="9.3f", scale=1.0):
    if v is None:
        return " " * 9
    return f"{v * scale:{spec}}"


def _horizon_table(per_horizon: list) -> str:
    """Console table: what each horizon cost, and what it bought."""
    rows = _horizon_timing_table(per_horizon)
    if not rows:
        return ""
    head = (f"  {'H':>3}{'evals':>7}{'wall/ev s':>11}{'plans/ev':>10}"
            f"{'nit/plan':>10}{'ms/nit':>9}{'obj/pln':>9}{'grad/pln':>10}"
            f"{'solve ms':>10}{'best dv':>10}{'box km':>9}")
    lines = ["  -- horizon cost (per-evaluation means over each horizon's sweep) --",
             head]
    for row, entry in zip(rows, [e for e in per_horizon if e.get("timing")]):
        res = entry["result"]
        lines.append(
            f"  {row['horizon']:>3}"
            f"{_fmt_h(row['evaluations'], '7.0f'):>7}"
            f"{_fmt_h(row['wall_s_per_eval'], '11.4f'):>11}"
            f"{_fmt_h(row['plans_per_eval'], '10.1f'):>10}"
            f"{_fmt_h(row['nit_per_solve'], '10.1f'):>10}"
            f"{_fmt_h(row['ms_per_nit'], '9.3f'):>9}"
            f"{_fmt_h(row['objective_evals_per_solve'], '9.1f'):>9}"
            f"{_fmt_h(row['grad_calls_per_solve'], '10.1f'):>10}"
            f"{_fmt_h(row['solve_ms'], '10.2f'):>10}"
            f"{res['dv']:>10.3f}"
            f"{res['max_abs_r_err'] / 1e3:>9.2f}")
    lines.append("  wall/ev = wall s per evaluated candidate; plans/ev = NMPC "
                 "replans per candidate; nit/plan = SLSQP iterations per replan; "
                 "ms/nit = per-iteration cost; obj/pln = cost() calls per replan "
                 "(SLSQP line search); grad/pln = gradient calls per replan. "
                 "lanes_per_grad (in per_horizon_timing) is the rollouts inside "
                 "one gradient: 0 for the adjoint, 3N+1 for finite differences "
                 "and for cpp's batched lanes.")
    return "\n".join(lines)


def stage_nmpc(args, cfg: EvalConfig, nmpc_backend: str) -> dict:
    _p = profiling.profiler()
    _log("\n=== stage 2/3: NMPC weights + trim, horizon sweep ===")
    _log(NMPC_SPACE.describe())
    _log(f"  horizons: {list(NMPC_HORIZONS)}")

    with _p.section("baseline"):
        baseline = evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=6)
    _log(f"  config defaults (H=6): {_fmt(baseline)}")

    per_horizon = []
    best_rep = None
    best_theta = None
    best_h = None
    _log(f"  nmpc backend: {_backend_label(nmpc_backend)}")
    # Each horizon is its own section: the sweep exists to compare horizons, so
    # their cost has to be separately readable (a bigger decision vector costs
    # more per solve, and that is a result, not an inconvenience).
    with _p.section("horizons"):
        for h in NMPC_HORIZONS:
            _log(f"  -- horizon H={h} --")
            ckpt = _ckpt_path(args, f"nmpc_h{h}")
            skip = _resume_count(args, ckpt, args.restarts)
            with _p.section(f"horizonH{h}"):
                rep, reports = multistart(
                    NMPC_SPACE, "nmpc", cfg, horizon=h, restarts=args.restarts,
                    seed=args.seed, maxiter=args.maxiter, workers=args.workers,
                    popsize=args.popsize, checkpoint_path=ckpt, skip_completed=skip,
                    nmpc_backend=nmpc_backend)
            _log(f"     best -> {_fmt(rep.result)}")
            per_horizon.append({"horizon": h,
                                "theta": [float(x) for x in rep.theta],
                                "result": rep.result.as_dict(),
                                "restarts": [r.as_dict() for r in reports],
                                "timing": _horizon_timing(_p, f"stage_nmpc.horizons.horizonH{h}")})
            if (best_rep is None
                    or rank_key(rep.result) < rank_key(best_rep.result)):
                best_rep, best_theta, best_h = rep, rep.theta, h

    _log("\n" + _horizon_table(per_horizon))
    _log(f"\n  best horizon H={best_h}: {_fmt(best_rep.result)}")
    return {
        "stage": "nmpc",
        "space": NMPC_SPACE.names,
        "horizon": best_h,
        "params": NMPC_SPACE.from_theta(best_theta),
        "theta": [float(x) for x in best_theta],
        "result": best_rep.result.as_dict(),
        "per_horizon": per_horizon,
        "per_horizon_timing": _horizon_timing_table(per_horizon),
        "baseline_result": baseline.as_dict(),
    }


# ---------------------------------------------------------------------------
# stage 3: joint
# ---------------------------------------------------------------------------

def stage_joint(args, cfg: EvalConfig, pid_out: dict, nmpc_out: dict,
                nmpc_backend: str) -> dict:
    _log("\n=== stage 3/3: joint NMPC+PID, seeded from stages 1 and 2 ===")
    _log(JOINT_SPACE.describe())
    h = int(nmpc_out["horizon"])
    seed = np.concatenate([np.asarray(nmpc_out["theta"], float),
                           np.asarray(pid_out["theta"], float)])
    ckpt = _ckpt_path(args, "joint")
    skip = _resume_count(args, ckpt, args.restarts)
    with profiling.profiler().section("sweep"):
        best, reports = multistart(
            JOINT_SPACE, "joint", cfg, horizon=h, restarts=args.restarts,
            seed=args.seed, maxiter=args.maxiter, workers=args.workers,
            popsize=args.popsize, theta_seed=seed,
            checkpoint_path=ckpt, skip_completed=skip,
            nmpc_backend=nmpc_backend)
    _log(f"  best NMPC -> {_fmt(best.result)}")
    if best.peer is not None:
        _log(f"  peer PID  -> {_fmt(best.peer)}")
    split = len(NMPC_SPACE)
    return {
        "stage": "joint",
        "space": JOINT_SPACE.names,
        "horizon": h,
        "nmpc_params": NMPC_SPACE.from_theta(best.theta[:split]),
        "pid_params": PID_SPACE.from_theta(best.theta[split:]),
        "theta": [float(x) for x in best.theta],
        "result": best.result.as_dict(),
        "peer_pid_result": best.peer.as_dict() if best.peer else None,
        "restarts": [r.as_dict() for r in reports],
        "seeded_from": {"nmpc_theta": nmpc_out["theta"],
                        "pid_theta": pid_out["theta"]},
    }


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------

def run_validation(pid_out: dict, nmpc_out: dict, joint_out: dict,
                   t_end: float, nmpc_backend: str = "numpy") -> dict:
    """Re-check the winners on the real simulation.

    ``nmpc_backend`` must be the *effective* backend from
    :func:`resolve_backends`. Note that ``cxx`` (whole sim inside one C++
    crossing) has no meaning here: validation runs the full heyoka 12-state
    dynamics with the Python-side SLSQP, so the closest prediction family is
    ``cpp``. Passing the requested flag instead would have made a ``cxx`` run
    validate through the numpy closures while reporting ``cxx``.
    """
    _p = profiling.profiler()
    _log(f"\n=== validation: heyoka + thruster allocation, t_end={t_end:.0f} s ===")
    _log(f"  validation backend: {_backend_label(_validation_backend(nmpc_backend))}")
    rows = []

    def _arm(label, fn):
        # One section per arm: the arms differ by two orders of magnitude in
        # cost (a PID arm has no solver at all), so a single combined number
        # would say nothing about which one is expensive.
        slug = label.lower().replace(" ", "_").replace("(", "").replace(")", "")
        with _p.section(f"arm_{slug}"):
            return fn()

    vbe = _validation_backend(nmpc_backend)
    combos = [
        ("PID (tuned)", lambda: validate_pid(pid_out["theta"], PID_SPACE,
                                            t_end=t_end)),
        ("NMPC (tuned)", lambda: validate_nmpc(nmpc_out["theta"], NMPC_SPACE,
                                               nmpc_out["horizon"], t_end=t_end,
                                               backend=vbe)),
        ("NMPC (config defaults)", lambda: validate_nmpc(
            NMPC_SPACE.seed_theta(), NMPC_SPACE, 6, t_end=t_end,
            backend=vbe)),
    ]
    if joint_out:
        split = len(NMPC_SPACE)
        combos.append(("NMPC (joint)", lambda: validate_nmpc(
            np.asarray(joint_out["theta"])[:split], NMPC_SPACE,
            joint_out["horizon"], t_end=t_end, backend=vbe)))
        combos.append(("PID (joint)", lambda: validate_pid(
            np.asarray(joint_out["theta"])[split:], PID_SPACE, t_end=t_end)))

    _log(f"  {'arm':<24}{'dv [m/s]':>11}{'prop [g]':>11}"
         f"{'max |r err| [km]':>18}{'final e_a [km]':>17}")
    for label, fn in combos:
        r = _arm(label, fn)
        r["label"] = label
        r["prop_g"] = propellant_g(r["dv"])
        rows.append(r)
        _log(f"  {label:<24}{r['dv']:>11.3f}{r['prop_g']:>11.1f}"
             f"{r['max_abs_r_err'] / 1e3:>18.2f}{r['final_e_a'] / 1e3:>17.3f}")
    return {"t_end": t_end, "arms": rows,
            "backend": {"requested": nmpc_backend, "used": vbe}}


# ---------------------------------------------------------------------------

def run_bench(args, nmpc_cfg: EvalConfig, pid_cfg: EvalConfig,
              nmpc_out: dict | None, pid_out: dict | None) -> dict:
    """Repeat one fast sim per arm and summarise the timing distribution.

    Why repeat rather than trust the sweep: within a single closed loop there
    are ~720 RK4 steps and ~72 NMPC solves, so ``total / calls`` already averages
    away per-call noise — but that average is over *correlated* work inside one
    trajectory, and it inherits whatever else the machine was doing for the whole
    run. Repeating the entire simulation gives independent repetitions, so
    ``std_s`` becomes a real uncertainty on "one fast sim costs X" and two arms
    can be compared with a spread rather than a single number.

    The interesting outputs are the per-section means: ``eval`` (whole
    evaluation), ``sim`` (rollout), ``sim.accel`` (controller), ``sim.rk4``
    (propagation) and, for the NMPC arms, ``nmpc.solve`` and ``nmpc.minimize``.
    Each of those sits far enough above a millisecond to be a measurement.
    """
    _p = profiling.profiler()
    reps = int(args.bench_reps)
    if reps <= 0:
        return {}
    if args.bench_theta == "tuned":
        if pid_out is None:
            raise SystemExit("--bench-theta tuned needs --stage pid results")
        pid_theta = np.asarray(pid_out["theta"], float)
        nmpc_theta = (np.asarray(nmpc_out["theta"], float) if nmpc_out
                      else NMPC_SPACE.seed_theta())
    else:
        pid_theta = PID_SPACE.seed_theta()
        nmpc_theta = NMPC_SPACE.seed_theta()

    arms: list[tuple[str, callable]] = [
        ("pid", lambda: evaluate_pid(pid_theta, pid_cfg)),
        ("nmpc", lambda: evaluate_nmpc(nmpc_theta, nmpc_cfg, horizon=6)),
    ]
    out: dict = {"reps": reps, "theta": args.bench_theta, "arms": {}}
    for name, fn in arms:
        snaps: list[dict[str, float]] = []
        for i in range(reps):
            # isolated(merge=False): each repetition is timed against a fresh
            # subtree, so repetitions are independent samples rather than one
            # undifferentiated sum. A discarded warm-up repetition is not
            # subtracted -- the cold-start JIT is a real cost of the first fast
            # sim and is reported as such via the first-repetition section.
            with _p.isolated("rep", merge=False) as sub:
                fn()
            if i == 0:
                warm = profiling.flatten(sub)
            else:
                snaps.append(profiling.flatten(sub))
            _log(f"    bench {name} rep {i + 1}/{reps}: "
                 f"{sum(profiling.flatten(sub).values()) * 1e3:.1f} ms")
        summary = profiling.summarize(snaps)
        total_mean = max((v["mean_s"] for k, v in summary.items()
                          if k == "eval"), default=0.0)
        _log("\n" + profiling.format_summary(summary, total_mean))
        out["arms"][name] = {"summary": summary, "cold_first_rep_s": warm}
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", default="all",
                    choices=["pid", "nmpc", "joint", "all"])
    ap.add_argument("--restarts", type=int, default=3)
    ap.add_argument("--maxiter", type=int, default=25)
    ap.add_argument("--popsize", type=int, default=None)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 4) - 2))
    ap.add_argument("--seed", type=int, default=20261007)
    ap.add_argument("--days", type=float, default=SEARCH_CFG.t_days,
                    help="closed-loop horizon used during the search")
    ap.add_argument("--spec-box", type=float, default=SEARCH_CFG.spec_box,
                    help="station-keeping box half-width [m] (feasibility)")
    ap.add_argument("--alloc-eff", type=float, default=SEARCH_CFG.alloc_eff,
                    help="commanded/achieved accel ratio of the jet allocator")
    ap.add_argument("--validate-t", type=float, default=7200.0,
                    help="validation horizon [s] on the full sim")
    ap.add_argument("--no-plot", action="store_true",
                    help="skip figure generation")
    ap.add_argument("--resume", action="store_true",
                    help="resume from per-restart checkpoint files in OUTDIR")
    ap.add_argument("--no-checkpoint", dest="checkpoint", action="store_false",
                    help="disable per-restart checkpoint files")
    ap.add_argument("--pid-backend", default="cpp", choices=["cpp", "numpy"],
                    help="PID-arm evaluator (cpp = fused C++ batch, gated by "
                         "parity test; numpy = reference)")
    ap.add_argument("--nmpc-backend", default="cpp",
                    choices=["cpp", "numpy", "cxx"],
                    help="NMPC-arm rollout (cpp = Heyoka C++ batch cost+grad, "
                         "~18x per solve warm, Taylor prediction; cxx = full C++ "
                         "closed-loop sim with embedded NLopt-SLSQP + early "
                         "abort, ~8x per sim; numpy = RK4 reference). Defaults "
                         "to cpp because that is the path the committed results "
                         "were produced with; if the extension is missing the "
                         "run downgrades to numpy, logs it, and records the "
                         "effective backend in the results JSON.")
    ap.add_argument("--no-timing", action="store_true",
                    help="disable the hierarchical timing profiler")
    ap.add_argument("--bench-reps", type=int, default=0,
                    help="after the sweep, re-run one fast sim this many times "
                         "per arm and report mean/std/min/max per section "
                         "(0 = off). Averaging repeated whole simulations is "
                         "what makes a sub-millisecond region measurable; a "
                         "single run's per-step time is timer noise.")
    ap.add_argument("--bench-theta", default="seed",
                    choices=["seed", "tuned"],
                    help="theta used for --bench-reps: the space seed point, or "
                         "the winning vector from the sweep")
    args = ap.parse_args()

    profiling.set_enabled(not args.no_timing)
    _p = profiling.profiler()

    nmpc_cfg = EvalConfig(t_days=args.days, spec_box=args.spec_box,
                          alloc_eff=args.alloc_eff)
    # the PID is flown as a DT_CONTROL zero-order-hold loop, so it is searched
    # at its own control period rather than the NMPC's
    pid_cfg = nmpc_cfg.with_dt(C.DT_CONTROL, C.DT_CONTROL)
    _log(f"nmpc search cfg: {nmpc_cfg}")
    _log(f"pid  search cfg: {pid_cfg}")
    _log(f"workers={args.workers} restarts={args.restarts} "
         f"maxiter={args.maxiter} seed={args.seed}")

    t_start = time.time()
    measured = measure_alloc_eff()
    _log(f"measured allocator efficiency: {measured:.3f} "
         f"(search uses {nmpc_cfg.alloc_eff})")
    pid_backend, nmpc_backend, backend_notes = resolve_backends(args)
    # Everything before the first stage is setup, not pipeline work, so it is
    # folded into one section rather than being mistaken for stage time.
    _p.reset()

    out: dict = {"config": {"days": nmpc_cfg.t_days,
                            "spec_box": nmpc_cfg.spec_box,
                            "alloc_eff": nmpc_cfg.alloc_eff,
                            "alloc_eff_measured": measured,
                            # requested vs effective, always both: a run that
                            # asked for cpp and silently got numpy must not be
                            # able to report "cpp"
                            "pid_backend": pid_backend,
                            "nmpc_backend": nmpc_backend,
                            "pid_backend_requested": args.pid_backend,
                            "nmpc_backend_requested": args.nmpc_backend,
                            "backend_fallbacks": backend_notes,
                            "nmpc_dt": [nmpc_cfg.dt_outer, nmpc_cfg.dt_inner],
                            "pid_dt": [pid_cfg.dt_outer, pid_cfg.dt_inner],
                            "restarts": args.restarts, "maxiter": args.maxiter,
                            "seed": args.seed, "workers": args.workers}}

    pid_out = nmpc_out = joint_out = None
    stage_wall: dict = {}
    if args.stage in ("pid", "all"):
        _t = time.time()
        with _p.section("stage_pid"):
            pid_out = stage_pid(args, pid_cfg, pid_backend == "cpp")
        stage_wall["pid"] = time.time() - _t
        _write("res_stage_pid.json", pid_out)
    if args.stage in ("nmpc", "all"):
        _t = time.time()
        with _p.section("stage_nmpc"):
            nmpc_out = stage_nmpc(args, nmpc_cfg, nmpc_backend)
        stage_wall["nmpc"] = time.time() - _t
        _write("res_stage_nmpc.json", nmpc_out)
    if args.stage in ("joint", "all"):
        if pid_out is None:
            raise SystemExit("--stage joint needs the stage 1 result; run "
                             "--stage all or pass it back from results")
        if nmpc_out is None:
            raise SystemExit("--stage joint needs the stage 2 result; run "
                             "--stage all or pass it back from results")
        _t = time.time()
        with _p.section("stage_joint"):
            joint_out = stage_joint(args, nmpc_cfg, pid_out, nmpc_out,
                                    nmpc_backend)
        stage_wall["joint"] = time.time() - _t

    if pid_out:
        out["pid"] = pid_out
    if nmpc_out:
        out["nmpc"] = nmpc_out
    if joint_out:
        out["joint"] = joint_out
    if pid_out and nmpc_out:
        with _p.section("validation"):
            out["validation"] = run_validation(pid_out, nmpc_out, joint_out,
                                               args.validate_t, nmpc_backend)

    out["wall_seconds"] = time.time() - t_start
    out["stage_wall_seconds"] = stage_wall
    # Full hierarchical tree. Includes the per-worker CPU that was shipped
    # back from the pool, so eval.total can exceed wall time under parallelism;
    # both are reported because the gap is the parallelism.
    out["timing"] = _p.to_dict()
    path = _write("res_optimization.json", out)
    _log(f"\nwrote {path}  ({out['wall_seconds'] / 60:.1f} min)")
    if not args.no_plot and pid_out and nmpc_out:
        from cubesat_12u_sim.optimization.plots import make_all_figures
        with _p.section("figures"):
            figs = make_all_figures(out, OUTDIR)
        for f in figs:
            _log(f"[info] wrote {f}")
    if args.bench_reps > 0:
        with _p.section("bench"):
            out["bench"] = run_bench(args, nmpc_cfg, pid_cfg, nmpc_out, pid_out)

    # Written after the bench so res_optimization.json carries the distributions
    # alongside the sweep tree.
    out["timing"] = _p.to_dict()
    _write("res_optimization.json", out)

    if profiling.is_enabled():
        _log("\n" + _p.report(f"run_optimize (workers={args.workers})"))
        _write("res_timing.json", {"wall_seconds": out["wall_seconds"],
                                   "config": out["config"],
                                   "timing": _p.to_dict()})
        _log(f"\n[info] wrote {_write('res_timing.txt', profiling.format_report('pipeline', _p.root))}")


if __name__ == "__main__":
    main()
