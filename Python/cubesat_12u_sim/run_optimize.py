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

    ./env/bin/python -m cubesat_12u_sim.run_optimize --stage all
    ./env/bin/python -m cubesat_12u_sim.run_optimize --stage nmpc --restarts 3
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

from . import config as C
from .optimization.driver import multistart, rank_key
from .optimization.objective import (PID_SEARCH_CFG, SEARCH_CFG, VALIDATE_CFG,
                                     EvalConfig, evaluate_nmpc, evaluate_pid,
                                     measure_alloc_eff)
from .optimization.params import (JOINT_SPACE, NMPC_HORIZONS, NMPC_SPACE,
                                  PID_SPACE)
from .optimization.validate import propellant_g, validate_nmpc, validate_pid

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "..", "results_optimization")


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

def stage_pid(args, cfg: EvalConfig) -> dict:
    _log("\n=== stage 1/3: PID gains (CMA-ES, %d restarts) ===" % args.restarts)
    _log(PID_SPACE.describe())
    baseline = evaluate_pid(PID_SPACE.seed_theta(), cfg)
    _log(f"  config defaults: {_fmt(baseline)}")
    ckpt = _ckpt_path(args, "pid")
    skip = _resume_count(args, ckpt, args.restarts)
    from .optimization.pidbatch import available as _cpp_pid_ok
    use_cpp = args.pid_backend == "cpp" and _cpp_pid_ok
    if args.pid_backend == "cpp" and not _cpp_pid_ok:
        _log("  warning: _pidbatch unavailable, falling back to numpy")
    _log(f"  pid backend: {'cpp-batch' if use_cpp else 'numpy'}")
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

def stage_nmpc(args, cfg: EvalConfig) -> dict:
    _log("\n=== stage 2/3: NMPC weights + trim, horizon sweep ===")
    _log(NMPC_SPACE.describe())
    _log(f"  horizons: {list(NMPC_HORIZONS)}")

    baseline = evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=6)
    _log(f"  config defaults (H=6): {_fmt(baseline)}")

    per_horizon = []
    best_rep = None
    best_theta = None
    best_h = None
    from .optimization.nmpcbatch import available as _cpp_nmpc_ok
    nmpc_backend = args.nmpc_backend
    if nmpc_backend == "cpp" and not _cpp_nmpc_ok:
        _log("  warning: _nmpcbatch unavailable, falling back to numpy")
        nmpc_backend = "numpy"
    _log(f"  nmpc backend: {'heyoka-cpp' if nmpc_backend == 'cpp' else 'numpy-rk4'}")
    for h in NMPC_HORIZONS:
        _log(f"  -- horizon H={h} --")
        ckpt = _ckpt_path(args, f"nmpc_h{h}")
        skip = _resume_count(args, ckpt, args.restarts)
        rep, reports = multistart(
            NMPC_SPACE, "nmpc", cfg, horizon=h, restarts=args.restarts,
            seed=args.seed, maxiter=args.maxiter, workers=args.workers,
            popsize=args.popsize, checkpoint_path=ckpt, skip_completed=skip,
            nmpc_backend=nmpc_backend)
        _log(f"     best -> {_fmt(rep.result)}")
        per_horizon.append({"horizon": h, "theta": [float(x) for x in rep.theta],
                            "result": rep.result.as_dict(),
                            "restarts": [r.as_dict() for r in reports]})
        if best_rep is None or rank_key(rep.result) < rank_key(best_rep.result):
            best_rep, best_theta, best_h = rep, rep.theta, h

    _log(f"\n  best horizon H={best_h}: {_fmt(best_rep.result)}")
    return {
        "stage": "nmpc",
        "space": NMPC_SPACE.names,
        "horizon": best_h,
        "params": NMPC_SPACE.from_theta(best_theta),
        "theta": [float(x) for x in best_theta],
        "result": best_rep.result.as_dict(),
        "per_horizon": per_horizon,
        "baseline_result": baseline.as_dict(),
    }


# ---------------------------------------------------------------------------
# stage 3: joint
# ---------------------------------------------------------------------------

def stage_joint(args, cfg: EvalConfig, pid_out: dict, nmpc_out: dict) -> dict:
    _log("\n=== stage 3/3: joint NMPC+PID, seeded from stages 1 and 2 ===")
    _log(JOINT_SPACE.describe())
    h = int(nmpc_out["horizon"])
    seed = np.concatenate([np.asarray(nmpc_out["theta"], float),
                           np.asarray(pid_out["theta"], float)])
    ckpt = _ckpt_path(args, "joint")
    skip = _resume_count(args, ckpt, args.restarts)
    from .optimization.nmpcbatch import available as _cpp_nmpc_ok2
    nmpc_backend = args.nmpc_backend
    if nmpc_backend == "cpp" and not _cpp_nmpc_ok2:
        _log("  warning: _nmpcbatch unavailable, falling back to numpy")
        nmpc_backend = "numpy"
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
    _log(f"\n=== validation: heyoka + thruster allocation, t_end={t_end:.0f} s ===")
    rows = []
    combos = [
        ("PID (tuned)", validate_pid(pid_out["theta"], PID_SPACE, t_end=t_end)),
        ("NMPC (tuned)", validate_nmpc(nmpc_out["theta"], NMPC_SPACE,
                                       nmpc_out["horizon"], t_end=t_end,
                                       backend=nmpc_backend)),
        ("NMPC (config defaults)", validate_nmpc(
            NMPC_SPACE.seed_theta(), NMPC_SPACE, 6, t_end=t_end,
            backend=nmpc_backend)),
    ]
    if joint_out:
        split = len(NMPC_SPACE)
        combos.append(("NMPC (joint)", validate_nmpc(
            np.asarray(joint_out["theta"])[:split], NMPC_SPACE,
            joint_out["horizon"], t_end=t_end, backend=nmpc_backend)))
        combos.append(("PID (joint)", validate_pid(
            np.asarray(joint_out["theta"])[split:], PID_SPACE, t_end=t_end)))

    _log(f"  {'arm':<24}{'dv [m/s]':>11}{'prop [g]':>11}"
         f"{'max |r err| [km]':>18}{'final e_a [km]':>17}")
    for label, r in combos:
        r["label"] = label
        r["prop_g"] = propellant_g(r["dv"])
        rows.append(r)
        _log(f"  {label:<24}{r['dv']:>11.3f}{r['prop_g']:>11.1f}"
             f"{r['max_abs_r_err'] / 1e3:>18.2f}{r['final_e_a'] / 1e3:>17.3f}")
    return {"t_end": t_end, "arms": rows}


# ---------------------------------------------------------------------------

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
    ap.add_argument("--nmpc-backend", default="numpy", choices=["cpp", "numpy"],
                    help="NMPC-arm rollout (cpp = Heyoka C++ batch cost+grad, "
                         "~11x per solve warm; numpy = RK4 reference)")
    args = ap.parse_args()

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

    out: dict = {"config": {"days": nmpc_cfg.t_days,
                            "spec_box": nmpc_cfg.spec_box,
                            "alloc_eff": nmpc_cfg.alloc_eff,
                            "alloc_eff_measured": measured,
                            "pid_backend": args.pid_backend,
                            "nmpc_backend": args.nmpc_backend,
                            "nmpc_dt": [nmpc_cfg.dt_outer, nmpc_cfg.dt_inner],
                            "pid_dt": [pid_cfg.dt_outer, pid_cfg.dt_inner],
                            "restarts": args.restarts, "maxiter": args.maxiter,
                            "seed": args.seed, "workers": args.workers}}

    pid_out = nmpc_out = joint_out = None
    stage_wall: dict = {}
    if args.stage in ("pid", "all"):
        _t = time.time()
        pid_out = stage_pid(args, pid_cfg)
        stage_wall["pid"] = time.time() - _t
        _write("res_stage_pid.json", pid_out)
    if args.stage in ("nmpc", "all"):
        _t = time.time()
        nmpc_out = stage_nmpc(args, nmpc_cfg)
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
        joint_out = stage_joint(args, nmpc_cfg, pid_out, nmpc_out)
        stage_wall["joint"] = time.time() - _t

    if pid_out:
        out["pid"] = pid_out
    if nmpc_out:
        out["nmpc"] = nmpc_out
    if joint_out:
        out["joint"] = joint_out
    if pid_out and nmpc_out:
        out["validation"] = run_validation(pid_out, nmpc_out, joint_out,
                                           args.validate_t, args.nmpc_backend)

    out["wall_seconds"] = time.time() - t_start
    out["stage_wall_seconds"] = stage_wall
    path = _write("res_optimization.json", out)
    _log(f"\nwrote {path}  ({out['wall_seconds'] / 60:.1f} min)")
    if not args.no_plot and pid_out and nmpc_out:
        from .optimization.plots import make_all_figures
        figs = make_all_figures(out, OUTDIR)
        for f in figs:
            _log(f"[info] wrote {f}")


if __name__ == "__main__":
    main()
