"""C++ batch evaluator for the PID arm (optional accelerator).

Mirrors :func:`objective.evaluate_pid` exactly (gated by parity test); one call
evaluates a whole CMA-ES population. Falls back to numpy when the compiled
``_pidbatch`` extension is unavailable.
"""
from __future__ import annotations

import numpy as np

try:
    from cubesat_12u_sim.native.cxxdrive import _pidbatch

    available = True
except Exception:
    _pidbatch = None  # type: ignore[assignment]
    available = False


def evaluate_pid_batch(thetas, cfg, space) -> list:
    """Evaluate a batch of log-space vectors; returns :class:`Result` list."""
    from .objective import Result, initial_state, score
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.control.nmpc import semi_major_axis

    thetas = [np.asarray(t, float) for t in thetas]
    P = np.array([[space.from_theta(t)[k] for k in space.names]
                  for t in thetas])
    out = np.zeros((len(thetas), 4))
    _pidbatch.pid_batch_eval(
        np.ascontiguousarray(P), np.ascontiguousarray(initial_state(cfg.v_frac)),
        cfg.dt_inner, int(round(cfg.t_end / cfg.dt_inner)),
        cfg.spec_box, cfg.w_viol, cfg.w_final, cfg.alloc_eff,
        C.MU_EARTH, C.R_ORBIT, C.PID_A_MAX, C.PID_INT_CLAMP, C.PID_DEADBAND,
        out)
    res = []
    for i in range(len(thetas)):
        dv, mx, ea, cost = (float(out[i, 0]), float(out[i, 1]),
                            float(out[i, 2]), float(out[i, 3]))
        if not np.isfinite(dv) or dv >= 1e299:
            from .objective import PENALTY_INFEASIBLE
            res.append(Result(PENALTY_INFEASIBLE, float("inf"), float("inf"),
                              float("inf"), False, "non-finite"))
        else:
            res.append(score(dv, mx, ea, cfg))
            assert abs(res[-1].cost - cost) < 1e-6 * max(abs(cost), 1.0), \
                "C++ cost disagrees with re-scored cost"
    return res
