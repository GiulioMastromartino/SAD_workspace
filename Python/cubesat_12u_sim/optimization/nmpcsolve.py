"""Full NMPC closed-loop sim in one C++ call (optional accelerator).

``closed_loop_batch`` runs the whole HierarchicalController rollout —
NLopt-SLSQP bulk plans (warm-started) + trim + alloc_eff + early abort —
with exactly one Python->C++ crossing per candidate. Python keeps scoring,
CMA-ES ask/tell, dispatch and bookkeeping (orchestrator only).

Parity gate: closed-loop dv within 5% + same feasibility verdict vs the
numpy path on seed theta (see test).
"""
from __future__ import annotations

import numpy as np

try:
    import heyoka  # noqa: F401  (libheyoka for dynamic_lookup)
    from cubesat_12u_sim.native.cxxdrive import _nmpcsolve

    available = True
except Exception:
    _nmpcsolve = None  # type: ignore[assignment]
    available = False

GAIN_KEYS = ("kp_r", "ki_r", "kd_r", "kp_t", "ki_t", "kp_n", "ki_n", "kd_n")


def closed_loop(theta, cfg, horizon: int, space,
                j_star: float = float("inf"),
                has_incumbent: bool = False,
                maxeval_cap: int = 0):
    """Run one candidate's closed loop in C++; return ``(dv, max_err, e_a)``."""
    from cubesat_12u_sim.control.adjoint import fuel_params
    from cubesat_12u_sim.core import config as C
    from .objective import TRACK_EDGE, TRACK_FRAC, initial_state

    p = space.from_theta(np.asarray(theta, float))
    n_outer = int(round(cfg.t_end / cfg.dt_outer))
    gains = {k: float(getattr(C, "PID_" + k.upper())) for k in GAIN_KEYS}
    fA, fx0, fg1 = fuel_params(C.NMPC_A_MAX, cfg.dt_outer)
    fA *= getattr(cfg, 'fuel_scale', 1.0)
    out = np.zeros((1, 10))
    _nmpcsolve.closed_loop_batch(
        np.ascontiguousarray(initial_state(cfg.v_frac)[None, :]),
        int(horizon), float(cfg.dt_outer), float(cfg.dt_inner), int(n_outer),
        int(cfg.maxiter), float(p["db_a"]), float(C.NMPC_A_MAX),
        float(p["trim_a_max"]), float(p["trim_dv_budget"]), 0.2,
        gains["kp_r"], gains["ki_r"], gains["kd_r"],
        gains["kp_t"], gains["ki_t"], gains["kp_n"], gains["ki_n"],
        gains["kd_n"], float(C.PID_INT_CLAMP), float(C.PID_A_MAX),
        float(C.PID_DEADBAND), float(C.MU_EARTH), float(C.R_ORBIT), 1e-12,
        float(cfg.alloc_eff), float(cfg.spec_box),
        TRACK_EDGE, TRACK_FRAC, float(cfg.w_final), fA, fx0, fg1,
        float(j_star), bool(has_incumbent), int(maxeval_cap), out)
    dv, mx, ea = float(out[0, 0]), float(out[0, 1]), float(out[0, 2])
    split = {"dv_bulk": float(out[0, 6]), "dv_trim": float(out[0, 7]),
             "fallbacks": int(out[0, 8]), "plans": int(out[0, 9])}
    tot = split["dv_bulk"] + split["dv_trim"]
    split["trim_frac"] = split["dv_trim"] / max(tot, 1e-12)
    return dv, mx, ea, {"aborted": bool(out[0, 4]), "nevals": int(out[0, 5]),
                        "fuel_split": split}
