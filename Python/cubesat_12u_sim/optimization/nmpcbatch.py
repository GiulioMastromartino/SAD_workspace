"""C++ batch cost/gradient for the NMPC shooting solver (optional accelerator).

Mirrors the ``cost`` closure in ``nmpc.ScipyShootingBackend.solve`` exactly
(gated by parity test). ``cost_batch`` evaluates B candidates in one call;
``grad_batch`` returns a forward-difference gradient in one call, which is
handed to SLSQP as ``jac=`` so each iteration costs 2 C++ calls instead of
~20 Python round-trips.
"""
from __future__ import annotations

import numpy as np

try:
    import heyoka  # noqa: F401  (loads libheyoka: _nmpcbatch links against
    # it with dynamic_lookup, same as _cxxdrive — bare import fails)
    from cubesat_12u_sim.native.cxxdrive import _nmpcbatch

    available = True
except Exception:
    _nmpcbatch = None  # type: ignore[assignment]
    available = False


def make_funcs(x0: np.ndarray, cfg):
    """Return ``(cost, jac, con_vals, con_jac)`` closures over C++ batch calls.

    Returns ``(None, None, None, None)`` when the extension is unavailable.
    Callers must handle the fallback (see ``nmpc.ScipyShootingBackend``).
    """
    from cubesat_12u_sim.control import adjoint
    from cubesat_12u_sim.core import config as C

    if not available:
        return None, None, None, None
    N, dt = int(cfg.horizon), float(cfg.dt_outer)
    x0a = np.ascontiguousarray(np.asarray(x0, float).ravel())
    # norm_penalty=0: the scipy backend enforces |u|<=a_max with a hard
    # NonlinearConstraint (same as the Python cost, which has no penalty term).
    # tol: Heyoka Taylor tolerance for the prediction rollout.
    #
    # The q_pos/q_vel/r_fuel slots are kept positionally but unused: the kernel's
    # cost now mirrors objective.run_scores (L1 fuel + box hinge), and those
    # three weights no longer exist in the objective. See nmpc_batch.cpp.
    fA, fx0, fg1 = adjoint.fuel_params(cfg.a_max, dt)
    fA *= float(getattr(cfg, 'fuel_scale', 1.0))
    args = (dt, N, float(cfg.w_track), float(cfg.w_track_frac),
            float(cfg.w_final), float(cfg.db_a), float(cfg.a_max), 0.0,
            float(cfg.box_ref), float(C.MU_EARTH), float(C.R_ORBIT), 1e-12,
            fA, fx0, fg1)

    def cost(z: np.ndarray) -> float:
        Z = np.ascontiguousarray(np.asarray(z, float).ravel()[None, :])
        X = np.ascontiguousarray(x0a[None, :])
        out = np.zeros(1)
        _nmpcbatch.cost_batch(X, Z, *args, out)
        return float(out[0])

    def jac(z: np.ndarray) -> np.ndarray:
        g = np.zeros(3 * N)
        _nmpcbatch.grad_batch(x0a, np.ascontiguousarray(
            np.asarray(z, float).ravel()), *args, g)
        return g

    def con_vals(Z: np.ndarray) -> np.ndarray:
        Z = np.ascontiguousarray(np.asarray(Z, float).reshape(1, 3 * N))
        vals = np.zeros((1, N))
        jac = np.zeros((1, N, 3 * N))
        _nmpcbatch.normviol_batch(Z, N, float(cfg.a_max), vals, jac)
        return vals.ravel()

    def con_jac(Z: np.ndarray) -> np.ndarray:
        Z = np.ascontiguousarray(np.asarray(Z, float).reshape(1, 3 * N))
        vals = np.zeros((1, N))
        jac = np.zeros((1, N, 3 * N))
        _nmpcbatch.normviol_batch(Z, N, float(cfg.a_max), vals, jac)
        return jac[0]

    return cost, jac, con_vals, con_jac
