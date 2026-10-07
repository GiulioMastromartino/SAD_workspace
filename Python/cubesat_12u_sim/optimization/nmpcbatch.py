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
    from ..cxxdrive import _nmpcbatch

    available = True
except Exception:
    _nmpcbatch = None  # type: ignore[assignment]
    available = False


def make_funcs(x0: np.ndarray, cfg):
    """Return ``(cost, jac, con_vals, con_jac)`` closures over C++ batch calls.

    Returns ``(None, None, None, None)`` when the extension is unavailable.
    Callers must handle the fallback (see ``nmpc.ScipyShootingBackend``).
    """
    from .. import config as C

    if not available:
        return None, None, None, None
    N, dt = int(cfg.horizon), float(cfg.dt_outer)
    x0a = np.ascontiguousarray(np.asarray(x0, float).ravel())
    # norm_penalty=0: the scipy backend enforces |u|<=a_max with a hard
    # NonlinearConstraint (same as the Python cost, which has no penalty term).
    # tol: Heyoka Taylor tolerance for the prediction rollout.
    args = (dt, N, float(cfg.q_pos), float(cfg.q_vel), float(cfg.r_fuel),
            float(cfg.db_a), float(cfg.a_max), 0.0,
            float(C.MU_EARTH), float(C.R_ORBIT), 1e-12)

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
