"""Caller-side stepping helpers (ADDENDUM 1: A3 + A7).

Two things deliberately live **outside** the equation of motion, in the caller's
stepping loop, exactly as ``native_rw`` and ``heyoka_rw.run_rw_mirror`` do:

**A3 -- motor-torque saturation is the caller's job.**  ``native_rw`` clamps
``u[i] = clamp(v, +-uMax)`` *before* writing it into the parameter block, and the
symbolic graph uses ``par[0..2]`` verbatim.  heyoka's Taylor integrator is
degraded by ``min``/``max``/``if`` in the expression graph, so nothing here may
embed a clamp in the system.  Use :func:`saturate_u` and write the result into
the parameter block.

**A7 -- the MRP shadow-set reset is the caller's job too.**  heyoka's propagator
has no per-step callback, so the switch ``sigma -> -sigma/|sigma|^2`` when
``|sigma|^2 > 1`` has to be applied after every step.  Use :func:`reset_mrp`,
which returns the switch delta so ``mrpSwitches`` can be accumulated.

:func:`step_loop` ties both together for the common "fixed-step control loop"
shape of ``run_rw_mirror``.
"""

from __future__ import annotations

from typing import Any, Callable, Iterable, Sequence

import numpy as np

from .layout import par_layout, sigma_slice, state_layout

__all__ = [
    "saturate_u",
    "u_max_of",
    "reset_mrp",
    "step_loop",
]

#: Default motor-torque saturation [Nm] if neither the system nor the caller
#: provides one: ``heyoka_rw.RW_U_MAX`` (HR16).
DEFAULT_U_MAX: float = 0.2

#: MRP shadow-set threshold: switch when |sigma|^2 exceeds 1.
MRP_SWITCH_SQ: float = 1.0


def saturate_u(u: Sequence[float] | float, u_max: float = DEFAULT_U_MAX) -> np.ndarray:
    """Clamp a motor-torque command to ``+-u_max`` (A3).

    Mirrors ``rwMotorTorque`` / ``rw_allocate``: the wheel torque command is
    saturated *before* it enters the parameter block; the ODE itself uses
    ``par[0..2]`` verbatim.  Returns a fresh ``(3,)`` float array.
    """
    v = np.asarray(u, dtype=float).reshape(-1)
    if v.size != 3:
        raise ValueError(f"u must be a 3-vector, got {v.size} component(s)")
    if not np.isfinite(u_max) or u_max < 0.0:
        raise ValueError(f"u_max must be a non-negative finite float, got {u_max!r}")
    return np.clip(v, -float(u_max), float(u_max))


def u_max_of(system: Any, default: float | None = None) -> float:
    """Read ``u_max`` from the system (A3), with a fallback.

    ``OmniSystem`` exposes ``u_max()``; if it does not (mock/older builds) fall
    back to ``default`` or the module :data:`DEFAULT_U_MAX`.
    """
    getter = getattr(system, "u_max", None)
    if callable(getter):
        try:
            value = float(getter())
        except Exception:  # pragma: no cover - defensive
            value = float("nan")
        if np.isfinite(value) and value >= 0.0:
            return value
    return float(DEFAULT_U_MAX if default is None else default)


def reset_mrp(state: Any, inplace: bool = True, sigma: Any = None,
              state_names: Any = None) -> tuple[Any, int]:
    """MRP shadow-set reset: ``sigma -> -sigma/|sigma|^2`` when ``|sigma|^2 > 1``.

    Returns ``(state, switched)`` where ``switched`` is ``1`` when a switch was
    applied and ``0`` otherwise, so a stepping loop can accumulate
    ``mrpSwitches`` the way ``native_rw`` does.  heyoka has no per-step
    callback, so this must be called by the caller after every propagation step.

    ``state`` may be an ``OmniTaylorAdaptive``'s ``.state`` view (in which case
    ``inplace=True`` writes straight into the integrator), a numpy array or any
    sequence.  ``sigma`` overrides the attitude block (default
    :func:`omniheyoka.sigma_indices`).
    """
    if sigma is None:
        sig = sigma_slice(state_names)
    elif isinstance(sigma, slice):
        sig = sigma
    else:
        idx = list(sigma)
        if not idx:
            raise ValueError("sigma index list is empty")
        sig = slice(int(idx[0]), int(idx[-1]) + 1)

    st = state
    if not isinstance(st, np.ndarray):
        # Sequences/lists cannot be slice-assigned; a fresh array carries the
        # change (an integrator's .state is already a writable numpy view, so
        # that case stays truly in-place).
        st = np.array(st, dtype=float, copy=not inplace)
    block = np.asarray(st, dtype=float)[sig]
    s2 = float(block @ block)
    if s2 > MRP_SWITCH_SQ:
        st[sig] = -block / s2
        return st, 1
    return st, 0


def step_loop(ta: Any, dt: float, n_steps: int = 1,
              torque_fn: Callable[[float, np.ndarray], Sequence[float]] | None = None,
              u_max: float | None = None, record: bool = True,
              mrp_reset: bool = True, state_names: Any = None) -> dict:
    """Fixed-step propagation loop with the caller-side A3 clamp and A7 reset.

    Parameters
    ----------
    ta
        An :class:`~omniheyoka.OmniTaylorAdaptive` (or anything heyoka-shaped).
    dt, n_steps
        Propagation grid.
    torque_fn
        ``f(t, state) -> raw 3-vector motor-torque command``.  Called *before*
        each step with the current time and state; the result is saturated by
        :func:`saturate_u` and written into the input block located via
        ``par_names()``.  ``None`` leaves the parameters untouched.
    u_max
        Saturation bound; defaults to ``ta.system.u_max()``.
    mrp_reset
        Apply :func:`reset_mrp` after every step and count switches.

    Returns
    -------
    dict
        ``{"t", "Y", "u", "mrp_switches"}`` -- ``Y`` is ``(dim, n+1)`` when
        ``record`` is true, else ``None``.
    """
    if n_steps < 0:
        raise ValueError("n_steps must be >= 0")
    dt = float(dt)

    names = state_names if state_names is not None else getattr(ta, "state_names", ())
    sig = sigma_slice(names) if len(names) else sigma_slice()
    pl = par_layout(getattr(ta, "system", None))
    u_block = None
    if len(pl) and "u1" in pl:
        u_block = pl.slice(["u1", "u2", "u3"])
    um = u_max_of(getattr(ta, "system", None)) if u_max is None else float(u_max)

    dim = int(ta.dim)
    t = np.zeros(n_steps + 1) if record else None
    Y = np.zeros((dim, n_steps + 1)) if record else None
    U = np.zeros((3, n_steps + 1)) if record else None
    switches = 0
    if record:
        t[0] = float(ta.time)
        Y[:, 0] = np.asarray(ta.state, dtype=float)
        U[:, 0] = np.nan

    for k in range(1, n_steps + 1):
        current_state = np.asarray(ta.state, dtype=float)
        if torque_fn is not None:
            raw = torque_fn(float(ta.time), current_state)
            u = saturate_u(raw, um)
            if u_block is not None:
                ta.pars[u_block] = u
        elif record:
            u = np.zeros(3)
        ta.propagate_for(dt)
        if mrp_reset:
            _, switched = reset_mrp(ta.state, inplace=True, sigma=sig)
            switches += int(switched)
        if record:
            t[k] = float(ta.time)
            Y[:, k] = np.asarray(ta.state, dtype=float)
            U[:, k] = u

    return {"t": t, "Y": Y, "u": U, "mrp_switches": switches}
