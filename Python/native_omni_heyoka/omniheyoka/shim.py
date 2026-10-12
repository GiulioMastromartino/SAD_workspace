"""``OmniTaylorAdaptive``: a fully heyoka-compatible integrator shim.

The whole point of ``native_omni_heyoka`` is that the symbolic equations of
motion are *composed at runtime* from effector contributions (Option A:
truncated-Taylor / symbolic scalars are carried through the back-substitution
solve, so the EOM falls out as an expression graph).  heyoka never needs to
evaluate the vector field numerically, so the integrator can be built directly
from that graph.

To be a drop-in replacement for ``heyoka.taylor_adaptive`` this class subclasses
the *real* heyoka class (``heyoka._core.taylor_adaptive_dbl``) rather than
wrapping it.  Consequently

* ``isinstance(ta, taylor_adaptive_dbl)`` is ``True``;
* every property (``state``, ``pars``, ``tc``, ``order``, ``tol``, ...) and every
  method (``propagate_for``, ``propagate_until``, ``step``, ``eval_taylor_map``,
  ...) is inherited verbatim -- nothing is re-implemented, nothing can drift;
* ``state`` / ``pars`` stay writable numpy views;
* custom attributes can be attached (pybind11 instances carry a ``__dict__``),
  which is how ``.system`` is exposed.

Usage::

    from omniheyoka import OmniTaylorAdaptive, rw_demo_system

    ta = OmniTaylorAdaptive(rw_demo_system(), y0, pars=..., tol=1e-12)
    ta.propagate_for(dt)              # exactly like heyoka
    ta.state, ta.pars, ta.time        # exactly like heyoka

Notes / gotchas (verified against heyoka 7.12.1):

* ``tol`` is **read-only** on the C++ class.  It is therefore a
  constructor-only argument here; there is deliberately no setter.
* ``dim`` is the state count.  The C++ class has **no** ``n_states`` /
  ``n_pars`` attribute, so they are added below as real Python properties.
* Some properties legitimately raise ``ValueError`` when not applicable
  (``t_events``, ``nt_events``, ``te_cooldowns``, ``tstate``, ``vargs``,
  ``vorder`` on an event-free / non-variational integrator).  That is normal
  library behaviour and is intentionally not papered over.
"""

from __future__ import annotations

from typing import Any, Iterable, Sequence

import numpy as np
import heyoka as hy
from heyoka._core import taylor_adaptive_dbl

from .layout import par_layout, par_slice

__all__ = ["OmniTaylorAdaptive", "DEFAULT_TOL", "ACCEL_LAYOUT"]

#: Default tolerance, matching ``cubesat_12u_sim.core.config.HEYOKA_TOL``.
DEFAULT_TOL: float = 1e-12

#: Layout of the acceleration vector returned by ``OmniSystem::accelerations``.
#: ``[r_ddot(3), omega_dot(3), wheel_dot(n_wheels)]`` -- note it is *not* the
#: full state derivative: the kinematic blocks (``r_dot``, ``sigma_dot``) are
#: not included, because those are pure kinematics that the symbolic build
#: writes out directly.
ACCEL_LAYOUT = "r_ddot[3], omega_dot[3], wheel_dot[n_wheels]"


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def _floats(x: Any) -> list[float]:
    """Coerce anything array-like into a flat ``list[float]`` (heyoka wants a list)."""
    arr = np.asarray(x, dtype=float)
    if arr.ndim != 1:
        arr = arr.reshape(-1)
    return [float(v) for v in arr]


def _is_symbolic_system(obj: Any) -> bool:
    """True for something that already *is* a heyoka system ``[(var, rhs), ...]``."""
    if not isinstance(obj, (list, tuple)) or not obj:
        return False
    for item in obj:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            return False
        for part in item:
            if not isinstance(part, hy.expression):
                return False
    return True


def _looks_like_omni_system(obj: Any) -> bool:
    """True for an object exposing the ``OmniSystem`` contract."""
    return callable(getattr(obj, "build_symbolic", None))


def _resolve_state_names(system: Any, override: Iterable[str] | None,
                          n_states: int | None) -> tuple[str, ...]:
    """Pick the state-variable names to hand to ``heyoka::make_vars``.

    Order of preference: explicit ``state_names=`` argument, then the system's
    own ``state_names()``, then generic ``x0, x1, ...`` (heyoka rejects names
    such as ``p`` that collide with the ``par`` namespace, so we never invent
    names that could collide).
    """
    if override is not None:
        names = tuple(str(n) for n in override)
    else:
        getter = getattr(system, "state_names", None)
        names = ()
        if callable(getter):
            try:
                raw = getter()
            except Exception:  # pragma: no cover - defensive
                raw = None
            if raw:
                names = tuple(str(n) for n in raw)
        if not names:
            if n_states is None:
                raise ValueError(
                    "could not determine the state count: the system exposes no "
                    "'state_names()' and no default state. Pass state_names=..."
                )
            names = tuple(f"x{i}" for i in range(int(n_states)))
    if len(set(names)) != len(names):
        raise ValueError(f"state names must be unique, got {names}")
    if any(n == "" or n.strip() != n for n in names):
        raise ValueError(f"invalid state name in {names}")
    return names


def _symbolic_system(system: Any, names: Sequence[str]) -> Any:
    """Call ``system.build_symbolic(...)`` tolerantly and validate the result."""
    if _is_symbolic_system(system):
        # Convenience: accept a hand-written heyoka system directly.
        return list(system)

    builder = getattr(system, "build_symbolic", None)
    if not callable(builder):
        raise TypeError(
            "OmniTaylorAdaptive expects an OmniSystem (an object with a "
            "'build_symbolic' method) or a heyoka system, got "
            f"{type(system).__name__!r}"
        )

    # DESIGN.md declares build_symbolic(const std::vector<std::string>&); the
    # WS2 sketch also shows a no-argument call. Support both arities without
    # masking genuine errors from inside the builder.
    try:
        sys_ = builder(list(names))
    except TypeError as exc:
        if "argument" not in str(exc):
            raise
        sys_ = builder()

    if not _is_symbolic_system(sys_):
        sys_ = list(sys_)
    if not sys_:
        raise ValueError("the system's build_symbolic() returned an empty system")
    return sys_


def _raise_unless_usable(system: Any) -> None:
    """Fail early with an actionable message when the native module is missing."""
    if _looks_like_omni_system(system):
        return
    if _is_symbolic_system(system):
        return
    module_name = type(system).__module__
    if module_name.startswith("omniheyoka.") and module_name.endswith("mock"):
        return  # the pure-Python mock backend is explicitly allowed
    from ._native import build_hint  # local import: only needed on the error path

    raise TypeError(
        "OmniTaylorAdaptive needs an OmniSystem built by the compiled "
        f"'omni_heyoka' extension (or a compatible mock). {build_hint()}"
    )


# ---------------------------------------------------------------------------
# the shim
# ---------------------------------------------------------------------------
class OmniTaylorAdaptive(taylor_adaptive_dbl):
    """``heyoka.taylor_adaptive`` subclass whose system comes from an ``OmniSystem``.

    Parameters
    ----------
    system
        A compiled ``omni_heyoka.OmniSystem`` (see
        ``native_omni_heyoka/DESIGN.md``) -- anything exposing
        ``build_symbolic(...)``, ``state_names()``, ``default_state()`` and
        ``default_pars()``.  A plain heyoka ``[(var, rhs), ...]`` system is also
        accepted for convenience.
    state
        Initial state vector.  If ``None``, ``system.default_state()`` is used.
    pars
        Parameter vector.  If ``None``, ``system.default_pars()`` is used.
        Resolved by name (A2) via ``system.par_names()``. Historical note:
        the hand-written reference used a fixed 19-vector (``u[3]``,
        ``mass``, ``I6``, ``invI6``, ``Js``); the composed system derives
        the inverse symbolically (no ``invI6``) and adds per-wheel
        ``tauF{i}`` plus thruster/env blocks, so counts are never hard-coded.
    tol
        Taylor-method tolerance.  Constructor-only: the underlying property is
        read-only.
    state_names
        Override the variable names used for ``heyoka::make_vars``.  Defaults to
        the system's own ``state_names()``.
    **kwargs
        Forwarded *verbatim* to ``taylor_adaptive_dbl`` (``high_accuracy``,
        ``compact_mode``, ``opt_level``, ``pars``-side effects, ...).

    Attributes
    ----------
    system
        The ``OmniSystem`` the integrator was built from (introspection and the
        numeric cross-check).
    """

    def __init__(self, system: Any, state: Any = None, *, pars: Any = None,
                 tol: float = DEFAULT_TOL, state_names: Iterable[str] | None = None,
                 **kwargs: Any):
        _raise_unless_usable(system)

        fp_type = kwargs.pop("fp_type", None)
        if fp_type is not None and fp_type is not float:
            raise TypeError(
                "OmniTaylorAdaptive is hard-bound to taylor_adaptive_dbl (double "
                f"precision) and cannot be built with fp_type={fp_type!r}."
            )

        # --- state / pars defaults come from the system itself -------------
        if state is None:
            default_state = getattr(system, "default_state", None)
            state = default_state() if callable(default_state) else []
        if pars is None:
            default_pars = getattr(system, "default_pars", None)
            pars = default_pars() if callable(default_pars) else []
        state_vec = _floats(state)
        pars_vec = _floats(pars)

        # --- names -> make_vars -> build_symbolic --------------------------
        n_states = len(state_vec) if state_vec else None
        names = _resolve_state_names(system, state_names, n_states)
        if state_vec and len(names) != len(state_vec):
            raise ValueError(
                f"state has {len(state_vec)} components but {len(names)} state "
                f"names were given/derived ({names[:8]}...)"
            )
        sys_ = _symbolic_system(system, names)
        if len(sys_) != len(names):
            raise ValueError(
                f"build_symbolic() returned {len(sys_)} equations for "
                f"{len(names)} state variables"
            )

        # NOTE: pars *must* be handed over as a plain list of floats, and tol
        # must go through the constructor (the property is read-only).
        super().__init__(sys_, state_vec, pars=pars_vec, tol=float(tol), **kwargs)

        # Custom attributes are fine: pybind11 instances carry a __dict__.
        self.system = system
        self._state_names = tuple(names)

    # -- convenience properties (absent from the C++ class) ----------------
    @property
    def n_states(self) -> int:
        """Number of state variables (== ``dim``; the C++ class has no such attr)."""
        return int(self.dim)

    @property
    def n_pars(self) -> int:
        """Number of parameters (== ``len(pars)``)."""
        return len(self.pars)

    @property
    def state_names(self) -> tuple[str, ...]:
        """Ordered state-variable names used for ``heyoka::make_vars``."""
        return getattr(self, "_state_names", ())

    @property
    def n_wheels(self) -> int:
        """Number of reaction wheels, derived via ``par_names()`` (A2).

        Prefers the system's own ``n_wheels``; otherwise counts the ``Js{i}``
        own-parameter slots in the derived layout; otherwise falls back to
        the A1 state count (only wheels add states beyond the 12 hub states).
        """
        getter = getattr(self.system, "n_wheels", None)
        if callable(getter):
            try:
                return int(getter())
            except Exception:  # pragma: no cover - defensive
                pass
        try:
            layout = par_layout(self.system)
            if len(layout):
                js_slots = [nm for nm in layout.names
                            if nm.startswith("Js") and nm[2:].isdigit()]
                if js_slots:
                    return int(len(js_slots))
                _ = par_slice("mass", from_names=layout)
        except Exception:  # pragma: no cover - defensive
            pass
        try:
            return max(int(self.dim) - 12, 0)
        except Exception:  # pragma: no cover - defensive
            return 0

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return (f"OmniTaylorAdaptive(dim={self.dim}, n_pars={len(self.pars)}, "
                f"order={self.order}, tol={self.tol:g}, "
                f"system={type(self.system).__name__})")

    # -- numeric <-> symbolic cross-check -----------------------------------
    def numeric_accelerations(self, state: Any = None,
                              pars: Any = None) -> np.ndarray:
        """Accelerations from the *same contributor code* that built the system.

        Delegates to ``system.accelerations(state, pars)`` (S = ``double``
        instantiation of the templated contributors), which is what the
        validation workstream compares against the symbolic ``c0``.

        Returns a flat ``np.ndarray`` laid out as ``[r_ddot(3), omega_dot(3),
        wheel_dot(n_wheels)]`` -- *not* the full state derivative (the
        kinematic blocks are pure kinematics and are written out directly by
        the symbolic build).  ``state`` defaults to the current integrator
        state, ``pars`` to the current parameters.
        """
        accel = getattr(self.system, "accelerations", None)
        if not callable(accel):
            raise TypeError(
                f"{type(self.system).__name__!r} does not implement "
                "'accelerations(state, pars)'"
            )
        st = self.state if state is None else np.asarray(state, dtype=float).reshape(-1)
        ps = self.pars if pars is None else np.asarray(pars, dtype=float).reshape(-1)
        out = accel([float(v) for v in st], [float(v) for v in ps])
        return np.asarray(out, dtype=float).reshape(-1)

    def _rhs_callable(self) -> Any:
        """Lazily compiled evaluator for the symbolic RHS (cached)."""
        fn = self.__dict__.get("_rhs_cfunc")
        if fn is None:
            if not _is_symbolic_system(self.sys):
                raise TypeError("the built system is not a heyoka [(var, rhs)] system")
            fn = hy.cfunc([rhs for _, rhs in self.sys],
                          [var for var, _ in self.sys])
            self._rhs_cfunc = fn
        return fn

    def symbolic_rhs(self, state: Any = None, pars: Any = None) -> np.ndarray:
        """Full state derivative of the *symbolic* system at ``(state, pars)``.

        Cheap and always well defined: the RHS expressions are compiled once
        with :func:`heyoka.cfunc` and then evaluated.  Use it to cross-check
        against :meth:`numeric_accelerations`; the acceleration blocks live
        at A1-derived positions (``r_ddot`` after ``r_dot``, ``omega_dot``
        after the attitude block, wheel blocks thereafter -- resolve via
        ``omniheyoka.layout`` helpers such as ``state_layout`` /
        ``sigma_slice`` instead of hard-coding offsets).

        Note that ``ta.tc[:, 1]`` cannot be used for this: heyoka 7.12.1 zeroes
        the ``tc`` array once a step has consumed it, so it reads as all zeros.
        """
        st = self.state if state is None else np.asarray(state, dtype=float).reshape(-1)
        ps = self.pars if pars is None else np.asarray(pars, dtype=float).reshape(-1)
        fn = self._rhs_callable()
        out = fn(inputs=np.asarray(st, dtype=float), pars=np.asarray(ps, dtype=float))
        return np.asarray(out, dtype=float).reshape(-1)
