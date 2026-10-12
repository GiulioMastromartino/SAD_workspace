"""Name-driven layout introspection (ADDENDUM 1: A1 + A2).

Two rules that used to be hard-coded in this package and no longer are:

**A1 -- state layout is derived.**  States are always, in order,
``r(3), v(3), sigma(3), omega(3)`` for the hub, followed by each contributor's
own states in registration order (a reaction wheel contributes ``W_i(3)`` per
wheel).  So 3 wheels -> 15 states named ``rx,ry,rz,vx,vy,vz,s1,s2,s3,omx,omy,omz,
W1,W2,W3`` (identical to ``heyoka_rw.STATE_NAMES`` and to ``native_rw``), one
wheel -> 14, four wheels -> 18, hub only -> 12.  ``system.state_names()`` is the
single source of truth.

**A2 -- parameter slots are assigned dynamically.**  The input-port block comes
first (one contiguous block per contributor declaring an input port, in
registration order; a reaction wheel declares the motor-torque input), then each
contributor's own parameters in registration order.  For the hub + 3RW preset
this reproduces ``native_rw`` / ``heyoka_rw.pack_rw_pars()`` exactly::

    u1,u2,u3, mass, Ixx,Iyy,Izz,Ixy,Ixz,Iyz,
    invIxx,invIyy,invIzz,invIxy,invIxz,invIyz, Js1,Js2,Js3        (19)

Hub-only gives ``mass, I6, invI6`` (13) with no leading input block.  Because
the layout depends on *registration*, downstream code must never hard-code an
index: use :func:`par_index` / :func:`par_slice` / :func:`par_layout` instead.
"""

from __future__ import annotations

from typing import Any, Iterable, Mapping, Sequence

import numpy as np

__all__ = [
    "HUB_STATE_NAMES",
    "UB_DEFAULTS",
    "Layout",
    "system_state_names",
    "system_par_names",
    "state_layout",
    "par_layout",
    "par_names",
    "par_index",
    "par_slice",
    "state_index",
    "state_slice",
    "r_slice",
    "v_slice",
    "sigma_slice",
    "sigma_indices",
    "omega_slice",
    "wheel_slice",
    "pars_for",
]

#: Canonical hub state block (A1): always first, always in this order.
HUB_STATE_NAMES: tuple[str, ...] = (
    "rx", "ry", "rz", "vx", "vy", "vz",
    "s1", "s2", "s3",
    "omx", "omy", "omz",
)

#: Parameter values used wherever the reference and the native build agree.
UB_DEFAULTS: dict[str, float] = {
    "u1": 0.0, "u2": 0.0, "u3": 0.0,
}


def _names_from(obj: Any, getter: str) -> tuple[str, ...]:
    """``obj.<getter>()`` -> tuple of names, or ``obj`` if it already is one."""
    if obj is None:
        return ()
    if isinstance(obj, str):
        return (obj,)
    if isinstance(obj, Mapping):
        return tuple(str(k) for k in obj)
    if isinstance(obj, np.ndarray):
        return tuple(str(v) for v in obj.tolist())
    if isinstance(obj, (list, tuple, set, frozenset)):
        if all(isinstance(v, str) for v in obj):
            return tuple(str(v) for v in obj)
    fn = getattr(obj, getter, None)
    if callable(fn):
        out = fn()
        if out is not None and not isinstance(out, int):
            return tuple(str(v) for v in out)
    raise TypeError(
        f"cannot read {getter!r} from {type(obj).__name__!r}; expected a "
        f"sequence of names or an object exposing {getter}()"
    )


def system_state_names(system: Any) -> tuple[str, ...]:
    """Ordered state names of ``system`` (an ``OmniSystem`` / integrator / list)."""
    return _names_from(system, "state_names")


def system_par_names(system: Any) -> tuple[str, ...]:
    """Ordered parameter names of ``system`` (an ``OmniSystem`` / integrator)."""
    return _names_from(system, "par_names")


class Layout:
    """A read-only name -> index mapping with a couple of conveniences."""

    def __init__(self, names: Sequence[str]) -> None:
        self._names = tuple(str(n) for n in names)
        self._index = {n: i for i, n in enumerate(self._names)}

    @property
    def names(self) -> tuple[str, ...]:
        return self._names

    def __len__(self) -> int:
        return len(self._names)

    def __iter__(self):
        return iter(self._names)

    def __contains__(self, name: object) -> bool:
        return name in self._index

    def __getitem__(self, key):
        if isinstance(key, str):
            return self._require(key)
        if isinstance(key, int):
            return self._names[key]
        if isinstance(key, slice):
            return self._names[key]
        return self.indices(key)

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return f"Layout({list(self._names)})"

    def _require(self, name: str) -> int:
        try:
            return self._index[str(name)]
        except KeyError:
            raise KeyError(
                f"{name!r} is not a known name; available: {list(self._names)}"
            ) from None

    def index(self, name: str) -> int:
        """Flat index of a single name."""
        return self._require(name)

    def indices(self, names: str | Iterable[str]) -> list[int]:
        """Flat indices of one name or several."""
        if isinstance(names, str):
            return [self._require(names)]
        keys = list(names)
        if not keys:
            return []
        idx = [self._require(k) for k in keys]
        if any(b - a != 1 for a, b in zip(idx, idx[1:])):
            raise ValueError(
                f"{list(keys)} is not a contiguous block (indices {idx}); use "
                "indices() instead of slice()"
            )
        return idx

    def slice(self, *names: str | Iterable[str]) -> slice:
        """Contiguous ``slice`` covering one or several names.

        ``layout.slice('Ixx')`` -> ``slice(4, 5)``; ``layout.slice('Ixx','Ixy')``
        and ``layout.slice(['Ixx','Ixy'])`` -> ``slice(4, 6)``.
        """
        if len(names) == 1 and not isinstance(names[0], str):
            keys = list(names[0])
        else:
            keys = list(names)
        if not keys:
            raise ValueError("no names given")
        idx = self.indices(keys)
        return slice(idx[0], idx[-1] + 1)


def state_layout(system: Any) -> Layout:
    """:class:`Layout` over ``system.state_names()``."""
    return Layout(system_state_names(system))


def par_layout(system: Any) -> Layout:
    """:class:`Layout` over ``system.par_names()`` (A2)."""
    return Layout(system_par_names(system))


# ---------------------------------------------------------------------------
# free functions taking either a system or a layout / name sequence
# ---------------------------------------------------------------------------
def _as_layout(names: Any, getter: str) -> Layout:
    if isinstance(names, Layout):
        return names
    return Layout(_names_from(names, getter))


def par_names(system: Any) -> tuple[str, ...]:
    """Ordered parameter names (alias of :func:`system_par_names`)."""
    return system_par_names(system)


def par_index(name: str, system: Any = None, *, names: Any = None) -> int:
    """Index of the parameter called ``name`` -- never hard-code it (A2).

    Pass the ``OmniSystem``/integrator, or an explicit name sequence.
    """
    layout = _as_layout(system if system is not None else names, "par_names")
    return layout.index(name)


def par_slice(names: str | Iterable[str], system: Any = None, *,
              from_names: Any = None) -> slice:
    """Contiguous parameter ``slice`` covering ``names``.

    ``par_slice(['Ixx', 'Ixy'], system)`` -> ``slice(4, 6)``.
    """
    layout = _as_layout(system if system is not None else from_names, "par_names")
    return layout.slice(names)


def state_index(name: str, system: Any = None, *, names: Any = None) -> int:
    """Index of the state called ``name`` (A1)."""
    layout = _as_layout(system if system is not None else names, "state_names")
    return layout.index(name)


def state_slice(names: str | Iterable[str], system: Any = None, *,
                from_names: Any = None) -> slice:
    """Contiguous state ``slice`` covering ``names``."""
    layout = _as_layout(system if system is not None else from_names, "state_names")
    return layout.slice(names)


def sigma_slice(system: Any = None, *, names: Any = None) -> slice:
    """Slice of the attitude (MRP) block, computed from ``state_names()``.

    Falls back to the canonical hub block ``slice(6, 9)`` when the layout is
    unknown -- valid whenever the attitude block is 3 wide, which it always is
    for a 3-parameter MRP set.
    """
    layout = _as_layout(system if system is not None else names, "state_names")
    if not len(layout):
        return slice(6, 9)
    try:
        return layout.slice(["s1", "s2", "s3"])
    except KeyError:
        return slice(6, 9)


def sigma_indices(system: Any = None, *, names: Any = None) -> slice:
    """A7 spelling of :func:`sigma_slice` (``omniheyoka.sigma_indices()``).

    No-argument call returns ``slice(6, 9)``.
    """
    if system is None and names is None:
        return slice(6, 9)
    return sigma_slice(system, names=names)


def r_slice(system: Any = None, *, names: Any = None) -> slice:
    """Slice of the inertial position block."""
    return _hub_block(["rx", "ry", "rz"], system, names)


def v_slice(system: Any = None, *, names: Any = None) -> slice:
    """Slice of the inertial velocity block."""
    return _hub_block(["vx", "vy", "vz"], system, names)


def omega_slice(system: Any = None, *, names: Any = None) -> slice:
    """Slice of the body angular-rate block."""
    return _hub_block(["omx", "omy", "omz"], system, names)


def _hub_block(keys: list[str], system: Any, names: Any) -> slice:
    layout = _as_layout(system if system is not None else names, "state_names")
    if not len(layout):
        base = {"rx": 0, "vx": 3, "omx": 9}[keys[0]]
        return slice(base, base + 3)
    return layout.slice(keys)


def wheel_slice(wheel: int = 1, system: Any = None, *,
                names: Any = None) -> slice:
    """Slice of a wheel's speed states: ``wheel=1`` -> ``W1``, etc."""
    layout = _as_layout(system if system is not None else names, "state_names")
    if not len(layout):
        return slice(12 + wheel - 1, 12 + wheel)
    return layout.slice([f"W{wheel}"])


# ---------------------------------------------------------------------------
# building parameter vectors by name
# ---------------------------------------------------------------------------
def pars_for(system: Any, values: Mapping[str, float] | None = None,
             default: float = 0.0, **kwargs: Any) -> list[float]:
    """Build a parameter vector *by name* (A2: never by index).

    ``pars_for(sys, {'mass': 750.0, 'Ixx': 900.0}, default=0.0)`` or
    ``pars_for(sys, mass=750.0, u1=0.1)``.  Unknown names raise ``KeyError``;
    unnamed slots get ``default``.
    """
    layout = par_layout(system)
    out = [float(default)] * len(layout)
    merged: dict[str, Any] = {}
    if values:
        merged.update(values)
    merged.update(kwargs)
    for key, value in merged.items():
        out[layout.index(key)] = float(value)
    return out
