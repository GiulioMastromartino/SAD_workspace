"""Ready-made ``OmniSystem`` configurations and A/B comparison helpers.

Two families of builders:

* **native** (default) -- drive a compiled ``omni_heyoka.OmniSystem`` and hand
  it to :class:`~omniheyoka.shim.OmniTaylorAdaptive`.  This is the runtime-
  composed system the package exists for.
* **ground truth** -- :func:`rw_demo_heyoka` (and the ``*_state`` / ``*_pars``
  helpers) reuse the independent hand-written reference in
  ``cubesat_12u_sim.backends.heyoka.heyoka_rw`` so any integrator built here can
  be A/B'd against it numerically.

All numbers are taken from ``native_rw`` defaults / ``heyoka_rw.py`` /
``cubesat_12u_sim.core.config``; nothing is invented.

Layout (A2: name-derived, see ``omniheyoka.layout`` and
``native_omni_heyoka/DESIGN.md``)::

    state = [r(3), v(3), sigma(3), omega(3), W(1) per wheel]
    pars  = [u1, tauF1, u2, tauF2, ... (2 per wheel), mass, I6(6), Js per wheel]

Historical note: the hand-written ``heyoka_rw`` reference used a fixed
19-vector ``[u(3), mass, I6, invI6, Js(3)]`` (with precomputed ``invI6``).
The runtime-composed system derives the inverse symbolically (no ``invI6``
slots) and adds the WP2 ``tauF{i}`` second input slot per wheel, so counts
are derived via ``par_names()`` -- never hard-coded.

The MRP shadow-set reset (``sigma -> -sigma/|sigma|^2`` when ``|sigma|^2 > 1``)
is *not* part of the ODE: the caller applies it, exactly as ``native_rw`` and
``heyoka_rw.run_rw_mirror`` do.
"""

from __future__ import annotations

from typing import Any, Iterable, Sequence

import numpy as np
import heyoka as hy

from cubesat_12u_sim.backends.heyoka import heyoka_rw as hrw
from cubesat_12u_sim.core import config as C

from .layout import par_index, par_layout

__all__ = [
    "RW_MASS", "RW_INERTIA", "RW_WHEEL_JS", "RW_U_MAX", "RW_MU",
    "RW_DEMO_WHEELS", "RW_STATE_NAMES", "RW_N_PARS",
    "CUBESAT_12U_MASS", "CUBESAT_12U_INERTIA",
    "build_system", "rw_demo_system", "cubesat_12u_system",
    "rw_demo_heyoka", "make_rw_integrator", "rw_demo_pars", "rw_demo_state",
    "available_backends", "wheel_specs",
]

# ---------------------------------------------------------------------------
# constants (single source of truth for the demo configuration)
# ---------------------------------------------------------------------------
#: Hub mass of the ``native_rw`` / scenarioAttitudeFeedbackRW baseline [kg].
RW_MASS: float = 750.0
#: Hub inertia about the centre of mass [kg m^2].
RW_INERTIA: np.ndarray = np.diag([900.0, 800.0, 600.0])
#: One HR16 rotor inertia [kg m^2] (== ``heyoka_rw.RW_JS``).
RW_WHEEL_JS: float = hrw.RW_JS
#: HR16 maximum motor torque [Nm] (== ``heyoka_rw.RW_U_MAX``).
RW_U_MAX: float = hrw.RW_U_MAX
#: Earth GM for the demo [m^3/s^2].
RW_MU: float = float(C.MU_EARTH)
#: Ordered state-variable names of the 15-state hub + 3RW system.
RW_STATE_NAMES: tuple[str, ...] = tuple(hrw.STATE_NAMES)
#: ``2**0.5``-free definition: the three orthogonal spin axes.
_BODY_AXES = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
#: Default wheel set: one HR16 on each body axis.
RW_DEMO_WHEELS: tuple[tuple[tuple[float, float, float], float], ...] = (
    (_BODY_AXES[0], RW_WHEEL_JS),
    (_BODY_AXES[1], RW_WHEEL_JS),
    (_BODY_AXES[2], RW_WHEEL_JS),
)
def _native_rw_par_names(n_wheels: int = 3) -> tuple[str, ...]:
    """Canonical A2 par names for hub + N wheels (registration order).

    Inputs first (interleaved ``u{i}``/``tauF{i}`` per wheel, WP2), then hub
    ``mass`` + ``I6``, then ``Js{i}``. No ``invI6`` (symbolic inverse).
    """
    n = int(n_wheels)
    names: list[str] = []
    for i in range(1, n + 1):
        names.extend([f"u{i}", f"tauF{i}"])
    names.extend(["mass", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"])
    for i in range(1, n + 1):
        names.append(f"Js{i}")
    return tuple(names)


#: Number of parameters for the hub + 3RW system, derived (A2) via
#: :func:`omniheyoka.layout.par_names` order (historical fixed value 19
#: covered the reference ``[u(3), mass, I6, invI6, Js(3)]`` layout).
RW_N_PARS: int = len(_native_rw_par_names(3))

# 12U CubeSat bus (``cubesat_12u_sim.core.config``).
CUBESAT_12U_MASS: float = float(C.MASS)
CUBESAT_12U_INERTIA: np.ndarray = np.asarray(C.INERTIA, dtype=float).copy()


# ---------------------------------------------------------------------------
# wheel / inertia helpers
# ---------------------------------------------------------------------------
def wheel_specs(wheels: Iterable[Any] | None) -> list[tuple[np.ndarray, float, float]]:
    """Normalise a wheel specification into ``[(gs, Js, Jt), ...]``.

    Each entry may be ``(gs, Js)`` or ``(gs, Js, Jt)``; ``Jt`` (transverse
    rotor inertia) defaults to ``Js`` -- the balanced-wheel model used by the
    HR16 reference.  ``gs`` does not need to be normalised here (the native side
    takes the spin axis as given); it is stored as a 3-vector.
    """
    if wheels is None:
        wheels = RW_DEMO_WHEELS
    out: list[tuple[np.ndarray, float, float]] = []
    for spec in wheels:
        gs, js, *rest = list(spec)
        gs_v = np.asarray(gs, dtype=float).reshape(-1)
        if gs_v.shape != (3,):
            raise ValueError(f"wheel spin axis must have 3 components, got {gs_v.tolist()}")
        jt = float(rest[0]) if rest else float(js)  # balanced rotor: Jt = Js
        out.append((gs_v, float(js), float(jt)))
    return out


def _inertia_9(inertia: np.ndarray | Sequence[float] | None) -> np.ndarray:
    I = RW_INERTIA if inertia is None else np.asarray(inertia, dtype=float)
    return I.reshape(3, 3)


# ---------------------------------------------------------------------------
# SWIG argument-shape tolerance
# ---------------------------------------------------------------------------
# DESIGN.md declares add_hub(mass, std::array<double,9>, std::array<double,3>)
# and add_reaction_wheel(std::array<double,3>, double, double).  How a SWIG
# %include <std_array.i> maps std::array to Python is a WS1 decision we cannot
# see from here, so array-valued arguments are offered in a couple of shapes
# and the one that works is remembered.
_SHAPE_CACHE: dict[str, int] = {}


def _shapes(value: np.ndarray) -> list[Any]:
    """Candidate Python representations of a numeric array, flat first."""
    flat = [float(v) for v in value.reshape(-1)]
    nested = [[float(v) for v in row] for row in np.asarray(value).reshape(-1, 3)]
    return [
        flat,
        np.asarray(value, dtype=float).reshape(-1),
        tuple(flat),
        np.asarray(value, dtype=float).reshape(3, 3) if value.size == 9 else flat,
        nested if value.size == 9 else flat,
    ]


def _invoke(obj: Any, method: str, *values: np.ndarray) -> Any:
    """``obj.method(*values)`` tolerating the SWIG array representation."""
    key = f"{type(obj).__name__}.{method}"
    preferred = _SHAPE_CACHE.get(key)
    candidates: list[list[Any]] = []
    for value in values:
        reps = _shapes(np.asarray(value, dtype=float))
        if preferred is not None:
            reps = [reps[preferred]] + [r for i, r in enumerate(reps) if i != preferred]
        candidates.append(reps)
    n = len(candidates)
    widths = [len(reps) for reps in candidates]
    # try the flat form of every argument first, then the others
    for combo in range(max(widths)):
        args = [candidates[i][min(combo, widths[i] - 1)] for i in range(n)]
        try:
            result = getattr(obj, method)(*args)
        except TypeError:
            continue
        _SHAPE_CACHE[key] = min(combo, widths[0] - 1)
        return result
    raise TypeError(
        f"could not call {method}() on {type(obj).__name__}: no accepted "
        f"argument shape worked for {[v.tolist() for v in values]}. This is a "
        "WS1/SWIG std::array typemap mismatch."
    )


# ---------------------------------------------------------------------------
# native system builders
# ---------------------------------------------------------------------------
def _new_native_system() -> Any:
    from ._native import require_omni_system_class

    return require_omni_system_class()()


def build_system(hub_mass: float = RW_MASS,
                 inertia: np.ndarray | None = None,
                 wheels: Iterable[Any] | None = None,
                 mu: float | None = RW_MU,
                 u_max: float = RW_U_MAX,
                 r_BcB_B: Sequence[float] = (0.0, 0.0, 0.0),
                 backend: str = "native") -> Any:
    """Build an :class:`OmniSystem` from parts.

    Parameters
    ----------
    hub_mass, inertia
        Hub mass [kg] and 3x3 (or 9-element) inertia about its centre of mass.
    r_BcB_B
        Centre-of-mass offset in body frame. Must be zero: the modelled case
        (anything else raises).
    wheels
        Iterable of ``(gs, Js)`` or ``(gs, Js, Jt)``; default three HR16 wheels
        on the body axes. Pass ``[]`` for a hub-only system.
    mu
        Central-body GM [m^3/s^2]; ``None`` disables the gravity contributor.
    u_max
        Motor-torque saturation [Nm].
    backend
        ``"native"`` (default) requires the compiled extension;
        ``"mock"`` uses the pure-Python delegating backend;
        ``"auto"`` prefers native and falls back to the mock.
    """
    if backend not in ("native", "mock", "auto"):
        raise ValueError(f"backend must be 'native', 'mock' or 'auto', got {backend!r}")

    if backend == "native":
        system = _new_native_system()
    else:
        from .mock import MockOmniSystem

        if backend == "auto":
            try:
                system = _new_native_system()
            except ImportError:
                system = MockOmniSystem()
        else:
            system = MockOmniSystem()

    I = _inertia_9(inertia)
    _invoke(system, "add_hub", np.asarray([hub_mass], dtype=float), I.reshape(-1),
            np.asarray(r_BcB_B, dtype=float))
    for gs, js, jt in wheel_specs(wheels):
        _invoke(system, "add_reaction_wheel", gs, np.asarray([js], dtype=float),
                np.asarray([jt], dtype=float))
    if mu is not None:
        system.set_gravity(float(mu))
    if u_max:
        system.set_u_max(float(u_max))
    return system


def rw_demo_system(hub_mass: float | None = None,
                   inertia: np.ndarray | None = None,
                   wheels: Iterable[Any] | None = None,
                   mu: float | None = RW_MU,
                   u_max: float | None = None,
                   backend: str = "native") -> Any:
    """The scenarioAttitudeFeedbackRW baseline: 750 kg hub, diag(900,800,600),
    three HR16 wheels (``Js = RW_JS``, ``uMax = 0.2``) on the body axes, Earth
    point-mass gravity.

    Matches ``native_rw`` defaults and ``heyoka_rw.RW_JS`` / ``RW_U_MAX``, so
    integrators built from it are directly comparable with
    :func:`rw_demo_heyoka`.
    """
    return build_system(
        hub_mass=RW_MASS if hub_mass is None else hub_mass,
        inertia=RW_INERTIA if inertia is None else inertia,
        wheels=RW_DEMO_WHEELS if wheels is None else wheels,
        mu=RW_MU if mu is None else mu,
        u_max=RW_U_MAX if u_max is None else u_max,
        backend=backend,
    )


def cubesat_12u_system(hub_mass: float | None = None,
                       inertia: np.ndarray | None = None,
                       mu: float | None = RW_MU,
                       u_max: float = 0.0,
                       backend: str = "native") -> Any:
    """Hub-only 12U CubeSat bus (``config.MASS``, ``config.INERTIA``), no
    wheels -- the general 12-state ``[r, v, sigma, omega]`` preset."""
    return build_system(
        hub_mass=CUBESAT_12U_MASS if hub_mass is None else hub_mass,
        inertia=CUBESAT_12U_INERTIA if inertia is None else inertia,
        wheels=(),
        mu=RW_MU if mu is None else mu,
        u_max=u_max,
        backend=backend,
    )


def available_backends() -> dict[str, bool]:
    """Which system backends can be constructed right now."""
    out: dict[str, bool] = {"native": False, "mock": True}
    try:
        _new_native_system()
        out["native"] = True
    except ImportError:
        pass
    return out


# ---------------------------------------------------------------------------
# state / parameter vectors
# ---------------------------------------------------------------------------
def rw_demo_state(r: Sequence[float] | None = None,
                  v: Sequence[float] | None = None,
                  sigma: Sequence[float] | None = None,
                  omega: Sequence[float] | None = None,
                  wheel_speeds: Sequence[float] | None = None) -> np.ndarray:
    """15-state vector for the RW demo.

    Defaults to a circular 12000 km equatorial orbit (``config.R0_ECI`` /
    ``V0_ECI``), zero MRP, ``config.OMEGA0_BODY`` body rates and the demo's
    ``RW_OMEGA0_RPM`` wheel speeds (100/200/300 rpm, converted to rad/s).
    """
    r_v = C.R0_ECI if r is None else np.asarray(r, dtype=float)
    v_v = C.V0_ECI if v is None else np.asarray(v, dtype=float)
    s_v = np.zeros(3) if sigma is None else np.asarray(sigma, dtype=float)
    w_v = C.OMEGA0_BODY if omega is None else np.asarray(omega, dtype=float)
    ws_v = (np.radians(hrw.RW_OMEGA0_RPM) if wheel_speeds is None
            else np.asarray(wheel_speeds, dtype=float))
    return hrw.pack_rw_state(r_v, v_v, s_v, w_v, ws_v)


def rw_demo_pars(u_motor: Sequence[float] | None = None,
                 mass: float = RW_MASS,
                 inertia: np.ndarray | None = None,
                 js: Sequence[float] | None = None,
                 tauF: Sequence[float] | None = None,
                 system: Any = None,
                 par_names: Sequence[str] | None = None) -> list[float]:
    """Parameter vector for the RW demo, resolved BY NAME (A2).

    Historical note: the hand-written ``heyoka_rw`` reference uses a fixed
    19-vector ``[u(3), mass, I6, invI6, Js(3)]`` via
    ``heyoka_rw.pack_rw_pars``; the runtime-composed system uses the derived
    layout ``[u1, tauF1, ... , mass, I6, Js...]`` (16 for 3 wheels, no
    ``invI6``). Defaults (masses, inertias, ``Js``, ``uMax``) are unchanged;
    only indexing is name-based. When ``system`` exposes ``par_names()`` it
    is the source of truth (extra slots default to 0.0); otherwise the
    canonical hub + N-wheel names are used.
    """
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import inertia_pars

    u_v = np.zeros(3) if u_motor is None else np.asarray(u_motor, dtype=float).reshape(-1)
    I = RW_INERTIA if inertia is None else np.asarray(inertia, dtype=float)
    I6, _invI6 = inertia_pars(I)
    js_v = np.full(3, RW_WHEEL_JS) if js is None else np.asarray(js, dtype=float).reshape(-1)
    t_v = np.zeros_like(js_v, dtype=float) if tauF is None else np.asarray(tauF, dtype=float).reshape(-1)
    if u_v.size != js_v.size or t_v.size != js_v.size:
        raise ValueError(f"u/tauF/js length mismatch: {u_v.size}/{t_v.size}/{js_v.size}")
    values: dict[str, float] = {"mass": float(mass)}
    for key, val in zip(("Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"), I6):
        values[key] = float(val)
    for i, (uu, tt, jj) in enumerate(zip(u_v, t_v, js_v), start=1):
        values[f"u{i}"] = float(uu)
        values[f"tauF{i}"] = float(tt)
        values[f"Js{i}"] = float(jj)
    if system is not None:
        layout = par_layout(system)
        out = [0.0] * len(layout)
        for key, val in values.items():
            if key in layout:
                out[layout.index(key)] = float(val)
        return out
    names: tuple[str, ...] = tuple(par_names) if par_names is not None else _native_rw_par_names(len(js_v))
    out2 = [0.0] * len(names)
    for key, val in values.items():
        try:
            out2[par_index(key, names=names)] = float(val)
        except KeyError:
            continue
    return out2


# ---------------------------------------------------------------------------
# integrators
# ---------------------------------------------------------------------------
def make_rw_integrator(y0: Sequence[float],
                       u_motor: Sequence[float] | None = None,
                       mass: float = RW_MASS,
                       inertia: np.ndarray | None = None,
                       js: Sequence[float] | None = None,
                       tol: float = float(C.HEYOKA_TOL),
                       high_accuracy: bool = bool(C.HEYOKA_HIGH_ACCURACY),
                       system: Any = None,
                       tauF: Sequence[float] | None = None,
                       **kwargs: Any) -> Any:
    """:class:`OmniTaylorAdaptive` for the RW demo -- the counterpart of
    ``heyoka_rw.make_rw_integrator``.  ``system`` lets you reuse an existing
    ``OmniSystem`` instead of building a fresh one. Pars are resolved by
    name (A2) against ``system.par_names()``."""
    from .shim import OmniTaylorAdaptive

    if system is None:
        system = rw_demo_system()
    return OmniTaylorAdaptive(
        system, np.asarray(y0, dtype=float).reshape(-1),
        pars=rw_demo_pars(u_motor, mass, inertia, js, tauF=tauF, system=system),
        tol=float(tol), high_accuracy=bool(high_accuracy), **kwargs,
    )


def rw_demo_heyoka(y0: Sequence[float],
                   u_motor: Sequence[float] | None = None,
                   mass: float = RW_MASS,
                   inertia: np.ndarray | None = None,
                   js: Sequence[float] | None = None,
                   tol: float = float(C.HEYOKA_TOL),
                   high_accuracy: bool = bool(C.HEYOKA_HIGH_ACCURACY),
                   **kwargs: Any) -> hy.taylor_adaptive:
    """Ground truth: the **hand-written** heyoka integrator for the same
    dynamics, built from ``heyoka_rw.build_rw_system()``.

    Compare it against :func:`make_rw_integrator` / ``OmniTaylorAdaptive`` for
    A/B parity -- this is the reference the validation workstream needs.
    """
    u = np.zeros(3) if u_motor is None else np.asarray(u_motor, dtype=float)
    I = RW_INERTIA if inertia is None else np.asarray(inertia, dtype=float)
    # Reference layout only: the hand-written system fixes 19 pars
    # [u(3), mass, I6, invI6, Js(3)]; native pars go through rw_demo_pars.
    ref_pars = hrw.pack_rw_pars(u, mass, I, js)
    return hy.taylor_adaptive(
        hrw.build_rw_system(), [float(v) for v in np.asarray(y0, dtype=float).reshape(-1)],
        pars=ref_pars,
        tol=float(tol), high_accuracy=bool(high_accuracy), **kwargs,
    )
