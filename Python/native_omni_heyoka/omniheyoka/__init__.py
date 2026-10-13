"""omniheyoka -- runtime-composed symbolic dynamics for heyoka.

The EOM are *composed at runtime* from effector contributions (hub, reaction
wheels, gravity) by the C++ ``OmniSystem`` (Option A: truncated-Taylor scalars
are carried through the back-substitution solve, so the equations of motion
fall out as an expression graph).  heyoka never evaluates the vector field
numerically, so the graph can be handed straight to ``taylor_adaptive``.

:class:`OmniTaylorAdaptive` subclasses the real heyoka class, so it is
indistinguishable from ``heyoka.taylor_adaptive`` downstream::

    from omniheyoka import OmniTaylorAdaptive, rw_demo_system, rw_demo_heyoka

    ta = OmniTaylorAdaptive(rw_demo_system(), y0, pars=..., tol=1e-12)
    ta.propagate_for(dt)
    ta.state, ta.pars, ta.time

The import of the compiled ``omni_heyoka`` extension is deliberately *lazy*:
``import omniheyoka`` works before WS1's module is built, so the shim can be
developed and tested against the pure-Python backend in :mod:`omniheyoka.mock`.
Constructing a *native* system before the build raises a clear ``ImportError``.
"""

from __future__ import annotations

from ._native import build_hint, load_native, native_available, require_native
from .shim import ACCEL_LAYOUT, DEFAULT_TOL, OmniTaylorAdaptive
from .driver import reset_mrp  # A7: caller-side MRP shadow-set reset (imports .layout + numpy only — no cycle)
from .layout import sigma_indices  # A7: attitude-block slice (stdlib + numpy only — no cycle)

__all__ = [
    "OmniTaylorAdaptive",
    "DEFAULT_TOL",
    "ACCEL_LAYOUT",
    "reset_mrp",
    "sigma_indices",
    "build_system",
    "rw_demo_system",
    "cubesat_12u_system",
    "rw_demo_heyoka",
    "rw_demo_state",
    "rw_demo_pars",
    "make_rw_integrator",
    "wheel_specs",
    "available_backends",
    "MockOmniSystem",
    "mock_rw_demo_system",
    "native_available",
    "load_native",
    "require_native",
    "build_hint",
    "__version__",
]

#: Version of the Python-side package (WS2).  The C++ extension carries its
#: own version; ``native_available()`` tells you whether it is loaded.
__version__ = "0.1.0"

try:  # the presets need the sibling cubesat_12u_sim reference package
    from .presets import (
        CUBESAT_12U_INERTIA,
        CUBESAT_12U_MASS,
        RW_INERTIA,
        RW_MASS,
        RW_MU,
        RW_N_PARS,
        RW_STATE_NAMES,
        RW_U_MAX,
        RW_WHEEL_JS,
        available_backends,
        build_system,
        cubesat_12u_system,
        make_rw_integrator,
        rw_demo_heyoka,
        rw_demo_pars,
        rw_demo_state,
        rw_demo_system,
        wheel_specs,
    )
except ImportError as _exc:  # pragma: no cover - environment problem
    _missing = getattr(_exc, "name", None)
    if _missing is not None and not str(_missing).startswith("cubesat_12u_sim"):
        raise
    raise ImportError(
        "omniheyoka: the presets reuse the hand-written reference in "
        "'cubesat_12u_sim.backends.heyoka.heyoka_rw', which could not be "
        f"imported ({_exc}). Run from the Python/ directory (or put it on "
        "sys.path) so the sibling package resolves."
    ) from _exc

try:
    from .mock import MockOmniSystem, mock_rw_demo_system
except ImportError as _exc:  # pragma: no cover - only if the reference moved
    MockOmniSystem = None  # type: ignore[assignment]
    mock_rw_demo_system = None  # type: ignore[assignment]
    del _exc
