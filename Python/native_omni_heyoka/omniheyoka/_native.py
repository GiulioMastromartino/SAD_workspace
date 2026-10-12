"""Lazy access to the compiled ``omni_heyoka`` extension module.

WS1 builds the SWIG module into this package, so the compiled artefact
normally lives at ``omniheyoka/omni_heyoka*.so`` (importable as
``omniheyoka.omni_heyoka``).  A plain top-level ``omni_heyoka`` is also
accepted, so an out-of-tree build keeps working.

Nothing here raises on a *missing* module: :func:`load_native` returns
``None`` instead, which lets ``import omniheyoka`` (and therefore the
``OmniTaylorAdaptive`` shim, plus the pure-Python mock backend) work before
the extension has been built.  Use :func:`require_native` when you actually
need the compiled object -- it raises a clear, actionable ``ImportError``.

Load failures that are *not* "the module is not there" (for instance a
``libheyoka`` rpath problem) are raised immediately rather than being
swallowed into a ``None``.
"""

from __future__ import annotations

import importlib
import os
from typing import Any

__all__ = ["load_native", "require_native", "native_available", "build_hint"]

# Candidate module names, most preferred first.
_CANDIDATES = ("omniheyoka.omni_heyoka", "omni_heyoka")

# Override hook: ``OMNI_HEYOKA_MODULE=/path/to/mod.so`` or a dotted module
# name.  Handy for out-of-tree builds and for the Stage-1/no-native workflow.
_ENV_VAR = "OMNI_HEYOKA_MODULE"


def _candidate_names() -> tuple[str, ...]:
    override = os.environ.get(_ENV_VAR)
    if override:
        return (override,)
    return _CANDIDATES


def _try_import(name: str) -> Any | None:
    """Import ``name``; return ``None`` only if it genuinely is not installed."""
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        # ``ModuleNotFoundError`` carries the missing module's name.  If the
        # missing thing is *not* the module we asked for, the import failed
        # for a deeper reason (a broken dependency) and must be surfaced.
        missing = getattr(exc, "name", None)
        if not isinstance(exc, ModuleNotFoundError) or (
            missing is not None and missing != name and missing != name.rpartition(".")[2]
        ):
            raise
        return None


def load_native() -> Any | None:
    """Return the compiled ``omni_heyoka`` module, or ``None`` if not built."""
    for name in _candidate_names():
        module = _try_import(name)
        if module is not None:
            return module
    return None


def native_available() -> bool:
    """``True`` if the compiled extension can be imported right now."""
    return load_native() is not None


def build_hint() -> str:
    """Human-readable instructions for building the native module."""
    return (
        "the compiled 'omni_heyoka' extension module is not available. Build "
        "the native_omni_heyoka package first:\n"
        "    cmake -S native_omni_heyoka -B native_omni_heyoka/build "
        "-DCMAKE_BUILD_TYPE=Release\n"
        "    cmake --build native_omni_heyoka/build\n"
        "or point " + _ENV_VAR + " at an already-built module.\n"
        "Until then you can exercise this package against the pure-Python "
        "mock backend (omniheyoka.mock / scratch_ws2/mock_system.py)."
    )


def require_native() -> Any:
    """Return the compiled ``omni_heyoka`` module or raise a clear error."""
    module = load_native()
    if module is None:
        raise ImportError(f"omniheyoka: {build_hint()}")
    return module


def require_omni_system_class() -> Any:
    """Return the compiled ``OmniSystem`` class or raise a clear error."""
    module = require_native()
    for attr in ("OmniSystem",):
        cls = getattr(module, attr, None)
        if cls is not None:
            return cls
    raise ImportError(
        f"omniheyoka: the compiled module {module.__name__!r} does not expose "
        f"'OmniSystem' (found: {sorted(a for a in dir(module) if not a.startswith('_'))}). "
        "This is a WS1/C++ contract mismatch -- the extension needs rebuilding "
        "from a system.hpp that declares class OmniSystem."
    )
