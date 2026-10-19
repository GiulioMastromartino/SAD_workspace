"""Full C++ Heyoka campaign driver (optional accelerator).

If the compiled ``_cxxdrive`` extension is unavailable, ``available`` is
False and callers must use the batch backend.
"""
from __future__ import annotations

try:
    from ._cxxdrive import run_chunk

    available = True
except Exception:  # ImportError and friends -> batch backend
    run_chunk = None  # type: ignore[assignment]
    available = False

__all__ = ["available", "run_chunk"]
