"""Batched C++ control tick (optional accelerator for heyoka_mc).

If the compiled ``_fastloop`` extension is unavailable, ``available`` is
False and callers must fall back to the numpy path.
"""
from __future__ import annotations

try:
    from ._fastloop import control_tick, tick_all

    available = True
except Exception:  # ImportError and friends -> numpy fallback
    control_tick = None  # type: ignore[assignment]
    tick_all = None  # type: ignore[assignment]
    available = False

__all__ = ["available", "control_tick", "tick_all"]
