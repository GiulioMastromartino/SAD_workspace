"""C++ Basilisk SysModel: Heyoka dynamics + full control loop in C++.

Drop-in replacement for Basilisk's spacecraft module. Reads
THRArrayOnTimeCmdMsg, runs the entire tick loop in C++, and writes
SCStatesMsg. All other Basilisk modules work unchanged.
"""
from __future__ import annotations

try:
    from ._cppdrive import run_tick

    available = True
except Exception:
    run_tick = None  # type: ignore[assignment]
    available = False

__all__ = ["available", "run_tick"]
