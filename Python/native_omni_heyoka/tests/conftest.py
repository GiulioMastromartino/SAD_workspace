"""Test-session path bootstrap (no test logic here).

Makes pytest runs independent of the invocation directory: the sibling
`cubesat_12u_sim` package (imported by `omniheyoka.presets` at
`omniheyoka/__init__` import time) resolves iff the `Python/` repo root is
on `sys.path`. Without this, running pytest from anywhere except `Python/`
turns every `importorskip("omniheyoka.*")` into a hard error instead of a
skip-or-pass, and host-gated tests fail for purely positional reasons.

Deliberately does NOT add `native_omni_heyoka/` itself: the tree copy of
`omniheyoka/` carries no compiled extension, so package imports must keep
resolving to the installed wheel.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO_ROOT = str(Path(__file__).resolve().parents[2])  # Python/
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


@pytest.fixture(autouse=True)
def _isolate_sys_path():
    """Undo probe-import sys.path surgery, per test.

    Several tests exec a sibling probe module by file path for its
    stdlib-only registry (``bench_stock_vs_omni``, ``hijack_demo``). Those
    probes insert their own dir / the repo root at import time -- and
    ``hijack_demo`` historically inserted ``native_omni_heyoka/`` itself,
    whose ``omniheyoka/`` tree copy carries no compiled extension. A leaked
    entry shadows the installed wheel for every LATER test in the session
    (``test_wp25_b2`` observed ``No module named 'omniheyoka.omni_heyoka'``
    only in whole-suite runs). Snapshot the entry path per test so the
    leak can never cross a test boundary.
    """
    saved = list(sys.path)
    try:
        yield
    finally:
        sys.path[:] = saved
