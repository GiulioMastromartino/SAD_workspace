"""Shared helpers for ALL audit-bundle probes (stdlib + numpy only at import).

Every probe must import from here and write ONLY to ``audit_bundle/results/``
via :func:`save_result` / :func:`log_text`.  Basilisk / heyoka imports are
lazy *inside* functions so a missing env never breaks collection.

Layout (all paths derived from ``__file__`` via pathlib)::

    audit_bundle/
      probes/common.py   <- this file
      probes/wp*.py
      probes/collect.py
      results/<name>.json + <name>.log
      run_all.sh

Python 3.11 compatible.  No heavy sims here.
"""

from __future__ import annotations

import datetime
import json
import platform
import sys
import traceback
from pathlib import Path

try:
    import numpy  # noqa: F401  (allowed light dep; not used directly)
    _NUMPY_OK = True
except Exception:
    _NUMPY_OK = False

# -- paths (relative to this file, never cwd) --------------------------------
_THIS_DIR: Path = Path(__file__).resolve().parent
BUNDLE_DIR: Path = _THIS_DIR.parent
RESULTS_DIR: Path = BUNDLE_DIR / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _os_tag() -> str:
    """'macOS' / 'Linux' / raw system name for env_info()."""
    s = platform.system()
    if s == "Darwin":
        return "macOS"
    if s == "Linux":
        return "Linux"
    return s


def safe_import_basilisk() -> bool:
    """Return True if ``import Basilisk`` works, else False.  Never raises."""
    try:
        import importlib

        importlib.import_module("Basilisk")
        return True
    except Exception:
        return False


def _basilisk_version() -> str:
    """Basilisk.__version__ or 'unknown'.  Never raises (lazy import)."""
    try:
        import importlib

        mod = importlib.import_module("Basilisk")
        return str(getattr(mod, "__version__", "unknown"))
    except Exception:
        return "unknown"


def _heyoka_version() -> str:
    """heyoka version string or 'unknown'.  Never raises (lazy import)."""
    try:
        import importlib

        mod = importlib.import_module("heyoka")
        for attr in ("__version__", "version"):
            v = getattr(mod, attr, None)
            if v is not None:
                return str(v() if callable(v) else v)
        return "unknown"
    except Exception:
        return "unknown"


def _bsk_sdk_includes() -> dict:
    """Describe the ``bsk_sdk`` package path + include listing.  Never raises."""
    info: dict = {"present": False, "path": None, "includes": []}
    try:
        import importlib.util

        spec = importlib.util.find_spec("bsk_sdk")
        if spec is None or spec.origin is None:
            return info
        info["present"] = True
        pkg_dir = Path(spec.origin).resolve().parent
        info["path"] = str(pkg_dir)
        # Look for an include/ dir next to (or inside) the package.
        for cand in (pkg_dir / "include", pkg_dir.parent / "include", pkg_dir):
            try:
                if cand.is_dir():
                    listing = sorted(
                        p.name for p in cand.iterdir() if p.suffix in (".h", ".hpp")
                    )[:50]
                    if listing:
                        info["includes"] = listing
                        info["include_dir"] = str(cand)
                        break
            except Exception:
                continue
    except Exception as exc:
        info["error"] = f"{type(exc).__name__}: {exc}"
    return info


def env_info() -> dict:
    """Environment fingerprint dict (pure stdlib + lazy imports)."""
    return {
        "timestamp_utc": _utc_now_iso(),
        "platform": platform.platform(),
        "os_tag": _os_tag(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "executable": sys.executable,
        "heyoka": _heyoka_version(),
        "basilisk": _basilisk_version(),
        "basilisk_importable": safe_import_basilisk(),
        "numpy_importable": _NUMPY_OK,
        "bsk_sdk": _bsk_sdk_includes(),
    }


def save_result(name: str, payload: dict) -> str:
    """Write ``results/<name>.json`` and return its path as str.

    Envelope: ``{timestamp, platform, python, basilisk, heyoka, payload}``
    plus ``env_info`` for forensics.  Creates RESULTS_DIR if needed.
    """
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    doc = {
        "name": name,
        "timestamp_utc": _utc_now_iso(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "basilisk": _basilisk_version(),
        "heyoka": _heyoka_version(),
        "env": env_info(),
        "payload": dict(payload),
    }
    path = RESULTS_DIR / f"{name}.json"
    path.write_text(json.dumps(doc, indent=2, default=str) + "\n", encoding="utf-8")
    return str(path)


def log_text(name: str, text: str) -> str:
    """Append ``text`` (one line block) to ``results/<name>.log``.  Returns path."""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    path = RESULTS_DIR / f"{name}.log"
    with path.open("a", encoding="utf-8") as fh:
        fh.write(text if text.endswith("\n") else text + "\n")
    return str(path)


def write_exception(name: str, exc: BaseException) -> str:
    """Save a traceback for probe ``name``; returns the ``.exception.log`` path."""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    tb = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    path = RESULTS_DIR / f"{name}.exception.log"
    path.write_text(tb, encoding="utf-8")
    # Also record a minimal JSON so collect.py sees the probe as errored.
    try:
        save_result(name, {"status": "error", "error": f"{type(exc).__name__}: {exc}"})
    except Exception:
        pass
    return str(path)
