"""Native Heyoka RW dynamics Basilisk extension. ISC License - see LICENSE."""
try:
    import heyoka  # noqa: F401  # first as fallback: if the linked libheyoka
    # ever fails rpath resolution, symbols still resolve from the loaded module
except ImportError:  # linked build does not need the Python package
    heyoka = None  # type: ignore[assignment]

from ._bsk_compatibility import check_basilisk_compatibility as _check_basilisk

_check_basilisk()
del _check_basilisk

from .heyokaMCDynamics import HeyokaMCDynamics
from .heyokaRWDynamics import HeyokaRWDynamics

__all__ = ["HeyokaMCDynamics", "HeyokaRWDynamics"]
