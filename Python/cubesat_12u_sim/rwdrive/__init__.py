"""C++ Heyoka RW dynamics extension."""
try:
    import heyoka  # noqa: F401  # must load first: _rwdrive resolves
    # heyoka symbols via dynamic_lookup from the already-loaded module
    from ._rwdrive import run_rw_mission, run_rw_tick

    available = True
except Exception as _e:  # extension not built for this platform
    import warnings

    warnings.warn(f"rwdrive unavailable: {_e!r}")
    run_rw_tick = None  # type: ignore[assignment]
    run_rw_mission = None  # type: ignore[assignment]
    available = False

__all__ = ["available", "run_rw_tick", "run_rw_mission"]
