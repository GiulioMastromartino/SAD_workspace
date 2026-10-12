"""Single-source thruster configuration for stock and omni arms.

Stock takes geometry through ``thrusterFactory.create(name, pos, dir,
MaxThrust, ...)``; ``OmniSpacecraft`` takes flat arrays via ``setThrusters``.
Writing the numbers twice invites transcription drift, so both arms are built
from one spec list::

    specs = [([0.1, 0.1, 0.0], [-1, 0, 0], 1.0), ...]   # (pos, dir, tmax)

    create_stock_thrusters(tf, specs)          # stock factory entries
    configure_omni_thrusters(sc, specs)        # flat arrays into the module

:func:`thruster_specs_from_factory` reads the geometry back out of a
configured factory (``thrLoc_B`` / ``thrDir_B`` / ``MaxThrust`` on each
``THRSimConfig``), so an existing factory can be audited against the omni
side. Basilisk imports are function-local: this module imports without it.
"""

from __future__ import annotations

from typing import Any, Sequence

__all__ = [
    'create_stock_thrusters',
    'omni_thruster_arrays',
    'configure_omni_thrusters',
    'thruster_specs_from_factory',
]


def _as_triplet(v: Sequence[float], what: str) -> list[float]:
    flat: list = []
    stack: list = [v]
    while stack:
        item = stack.pop(0)
        if isinstance(item, (list, tuple)):
            stack[0:0] = list(item)
        else:
            flat.append(float(item))
    if len(flat) != 3:
        raise ValueError(f'thruster {what} needs 3 components, got {flat}')
    return flat


def create_stock_thrusters(tf: Any, specs: Sequence[tuple],
                           cls: str = 'TEST_Thruster', **kwargs: Any) -> None:
    """Create stock factory entries from a shared spec list.

    Each spec is ``(position[3], direction[3], max_thrust)`` — the same order
    as ``thrusterFactory.create``. Extra kwargs (``cutoffFrequency``,
    ``useMinPulseTime``) go to every ``create`` call.
    """
    for pos, direction, tmax in specs:
        tf.create(cls, _as_triplet(pos, 'position'),
                  _as_triplet(direction, 'direction'),
                  MaxThrust=float(tmax), **kwargs)


def omni_thruster_arrays(specs: Sequence[tuple]) -> tuple[list, list, list]:
    """Flat ``(dirs, pos, tmax)`` arrays for ``OmniSpacecraft.setThrusters``."""
    dirs: list = []
    pos: list = []
    tmax: list = []
    for p, d, t in specs:
        dirs.extend(_as_triplet(d, 'direction'))
        pos.extend(_as_triplet(p, 'position'))
        tmax.append(float(t))
    return dirs, pos, tmax


def configure_omni_thrusters(sc: Any, specs: Sequence[tuple]) -> None:
    """Push a shared spec list into an ``OmniSpacecraft``."""
    dirs, pos, tmax = omni_thruster_arrays(specs)
    sc.setThrusters(dirs, pos, tmax)


def thruster_specs_from_factory(tf: Any) -> list[tuple[list, list, float]]:
    """Read ``(pos, dir, tmax)`` specs back out of a configured factory."""
    out = []
    for entry in tf.thrusterList.values():
        out.append((_as_triplet(entry.thrLoc_B, 'position'),
                    _as_triplet(entry.thrDir_B, 'direction'),
                    float(entry.MaxThrust)))
    return out
