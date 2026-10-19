"""Runtime OBJ/MTL generation for the Vizard spacecraft model.

Purely a visualization asset: nothing here feeds the dynamics, so the hub
inertia and mass in `config` are untouched. Models applied to spacecraft are
interpreted in meters, so the geometry below is authored directly in the body
frame at true scale and `createCustomModel` is called with scale=[1, 1, 1].

The file is written to a deterministic path and deliberately not deleted: Vizard
re-imports the OBJ when a saved `.bin` is played back later.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


class _ObjBuilder:
    """Minimal Wavefront OBJ writer with per-face normals and materials."""

    def __init__(self) -> None:
        self.lines: list[str] = []
        self.vertexCount = 0
        self.normalCount = 0

    def add_box(self, mins, maxs, material: str) -> None:
        """Append an axis-aligned box as six quads wound outward."""
        x0, y0, z0 = mins
        x1, y1, z1 = maxs
        faces = (
            ((0.0, 0.0, 1.0), ((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))),
            ((0.0, 0.0, -1.0), ((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0))),
            ((0.0, 1.0, 0.0), ((x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0))),
            ((0.0, -1.0, 0.0), ((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))),
            ((1.0, 0.0, 0.0), ((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))),
            ((-1.0, 0.0, 0.0), ((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0))),
        )
        for normal, corners in faces:
            for corner in corners:
                self.lines.append(
                    f"v {corner[0]:.6f} {corner[1]:.6f} {corner[2]:.6f}"
                )
            self.vertexCount += 4
            self.lines.append(f"vn {normal[0]:.1f} {normal[1]:.1f} {normal[2]:.1f}")
            self.normalCount += 1
            base = self.vertexCount - 3
            index = self.normalCount
            self.lines.append(f"usemtl {material}")
            self.lines.append(
                f"f {base}/{index} {base + 1}/{index} {base + 2}/{index} {base + 3}/{index}"
            )

    def render(self, name: str, mtlName: str) -> str:
        header = [f"# {name}", f"mtllib {mtlName}"]
        return "\n".join(header + self.lines) + "\n"


_BUS_MATERIAL = """newmtl bus
Ka 0.180 0.180 0.190
Kd 0.720 0.725 0.745
Ks 0.250 0.250 0.260
Ns 48.0
d 1.0
illum 2
"""

_PANEL_MATERIAL = """newmtl panel
Ka 0.010 0.012 0.040
Kd 0.045 0.075 0.320
Ks 0.620 0.640 0.720
Ns 90.0
d 1.0
illum 2
"""


def write_panelled_bus(
    objPath: str | Path,
    lx: float,
    ly: float,
    lz: float,
    deploy: float,
    thickness: float,
    normal_B: tuple[float, float, float],
    deployAxis_B: tuple[float, float, float],
) -> Path:
    """Write a bus with two deployed solar wings, in meters.

    The wing geometry is driven entirely by `normal_B` and `deployAxis_B`, which
    are supplied by the caller and are independent of the sun-pointing constraint
    axis. `normal_B` is the panel normal and `deployAxis_B` is the body axis the
    wings extend along; the two must be different body axes, and the panel plane
    is the plane containing `deployAxis_B`, so the cells face along +/-`normal_B`.

    Args:
        objPath: destination `.obj` path; the matching `.mtl` sits alongside.
        lx, ly, lz: bus edge lengths [m].
        deploy: wing length measured outboard of each bus face [m].
        thickness: wing plate thickness [m].
        normal_B: panel normal (cells face along +/-this axis).
        deployAxis_B: body axis the wings extend along.

    Returns:
        The written `.obj` path.
    """
    normal = _axis_index(normal_B, "normal_B")
    deployIndex = _axis_index(deployAxis_B, "deployAxis_B")
    if normal == deployIndex:
        raise ValueError(
            f"deployAxis_B {tuple(deployAxis_B)} must be a different body axis "
            f"from normal_B {tuple(normal_B)}"
        )

    objPath = Path(objPath)
    objPath.parent.mkdir(parents=True, exist_ok=True)
    mtlName = f"{objPath.stem}.mtl"

    half = np.array([lx, ly, lz], dtype=float) / 2.0
    halfThickness = thickness / 2.0
    widthIndex = ({0, 1, 2} - {normal, deployIndex}).pop()

    builder = _ObjBuilder()
    builder.add_box(-half, half, "bus")
    for sign in (1.0, -1.0):
        lo, hi = np.empty(3), np.empty(3)
        # Wings reach `deploy` outboard of the bus face on the deploy axis, are
        # one plate thickness thick along the panel normal, and span the bus
        # width on the remaining axis.
        inner = sign * half[deployIndex]
        outer = sign * (half[deployIndex] + deploy)
        lo[deployIndex], hi[deployIndex] = min(inner, outer), max(inner, outer)
        lo[normal], hi[normal] = -halfThickness, halfThickness
        lo[widthIndex], hi[widthIndex] = -half[widthIndex], half[widthIndex]
        builder.add_box(lo, hi, "panel")

    objPath.write_text(builder.render("12U bus with deployed solar wings", mtlName))
    objPath.with_suffix(".mtl").write_text(_BUS_MATERIAL + _PANEL_MATERIAL)
    return objPath


def _axis_index(vector, name: str) -> int:
    """Index of the body axis a direction refers to; must be axis-aligned.

    Uses the magnitude because a signed unit vector such as (0, 0, -1) must
    resolve to the z axis, which a plain argmax would get wrong.
    """
    axis = np.asarray(vector, dtype=float)
    if axis.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {axis.shape}")
    if not np.isclose(np.linalg.norm(axis), 1.0, atol=1e-9):
        raise ValueError(f"{name} must be a unit vector, got {np.linalg.norm(axis)}")
    nonzero = np.flatnonzero(np.abs(axis) > 1e-9)
    if nonzero.size != 1:
        raise ValueError(
            f"{name} must be aligned with a single body axis, got {tuple(axis)}"
        )
    return int(nonzero[0])
