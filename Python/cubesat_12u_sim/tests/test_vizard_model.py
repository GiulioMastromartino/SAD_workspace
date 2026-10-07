import numpy as np
import pytest
from cubesat_12u_sim import config as C
from cubesat_12u_sim import vizard_model as vm


def _vertices(obj_text):
    rows = [ln.split()[1:] for ln in obj_text.splitlines() if ln.startswith("v ")]
    return np.array([[float(x) for x in row] for row in rows])


def test_inertia_matches_solid_prism():
    assert np.isclose(C.IX, C.MASS / 12.0 * (C.LY**2 + C.LZ**2))
    assert np.isclose(C.IY, C.MASS / 12.0 * (C.LX**2 + C.LZ**2))
    assert np.isclose(C.IZ, C.MASS / 12.0 * (C.LX**2 + C.LY**2))
    assert np.allclose(np.diag(C.INERTIA), [C.IX, C.IY, C.IZ])
    assert np.allclose(C.INV_INERTIA @ C.INERTIA, np.eye(3))


def test_panelled_bus_geometry_in_metres(tmp_path):
    obj = vm.write_panelled_bus(
        tmp_path / "bus.obj", lx=C.LX, ly=C.LY, lz=C.LZ,
        deploy=C.VIZ_PANEL_DEPLOY, thickness=C.VIZ_PANEL_THICKNESS,
        normal_B=C.VIZ_PANEL_NORMAL_B, deployAxis_B=C.VIZ_PANEL_DEPLOY_AXIS_B,
    )
    verts = _vertices(obj.read_text())
    lo, hi = verts.min(axis=0), verts.max(axis=0)

    # Bus occupies the configured envelope; wings deploy symmetrically in +/-X.
    assert np.allclose([lo[1], hi[1]], [-C.LY / 2, C.LY / 2])
    assert np.allclose([lo[2], hi[2]], [-C.LZ / 2, C.LZ / 2])
    assert np.isclose(hi[0] - lo[0], C.LX + 2 * C.VIZ_PANEL_DEPLOY)
    assert np.allclose(lo, -hi)  # symmetric about the body origin

    text = obj.read_text()
    assert text.count("\nf ") == 18  # three boxes, six quads each
    assert "newmtl panel" in obj.with_suffix(".mtl").read_text()
    assert "usemtl panel" in text


def test_viz_geometry_does_not_feed_inertia():
    """Solar-wing visualization must not perturb mass properties."""
    before = C.INERTIA.copy()
    beforeInv = C.INV_INERTIA.copy()
    beforeMass = C.MASS
    vm.write_panelled_bus(
        C.VIZ_MODEL_DIR + "/_invariant_check.obj", lx=C.LX, ly=C.LY, lz=C.LZ,
        deploy=C.VIZ_PANEL_DEPLOY, thickness=C.VIZ_PANEL_THICKNESS,
        normal_B=C.VIZ_PANEL_NORMAL_B, deployAxis_B=C.VIZ_PANEL_DEPLOY_AXIS_B,
    )
    assert np.array_equal(C.INERTIA, before)
    assert np.array_equal(C.INV_INERTIA, beforeInv)
    assert C.MASS == beforeMass


def test_wing_normal_follows_the_requested_axis(tmp_path_factory):
    """The thin axis of the rendered wings must equal the panel normal."""
    for normal, deploy in (
        ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
        ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ):
        path = tmp_path_factory.mktemp("wing") / "m.obj"
        obj = vm.write_panelled_bus(
            path, lx=C.LX, ly=C.LY, lz=C.LZ,
            deploy=C.VIZ_PANEL_DEPLOY, thickness=C.VIZ_PANEL_THICKNESS,
            normal_B=normal, deployAxis_B=deploy,
        )
        verts = _vertices(obj.read_text())
        half = np.array([C.LX, C.LY, C.LZ]) / 2.0
        wings = verts[~np.all(np.abs(verts) <= half + 1e-9, axis=1)]
        span = np.ptp(wings, axis=0)
        # Plate is thinnest along the normal, and reaches exactly `deploy` out.
        assert int(np.argmin(span)) == int(np.argmax(np.abs(normal)))
        dep = int(np.argmax(np.abs(np.array(deploy, float))))
        assert np.max(np.abs(wings[:, dep])) - half[dep] == pytest.approx(
            C.VIZ_PANEL_DEPLOY
        )


def test_wing_builder_rejects_bad_axes(tmp_path):
    with pytest.raises(ValueError):  # same axis for both
        vm.write_panelled_bus(
            tmp_path / "a.obj", lx=C.LX, ly=C.LY, lz=C.LZ,
            deploy=C.VIZ_PANEL_DEPLOY, thickness=C.VIZ_PANEL_THICKNESS,
            normal_B=(0.0, 0.0, 1.0), deployAxis_B=(0.0, 0.0, 1.0),
        )
    with pytest.raises(ValueError):  # not axis-aligned
        vm.write_panelled_bus(
            tmp_path / "b.obj", lx=C.LX, ly=C.LY, lz=C.LZ,
            deploy=C.VIZ_PANEL_DEPLOY, thickness=C.VIZ_PANEL_THICKNESS,
            normal_B=(0.707, 0.707, 0.0), deployAxis_B=(0.0, 0.0, 1.0),
        )
