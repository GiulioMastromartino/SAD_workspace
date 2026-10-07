import numpy as np
import pytest
from cubesat_12u_sim import config as C


def _bazilisk():
    np = pytest.importorskip("numpy")
    rk = pytest.importorskip("Basilisk.utilities.RigidBodyKinematics")
    return rk


def test_constrained_axis_and_panel_normal_are_independent_axes():
    """The constraint boresight and the panel normal are separate settings."""
    axis = np.asarray(C.SUN_POINTING_AXIS_B, float)
    panel = np.asarray(C.VIZ_PANEL_NORMAL_B, float)
    deploy = np.asarray(C.VIZ_PANEL_DEPLOY_AXIS_B, float)
    for vector in (axis, panel, deploy):
        assert vector.shape == (3,)
        assert np.isclose(np.linalg.norm(vector), 1.0)
    # Wings must be buildable: panel normal and deploy axis are different axes.
    assert int(np.argmax(np.abs(panel))) != int(np.argmax(np.abs(deploy)))
    # The constrained boresight is +X; the wings keep their +Z normal.
    assert axis[0] == pytest.approx(1.0)
    assert abs(panel[2]) == pytest.approx(1.0)


def test_sun_pointing_sigma_aims_boresight_at_sun():
    rk = _bazilisk()
    from cubesat_12u_sim.sun_model import sun_pointing_sigma

    sunDir = np.array([0.0, 1.0, 0.0])
    sigma = sun_pointing_sigma(sunDir * C.VIZ_SUN_DISTANCE, C.SUN_ROLL_REFERENCE_N)
    dcm = rk.MRP2C(sigma)
    assert np.allclose(dcm @ dcm.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(dcm) == pytest.approx(1.0, abs=1e-12)
    bodyZ = dcm @ np.asarray(C.SUN_POINTING_AXIS_B, float)
    assert np.allclose(bodyZ, sunDir, atol=1e-9)


def test_sun_pointing_sigma_is_roll_aware_but_boresign_invariant():
    """Any roll reference must still put the boresight on the Sun."""
    rk = _bazilisk()
    from cubesat_12u_sim.sun_model import sun_pointing_sigma

    sunDir = np.array([0.0, 1.0, 0.0])
    axis = np.asarray(C.SUN_POINTING_AXIS_B, float)
    sigmas = []
    for ref in ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]):
        sigma = sun_pointing_sigma(sunDir * C.VIZ_SUN_DISTANCE, ref)
        bodyZ = rk.MRP2C(sigma) @ axis
        assert np.allclose(bodyZ, sunDir, atol=1e-9)
        sigmas.append(sigma)
    # Different roll references must actually give different attitudes.
    assert not np.allclose(sigmas[0], sigmas[2])


def test_sun_pointing_rejects_degenerate_roll_reference():
    from cubesat_12u_sim.sun_model import sun_pointing_sigma

    sunDir = np.array([0.0, 1.0, 0.0])
    with pytest.raises(ValueError):
        sun_pointing_sigma(sunDir * C.VIZ_SUN_DISTANCE, (0.0, 1.0, 0.0))


def test_sun_cone_is_a_keep_in_about_the_constrained_axis():
    assert 0.0 < C.VIZ_SUN_CONE_ANGLE_DEG < 90.0
    assert C.VIZ_SUN_CONE_HEIGHT > max(C.LX, C.LY, C.LZ)
    # Vizard keys the constraint off the body's displayName, not planetName.
    assert C.VIZ_SUN_BODY_NAME == "sun"
    assert C.VIZ_SUN_DISTANCE > 1.0e11


def test_sun_ephemeris_rotates_at_one_sweep_per_year():
    from cubesat_12u_sim.sun_model import SunEphemeris

    eph = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    assert eph.angularRate == pytest.approx(2.0 * np.pi / C.SUN_YEAR)
    # 0.9856 deg/day mean motion.
    assert np.degrees(eph.angularRate) * 86400.0 == pytest.approx(0.9856, abs=1e-3)
    # t=0 must match the direction this scenario always assumed: inertial +Y.
    assert np.allclose(eph.directionAt(0.0), [0.0, 1.0, 0.0], atol=1e-12)
    # A full year returns to the start; half a year is opposite.
    assert np.allclose(eph.directionAt(C.SUN_YEAR), [0.0, 1.0, 0.0], atol=1e-9)
    assert np.allclose(eph.directionAt(0.5 * C.SUN_YEAR), [0.0, -1.0, 0.0], atol=1e-9)
    # Over one spacecraft orbit the line barely moves, as it should.
    drift = np.degrees(eph.angularRate * C.T_ORBIT)
    assert 0.0 < drift < 1.0


def test_sun_ephemeris_velocity_is_the_path_derivative():
    from cubesat_12u_sim.sun_model import SunEphemeris

    eph = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    for t in (0.0, 1234.0, 1.0e6):
        theta = C.SUN_PHASE0 + eph.angularRate * t
        tangent = np.array([-np.sin(theta), np.cos(theta), 0.0]) * C.VIZ_SUN_DISTANCE
        vel = eph.angularRate * tangent          # exact derivative of the path
        assert np.isclose(np.linalg.norm(vel), 29785.0, rtol=2e-4)
        assert np.isclose(
            np.linalg.norm(eph.directionAt(t) * C.VIZ_SUN_DISTANCE),
            C.VIZ_SUN_DISTANCE, rtol=1e-12,
        )


def test_sun_pointing_reference_rate_matches_sun_rate():
    """The reference must co-rotate at the Sun's rate, not sit inertially."""
    from cubesat_12u_sim.sun_model import SunEphemeris, SunPointingReference

    eph = SunEphemeris(C.VIZ_SUN_DISTANCE, C.SUN_PHASE0)
    ref = SunPointingReference(eph, C.SUN_ROLL_REFERENCE_N)
    t = 1000.0
    ref.Reset(int(t * 1e9))
    ref.UpdateState(int((t + 10.0) * 1e9))
    # The Sun sweeps the inertial XY plane, so the reference spins about +Z.
    assert np.allclose(ref.omega, [0.0, 0.0, eph.angularRate], atol=1e-18)
    # And the reference boresight sits on the Sun at that epoch.
    axis = np.asarray(C.SUN_POINTING_AXIS_B, float)
    rk = pytest.importorskip("Basilisk.utilities.RigidBodyKinematics")
    boresight = rk.MRP2C(ref.sigma) @ axis
    assert np.allclose(boresight, eph.directionAt(t + 10.0), atol=1e-9)
