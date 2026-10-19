"""Attitude-frame conventions used by the sun-pointing target."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import sad_py
from SAD_Project.core import dynamics as D
from SAD_Project.core import sun_model as SM


def _quat_z(angle):
    return np.array([0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0)])


def test_body_from_inertial_rotation():
    # +90 deg about z: inertial +X seen in the body frame is body -Y.
    q = _quat_z(np.pi / 2.0)
    R = D.quat_to_dcm(q)
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0], atol=1e-12)
    # The C++ and Python implementations must agree.
    assert np.allclose(np.asarray(sad_py.quat_to_dcm(q)), R, atol=1e-12)


def test_dcm_quat_roundtrip():
    q = D.quat_normalize(np.array([0.2, -0.3, 0.1, 0.9]))
    R = D.quat_to_dcm(q)
    q2 = D.dcm_to_quat(R)
    assert abs(abs(np.dot(q, q2)) - 1.0) < 1e-12
    assert np.allclose(D.quat_to_dcm(q2), R, atol=1e-12)


def test_sun_target_points_body_x_at_sun():
    for sun in (np.array([1.0, 0.0, 0.0]),
                np.array([0.6, 0.8, 0.0]),
                np.array([0.2, -0.5, 0.84])):
        sun = sun / np.linalg.norm(sun)
        q = SM.sun_pointing_quaternion(sun, np.array([0.0, 0.0, 1.0]))
        body_sun = D.quat_to_dcm(q) @ sun
        assert np.allclose(body_sun, [1.0, 0.0, 0.0], atol=1e-12)
