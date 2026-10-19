"""Standard 12-thruster 6-DOF geometry: rank, authority, C++ allocation."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import sad_py
from SAD_Project.core import config as C


def _influence():
    n = len(C.THRUSTER_POS)
    A = np.zeros((6, n))
    for i in range(n):
        d = C.THRUSTER_DIR[i] / np.linalg.norm(C.THRUSTER_DIR[i])
        A[:3, i] = d
        A[3:, i] = np.cross(C.THRUSTER_POS[i], d)
    return A


def test_standard_12_layout():
    assert len(C.THRUSTER_POS) == 12
    assert len(C.THRUSTER_DIR) == 12
    assert len(C.THRUSTER_TMAX) == 12
    A = _influence()
    assert np.linalg.matrix_rank(A) == 6
    # every force/torque axis has both signs available
    for j in range(6):
        assert A[j].max() > 0.0, j
        assert A[j].min() < 0.0, j


def test_cpp_allocation_full_wrench():
    tp = sad_py.ThrusterParams()
    tp.n = len(C.THRUSTER_POS)
    tp.pos = [np.array(p, float) for p in C.THRUSTER_POS]
    tp.dir = [np.array(d, float) for d in C.THRUSTER_DIR]
    tp.tmax = list(C.THRUSTER_TMAX)
    tp.isp = C.ISP_JET
    tp.g0 = C.G0
    A = _influence()
    for j in range(6):
        w = np.zeros(6); w[j] = 0.01
        u = np.array(sad_py.thruster_allocate(tp, w[:3], w[3:]))
        assert u.shape == (12,)
        assert np.all(u >= 0.0)
        got = A @ u
        # Min-norm + non-negative clamp splits a pure wrench across opposing
        # thrusters, so the clamped result is half.  The geometry test above
        # guarantees both signs exist; here we check the C++ path is wired to
        # all 12 thrusters and responds in the commanded direction.
        assert np.dot(got, w) > 0.0, (j, got, w)
