import numpy as np
from cubesat_12u_sim import config as C
from cubesat_12u_sim.dynamics import euler_to_dcm, pack_state
from cubesat_12u_sim.actuators import jets_to_wrench
from cubesat_12u_sim.propagate import propagate


def test_dcm_orthonormal():
    R = euler_to_dcm(0.1, 0.2, 0.3)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)


def test_jets_zero():
    F, t = jets_to_wrench(np.zeros(12))
    assert np.allclose(F, 0) and np.allclose(t, 0)


def test_freefall_short():
    sol = propagate(t_end=60.0, dt_log=10.0)
    assert sol.success
    r_norm = np.linalg.norm(sol.y[0:3, -1])
    assert abs(r_norm - C.R_ORBIT) / C.R_ORBIT < 0.01
