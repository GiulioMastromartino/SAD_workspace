import numpy as np
from cubesat_12u_sim import config as C
from cubesat_12u_sim.controller import SemiMajorPID
from cubesat_12u_sim.heyoka_propagate import propagate_closedloop_orbit


def test_semimajor_settles_and_coasts():
    pid = SemiMajorPID()
    c = propagate_closedloop_orbit(t_end=10 * 86400.0, dt_control=600.0,
                                   r0=C.R0_ECI, v0=C.V0_ECI * 0.995,
                                   pid=pid, tol=1e-10)
    assert np.all(np.isfinite(c["y"]))
    e = np.linalg.norm(c["y"][0:3, :], axis=0) - C.R_ORBIT
    assert np.abs(e[-1]) < 1000.0  # recovered to sub-km
    t = c["t"] / 86400.0
    # coasts after recovery: <0.5 m/s burned in the last 5 days
    assert c["dv"][-1] - c["dv"][np.searchsorted(t, 5.0)] < 0.5
