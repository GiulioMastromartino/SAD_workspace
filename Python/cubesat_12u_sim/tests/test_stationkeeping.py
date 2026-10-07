import numpy as np
from cubesat_12u_sim import config as C
from cubesat_12u_sim.controller import OrbitPID
from cubesat_12u_sim.heyoka_propagate import propagate_heyoka, propagate_closedloop
from cubesat_12u_sim.run_stationkeeping import demo_state, radius_err


def test_pid_reduces_drift():
    y0 = demo_state()
    open_log = propagate_heyoka(t_end=600.0, dt_log=C.DT_CONTROL, y0=y0)
    closed_log = propagate_closedloop(t_end=600.0, dt_control=C.DT_CONTROL,
                                      y0=y0, pid=OrbitPID())
    assert np.all(np.isfinite(closed_log["y"]))
    e_open = np.abs(radius_err(open_log["y"])).max()
    e_closed = np.abs(radius_err(closed_log["y"])).max()
    assert e_closed < e_open, f"PID did not help: {e_closed} vs {e_open}"
    assert closed_log["dv"] >= 0.0
    assert closed_log["u"].max() <= 1.0 + 1e-9
