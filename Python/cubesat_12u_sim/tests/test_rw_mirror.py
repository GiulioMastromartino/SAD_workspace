import numpy as np

from cubesat_12u_sim.heyoka_attitude import mrp_diff
from cubesat_12u_sim.heyoka_rw import (
    N_PARS_RW,
    pack_rw_pars,
    pack_rw_state,
    rw_allocate,
    run_rw_mirror,
)


def test_rw_allocate_sign_and_clip():
    # Wheel torque must NEGATE bus demand (verified sign-for-sign vs stock).
    u = rw_allocate(np.array([-0.38, -0.4, 0.15]))
    assert np.allclose(u, [0.2, 0.2, -0.15])
    u = rw_allocate(np.zeros(3))
    assert np.allclose(u, 0.0)


def test_rw_par_layout():
    assert N_PARS_RW == 19
    y0 = pack_rw_state(np.zeros(3), np.zeros(3), np.zeros(3),
                       np.zeros(3), np.zeros(3))
    assert len(y0) == 15
    assert len(pack_rw_pars(np.zeros(3), 750.0, np.diag([900.0, 800.0, 600.0]))) == 19


def test_rw_freeflight_parity():
    from cubesat_12u_sim.bsk_rw_stock import run_stock_rw
    from cubesat_12u_sim.heyoka_rw import make_rw_integrator
    s = run_stock_rw(duration=60.0, K=0.0, P=0.0)
    y0 = pack_rw_state(s["r_CN_N"][0], s["v_CN_N"][0], s["sigma_BN"][0],
                       s["omega_BN_B"][0], s["wheel_speeds"][0])
    ta = make_rw_integrator(y0)
    for t in s["t"][1:]:
        ta.propagate_for(float(t - ta.time))
    st = np.asarray(ta.state, float)
    assert np.linalg.norm(st[0:3] - s["r_CN_N"][-1]) < 0.01
    assert np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(st[6:9], s["sigma_BN"][-1])))) < 0.01


def test_rw_closedloop_parity():
    from cubesat_12u_sim.bsk_rw_stock import run_stock_rw
    s = run_stock_rw(duration=600.0)
    y0 = pack_rw_state(s["r_CN_N"][0], s["v_CN_N"][0], s["sigma_BN"][0],
                       s["omega_BN_B"][0], s["wheel_speeds"][0])
    h = run_rw_mirror(duration=600.0, dt=0.1, y0=y0)
    Y = h["Y"]
    n = Y.shape[1] - 1
    assert np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(Y[6:9, n], s["sigma_BN"][-1])))) < 1e-3
    assert np.degrees(np.linalg.norm(
        Y[9:12, n] - s["omega_BN_B"][-1])) < 1e-4
