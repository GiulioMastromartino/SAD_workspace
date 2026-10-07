import numpy as np

from cubesat_12u_sim import config as C
from cubesat_12u_sim.heyoka_attitude import (
    build_mrp_system,
    make_mrp_integrator,
    mrp_diff,
    mrp_from_dcm,
    mrp_to_dcm,
    pack_mrp_state,
    pack_pars,
    propagate_closedloop_mrp,
)
from cubesat_12u_sim.actuators import (
    allocate_cached,
    allocation_matrices,
    allocation_pinv,
    quantize_duty_cycle,
)
from cubesat_12u_sim.monte_carlo import MonteCarloConfig


def test_mrp_dcm_orthonormal():
    R = mrp_to_dcm(np.array([0.1, -0.2, 0.3]))
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(R) - 1.0) < 1e-12


def test_mrp_diff_identity():
    assert np.allclose(mrp_diff(np.array([0.1, 0.2, -0.15]),
                                np.array([0.1, 0.2, -0.15])), 0.0, atol=1e-12)


def test_mrp_system_builds_and_steps():
    y0 = pack_mrp_state(C.R0_ECI, C.V0_ECI,
                        np.array([0.1, -0.2, 0.3]), C.OMEGA0_BODY)
    ta = make_mrp_integrator(y0)
    ta.propagate_for(1.0)
    assert ta.time == 1.0
    assert len(ta.state) == 12
    assert len(pack_pars(np.zeros(3), np.zeros(3), C.MASS, C.INERTIA)) == 19


def test_closedloop_mrp_shape_and_cone():
    mc = MonteCarloConfig()
    res = propagate_closedloop_mrp(100, mc)
    assert res["kpis"]["inside_sun_cone"] in (True, False)
    assert res["timeseries"]["time"][-1] == mc.t_total
    # sun-pointing must converge into the cone for the nominal seed
    assert res["kpis"]["final_sun_angle_deg"] < mc.sun_cone_angle_deg
    assert res["kpis"]["total_propellant_g"] > 0


def test_batch_runner_smoke():
    from cubesat_12u_sim.heyoka_mc import run_heyoka_monte_carlo
    res = run_heyoka_monte_carlo(num_runs=2, base_seed=100,
                                 mc_cfg=MonteCarloConfig(), verbose=False)
    assert len(res["runs"]) == 2
    assert res["runs"][0]["timeseries"]["time"][-1] == MonteCarloConfig().t_total


def test_mrp_from_dcm_matches_basilisk():
    from Basilisk.utilities.RigidBodyKinematics import C2MRP
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(100):
        s = rng.normal(size=3) * 0.9
        R = mrp_to_dcm(s)
        worst = max(worst, np.linalg.norm(mrp_from_dcm(R) - np.asarray(C2MRP(R))))
    assert worst < 1e-12


def test_allocate_cached_matches_pinv():
    P = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)
    rng = np.random.default_rng(2)
    worst = 0.0
    for _ in range(20):
        tq = rng.normal(size=3) * 0.05
        worst = max(worst, np.abs(
            allocation_pinv(np.zeros(3), tq) - allocate_cached(P, np.zeros(3), tq)).max())
    assert worst < 1e-12


def test_quantize_levels():
    q = quantize_duty_cycle(np.array([0.0, 0.1, 0.49, 0.5, 0.9, 1.0]))
    assert np.allclose(q, [0.0, 1 / 6, 1 / 6, 5 / 6, 5 / 6, 1.0])


def test_batch_sun_angle_matches_loop():
    """Regression: batched sun-angle contraction must equal dcm @ sun per sample."""
    from Basilisk.utilities.RigidBodyKinematics import MRP2C
    from cubesat_12u_sim.heyoka_attitude import mrp_to_dcm_batch, sun_direction_at
    rng = np.random.default_rng(0)
    L, B = 37, 5
    SG = rng.normal(size=(L, B, 3)) * 0.3
    bore = np.array([1.0, 0.0, 0.0])
    sun_dirs = np.array([sun_direction_at(float(t)) for t in np.linspace(0, 300, L)])
    dcm_all = mrp_to_dcm_batch(SG.reshape(L * B, 3)).reshape(L, B, 3, 3)
    sun_b = np.einsum("lbij,lj->lbi", dcm_all, sun_dirs)
    sang = np.degrees(np.arccos(np.clip(np.einsum("lbj,j->lb", sun_b, bore), -1.0, 1.0)))
    worst = 0.0
    for l in range(L):
        for j in range(B):
            dcm = np.asarray(MRP2C(SG[l, j]), float)
            sb = dcm @ sun_dirs[l]
            ref = np.degrees(np.arccos(np.clip(float(bore @ sb), -1.0, 1.0)))
            worst = max(worst, abs(ref - sang[l, j]))
    assert worst < 1e-9


def test_jobs_parity():
    from cubesat_12u_sim.heyoka_mc import run_heyoka_monte_carlo
    mc = MonteCarloConfig()
    a = run_heyoka_monte_carlo(num_runs=4, base_seed=100, mc_cfg=mc,
                               verbose=False, n_jobs=1)
    b = run_heyoka_monte_carlo(num_runs=4, base_seed=100, mc_cfg=mc,
                               verbose=False, n_jobs=2)
    fa = [r["kpis"]["total_propellant_g"] for r in a["runs"]]
    fb = [r["kpis"]["total_propellant_g"] for r in b["runs"]]
    assert np.allclose(fa, fb, atol=0.0)
