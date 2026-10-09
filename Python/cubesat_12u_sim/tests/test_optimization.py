"""Tests for the optimisation harness (not the controllers it tunes)."""
import numpy as np
import pytest

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.optimization import objective as O
from cubesat_12u_sim.optimization.driver import rank_key, multistart
from cubesat_12u_sim.optimization.params import (NMPC_SPACE, PID_SPACE,
                                                 JOINT_SPACE, Param, ParamSpace)
from cubesat_12u_sim.optimization.validate import propellant_g


# --- parameter spaces ------------------------------------------------------

def test_space_roundtrips_through_log_space():
    space = PID_SPACE
    rng = np.random.default_rng(0)
    for _ in range(20):
        theta = space.random_theta(rng)
        params = space.from_theta(theta)
        assert np.allclose(space.to_theta(params), theta)


def test_space_decoding_respects_bounds():
    space = PID_SPACE
    lb, ub = space.bounds()
    # far-outside inputs clamp instead of raising or escaping the bounds
    params = space.from_theta(np.full(len(space), 1e6))
    assert all(np.isfinite(v) and v > 0 for v in params.values())
    assert np.allclose(space.to_theta(params), ub)
    assert np.allclose(space.from_theta(np.full(len(space), -1e6))["kp_r"],
                       float(np.exp(lb[0])))


def test_space_seed_is_inside_bounds():
    for space in (PID_SPACE, NMPC_SPACE, JOINT_SPACE):
        lb, ub = space.bounds()
        s = space.seed_theta()
        assert np.all(s >= lb) and np.all(s <= ub)


def test_param_rejects_out_of_range_default():
    with pytest.raises(ValueError):
        Param("bad", -1.0, 1.0, 5.0)


def test_joint_space_is_concatenation():
    assert len(JOINT_SPACE) == len(NMPC_SPACE) + len(PID_SPACE)
    assert JOINT_SPACE.names[:len(NMPC_SPACE)] == NMPC_SPACE.names


# --- objective -------------------------------------------------------------

def test_measured_alloc_efficiency_matches_configured_default():
    """The surrogate's fidelity constant must be the real allocator's value."""
    assert abs(O.measure_alloc_eff() - O.SEARCH_CFG.alloc_eff) < 1e-6
    assert O.SEARCH_CFG.alloc_eff < 1.0, "allocator cannot exceed commanded"


def test_surrogate_matches_full_simulation():
    """Surrogate vs the real heyoka+thruster path, same dt and horizon.

    Guards the alloc_eff correction: without it the surrogate reports roughly
    half the dv and a much tighter box than the vehicle actually achieves.
    """
    from cubesat_12u_sim.optimization.validate import validate_pid
    cfg = O.EvalConfig(t_days=2.0 / 24.0, dt_outer=10.0, dt_inner=1.0)
    res = O.evaluate_pid(PID_SPACE.seed_theta(), cfg)
    full = validate_pid(PID_SPACE.seed_theta(), PID_SPACE,
                        t_end=2.0 / 24.0 * 86400.0, dt_outer=10.0, dt_inner=1.0)
    assert abs(res.dv - full["dv"]) / max(full["dv"], 1e-9) < 0.02
    assert abs(res.max_abs_r_err - full["max_abs_r_err"]) < 5e3


def test_score_penalises_box_violation_but_keeps_feasible_cheaper():
    cfg = O.SEARCH_CFG
    inside = O.score(100.0, cfg.spec_box * 0.9, 0.0, cfg)
    outside = O.score(1.0, cfg.spec_box * 2.0, 0.0, cfg)
    assert inside.feasible and not outside.feasible
    # a much cheaper but infeasible candidate must still lose on rank
    assert rank_key(inside) < rank_key(outside)
    assert "box exceeded" in outside.notes


def test_score_penalty_monotone_in_excursion():
    cfg = O.SEARCH_CFG
    dv = 10.0
    costs = [O.score(dv, cfg.spec_box * f, 0.0, cfg).cost
             for f in (1.0, 1.1, 1.5, 2.0)]
    assert costs == sorted(costs)


def test_nonfinite_evaluation_is_ranked_last():
    bad = O.score(float("nan"), 0.0, 0.0, O.SEARCH_CFG)
    good = O.score(50.0, 100.0, 0.0, O.SEARCH_CFG)
    assert not bad.feasible and bad.cost >= O.PENALTY_INFEASIBLE
    assert rank_key(good) < rank_key(bad)


def test_simulation_rejects_nonfinite_control():
    cfg = O.EvalConfig(t_days=0.01, dt_outer=60.0, dt_inner=60.0)
    x, dv, mx = O.simulate(lambda r, v, dt: np.full(3, np.nan), cfg)
    assert not np.isfinite(dv) and not np.isfinite(mx)


def test_coasting_controller_has_zero_dv():
    cfg = O.EvalConfig(t_days=0.01, dt_outer=60.0, dt_inner=60.0)
    _, dv, _ = O.simulate(lambda r, v, dt: np.zeros(3), cfg)
    assert dv == 0.0


def test_cache_returns_identical_results():
    cfg = O.EvalConfig(t_days=0.02, dt_outer=60.0, dt_inner=60.0)
    cache = {}
    theta = PID_SPACE.seed_theta()
    a = O.evaluate_pid(theta, cfg, cache)
    n_after_first = len(cache)
    b = O.evaluate_pid(theta, cfg, cache)
    assert len(cache) == n_after_first, "second call should hit the cache"
    assert a.as_dict() == b.as_dict()


def test_evaluation_is_deterministic():
    cfg = O.EvalConfig(t_days=0.02, dt_outer=60.0, dt_inner=60.0)
    th = PID_SPACE.seed_theta()
    assert (O.evaluate_pid(th, cfg).as_dict()
            == O.evaluate_pid(th, cfg).as_dict())


# --- controllers under optimisation ---------------------------------------

def test_tuned_pid_matches_stock_orbit_pid_control_law():
    """Instance-local gains must not change the law itself."""
    from cubesat_12u_sim.control.controller import OrbitPID
    gains = PID_SPACE.from_theta(PID_SPACE.seed_theta())
    tuned = O.make_pid(dict(gains))
    stock = OrbitPID(box_r=gains["box_r"])
    r = C.R0_ECI.copy() * (1.0 + 1e-4)
    v = C.V0_ECI.copy() * 0.995
    for ctrl in (tuned, stock):
        ctrl.reset()
    a_t, a_s = tuned.accel_command(r, v, 10.0), stock.accel_command(r, v, 10.0)
    assert np.allclose(a_t, a_s)


def test_tuned_pid_gains_do_not_leak_into_config():
    from cubesat_12u_sim.control.controller import OrbitPID
    before = (C.PID_KP_R, C.PID_KD_R)
    theta = PID_SPACE.random_theta(np.random.default_rng(3))
    O.evaluate_pid(theta, O.EvalConfig(t_days=0.01, dt_outer=10.0, dt_inner=10.0))
    assert (C.PID_KP_R, C.PID_KD_R) == before


def test_pid_box_coasts_and_leaks_integrators():
    gains = PID_SPACE.from_theta(PID_SPACE.seed_theta())
    ctrl = O.make_pid(dict(gains, box_r=1e6))  # wider than any excursion
    ctrl.int_r = 100.0
    a = ctrl.accel_command(C.R0_ECI.copy(), C.V0_ECI.copy() * 0.995, 10.0)
    assert np.allclose(a, 0.0)
    assert ctrl.int_r == pytest.approx(90.0)


# --- driver ----------------------------------------------------------------

def test_multistart_ranks_feasible_above_infeasible():
    cfg = O.EvalConfig(t_days=0.01, dt_outer=60.0, dt_inner=60.0)
    best, reports = multistart(
        PID_SPACE, "pid", cfg, horizon=0, restarts=1, seed=1, maxiter=2,
        workers=1, popsize=4, verbose=False)
    assert len(reports) == 1
    assert rank_key(best.result) == min(rank_key(r.result) for r in reports)


def test_checkpoint_roundtrip_and_resume(tmp_path):
    import json
    from cubesat_12u_sim.optimization.driver import multistart
    cfg = O.EvalConfig(t_days=0.01, dt_outer=60.0, dt_inner=60.0)
    ckpt = str(tmp_path / "ckpt.jsonl")
    best, reports = multistart(
        PID_SPACE, "pid", cfg, horizon=0, restarts=2, seed=1, maxiter=2,
        workers=1, popsize=4, verbose=False, checkpoint_path=ckpt)
    assert len(reports) == 2
    lines = [l for l in open(ckpt) if l.strip()]
    assert len(lines) == 2
    assert json.loads(lines[0])["restart"] == 0
    # resume: skip 1, reload restart 0 from file, run restart 1 fresh
    best2, reports2 = multistart(
        PID_SPACE, "pid", cfg, horizon=0, restarts=2, seed=1, maxiter=2,
        workers=1, popsize=4, verbose=False, checkpoint_path=ckpt,
        skip_completed=1)
    assert len(reports2) == 2
    assert np.allclose(reports2[0].theta, reports[0].theta)
    assert reports2[0].result.as_dict() == reports[0].result.as_dict()


def test_pid_cpp_batch_matches_numpy():
    """Parity gate for the C++ PID surrogate (Phase 1a).

    Seed + random + bound-adversarial thetas must agree to 1e-6 relative;
    measured worst is ~2e-9.
    """
    pidbatch = pytest.importorskip("cubesat_12u_sim.optimization.pidbatch")
    if not pidbatch.available:
        pytest.skip("no _pidbatch extension")
    cfg = O.EvalConfig(t_days=0.1, dt_outer=10.0, dt_inner=10.0)
    rng = np.random.default_rng(7)
    lb, ub = PID_SPACE.bounds()
    thetas = [PID_SPACE.seed_theta(), PID_SPACE.random_theta(rng), lb, ub]
    got = pidbatch.evaluate_pid_batch(thetas, cfg, PID_SPACE)
    for t, r in zip(thetas, got):
        ref = O.evaluate_pid(t, cfg)
        for k in ("dv", "max_abs_r_err", "final_e_a", "cost"):
            a, b = getattr(r, k), getattr(ref, k)
            assert abs(a - b) <= 1e-6 * max(abs(b), 1e-9), (k, a, b)


def test_nmpc_cpp_cost_matches_python():
    """Parity gate for the Heyoka C++ shooting cost (Phase 1b).

    Taylor prediction is more accurate than the 600 s RK4 step, so exact
    agreement is not expected: gate is 1e-3 relative (measured ~3e-4).
    """
    nb = pytest.importorskip("cubesat_12u_sim.optimization.nmpcbatch")
    if not nb.available:
        pytest.skip("no _nmpcbatch extension")
    from cubesat_12u_sim.control.nmpc import (NMPCConfig, _rk4_step_orbit,
                                      semi_major_axis)
    r, v = C.R0_ECI.copy(), C.V0_ECI.copy() * 0.995
    x0 = np.concatenate([r, v])
    cfg = NMPCConfig(dt_outer=600.0, horizon=6, maxiter=50)

    def py_cost(U):
        """Longhand reference for the C++ kernel's cost.

        Written out from the cost definition rather than by calling adjoint's
        helpers, so it is an independent expression: fuel is quadratic below
        the knee and exponential above, position is radius-error tracking that
        is linear inside ``box_frac`` and logarithmic beyond.
        """
        N, dt = 6, 600.0
        traj = np.zeros((N + 1, 6))
        traj[0] = x0
        for k in range(N):
            traj[k + 1] = _rk4_step_orbit(traj[k], U[k], dt)
        knee = float(__import__("cubesat_12u_sim.control.adjoint",
                                fromlist=["FUEL_KNEE"]).FUEL_KNEE)
        g1 = (1.0 if 1.0 <= knee else knee * knee * np.exp(1.0 - knee))
        A_fuel = cfg.a_max * dt
        s = cfg.w_track / (cfg.w_track_frac *
                           (1.0 + np.log(1.0 / cfg.w_track_frac)))
        c = 0.0
        for k in range(1, N + 1):
            rk = traj[k, 0:3]
            xf = float(np.linalg.norm(U[k - 1])) / cfg.a_max
            g = (xf * xf if xf <= knee else knee * knee * np.exp(xf - knee))
            c += A_fuel * g / g1
            xr = abs(float(np.linalg.norm(rk)) - C.R_ORBIT) / cfg.box_ref
            c += (s * xr if xr <= cfg.w_track_frac
                  else s * cfg.w_track_frac *
                  (1.0 + np.log(xr / cfg.w_track_frac)))
        aT = semi_major_axis(traj[N, 0:3], traj[N, 3:6])
        e_term = (aT - C.R_ORBIT) / 50e3
        if abs(aT - C.R_ORBIT) < cfg.db_a:
            e_term = 0.0
        return c + cfg.w_final * e_term**2

    cost, _, _, _ = nb.make_funcs(x0, cfg)
    rng = np.random.default_rng(0)
    for U in (np.zeros((6, 3)), rng.normal(0, 0.02, (6, 3))):
        ref, got = py_cost(U), cost(U.ravel())
        assert abs(got - ref) <= 1e-3 * max(abs(ref), 1e-9), (got, ref)


def test_nmpc_cpp_grad_matches_python_finite_diff():
    """The jac handed to SLSQP must match finite differences of the cost."""
    nb = pytest.importorskip("cubesat_12u_sim.optimization.nmpcbatch")
    if not nb.available:
        pytest.skip("no _nmpcbatch extension")
    from cubesat_12u_sim.control.nmpc import NMPCConfig
    r, v = C.R0_ECI.copy(), C.V0_ECI.copy() * 0.995
    x0 = np.concatenate([r, v])
    cfg = NMPCConfig(dt_outer=600.0, horizon=3, maxiter=50)
    cost, jac, _, _ = nb.make_funcs(x0, cfg)
    rng = np.random.default_rng(1)
    z = rng.normal(0, 0.01, 9)
    g = jac(z)
    h = 1e-6
    num = np.array([(cost(z + h * e) - cost(z - h * e)) / (2 * h)
                    for e in np.eye(9)])
    # deadband kinks make a few components non-smooth; check the smooth bulk
    err = np.abs(g - num) / np.maximum(np.abs(num), 1e-3)
    assert np.median(err) < 5e-2, err
    assert np.all(np.isfinite(g))


def test_nmpc_cxx_closed_loop_agrees():
    """Full-C++ sim gate: same verdict, dv within 5% vs numpy path."""
    from cubesat_12u_sim.optimization import nmpcsolve as NS
    if not NS.available:
        pytest.skip("no _nmpcsolve extension")
    cfg = O.EvalConfig()
    th = NMPC_SPACE.seed_theta()
    ref = O.evaluate_nmpc(th, cfg, 6)
    dv, mx, ea, info = NS.closed_loop(th, cfg, 6, NMPC_SPACE)
    got = O.score(dv, mx, ea, cfg)
    assert got.feasible == ref.feasible
    assert abs(got.dv - ref.dv) <= 0.05 * max(abs(ref.dv), 1e-9), \
        (got.dv, ref.dv)
    assert not info["aborted"]


def test_nmpc_cxx_early_abort_only_when_dominated():
    """Abort fires on j_star=0 (nothing beats it); off at j_star=inf."""
    from cubesat_12u_sim.optimization import nmpcsolve as NS
    if not NS.available:
        pytest.skip("no _nmpcsolve extension")
    cfg = O.EvalConfig(t_days=0.1, dt_outer=600.0, dt_inner=60.0)
    th = NMPC_SPACE.seed_theta()
    dv, mx, ea, info = NS.closed_loop(th, cfg, 6, NMPC_SPACE,
                                      j_star=0.0, has_incumbent=True)
    assert info["aborted"]
    assert not np.isfinite(dv) or dv >= 1e299
    dv2, _, _, info2 = NS.closed_loop(th, cfg, 6, NMPC_SPACE)
    assert not info2["aborted"] and np.isfinite(dv2)


def test_nmpc_stacked_normviol_matches_lambdas():
    """One C++ normviol call == N per-stage Python norms + analytic rows."""
    nb = pytest.importorskip("cubesat_12u_sim.optimization.nmpcbatch")
    if not nb.available:
        pytest.skip("no _nmpcbatch extension")
    import heyoka  # noqa: F401  (libheyoka for dynamic_lookup)
    from cubesat_12u_sim.native.cxxdrive import _nmpcbatch
    rng = np.random.default_rng(2)
    N, B = 6, 3
    Z = np.ascontiguousarray(rng.normal(0, 0.03, (B, 3 * N)))
    vals = np.zeros((B, N))
    jac = np.zeros((B, N, 3 * N))
    _nmpcbatch.normviol_batch(Z, N, 0.05, vals, jac)
    for b in range(B):
        for k in range(N):
            u = Z[b, 3 * k:3 * k + 3]
            assert abs(vals[b, k] - np.linalg.norm(u)) < 1e-12
            nk = np.linalg.norm(u)
            row = np.zeros(3 * N)
            if nk > 1e-12:
                row[3 * k:3 * k + 3] = u / nk
            assert np.allclose(jac[b, k], row)


def test_nmpc_cpp_solve_agrees_and_caps():
    """Closed solve: same verdict, dv within 5%, norm cap respected."""
    from cubesat_12u_sim.control.nmpc import NMPCConfig, ScipyShootingBackend
    pytest.importorskip("cubesat_12u_sim.optimization.nmpcbatch",
                        reason="no _nmpcbatch extension")
    from cubesat_12u_sim.optimization import nmpcbatch as nb
    if not nb.available:
        pytest.skip("no _nmpcbatch extension")
    r, v = C.R0_ECI.copy(), C.V0_ECI.copy() * 0.995
    x0 = np.concatenate([r, v])
    cfg = NMPCConfig(dt_outer=600.0, horizon=4, maxiter=30)
    s1 = ScipyShootingBackend(use_cpp=False).solve(x0, cfg)
    s2 = ScipyShootingBackend(use_cpp=True).solve(x0, cfg)
    assert s1.success and s2.success
    assert abs(s2.cost - s1.cost) <= 0.05 * max(abs(s1.cost), 1e-9)
    assert np.all(np.linalg.norm(s2.u_seq, axis=1) <= cfg.a_max + 1e-9)


def test_multistart_survives_a_raising_restart(monkeypatch):
    import cubesat_12u_sim.optimization.driver as drv

    def boom(*a, **k):
        raise RuntimeError("solver exploded")

    monkeypatch.setattr(drv, "run_restart", boom)
    with pytest.raises(RuntimeError, match="every CMA-ES restart failed"):
        drv.multistart(PID_SPACE, "pid", O.SEARCH_CFG, 0, restarts=2, seed=1,
                       maxiter=1, workers=1, verbose=False)


# --- validation ------------------------------------------------------------

def test_propellant_is_zero_for_zero_dv_and_monotone():
    assert propellant_g(0.0) == pytest.approx(0.0)
    assert propellant_g(10.0) < propellant_g(20.0)
    # rocket equation closed form for 20 kg at Isp 220 s
    import math
    expected = C.MASS * (1 - math.exp(-15.0 / (C.ISP_JET * C.G0))) * 1e3
    assert propellant_g(15.0) == pytest.approx(expected, rel=1e-9)
