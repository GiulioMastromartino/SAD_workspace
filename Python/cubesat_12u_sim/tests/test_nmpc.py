import numpy as np

from cubesat_12u_sim import config as C
from cubesat_12u_sim.nmpc import HighLevelNMPC, NMPCConfig, ScipyShootingBackend
from cubesat_12u_sim.hierarchical import FastTrimPID, HierarchicalController
from cubesat_12u_sim.actuators import allocation_pinv, jets_to_wrench


def _circ_state(dv_frac: float = -0.005):
    r = C.R0_ECI.copy()
    v = C.V0_ECI.copy() * (1.0 + dv_frac)
    return r, v


def test_nmpc_output_respects_caps():
    cfg = NMPCConfig(dt_outer=10.0, horizon=3, maxiter=20)
    nmpc = HighLevelNMPC(cfg, ScipyShootingBackend())
    r, v = _circ_state()
    a = nmpc.accel_command(r, v, 10.0)
    assert a.shape == (3,)
    assert np.all(np.abs(a) <= cfg.a_max + 1e-12)
    assert np.all(np.isfinite(a))


def test_nmpc_solver_failure_fallback_safe():
    class Boom(ScipyShootingBackend):
        def solve(self, x0, cfg):
            raise RuntimeError("boom")

    nmpc = HighLevelNMPC(NMPCConfig(dt_outer=10.0, horizon=2, fallback="zero"), Boom())
    r, v = _circ_state()
    a = nmpc.accel_command(r, v, 10.0)
    assert np.allclose(a, 0.0)
    assert nmpc.stats["fallbacks"] == 1


def test_trim_budget_latches_off():
    trim = FastTrimPID(authority=0.005, dv_budget=0.01)
    r, v = _circ_state()
    spent = []
    for _ in range(50):
        a = trim.accel_command(r, v, 1.0)
        spent.append(float(np.linalg.norm(a)))
    # budget 0.01 m/s at ~1e-3 m/s^2 levels exhausts quickly, then coasts at 0
    assert spent[-1] == 0.0
    assert trim.dv_spent_window >= trim.dv_budget - 1e-12


def test_hierarchical_allocates_feasible_wrench():
    ctrl = HierarchicalController(NMPCConfig(dt_outer=10.0, horizon=2, maxiter=20),
                                  FastTrimPID())
    r, v = _circ_state()
    euler = np.radians([5.0, 2.0, 0.0])
    u, F, tau = ctrl.throttle_command(r, v, euler, 1.0, np.zeros(3))
    assert u.shape == (12,)
    assert np.all(u >= 0.0) and np.all(u <= 1.0)
    F2, t2 = jets_to_wrench(u)
    assert np.allclose(F, F2) and np.allclose(tau, t2)


def test_backend_swappable_interface():
    from cubesat_12u_sim.nmpc import SolverBackend, NMPCSolution

    class Dummy(SolverBackend):
        def solve(self, x0, cfg):
            n = cfg.horizon
            return NMPCSolution(np.zeros(3), np.zeros(3),
                                np.zeros((n, 3)), np.zeros((n + 1, 6)),
                                0.0, True, {})

    ctrl = HierarchicalController(NMPCConfig(dt_outer=10.0, horizon=2), FastTrimPID())
    ctrl.nmpc.backend = Dummy()  # drop-in replace (future casadi/acados path)
    r, v = _circ_state()
    a = ctrl.accel_command(r, v, 1.0)
    assert a.shape == (3,)


def test_energy_metric_constant_on_coast():
    from cubesat_12u_sim.nmpc import semi_major_axis, _rk4_step_orbit
    r, v = C.R0_ECI.copy(), C.V0_ECI.copy() * 0.995
    a0 = semi_major_axis(r, v)
    x = np.concatenate([r, v])
    for _ in range(10):
        x = _rk4_step_orbit(x, np.zeros(3), 60.0)
    a1 = semi_major_axis(x[0:3], x[3:6])
    assert abs(a1 - a0) < 1.0  # energy conserved ballistically (m-level)
    # radius DOES oscillate on the same arc (why radius-cost burns forever)
    assert abs(float(np.linalg.norm(x[0:3])) - float(np.linalg.norm(r))) > 10.0


def test_deadband_coasts_when_recovered():
    from cubesat_12u_sim.nmpc import SolverBackend, NMPCSolution, semi_major_axis
    r, v = C.R0_ECI.copy(), C.V0_ECI.copy()  # already circular: e_a ~ 0
    assert abs(semi_major_axis(r, v) - C.R_ORBIT) < C.NMPC_DB_A
    nmpc = HighLevelNMPC(NMPCConfig(dt_outer=600.0, horizon=6, fallback="zero"))
    a = nmpc._fallback_accel(r, v)
    assert np.allclose(a, 0.0)  # fallback itself must coast inside deadband


def test_explicit_plan_no_double_counter():
    from cubesat_12u_sim.nmpc import SolverBackend, NMPCSolution
    calls = []

    class Count(SolverBackend):
        def solve(self, x0, cfg):
            calls.append(1)
            n = cfg.horizon
            return NMPCSolution(np.full(3, 1e-3), np.zeros(3),
                                np.zeros((n, 3)), np.zeros((n + 1, 6)),
                                0.0, True, {})

    ctrl = HierarchicalController(NMPCConfig(dt_outer=10.0, horizon=2), FastTrimPID())
    ctrl.nmpc.backend = Count()
    r, v = _circ_state()
    for _ in range(25):  # 25 x 1 s inner ticks = 2.5 outer windows
        ctrl.accel_command(r, v, 1.0)
    assert len(calls) == 3  # ticks 0, 10, 20 — exactly one solve per outer window
