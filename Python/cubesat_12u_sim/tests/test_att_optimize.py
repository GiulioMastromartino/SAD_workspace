"""Tests for the attitude-loop hypercube sweep.

The expensive properties are covered by tests that assert *structure* rather
than running a 300 s campaign: the design's reproducibility, the reduction
algebra, and the KPI parity against the reference implementation. One
end-to-end candidate is evaluated for real, at a small lane count, because the
hand-written ``run_chunk`` call is easy to get subtly wrong and a wrong
argument is silent.
"""
from __future__ import annotations

import numpy as np
import pytest

from cubesat_12u_sim.optimization import attplots  # noqa: F401  (import check)
from cubesat_12u_sim.optimization.attdesign import build_bundle
from cubesat_12u_sim.optimization.attobjective import (
    AttWeights, reduce_runs, run_scores)
from cubesat_12u_sim.uncertainty.monte_carlo import MonteCarloConfig


def _kpis(fuel, success=None, d_set=None, s_set=None, ss_max=None, n=None):
    n = n or len(fuel)
    fuel = np.asarray(fuel, float)
    return {"fuel": fuel,
            "d_set": np.zeros(n) if d_set is None else np.asarray(d_set, float),
            "s_set": np.zeros(n) if s_set is None else np.asarray(s_set, float),
            "ss_max": np.zeros(n) if ss_max is None else np.asarray(ss_max, float),
            "fin_rate": np.zeros(n), "fin_slew": np.zeros(n),
            "success": (np.ones(n, bool) if success is None
                        else np.asarray(success, bool))}


def _mc():
    return MonteCarloConfig()


# --------------------------------------------------------------------------
# design
# --------------------------------------------------------------------------

def test_design_is_reproducible():
    """Common random numbers: two builds with one seed are bit-identical.

    The whole cost function rests on this. If it broke, every candidate would
    be scored against different dispersions and CMA-ES would fit noise.
    """
    a = build_bundle(n_runs=10, base_seed=7)
    b = build_bundle(n_runs=10, base_seed=7)
    assert np.array_equal(a.sigma0, b.sigma0)
    assert np.array_equal(a.omega0, b.omega0)
    assert np.array_equal(a.mass, b.mass)
    assert np.array_equal(a.REFS, b.REFS)
    assert a.seeds == b.seeds


def test_design_seed_changes_the_ensemble():
    a = build_bundle(n_runs=10, base_seed=7)
    b = build_bundle(n_runs=10, base_seed=8)
    assert not np.array_equal(a.sigma0, b.sigma0)


def test_bundle_shapes_are_consistent():
    b = build_bundle(n_runs=12, base_seed=3)
    n = 12
    assert b.r0.shape == (n, 3)
    assert b.MF.shape == (n, 3, 12)
    assert b.MT.shape == (n, 3, 12)
    assert b.REFS.shape[1] == n
    assert b.PH.shape[0] == b.REFS.shape[0]
    assert len(b.seeds) == n
    assert len(b.disps) == n


# --------------------------------------------------------------------------
# space
# --------------------------------------------------------------------------

def test_space_covers_every_gain_exactly_once():
    from cubesat_12u_sim.optimization.attparams import ATT_SPACE

    assert ATT_SPACE.names == ["K", "P", "obs_gain", "obs_bias_gain",
                               "ctrl_scale"]
    p = ATT_SPACE.from_theta(ATT_SPACE.seed_theta())
    from cubesat_12u_sim.core import config as C
    assert p["K"] == pytest.approx(C.ACS_K)
    assert p["P"] == pytest.approx(C.ACS_P)
    assert p["obs_gain"] == pytest.approx(C.ACS_OBSERVER_GAIN)
    assert p["obs_bias_gain"] == pytest.approx(C.ACS_OBSERVER_BIAS_GAIN)
    assert p["ctrl_scale"] == pytest.approx(C.THRUSTER_CONTROL_SCALE)


# --------------------------------------------------------------------------
# reduction
# --------------------------------------------------------------------------

def test_reduction_penalises_a_worse_tail():
    """Same mean fuel, worse worst-decile must score worse."""
    w = AttWeights()
    n = 100
    tight = np.full(n, 10.0)
    flat = np.concatenate([np.full(90, 600.0 / 90.0), np.full(10, 40.0)])
    assert flat.mean() == pytest.approx(tight.mean())

    c_tight, st_t = reduce_runs(_kpis(tight), _mc(), w)
    c_flat, st_f = reduce_runs(_kpis(flat), _mc(), w)
    assert st_f.tail_hi > st_t.tail_hi
    assert c_flat > c_tight


def test_infeasible_runs_cannot_buy_a_cheap_tail():
    """Failed runs must cost more than a clean ensemble at the same fuel."""
    w = AttWeights()
    n = 100
    fuel = np.full(n, 10.0)
    c_clean, st_clean = reduce_runs(_kpis(fuel), _mc(), w)

    success = np.ones(n, bool)
    success[50:] = False
    cost, st = reduce_runs(_kpis(fuel, success=success), _mc(), w)

    assert st_clean.n_infeasible == 0
    assert st.n_infeasible == 50
    assert st.feasible_frac == pytest.approx(0.5)
    assert cost > c_clean
    assert cost - c_clean > 0.9 * w.w_infeasible * 0.5


def test_all_infeasible_is_the_flat_penalty():
    w = AttWeights()
    n = 40
    cost, st = reduce_runs(_kpis(np.full(n, 5.0),
                                 success=np.zeros(n, bool)), _mc(), w)
    assert st.feasible_frac == 0.0
    assert cost >= 1e9


def test_empty_input_does_not_raise():
    cost, st = reduce_runs(_kpis(np.zeros(0), n=0), _mc(), AttWeights())
    assert st.n_infeasible == 0
    assert cost >= 1e9


def test_time_term_prices_a_full_phase():
    """Settling at the phase end must cost w_time per phase used."""
    w = AttWeights()
    mc = _mc()
    n = 10
    instant = _kpis(np.full(n, 5.0), d_set=np.zeros(n), s_set=np.zeros(n))
    slow = _kpis(np.full(n, 5.0), d_set=np.full(n, mc.t_detumble),
                 s_set=np.full(n, mc.t_slew))
    c_fast, _ = reduce_runs(instant, mc, w)
    c_slow, _ = reduce_runs(slow, mc, w)
    # same fuel, same deciles, same feasibility: only the time term differs
    assert c_slow - c_fast == pytest.approx(2 * w.w_time)


def test_sun_hinge_is_zero_inside_the_cone():
    w = AttWeights()
    mc = _mc()
    n = 10
    inside = _kpis(np.full(n, 5.0),
                   ss_max=np.full(n, mc.sun_cone_angle_deg * 0.5))
    c_in, _ = reduce_runs(inside, mc, w)
    c_fuel, _ = reduce_runs(_kpis(np.full(n, 5.0)), mc, w)
    assert c_in == pytest.approx(c_fuel)


# --------------------------------------------------------------------------
# end to end (small, but real C++)
# --------------------------------------------------------------------------

@pytest.mark.slow
def test_candidate_evaluates_through_the_full_cpp_pipeline():
    """One real candidate, 8 lanes, through run_chunk.

    Exists to catch argument-list errors in the hand-written kernel call,
    which are otherwise silent: a misordered float still returns numbers, just
    the wrong ones.
    """
    pytest.importorskip("heyoka")
    from cubesat_12u_sim.optimization.attobjective import evaluate_candidate
    from cubesat_12u_sim.optimization.attparams import ATT_SPACE

    b = build_bundle(n_runs=8, base_seed=1)
    r = evaluate_candidate(ATT_SPACE.seed_theta(), b, MonteCarloConfig(),
                           dt=0.02, log_every=50, g0=9.80665, tol=1e-9,
                           quantize=True)
    assert np.isfinite(r.cost)
    assert r.stats.fuel.size == 8
    assert 0.0 <= r.stats.feasible_frac <= 1.0
    if r.stats.feasible_frac > 0:
        assert r.stats.tail_lo <= r.stats.tail_hi


@pytest.mark.slow
def test_kpis_match_the_reference_implementation():
    """Lean KPI vectors must equal ``_results_from_arrays`` per lane.

    The sweep path deliberately skips timeseries storage and per-run dicts.
    If a threshold, mask, or ref-table ever drifts from the reference, every
    cost the sweep computes is silently biased -- so this pins the two
    together. Same binary + same seeds => agreement to floating-point order.
    """
    pytest.importorskip("heyoka")
    import cubesat_12u_sim.optimization.attobjective as AO
    from cubesat_12u_sim.backends.heyoka import heyoka_mc as HM
    from cubesat_12u_sim.optimization.attparams import ATT_SPACE

    B, SEED = 8, 1
    mc = MonteCarloConfig()
    seeds = list(range(SEED, SEED + B))
    ref = HM._run_batch_cxx(seeds, mc, run_offset=0, quantize_thrusters=True)

    bundle = build_bundle(n_runs=B, base_seed=SEED, mc_cfg=mc,
                          task_step=0.02, log_step=1.0)
    r = AO.evaluate_candidate(ATT_SPACE.seed_theta(), bundle, mc,
                              dt=0.02, log_every=50, g0=9.80665, tol=1e-12,
                              quantize=True)
    assert r.stats.fuel.size == B
    # per-lane agreement against the reference dicts
    from cubesat_12u_sim.native import cxxdrive as _cxx
    from cubesat_12u_sim.core import config as C
    out = _cxx.run_chunk(
        np.ascontiguousarray(bundle.r0), np.ascontiguousarray(bundle.v0),
        np.ascontiguousarray(bundle.sigma0),
        np.ascontiguousarray(bundle.omega0),
        np.ascontiguousarray(bundle.mass),
        np.ascontiguousarray(bundle.I6),
        np.ascontiguousarray(bundle.invI6),
        np.ascontiguousarray(bundle.MF), np.ascontiguousarray(bundle.MT),
        np.ascontiguousarray(bundle.tmax), np.ascontiguousarray(bundle.isp),
        np.ascontiguousarray(bundle.gyro_bias),
        np.ascontiguousarray(bundle.gyro_arw),
        np.ascontiguousarray(np.asarray(bundle.seeds, dtype=np.uint64)),
        np.ascontiguousarray(bundle.P_nom),
        np.ascontiguousarray(bundle.REFS),
        np.ascontiguousarray(bundle.PH),
        np.ascontiguousarray(bundle.sun_dirs),
        np.ascontiguousarray(bundle.bore),
        float(C.ACS_K), float(C.ACS_P),
        float(C.ACS_OBSERVER_GAIN), float(C.ACS_OBSERVER_BIAS_GAIN),
        float(C.THRUSTER_CONTROL_SCALE), 0.02, bundle.n_steps, 50,
        float(C.G0), 0.1, 2.5, 15.0, True, 0, 1e-12, False,
        float(C.MU_EARTH), bool(C.USE_GRAVITY_GRADIENT))
    om = np.degrees(np.asarray(out["omega_norm"], float)).T
    se = np.asarray(out["sigma_err"], float).T
    sang = np.degrees(np.asarray(out["sun_angle"], float)).T
    pf = np.asarray(out["phase_fuel"], float)
    times = np.arange(om.shape[1]) * 50 * 0.02
    mine = AO._kpis(om, se, sang, pf, times, mc)
    worst = 0.0
    for j in range(B):
        k = ref[j]["kpis"]
        worst = max(worst,
                    abs(mine["d_set"][j] - k["detumble_settle_time_s"]),
                    abs(mine["s_set"][j] - k["slew_settle_time_s"]),
                    abs(mine["fuel"][j] - k["total_propellant_g"]))
        assert mine["success"][j] == ref[j]["success"]
    assert worst < 1e-9


@pytest.mark.slow
def test_same_theta_gives_identical_cost():
    """Determinism: the frozen ensemble must make the cost reproducible."""
    pytest.importorskip("heyoka")
    from cubesat_12u_sim.optimization.attobjective import evaluate_candidate
    from cubesat_12u_sim.optimization.attparams import ATT_SPACE

    b = build_bundle(n_runs=8, base_seed=1)
    kw = dict(dt=0.02, log_every=50, g0=9.80665, tol=1e-9, quantize=True)
    a = evaluate_candidate(ATT_SPACE.seed_theta(), b, MonteCarloConfig(), **kw)
    c = evaluate_candidate(ATT_SPACE.seed_theta(), b, MonteCarloConfig(), **kw)
    assert a.cost == pytest.approx(c.cost, rel=1e-12)


@pytest.mark.slow
def test_gains_change_the_result():
    """A different controller must produce a different ensemble outcome."""
    pytest.importorskip("heyoka")
    import math
    from cubesat_12u_sim.optimization.attobjective import evaluate_candidate
    from cubesat_12u_sim.optimization.attparams import ATT_SPACE

    b = build_bundle(n_runs=8, base_seed=1)
    kw = dict(dt=0.02, log_every=50, g0=9.80665, tol=1e-9, quantize=True)
    a = evaluate_candidate(ATT_SPACE.seed_theta(), b, MonteCarloConfig(), **kw)
    th = ATT_SPACE.seed_theta()
    th[0] = math.log(0.4)  # K x10
    d = evaluate_candidate(th, b, MonteCarloConfig(), **kw)
    assert a.cost != pytest.approx(d.cost, rel=1e-9)
