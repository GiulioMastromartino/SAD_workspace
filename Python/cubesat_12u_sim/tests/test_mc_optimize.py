"""Tests for the Monte-Carlo hypercube sweep.

The expensive properties are covered by tests that assert *structure* rather
than running a 250-run ensemble: the design's reproducibility, the reduction
algebra, the discrete horizon decoding, and the rank/abort semantics. One
end-to-end candidate is evaluated for real, at a reduced run count, because the
C++ argument list is easy to get subtly wrong and a wrong argument is silent.
"""
from __future__ import annotations

import numpy as np
import pytest

from cubesat_12u_sim.optimization import mcplots  # noqa: F401  (import check)
from cubesat_12u_sim.optimization.mcdesign import MCConfig, build_design
from cubesat_12u_sim.optimization.mcobjective import (TailWeights, est_nmpc_s,
                                         evaluate_candidate, reduce_arm,
                                         run_scores)


# --------------------------------------------------------------------------
# design
# --------------------------------------------------------------------------

def test_design_is_reproducible():
    """Common random numbers: two builds with one seed are bit-identical.

    This is the property the whole cost function rests on. If it broke, every
    candidate would be scored against different launches and CMA-ES would be
    fitting sampling noise.
    """
    a = build_design(MCConfig(n_runs=50, n_vehicles=5, seed=7))
    b = build_design(MCConfig(n_runs=50, n_vehicles=5, seed=7))
    assert np.array_equal(a.x0, b.x0)
    assert [v.as_dict() for v in a.vehicles] == [v.as_dict() for v in b.vehicles]


def test_design_seed_changes_the_ensemble():
    a = build_design(MCConfig(n_runs=50, n_vehicles=5, seed=7))
    b = build_design(MCConfig(n_runs=50, n_vehicles=5, seed=8))
    assert not np.array_equal(a.x0, b.x0)


def test_design_partition_is_exact():
    cfg = MCConfig(n_runs=50, n_vehicles=5)
    d = build_design(cfg)
    assert d.n_runs == 50
    assert len(d.vehicles) == 5
    # every run belongs to exactly one group, and groups are equal-sized
    idx = np.concatenate([d.group_runs(g) for g in range(5)])
    assert sorted(idx.tolist()) == list(range(50))
    for g in range(5):
        assert d.group_runs(g).size == cfg.n_injections


def test_design_rejects_bad_split():
    with pytest.raises(ValueError):
        build_design(MCConfig(n_runs=50, n_vehicles=7))
    with pytest.raises(ValueError):
        build_design(MCConfig(n_runs=10, n_vehicles=50))


def test_design_dispersions_are_ordered():
    """The nominal slow injection must dominate the repeatability spread."""
    d = build_design(MCConfig(n_runs=4000, n_vehicles=40, seed=3))
    dv = np.linalg.norm(d.x0[:, 3:6], axis=1) / 4657.0 - 1.0
    assert dv.mean() < -1e-3           # slow, not fast
    assert dv.std() > 0.5e-3           # and genuinely dispersed
    dr = np.linalg.norm(d.x0[:, 0:3], axis=1) / 18378137.0 - 1.0
    assert abs(dr.mean()) < 1e-4       # position error is zero-mean


# --------------------------------------------------------------------------
# reduction
# --------------------------------------------------------------------------

def test_run_scores_match_the_single_run_score():
    """Per-run cost must agree with objective.score on the same numbers."""
    from cubesat_12u_sim.optimization.objective import EvalConfig, score

    cfg = EvalConfig()
    rng = np.random.default_rng(0)
    dv = rng.uniform(1.0, 50.0, 8)
    box = rng.uniform(0.0, 60e3, 8)
    ea = rng.uniform(-20e3, 20e3, 8)
    got = run_scores(dv, box, ea, cfg.spec_box, cfg.w_viol, cfg.w_final)
    for i in range(8):
        assert got[i] == pytest.approx(score(dv[i], box[i], ea[i], cfg).cost)


def test_reduction_penalises_a_worse_tail():
    """Same mean, worse worst-decile must score worse.

    This is the entire reason the tails are in the cost: without them the two
    ensembles below are indistinguishable and the tail risk would be free.

    The flat ensemble is built to *match* the tight one's mean exactly: 90 runs
    at 6.667 plus 10 runs at 40 averages 10, the same as 100 runs at 10. A
    cheaper-looking body of runs pays for one ruinous tail.
    """
    w = TailWeights()
    n = 100
    box = np.zeros(n)
    ea = np.zeros(n)

    tight = np.full(n, 10.0)
    flat = np.concatenate([np.full(90, 600.0 / 90.0), np.full(10, 40.0)])
    assert flat.mean() == pytest.approx(tight.mean())

    c_tight, st_t = reduce_arm(tight, box, ea, 25e3, 1e3, 0.05, w)
    c_flat, st_f = reduce_arm(flat, box, ea, 25e3, 1e3, 0.05, w)
    assert st_f.tail_hi > st_t.tail_hi
    assert c_flat > c_tight


def test_reduction_rewards_consistency_over_variance():
    """A spread ensemble must not beat a tight one at equal mean.

    Guards the sign of the ``w_lo`` term. If it were added instead of
    subtracted, a bimodal ensemble would beat a tight one and the objective
    would be maximising variance.

    Both ensembles have mean 25.4; one is flat, the other splits into a cheap
    decile and an expensive body.
    """
    w = TailWeights()
    n = 100
    box = np.zeros(n)
    ea = np.zeros(n)

    consistent = np.full(n, 25.4)
    bimodal = np.concatenate([np.full(90, 28.0), np.full(10, 2.0)])
    assert bimodal.mean() == pytest.approx(consistent.mean())

    c_con, st_con = reduce_arm(consistent, box, ea, 25e3, 1e3, 0.05, w)
    c_bim, st_bim = reduce_arm(bimodal, box, ea, 25e3, 1e3, 0.05, w)

    assert st_bim.tail_lo < st_con.tail_lo      # the cheap decile
    assert st_bim.tail_hi > st_con.tail_hi      # the expensive body
    # the tight ensemble wins despite the spread one having the better floor
    assert c_con < c_bim


def test_infeasible_runs_cannot_buy_a_cheap_tail():
    """Half the runs outside the box must cost more than a clean ensemble.

    The clean reference is the same 100 runs with the same dv and no box
    violations, so the only difference is feasibility.
    """
    w = TailWeights()
    n = 100
    ea = np.zeros(n)
    dv = np.full(n, 10.0)

    c_clean, st_clean = reduce_arm(dv, np.zeros(n), ea, 25e3, 1e3, 0.05, w)

    box = np.zeros(n)
    box[50:] = 60e3          # 50 runs blow the 25 km box
    cost, st = reduce_arm(dv, box, ea, 25e3, 1e3, 0.05, w)

    assert st_clean.n_infeasible == 0
    assert st.n_infeasible == 50
    assert st.feasible_frac == pytest.approx(0.5)
    assert cost > c_clean
    # the gap is dominated by the infeasible penalty, not by dv
    assert cost - c_clean > 0.9 * w.w_infeasible * 0.5


def test_all_infeasible_is_the_flat_penalty():
    w = TailWeights()
    n = 40
    cost, st = reduce_arm(np.full(n, 5.0), np.full(n, 90e3), np.zeros(n),
                          25e3, 1e3, 0.05, w)
    assert st.feasible_frac == 0.0
    assert cost >= 1e9


def test_nonfinite_runs_are_infeasible_not_crashes():
    w = TailWeights()
    n = 20
    dv = np.full(n, np.inf)
    dv[:5] = 12.0
    cost, st = reduce_arm(dv, np.zeros(n), np.zeros(n), 25e3, 1e3, 0.05, w)
    assert np.isfinite(cost)
    assert st.feasible_frac == pytest.approx(0.25)


def test_empty_input_does_not_raise():
    cost, st = reduce_arm(np.zeros(0), np.zeros(0), np.zeros(0), 25e3, 1e3,
                          0.05, TailWeights())
    assert st.n_infeasible == 0
    assert cost >= 1e9


# --------------------------------------------------------------------------
# hypercube / horizon
# --------------------------------------------------------------------------

def test_horizon_is_a_discrete_coordinate():
    from cubesat_12u_sim.optimization.params import HYPER_SPACE, MC_HORIZONS

    p = HYPER_SPACE.from_theta(HYPER_SPACE.seed_theta())
    assert p["horizon"] in MC_HORIZONS
    # any continuous proposal decodes to a legal horizon
    for t in (-50.0, -1.0, 0.0, 1.0, 3.0, 10.0, 50.0):
        assert HYPER_SPACE.from_theta(
            np.full(len(HYPER_SPACE), t))["horizon"] in MC_HORIZONS


def test_horizon_is_capped_at_six():
    """The searched horizon set is capped, and nothing above H=6 is reachable.

    The cap is a cost/straggler decision documented in ``params``: H=8 and H=12
    cost several times an H=4 candidate and, because the horizon is a free
    coordinate in a population that is otherwise latency-bound, they strand the
    generation on whichever worker drew one.
    """
    from cubesat_12u_sim.optimization.params import (HYPER_SPACE, MC_HORIZONS,
                                       NMPC_HORIZONS)

    assert max(MC_HORIZONS) == 6
    assert set(MC_HORIZONS) <= set(NMPC_HORIZONS)
    # the staged sweep keeps the wider set: it is not latency-bound
    assert max(NMPC_HORIZONS) == 12

    lb, ub = HYPER_SPACE.bounds()
    lo_h, hi_h = float(np.exp(lb[-1])), float(np.exp(ub[-1]))
    assert (lo_h, hi_h) == (2.0, 6.0)
    # a proposal at the very top of the horizon's log range still snaps to <= 6
    assert HYPER_SPACE.from_theta(
        np.concatenate([np.zeros(len(HYPER_SPACE) - 1), [hi_h]]))["horizon"] <= 6


def test_hypercube_is_one_box_over_every_parameter():
    from cubesat_12u_sim.optimization.params import (HYPER_SPACE, NMPC_SPACE,
                                       PID_SPACE)

    names = HYPER_SPACE.names
    assert len(HYPER_SPACE) == len(NMPC_SPACE) + len(PID_SPACE) + 1
    assert set(NMPC_SPACE.names) | set(PID_SPACE.names) | {"horizon"} == set(names)
    assert len(set(names)) == len(names)      # no duplicated coordinate


def test_horizon_estimate_is_monotone():
    prev = 0.0
    for h in (2, 4, 6, 8, 12):
        v = est_nmpc_s(h, 250)
        assert v > prev
        prev = v


def test_over_budget_horizon_is_rejected_not_simulated():
    from cubesat_12u_sim.optimization.params import HYPER_SPACE

    d = build_design(MCConfig(n_runs=10, n_vehicles=5))
    th = HYPER_SPACE.seed_theta()
    th[-1] = np.log(6)                        # the most expensive allowed choice
    r = evaluate_candidate(th, d, dt_outer=600.0, dt_inner=60.0, t_end=43200.0,
                           spec_box=25e3, w_viol=1e3, w_final=0.05, maxiter=50,
                           max_candidate_s=0.0)
    assert r.cost >= 1e9
    assert r.params.get("_rejected")


# --------------------------------------------------------------------------
# end to end (reduced, but real C++)
# --------------------------------------------------------------------------

@pytest.mark.slow
def test_candidate_evaluates_through_the_full_cpp_pipeline():
    """One real candidate, 20 runs, both arms, through the C++ kernels.

    Exists to catch argument-list errors in the hand-written kernel calls,
    which are otherwise silent: a misordered float still returns numbers, just
    the wrong ones.
    """
    pytest.importorskip("heyoka")
    from cubesat_12u_sim.optimization.params import HYPER_SPACE

    d = build_design(MCConfig(n_runs=20, n_vehicles=5, seed=1))
    r = evaluate_candidate(HYPER_SPACE.seed_theta(), d, dt_outer=600.0,
                           dt_inner=60.0, t_end=0.02 * 86400.0, spec_box=25e3,
                           w_viol=1e3, w_final=0.05, maxiter=20)
    assert np.isfinite(r.cost)
    assert r.nmpc.dv.size == 20
    # both arms produced finite dv for at least some runs
    assert np.isfinite(r.nmpc.dv_mean) or r.nmpc.feasible_frac == 0.0
    assert 0.0 <= r.pid.feasible_frac <= 1.0
    # the reported decile means must actually bound the feasible runs
    if r.nmpc.feasible_frac > 0:
        assert r.nmpc.tail_lo <= r.nmpc.tail_hi


@pytest.mark.slow
def test_same_theta_gives_identical_cost():
    """Determinism: common random numbers must make the cost reproducible."""
    pytest.importorskip("heyoka")
    from cubesat_12u_sim.optimization.params import HYPER_SPACE

    d = build_design(MCConfig(n_runs=20, n_vehicles=5, seed=1))
    kw = dict(dt_outer=600.0, dt_inner=60.0, t_end=0.02 * 86400.0,
              spec_box=25e3, w_viol=1e3, w_final=0.05, maxiter=20)
    a = evaluate_candidate(HYPER_SPACE.seed_theta(), d, **kw)
    b = evaluate_candidate(HYPER_SPACE.seed_theta(), d, **kw)
    assert a.cost == pytest.approx(b.cost, rel=1e-12)
    assert a.nmpc.dv_mean == pytest.approx(b.nmpc.dv_mean, rel=1e-12)


@pytest.mark.slow
def test_evaluate_candidate_actually_runs_both_arms(capsys):
    """Neither arm may silently fall through to the infeasible penalty.

    Regression guard. ``evaluate_candidate`` wraps each arm's evaluation in a
    bare ``except: pass`` so one bad vehicle group cannot abort a generation --
    which is right, but it also means a systematic wiring error (a missing
    keyword argument, a renamed space entry) shows up only as "every candidate
    is infeasible", with the NameError buried. That happened once: ``w_track``
    was added to ``_nmpc_group`` but not to its caller, and the sweep reported a
    flat 1e9 for the whole population.

    So this turns the swallow on (``_TRACE``) and asserts no traceback appears.
    An arm that runs and is merely *infeasible* is correct behaviour and must not
    trip this: the PID arm at space defaults really does leave the box, so
    "finite dv" is not a usable assertion here.
    """
    pytest.importorskip("heyoka")
    import cubesat_12u_sim.optimization.mcobjective as M
    from cubesat_12u_sim.optimization.params import HYPER_SPACE

    M._TRACE = True
    try:
        d = build_design(MCConfig(n_runs=20, n_vehicles=5, seed=1))
        r = evaluate_candidate(HYPER_SPACE.seed_theta(), d, dt_outer=600.0,
                               dt_inner=60.0, t_end=0.5 * 86400.0,
                               spec_box=25e3, w_viol=1e3, w_final=0.05,
                               maxiter=50, tol=1e-12)
    finally:
        M._TRACE = False
    cap = capsys.readouterr()
    assert "Traceback" not in cap.out + cap.err, (
        "an arm raised and was swallowed; see the traceback above")

    # Exactly HOW feasible the NMPC arm is at the seed is not a contract: the
    # fuel price moves the seed's operating point, so a seed that was 95%
    # feasible at one scale can be 50% at another. What matters is that it is
    # not all-infeasible -- an all-infeasible seed means every candidate in
    # generation 1 sits at the same penalty constant and CMA-ES has no gradient
    # to descend.
    assert np.all(np.isfinite(r.nmpc.dv))
    assert 0.0 < r.nmpc.feasible_frac
    # The PID arm at defaults leaves the 25 km box, so it may legitimately be
    # fully infeasible; what matters is that the verdict came from evaluating,
    # not from failing to evaluate.
    assert r.pid.n_infeasible in (0, 20)


@pytest.mark.slow
def test_vehicle_dispersion_changes_the_result():
    """A different frozen design must produce a different ensemble outcome."""
    pytest.importorskip("heyoka")
    from cubesat_12u_sim.optimization.params import HYPER_SPACE

    kw = dict(dt_outer=600.0, dt_inner=60.0, t_end=0.02 * 86400.0,
              spec_box=25e3, w_viol=1e3, w_final=0.05, maxiter=20)
    th = HYPER_SPACE.seed_theta()
    a = evaluate_candidate(th, build_design(MCConfig(n_runs=20, n_vehicles=5,
                                                     seed=1)), **kw)
    b = evaluate_candidate(th, build_design(MCConfig(n_runs=20, n_vehicles=5,
                                                     seed=99)), **kw)
    assert a.cost != pytest.approx(b.cost, rel=1e-9)

def test_stagnation_detector_separates_descent_from_plateau():
    """The restart trigger must not fire on a still-descending sweep.

    This is the guard on the new force-restart path: a false positive burns a
    doubled population for nothing, a false negative lets a basin-bound sweep
    run to its full budget.
    """
    from cubesat_12u_sim.tuning.run_mc_optimize import _stagnant

    def mk(costs, feas=1.0):
        return [{"best_cost": c,
                 "best": {"nmpc": {"feasible_frac": feas}}} for c in costs]

    # a genuine geometric ramp, so the second half IS an improvement over the
    # first. (A sequence that descends and then repeats is flat by construction
    # and must read as stagnant -- that is the case below.)
    descending = mk([400.0 / (1.25 ** i) for i in range(16)])
    assert _stagnant(descending, 8, 0.02)[0] is False

    plateau = mk([50, 48, 47, 47.2, 47.1, 47.0, 47.05, 47.0] * 2)
    assert _stagnant(plateau, 8, 0.02)[0] is True

    # all-infeasible: cost is flat AND there is nothing feasible to improve
    stuck = mk([2e9] * 20, feas=0.0)
    assert _stagnant(stuck, 8, 0.02)[0] is True

    # not enough history to judge -- must not fire
    assert _stagnant(mk([100, 90, 80, 70]), 8, 0.02)[0] is False
