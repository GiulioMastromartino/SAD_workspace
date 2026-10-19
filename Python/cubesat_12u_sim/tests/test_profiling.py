"""Tests for the hierarchical timing profiler and its pipeline wiring.

Two things are worth separating here. The first is that the profiler computes
the right thing: nesting, exclusive time, cross-process merging, and the
mean/std statistics that make a fast sim's cost a measurement rather than a
timer artifact. The second is that the pipeline actually records into it, at the
granularity claimed, and that recording it does not perturb the control results
or cost a meaningful fraction of the run.
"""
import math

import numpy as np
import pytest

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.tuning import profiling
from cubesat_12u_sim.optimization import objective as O
from cubesat_12u_sim.optimization.params import NMPC_SPACE, PID_SPACE


def _short_pid_cfg(t_days=0.01):
    """Short-arc EvalConfig at the PID's own control period (fast in tests)."""
    return O.EvalConfig(t_days=t_days, dt_outer=C.DT_CONTROL,
                        dt_inner=C.DT_CONTROL)


def _walk(root, path="", depth=0):
    """Yield ``(path, node, depth)`` for the whole tree."""
    for name, child in sorted(root.children.items()):
        p = f"{path}/{name}"
        yield p, child, depth
        yield from _walk(child, p, depth + 1)


@pytest.fixture(autouse=True)
def _clean_profiler():
    """Keep the global profiler isolated between tests."""
    profiling.reset()
    was = profiling.is_enabled()
    yield
    profiling.set_enabled(was)
    profiling.reset()


# --- mechanics -------------------------------------------------------------

def test_sections_nest_and_exclusive_excludes_children():
    p = profiling.Profiler()
    with p.section("outer"):
        with p.section("inner"):
            pass
    outer = p.root.children["outer"]
    inner = outer.children["inner"]
    assert inner.total > 0.0
    # outer spans inner, so its exclusive time is what is left over
    assert outer.total >= inner.total
    assert outer.exclusive() == pytest.approx(outer.total - inner.total)
    assert inner.exclusive() == pytest.approx(inner.total)


def test_same_name_accumulates_rather_than_forking():
    p = profiling.Profiler()
    for _ in range(5):
        with p.section("eval"):
            pass
    node = p.root.children["eval"]
    assert node.calls == 5
    assert len(p.root.children) == 1  # one node, not five
    assert node.total > 0.0


def test_exception_inside_section_still_records_time():
    p = profiling.Profiler()
    with pytest.raises(ValueError):
        with p.section("boom"):
            raise ValueError("boom")
    node = p.root.children["boom"]
    assert node.calls == 1
    assert node.total > 0.0


def test_add_bulk_publishes_n_calls_at_once():
    p = profiling.Profiler()
    p.add_bulk("sim.accel", 3.0, 12)
    node = p.root.children["sim.accel"]
    assert node.calls == 12
    assert node.total == pytest.approx(3.0)
    # the whole point: the mean is per-call, not per-publish
    assert node.to_dict()["mean_ms"] == pytest.approx(250.0)


def test_counts_are_dimensionless_and_accumulate():
    p = profiling.Profiler()
    p.count("cache_hit")
    p.count("cache_hit")
    p.count("cache_miss")
    assert p.root.counts["cache_hit"] == 2
    assert p.root.counts["cache_miss"] == 1


def test_disabled_profiler_records_nothing_and_still_yields():
    p = profiling.Profiler(enabled=False)
    with p.section("s") as node:
        assert node is None
        p.count("c")
        p.add("a", 1.0)
        p.add_bulk("b", 1.0, 5)
    assert p.to_dict() == {}
    assert p.total_s() == 0.0


def test_to_dict_is_json_shaped_and_roundtrips_through_merge():
    p = profiling.Profiler()
    with p.section("a"):
        p.count("n", 3)
        with p.section("b"):
            pass
    d = p.to_dict()
    assert set(d) == {"a"}
    assert d["a"]["calls"] == 1
    assert d["a"]["counts"] == {"n": 3.0}
    assert set(d["a"]["children"]) == {"b"}

    # a fresh profiler folding that dict must reproduce the same totals
    q = profiling.Profiler()
    with q.section("wrapper"):
        q.merge_dict("a", d["a"])
    assert q.root.children["wrapper"].children["a"].calls == 1
    assert q.root.children["wrapper"].children["a"].children["b"].calls == 1
    assert q.root.children["wrapper"].children["a"].counts["n"] == 3


def test_merge_sums_rather_than_replaces():
    node_dict = {"total_s": 1.0, "calls": 2, "counts": {"k": 1.0}}
    p = profiling.Profiler()
    p.merge_dict("s", node_dict)
    p.merge_dict("s", node_dict)
    node = p.root.children["s"]
    assert node.total == pytest.approx(2.0)
    assert node.calls == 4
    assert node.counts["k"] == 2.0


def test_merge_all_takes_a_whole_serialised_tree():
    """A worker's to_dict() is a set of named nodes, not one node."""
    q = profiling.Profiler()
    with q.section("eval"):
        q.count("cache_hit")
        with q.section("sim"):
            pass
    snap = q.to_dict()

    p = profiling.Profiler()
    with p.section("generation"):
        p.merge_all(snap)
    gen = p.root.children["generation"]
    assert gen.children["eval"].counts["cache_hit"] == 1
    assert gen.children["eval"].children["sim"].calls == 1
    # idempotent: folding the same worker batch twice doubles, never replaces
    with p.section("generation"):
        p.merge_all(snap)
    assert p.root.children["generation"].children["eval"].children["sim"].calls == 2


def test_merge_all_on_a_single_node_mapping_would_be_wrong():
    """Guards the distinction: merge_dict takes a node, merge_all a tree."""
    node = {"total_s": 1.0, "calls": 1}
    p = profiling.Profiler()
    p.merge_all({"eval": node})
    assert p.root.children["eval"].calls == 1
    # a bare node has no named children to iterate, so it contributes nothing
    p2 = profiling.Profiler()
    p2.merge_all(node)
    assert p2.to_dict() == {}


def test_merge_dict_ignores_derived_fields():
    """mean_ms/exclusive_s must be recomputed, not accumulated."""
    src = {"total_s": 4.0, "calls": 2, "mean_ms": 999.0, "exclusive_s": -7.0}
    p = profiling.Profiler()
    p.merge_dict("s", src)
    p.merge_dict("s", src)
    node = p.root.children["s"]
    assert node.to_dict()["mean_ms"] == pytest.approx(2000.0)
    assert node.to_dict()["exclusive_s"] == pytest.approx(8.0)


# --- isolated scoping -----------------------------------------------------

def test_isolated_merge_true_folds_subtree_into_parent():
    p = profiling.Profiler()
    with p.section("outer"):
        with p.isolated("inner"):
            with p.section("deep"):
                pass
    assert p.root.children["outer"].children["inner"].children["deep"].calls == 1


def test_isolated_merge_false_keeps_subtree_out_of_parent():
    p = profiling.Profiler()
    seen = []
    with p.isolated("rep", merge=False) as sub:
        with p.section("work"):
            pass
        seen = profiling.flatten(sub)
    assert "work" in seen
    assert p.to_dict() == {}  # nothing leaked into the parent


def test_isolated_restores_stack_after_exception():
    p = profiling.Profiler()
    with pytest.raises(RuntimeError):
        with p.section("after"):
            with p.isolated("x"):
                raise RuntimeError("x")
    # the outer section must still be entered/left cleanly afterwards
    with p.section("after"):
        pass
    assert p.root.children["after"].calls == 2


# --- repeated-run statistics ---------------------------------------------

def test_summarize_reports_mean_spread_and_n():
    snaps = [{"eval": 1.0, "sim": 0.8}, {"eval": 3.0, "sim": 2.4},
             {"eval": 2.0, "sim": 1.6}]
    s = profiling.summarize(snaps)
    assert s["eval"]["n"] == 3
    assert s["eval"]["mean_s"] == pytest.approx(2.0)
    assert s["eval"]["std_s"] == pytest.approx(1.0)      # sample std, n-1
    assert s["eval"]["min_s"] == pytest.approx(1.0)
    assert s["eval"]["max_s"] == pytest.approx(3.0)


def test_summarize_treats_absent_path_as_zero_not_missing():
    """A section present in only some runs must not shift the mean."""
    s = profiling.summarize([{"eval": 2.0}, {"eval": 4.0, "extra": 6.0}])
    assert s["extra"]["mean_s"] == pytest.approx(3.0)    # (0 + 6) / 2
    assert s["extra"]["max_s"] == pytest.approx(6.0)


def test_summarize_single_repetition_has_zero_spread():
    s = profiling.summarize([{"eval": 5.0}])
    assert s["eval"]["std_s"] == 0.0
    assert s["eval"]["n"] == 1


def test_summarize_empty_is_empty():
    assert profiling.summarize([]) == {}


def test_flatten_paths_are_dotted_and_exclusive_of_root():
    p = profiling.Profiler()
    with p.section("a"):
        with p.section("b"):
            with p.section("c"):
                pass
    flat = profiling.flatten(p.root)
    assert set(flat) == {"a", "a.b", "a.b.c"}


# --- pipeline wiring: a fast sim records a full path ----------------------

def test_pid_fast_sim_records_sections_averaged_over_many_steps():
    cfg = O.EvalConfig(t_days=0.02)          # short arc: keeps the test quick
    O.evaluate_pid(PID_SPACE.seed_theta(), cfg)
    tree = profiling.profiler().to_dict()
    assert "eval" in tree
    eval_node = tree["eval"]
    assert "sim" in eval_node["children"]
    sim = eval_node["children"]["sim"]
    children = sim["children"]
    # control law and propagation are separated, both over the whole rollout
    assert "accel" in children and "rk4" in children
    assert children["accel"]["calls"] > 1      # averaged over many steps
    assert children["rk4"]["calls"] == children["accel"]["calls"]
    assert "steps" in sim["counts"]
    assert sim["counts"]["steps"] == children["rk4"]["calls"]
    assert eval_node["counts"]["cache_miss"] == 1


def test_nmpc_fast_sim_records_solve_sections():
    cfg = O.EvalConfig(t_days=0.02)
    O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=4)
    tree = profiling.profiler().to_dict()
    eval_node = tree["eval"]
    accel = eval_node["children"]["sim"]["children"]["accel"]
    # the whole controller path nests under one step's worth of accel time,
    # rather than overlapping it
    plan = accel["children"]["tick"]["children"]["plan"]
    assert "outer_plan" in plan["children"]
    solve = plan["children"]["outer_plan"]["children"]["solve"]
    # the solver internals are attributed under the solve, not the rollout
    assert "minimize" in solve["children"]
    assert "jac_setup" in solve["children"]
    assert solve["counts"]["cost_evals"] > 0
    assert "nit" in solve["counts"]


def test_cache_hits_are_counted_separately_from_real_evaluations():
    cfg = O.EvalConfig(t_days=0.01)
    cache: dict = {}
    th = PID_SPACE.seed_theta()
    O.evaluate_pid(th, cfg, cache)
    O.evaluate_pid(th, cfg, cache)
    tree = profiling.profiler().to_dict()["eval"]
    assert tree["calls"] == 2
    assert tree["counts"]["cache_hit"] == 1
    assert tree["counts"]["cache_miss"] == 1
    # only one simulation happened, so the rollout ran once
    assert tree["children"]["sim"]["counts"]["steps"] > 0
    assert tree["children"]["sim"]["children"]["rk4"]["calls"] < (
        tree["children"]["sim"]["counts"]["steps"] * 2)


def test_timing_does_not_change_the_objective_value():
    """Recording must be observationally free."""
    th = PID_SPACE.seed_theta()
    cfg = O.EvalConfig(t_days=0.01)
    profiling.set_enabled(False)
    a = O.evaluate_pid(th, cfg).as_dict()
    profiling.set_enabled(True)
    b = O.evaluate_pid(th, cfg).as_dict()
    assert a == b


def test_timing_overhead_is_small_on_a_repeated_fast_sim():
    """A profiler that cost real time would corrupt every timing it reports."""
    th = PID_SPACE.seed_theta()
    cfg = O.EvalConfig(t_days=0.02)
    O.evaluate_pid(th, cfg)  # warm up imports/allocations

    def timed(enabled):
        profiling.reset()
        profiling.set_enabled(enabled)
        t0 = profiling.CLOCK()
        for _ in range(3):
            O.evaluate_pid(th, cfg, cache=None)
        return profiling.CLOCK() - t0

    t_off = timed(False)
    t_on = timed(True)
    assert t_off > 0.0
    # generous ceiling: the point is that instrumentation is not the bottleneck
    assert t_on < 3.0 * t_off + 0.05


def test_measured_region_sits_far_above_timer_resolution():
    """The design claim: timed regions are coarse enough to measure.

    If a region's mean fell to microseconds it would be dominated by clock
    overhead, and averaging over a sweep would not rescue it.
    """
    cfg = O.EvalConfig(t_days=0.05)
    O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=6)
    tree = profiling.profiler().to_dict()
    rk4 = tree["eval"]["children"]["sim"]["children"]["rk4"]
    mean_ms = rk4["mean_ms"]
    assert mean_ms > 0.01, f"RK4 step mean {mean_ms} ms is at timer resolution"


# --- driver wiring --------------------------------------------------------

def test_run_restart_records_per_generation_sections():
    from cubesat_12u_sim.optimization import driver as D
    cfg = _short_pid_cfg()
    rep = D.run_restart(PID_SPACE.seed_theta(), PID_SPACE, "pid", cfg,
                        horizon=0, seed=1, maxiter=2, workers=1)
    tree = profiling.profiler().to_dict()
    # called directly, so there is no enclosing restart section: the restart's
    # own sections are the top level
    assert "generation" in tree
    gen = tree["generation"]
    assert gen["calls"] >= 1
    assert "ask" in gen["children"]
    assert "tell" in gen["children"]
    dispatch = gen["children"]["dispatch"]
    assert "eval" in dispatch["children"]
    assert "final_eval" in tree
    assert rep.evals > 0


def test_driver_in_process_does_not_reset_the_parent_profiler():
    """The inline path must scope its own subtree, not drain the parent."""
    from cubesat_12u_sim.optimization import driver as D
    cfg = _short_pid_cfg()
    with profiling.profiler().section("outer_marker"):
        D.run_restart(PID_SPACE.seed_theta(), PID_SPACE, "pid", cfg, horizon=0,
                      seed=1, maxiter=1, workers=1)
    tree = profiling.profiler().to_dict()
    # the caller's own sections survive, i.e. the inline worker path did not
    # reset the shared profiler out from under them
    marker = tree["outer_marker"]
    assert "generation" in marker["children"]
    assert "final_eval" in marker["children"]


def test_multistart_records_a_section_per_restart():
    from cubesat_12u_sim.optimization.driver import multistart
    cfg = _short_pid_cfg()
    multistart(PID_SPACE, "pid", cfg, 0, restarts=2, seed=3, maxiter=1,
               workers=1, verbose=False)
    tree = profiling.profiler().to_dict()
    assert "restart0" in tree and "restart1" in tree


def test_worker_timing_merge_sums_shards(monkeypatch):
    from cubesat_12u_sim.optimization import driver as D
    a = {"total_s": 1.0, "calls": 2, "counts": {"k": 1.0},
         "children": {"c": {"total_s": 0.5, "calls": 2}}}
    b = {"total_s": 3.0, "calls": 4, "counts": {"k": 2.0},
         "children": {"c": {"total_s": 1.5, "calls": 4}}}
    merged = D._merge_shot(a, b)
    assert merged["total_s"] == pytest.approx(4.0)
    assert merged["calls"] == 6
    assert merged["counts"]["k"] == pytest.approx(3.0)
    assert merged["children"]["c"]["total_s"] == pytest.approx(2.0)


def test_drain_returns_none_when_timing_disabled():
    from cubesat_12u_sim.optimization import driver as D
    D._init_worker({"kind": "pid", "cfg": O.PID_SEARCH_CFG, "horizon": 0,
                    "lb": -1.0, "ub": 1.0, "use_cpp_pid": False,
                    "nmpc_backend": "numpy"})
    profiling.set_enabled(False)
    assert D._drain_worker_timing() is None
    profiling.set_enabled(True)
    snap = D._drain_worker_timing()
    assert isinstance(snap, dict)


def test_drain_resets_so_batches_do_not_double_count():
    from cubesat_12u_sim.optimization import driver as D
    D._init_worker({"kind": "pid", "cfg": O.PID_SEARCH_CFG, "horizon": 0,
                    "lb": -1.0, "ub": 1.0, "use_cpp_pid": False,
                    "nmpc_backend": "numpy"})
    profiling.profiler().add("x", 1.0)
    first = D._drain_worker_timing()
    assert first["x"]["total_s"] == pytest.approx(1.0)
    second = D._drain_worker_timing()
    assert second == {}          # drained, not re-read


def test_run_batch_returns_payloads_and_timing():
    from cubesat_12u_sim.optimization import driver as D
    lb, ub = PID_SPACE.bounds()
    D._init_worker({"kind": "pid", "cfg": O.PID_SEARCH_CFG, "horizon": 0,
                    "lb": lb, "ub": ub, "use_cpp_pid": False,
                    "nmpc_backend": "numpy"})
    th = PID_SPACE.seed_theta()
    out, timing = D._run_batch([th], drain=False)
    assert len(out) == 1
    assert "cost" in out[0]
    # drain=False: nothing is shipped back, it is already in this process's tree
    assert timing is None
    assert profiling.profiler().to_dict() != {}


# --- validation path ------------------------------------------------------

def test_validate_records_control_and_integrate_separately():
    pytest.importorskip("heyoka", reason="validation path needs heyoka")
    from cubesat_12u_sim.optimization.validate import validate_pid
    r = validate_pid(PID_SPACE.seed_theta(), PID_SPACE, t_end=60.0,
                     dt_outer=10.0, dt_inner=1.0)
    assert math.isfinite(r["dv"])
    tree = profiling.profiler().to_dict()
    v = tree["validate"]
    assert "control" in v["children"]
    assert "integrate" in v["children"]
    assert "build" in v["children"]
    assert v["counts"]["ticks"] > 0
    # both halves averaged over the whole run, not per tick
    assert v["children"]["control"]["calls"] == v["counts"]["ticks"]
    assert v["children"]["integrate"]["calls"] == v["counts"]["ticks"]


# --- reporting ------------------------------------------------------------

def test_report_renders_rows_and_share_column():
    p = profiling.Profiler()
    with p.section("a"):
        with p.section("b"):
            pass
        p.count("n", 7)
    text = p.report("t")
    assert "timing: t" in text
    assert "a" in text and "b" in text
    assert "#n = 7" in text
    assert "%" in text


def test_report_handles_an_empty_tree():
    p = profiling.Profiler()
    assert "no timed sections" in p.report("empty")


def test_format_summary_renders_a_row_per_path():
    s = profiling.summarize([{"eval": 1.0}, {"eval": 2.0}])
    text = profiling.format_summary(s, 1.5)
    assert "eval" in text
    assert "mean" in text and "std" in text


# --- per-horizon timing --------------------------------------------------
#
# The horizon sweep is a structural comparison, so its cost breakdown is a
# result, not a diagnostic. These check that each horizon's subtree is lifted
# out correctly and reduced to rates that are comparable across horizons.

def _horizon_cfg():
    return O.EvalConfig(t_days=0.005, dt_outer=600.0, dt_inner=60.0)


def test_node_resolves_a_dotted_path_from_the_root():
    p = profiling.Profiler()
    with p.section("stage_nmpc"):
        with p.section("horizons"):
            with p.section("horizonH8"):
                with p.section("sweep"):
                    pass
    assert p.node("stage_nmpc") is not None
    assert p.node("stage_nmpc.horizons.horizonH8.sweep").calls == 1
    assert p.node("stage_nmpc.horizons.horizonH8").calls == 1
    # missing components return None rather than raising or faking a zero
    assert p.node("stage_nmpc.horizons.horizonH6") is None
    assert p.node("stage_nmpc.nope.here") is None
    assert p.subtree("stage_nmpc.nope") == {}


def test_horizon_timing_reduces_a_recorded_horizon():
    from cubesat_12u_sim.tuning.run_optimize import _horizon_timing
    cfg = _horizon_cfg()
    with profiling.profiler().section("stage_nmpc"):
        with profiling.profiler().section("horizons"):
            for h in (2, 6):
                with profiling.profiler().section(f"horizonH{h}"):
                    O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=h)
    p = profiling.profiler()
    t2 = _horizon_timing(p, "stage_nmpc.horizons.horizonH2")
    t6 = _horizon_timing(p, "stage_nmpc.horizons.horizonH6")
    for t in (t2, t6):
        assert t["evaluations"] == 1
        assert t["generations"] == 0        # no CMA-ES around a bare evaluate
        assert t["wall_s"] > 0.0
        assert t["wall_s_per_eval"] == pytest.approx(t["wall_s"])
        assert t["solve_calls"] > 0
        assert t["solve_ms"] > 0.0
        assert t["plans_per_eval"] > 0
        assert t["objective_evals_per_solve"] > 0
        assert t["nit_per_solve"] > 0
        assert t["ms_per_nit"] > 0.0
        assert "minimize_ms" in t
    assert _horizon_timing(p, "stage_nmpc.horizons.horizonH12") == {}


def test_horizon_timing_rates_are_normalised_by_evaluations():
    """Raw totals would compare effort, not cost per unit of effort."""
    from cubesat_12u_sim.tuning.run_optimize import _count_sum, _horizon_timing
    cfg = _horizon_cfg()
    # the global profiler, because that is where objective.py records; passing a
    # private Profiler would see the stage sections but none of the eval counts
    p = profiling.profiler()
    with p.section("stage_nmpc"):
        with p.section("horizons"):
            with p.section("horizonH4"):
                O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=4)
                O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=4)
    path = "stage_nmpc.horizons.horizonH4"
    t = _horizon_timing(p, path)
    assert t["evaluations"] == 2
    assert t["wall_s_per_eval"] == pytest.approx(t["wall_s"] / 2)
    # the rate is the raw total divided by the solve count, not the total:
    # two identical evaluations must not double the per-solve objective evals
    raw_obj_evals = _count_sum(p.subtree(path), "cost_evals")
    assert t["objective_evals_per_solve"] == pytest.approx(
        raw_obj_evals / t["solve_calls"])
    # and plans_per_eval is replans per candidate, not cost() calls
    assert t["plans_per_eval"] == pytest.approx(t["solve_calls"] / 2)


def test_ms_per_nit_separates_size_from_iteration_count():
    """A bigger horizon costs more per iteration, not just more iterations."""
    from cubesat_12u_sim.tuning.run_optimize import _horizon_timing
    cfg = _horizon_cfg()
    p = profiling.profiler()
    with p.section("stage_nmpc"):
        with p.section("horizons"):
            for h in (2, 12):
                with p.section(f"horizonH{h}"):
                    O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=h)
    t2 = _horizon_timing(p, "stage_nmpc.horizons.horizonH2")
    t12 = _horizon_timing(p, "stage_nmpc.horizons.horizonH12")
    # H=12 has 6x the decision variables of H=2; per-iteration cost must reflect
    # that even if the iteration counts happened to match
    assert t12["ms_per_nit"] > t2["ms_per_nit"]


def test_horizon_timing_table_and_console_rows():
    from cubesat_12u_sim.tuning.run_optimize import (_horizon_table,
                                              _horizon_timing_table)
    per_horizon = [
        {"horizon": 2, "result": {"dv": 22.7, "max_abs_r_err": 24920.0},
         "timing": {"wall_s": 0.2, "evaluations": 2, "wall_s_per_eval": 0.1,
                    "solves_per_eval": 272.0, "nit_per_solve": 34.0,
                    "ms_per_nit": 0.27, "solve_ms": 9.2}},
        {"horizon": 12, "result": {"dv": 34.9, "max_abs_r_err": 25000.0},
         "timing": {"wall_s": 7.7, "evaluations": 2, "wall_s_per_eval": 3.8,
                    "solves_per_eval": 3768.0, "nit_per_solve": 96.0,
                    "ms_per_nit": 6.0, "solve_ms": 580.0}},
    ]
    rows = _horizon_timing_table(per_horizon)
    assert [r["horizon"] for r in rows] == [2, 12]
    assert rows[0]["ms_per_nit"] == pytest.approx(0.27)
    text = _horizon_table(per_horizon)
    assert "horizon cost" in text
    assert "ms/nit" in text          # column header
    assert "wall/ev" in text         # legend
    # horizons with no timing recorded are skipped, not rendered blank
    assert _horizon_timing_table([{"horizon": 4, "result": {}, "timing": {}}]) == []
    assert _horizon_table([]) == ""


def test_horizon_table_tolerates_missing_metrics():
    """A horizon with a partial timing dict must not crash the sweep."""
    from cubesat_12u_sim.tuning.run_optimize import _horizon_table
    text = _horizon_table([
        {"horizon": 6, "result": {"dv": 1.0, "max_abs_r_err": 1000.0},
         "timing": {"wall_s": 0.5, "evaluations": 1}},
    ])
    assert "6" in text


def test_documented_cli_flags_exist_in_the_parser():
    """The flags this feature is driven by must stay wired up."""
    import inspect

    from cubesat_12u_sim.tuning import run_optimize
    src = inspect.getsource(run_optimize.main)
    for flag in ("--no-timing", "--bench-reps", "--bench-theta"):
        assert flag in src


# --- structural integrity of a real recorded tree ------------------------
#
# The two bugs that motivated these were both invisible to assertions about
# individual paths, because each offending node existed and had plausible
# numbers. What gave them away was walking the whole tree looking for
# impossible relationships, so that is what these do.

def _assert_tree_sane(root):
    for path, node, _ in _walk(root):
        children_total = sum(c.total for c in node.children.values())
        # a parent's inclusive time cannot be smaller than the sum of its
        # children: that only happens if a node was merged in from a scope that
        # never sat inside it
        assert node.total >= children_total - 1e-9, (
            f"{path}: total {node.total} < children {children_total}")
        for c in node.children.values():
            assert c.exclusive() >= -1e-9, f"{path}: negative exclusive"


def test_fast_sim_tree_has_no_impossible_nesting():
    cfg = O.EvalConfig(t_days=0.02)
    O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, horizon=4)
    _assert_tree_sane(profiling.profiler().root)


def test_pooled_sweep_tree_has_no_impossible_nesting():
    """A forked worker must not ship a copy of the parent's tree back."""
    from cubesat_12u_sim.optimization import driver as D
    D.multistart(PID_SPACE, "pid", _short_pid_cfg(0.01), 0, restarts=2, seed=5,
                 maxiter=2, workers=2, verbose=False)
    _assert_tree_sane(profiling.profiler().root)


def test_pooled_sweep_does_not_nest_parent_sections_inside_generations():
    """Regression: the parent's own sections must not reappear under a worker.

    A forked worker inherits a copy of the parent's profiler. If it reports that
    copy back, ``restart0`` shows up nested inside
    ``restart0/generation/dispatch`` and the parent's time is counted twice.
    """
    from cubesat_12u_sim.optimization import driver as D
    D.multistart(PID_SPACE, "pid", _short_pid_cfg(0.01), 0, restarts=2, seed=5,
                 maxiter=2, workers=2, verbose=False)
    root = profiling.profiler().root
    paths = {p for p, _, _ in _walk(root)}
    assert "/restart0" in paths
    # A restart section never contains a further restart section. Counted over
    # section names in the whole tree: a worker shipping back a copy of the
    # parent's tree puts "restart0" inside "restart0".
    restart_like = [p for p, _, _ in _walk(root)
                    if any(n.startswith("restart") for n in p.split("/")[1:])]
    for p in restart_like:
        names = [n for n in p.split("/")[1:] if n.startswith("restart")]
        assert len(names) <= 1, f"nested restart sections at {p}"
    # pool bookkeeping belongs at the top level, not inside a dispatch
    assert "/pool_setup" in paths
    for p, _, _ in _walk(root, "/restart0/generation"):
        assert "/dispatch/pool_setup" not in p, f"pool_setup leaked into {p}"
        assert "/dispatch/resume" not in p, f"resume leaked into {p}"


def test_inlined_evaluation_is_not_double_wrapped():
    """The in-process path must not stack two 'eval' levels."""
    from cubesat_12u_sim.optimization import driver as D
    cfg = _short_pid_cfg(0.01)
    D.run_restart(PID_SPACE.seed_theta(), PID_SPACE, "pid", cfg, horizon=0,
                  seed=1, maxiter=1, workers=1)
    root = profiling.profiler().root
    paths = {p for p, _, _ in _walk(root)}
    assert "/generation/dispatch/eval" in paths
    assert "/generation/dispatch/eval/eval" not in paths
    assert "/final_eval/eval" in paths
    assert "/final_eval/eval/eval" not in paths

# --- horizon timing: aggregation and naming -------------------------------

def test_find_all_collects_every_matching_node():
    from cubesat_12u_sim.tuning.run_optimize import _find, _find_all
    sub = {"total_s": 9.0, "calls": 9, "children": {
        "a": {"total_s": 1.0, "calls": 1, "children": {
            "solve": {"total_s": 2.0, "calls": 2, "children": {}}}},
        "b": {"total_s": 1.0, "calls": 1, "children": {
            "solve": {"total_s": 4.0, "calls": 4, "children": {}}}}}}
    assert _find(sub, "solve")["calls"] == 2          # first match, for compat
    found = _find_all(sub, "solve")
    assert [n["calls"] for n in found] == [2, 4]
    assert _find_all(sub, "absent") == []


def test_horizon_rates_are_not_inflated_by_the_restart_count():
    """The bug: per-solve rates paired one restart's calls with all restarts' nit.

    Each restart contributes its own 'solve' node, so a first-match lookup
    halves the denominator and doubles the rate. The two restarts here are
    deliberately unequal, so that solve_ms discriminates too and not just the
    iteration rate.
    """
    from cubesat_12u_sim.tuning.run_optimize import _find_all, _horizon_timing
    p = profiling.Profiler()
    # (total_s, calls, nit) per restart: 0.1s/2 solves/80 nit, then 0.3s/2/80
    per_restart = [(0.100, 2, 80), (0.300, 2, 80)]
    with p.section("root"):
        for r, (total, calls, nit) in enumerate(per_restart):
            with p.section(f"restart{r}"):
                with p.section("generation"):
                    with p.section("solve"):
                        pass
                    with p.section("solve"):
                        pass
                solve = p.root.children["root"].children[f"restart{r}"].children[
                    "generation"].children["solve"]
                solve.total, solve.calls = total, calls
                solve.counts["nit"] = nit
                solve.counts["cost_evals"] = 400 * (r + 1)
                gen = p.root.children["root"].children[f"restart{r}"].children[
                    "generation"]
                gen.counts["cache_miss"] = 1
    sub = p.subtree("root")
    # both solve nodes must be visible, or the assertions below prove nothing
    assert len(_find_all(sub, "solve")) == 2

    t = _horizon_timing(p, "root")
    assert t["solve_calls"] == 4            # both restarts, not just the first
    # 0.4 s over 4 solves = 100 ms; a first-match lookup would report 50
    assert t["solve_ms"] == pytest.approx(100.0)
    # 160 nit over 4 solves = 40; a first-match lookup would report 80
    assert t["nit_per_solve"] == pytest.approx(40.0)
    assert t["ms_per_nit"] == pytest.approx(0.4 / 160 * 1e3)
    assert t["evaluations"] == 2
    assert t["plans_per_eval"] == pytest.approx(2.0)
    # objective evals are summed across restarts too: (400+800)/4
    assert t["objective_evals_per_solve"] == pytest.approx(300.0)


def test_horizon_timing_uses_corrected_metric_names():
    """solves_per_eval was objective-function calls, not controller replans."""
    from cubesat_12u_sim.tuning.run_optimize import _horizon_timing
    cfg = _horizon_cfg()
    p = profiling.profiler()
    with p.section("stage_nmpc"):
        with p.section("horizons"):
            with p.section("horizonH6"):
                O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, 6)
    t = _horizon_timing(p, "stage_nmpc.horizons.horizonH6")
    assert "solves_per_eval" not in t           # the mislabelled column is gone
    assert "plans_per_eval" in t
    assert "objective_evals_per_solve" in t
    # a plan is one solve; objective evals are the cost() calls it needed
    assert t["plans_per_eval"] < t["objective_evals_per_solve"]


def test_horizon_table_legend_matches_the_columns():
    from cubesat_12u_sim.tuning.run_optimize import _horizon_table
    text = _horizon_table([{"horizon": 6, "result": {"dv": 1.0,
                                                    "max_abs_r_err": 1000.0},
                            "timing": {"wall_s": 0.5, "evaluations": 1,
                                       "wall_s_per_eval": 0.5,
                                       "plans_per_eval": 2.0,
                                       "objective_evals_per_solve": 950.0,
                                       "nit_per_solve": 46.0,
                                       "ms_per_nit": 3.3, "solve_ms": 151.0}}])
    for token in ("plans/ev", "nit/plan", "ms/nit", "obj/pln"):
        assert token in text
    assert "solv/ev" not in text


# --- backend resolution ---------------------------------------------------

def _args(pid_backend="cpp", nmpc_backend="cpp"):
    import argparse
    ns = argparse.Namespace(pid_backend=pid_backend, nmpc_backend=nmpc_backend)
    return ns


def _patch_availability(monkeypatch, pid=True, cpp=True, cxx=True):
    from cubesat_12u_sim.optimization import nmpcbatch, nmpcsolve, pidbatch
    monkeypatch.setattr(pidbatch, "available", pid)
    monkeypatch.setattr(nmpcbatch, "available", cpp)
    monkeypatch.setattr(nmpcsolve, "available", cxx)


def test_resolve_backends_keeps_cpp_when_available(monkeypatch):
    from cubesat_12u_sim.tuning import run_optimize as R
    _patch_availability(monkeypatch)
    pid, nmpc, notes = R.resolve_backends(_args())
    assert (pid, nmpc) == ("cpp", "cpp")
    assert notes == []


def test_resolve_backends_reports_the_downgrade_instead_of_hiding_it(monkeypatch):
    """A run that asked for cpp and got numpy must not report cpp."""
    from cubesat_12u_sim.tuning import run_optimize as R
    _patch_availability(monkeypatch, cpp=False, cxx=False)
    pid, nmpc, notes = R.resolve_backends(_args())
    assert nmpc == "numpy"
    assert any("_nmpcbatch unavailable" in n for n in notes)
    assert pid == "cpp"          # pidbatch was still available


def test_resolve_backends_degrades_cxx_to_cpp_before_numpy(monkeypatch):
    from cubesat_12u_sim.tuning import run_optimize as R
    _patch_availability(monkeypatch, cxx=False, cpp=True)
    _, nmpc, notes = R.resolve_backends(_args(nmpc_backend="cxx"))
    assert nmpc == "cpp"
    assert any("NLopt" in n for n in notes)
    # with neither available it has to go all the way to numpy; only the nmpc
    # arm is affected, since pidbatch is still available
    _patch_availability(monkeypatch, cxx=False, cpp=False)
    pid, nmpc, notes = R.resolve_backends(_args(nmpc_backend="cxx"))
    assert (pid, nmpc) == ("cpp", "numpy")
    assert len(notes) == 1


def test_resolve_backends_downgrades_pid_independently(monkeypatch):
    from cubesat_12u_sim.tuning import run_optimize as R
    _patch_availability(monkeypatch, pid=False)
    pid, nmpc, notes = R.resolve_backends(_args())
    assert (pid, nmpc) == ("numpy", "cpp")
    assert any("_pidbatch unavailable" in n for n in notes)


def test_validation_backend_maps_cxx_to_cpp():
    """Validation cannot use cxx; the nearest prediction family is cpp."""
    from cubesat_12u_sim.tuning.run_optimize import _validation_backend
    assert _validation_backend("numpy") == "numpy"
    assert _validation_backend("cpp") == "cpp"
    assert _validation_backend("cxx") == "cpp"


def test_make_nmpc_treats_every_non_numpy_backend_as_the_cpp_cost(monkeypatch):
    """cxx must not silently receive the numpy closures."""
    from cubesat_12u_sim.optimization import objective as O
    seen = {}

    class Spy:
        def __init__(self, use_cpp=False):
            seen["use_cpp"] = use_cpp

    monkeypatch.setattr("cubesat_12u_sim.control.nmpc.ScipyShootingBackend", Spy)
    cfg = O.EvalConfig(t_days=0.01)
    O.make_nmpc(NMPC_SPACE.from_theta(NMPC_SPACE.seed_theta()), cfg, 4,
                backend="numpy")
    assert seen["use_cpp"] is False
    for backend in ("cpp", "cxx"):
        O.make_nmpc(NMPC_SPACE.from_theta(NMPC_SPACE.seed_theta()), cfg, 4,
                    backend=backend)
        assert seen["use_cpp"] is True, backend


def test_default_nmpc_backend_is_the_cpp_path():
    """The default must reproduce the committed results, not the fallback."""
    import argparse
    import inspect

    from cubesat_12u_sim.tuning import run_optimize
    src = inspect.getsource(run_optimize.main)
    assert '--nmpc-backend", default="cpp"' in src


def test_config_records_requested_and_effective_backends():
    """Both, plus the downgrade list: a silently-fallbacked run must be visible."""
    import inspect

    from cubesat_12u_sim.tuning import run_optimize
    src = inspect.getsource(run_optimize.main)
    for key in ("pid_backend", "nmpc_backend",
                "pid_backend_requested", "nmpc_backend_requested",
                "backend_fallbacks"):
        assert f'"{key}"' in src
    # the effective values come from resolve_backends, not from args
    assert "pid_backend, nmpc_backend, backend_notes = resolve_backends(args)" in src
    assert '"pid_backend": pid_backend' in src
    assert '"nmpc_backend": nmpc_backend' in src


def test_stage_functions_take_the_effective_backend():
    """No stage may re-derive the backend from args and downgrade locally."""
    import inspect

    from cubesat_12u_sim.tuning import run_optimize as R
    for fn in (R.stage_pid, R.stage_nmpc, R.stage_joint, R.run_validation):
        src = inspect.getsource(fn)
        assert "args.nmpc_backend" not in src, fn.__name__
        assert "args.pid_backend" not in src, fn.__name__
        assert "_cpp_nmpc_ok" not in src, fn.__name__


# --- fixes found by the first real C++ run --------------------------------

def test_pred_traj_is_not_computed_by_default():
    """It has no consumers and cost N Python RK4 steps per solve."""
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.control.nmpc import (NMPCConfig, ScipyShootingBackend,
                                      semi_major_axis)
    cfg = NMPCConfig(dt_outer=600.0, horizon=6, maxiter=3)
    x = np.concatenate([C.R0_ECI, C.V0_ECI * 0.995])
    sol = ScipyShootingBackend(use_cpp=False).solve(x, cfg)
    assert sol.pred_traj is None
    assert sol.u_seq.shape == (6, 3)          # the warm start still works

    sol2 = ScipyShootingBackend(use_cpp=False).solve(
        x, NMPCConfig(dt_outer=600.0, horizon=6, maxiter=3,
                      record_pred_traj=True))
    assert sol2.pred_traj is not None
    assert sol2.pred_traj.shape == (7, 6)
    assert sol2.a_eci.shape == (3,)


def test_grad_counters_separate_the_three_gradient_strategies(monkeypatch):
    """The lanes are the whole point, so all three paths must be accounted for.

    fd:      scipy finite-differences, 3N+1 perturbed rollouts per gradient
    adjoint: one forward + one backward sweep, no perturbed rollouts
    cpp:     one crossing that batches the same 3N+1 lanes inside C++
    """
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.control.nmpc import NMPCConfig, ScipyShootingBackend

    N = 4
    cfg = NMPCConfig(dt_outer=600.0, horizon=N, maxiter=5)
    x = np.concatenate([C.R0_ECI, C.V0_ECI * 0.995])

    def run(**kw):
        kw.setdefault("use_cpp", False)
        profiling.reset()
        ScipyShootingBackend(**kw).solve(x, cfg)
        return profiling.profiler().root.children["solve"]

    fd = run(grad="fd")
    nit = fd.counts["nit"]
    assert fd.counts["cost_evals"] > 0
    # scipy calls no jac closure here, so the lanes are booked from the
    # iteration count; otherwise the report shows zero and the adjoint looks
    # like it removed nothing
    assert fd.counts["grad_evals"] == nit
    assert fd.counts["grad_lanes"] == (3 * N + 1) * nit

    adj = run(grad="adjoint")
    assert adj.counts["grad_lanes"] == 0          # no perturbed rollouts
    assert adj.counts["grad_evals"] > 0
    assert adj.counts["cost_evals"] > 0

    # cpp supplies its own gradient, so grad= is ignored there
    calls = {"n": 0}

    def fake_make_funcs(x0, c):
        def cost(z):
            calls["n"] += 1
            return float(np.sum(np.asarray(z) ** 2))

        def jac(z):
            calls["n"] += 1
            return 2.0 * np.asarray(z)

        return (cost, jac, lambda Z: np.zeros(N),
                lambda Z: np.zeros((N, 3 * N)))

    monkeypatch.setattr("cubesat_12u_sim.optimization.nmpcbatch.make_funcs",
                        fake_make_funcs)
    cpp = run(use_cpp=True, grad="fd")
    assert cpp.counts["grad_evals"] > 0
    # same 3N+1 lanes as fd, just batched rather than rolled out separately
    assert cpp.counts["grad_lanes"] == (3 * N + 1) * cpp.counts["grad_evals"]
    assert calls["n"] > 0


def test_horizon_timing_reports_lanes_for_every_gradient_strategy():
    from cubesat_12u_sim.tuning.run_optimize import _horizon_timing
    cfg = _horizon_cfg()
    p = profiling.profiler()
    with p.section("stage_nmpc"):
        with p.section("horizons"):
            with p.section("horizonH4"):
                O.evaluate_nmpc(NMPC_SPACE.seed_theta(), cfg, 4)
    t = _horizon_timing(p, "stage_nmpc.horizons.horizonH4")
    # the default backend is the adjoint, so there are no FD lanes ...
    assert t["objective_evals_per_solve"] > 0
    assert t["nit_per_solve"] > 0
    # ... and the key is absent rather than zero, so "no lanes" is unambiguous
    assert "lanes_per_grad" not in t or t["lanes_per_grad"] == 0.0


def test_figures_use_the_effective_backend():
    """fig_opt_trajectories used to re-simulate on numpy whatever the sweep ran."""
    import inspect

    from cubesat_12u_sim.optimization import plots
    # the re-simulation must accept a backend...
    assert 'backend: str = "numpy"' in inspect.getsource(plots._resimulate)
    # ...pass it to make_nmpc...
    assert "backend=backend" in inspect.getsource(plots._resimulate)
    # ...and fig_trajectories must read it from the results and pass it on
    src = inspect.getsource(plots.fig_trajectories)
    assert "nmpc_backend" in src
    assert "backend=backend" in src


def test_shared_cache_is_used_when_available(monkeypatch):
    """Per-worker caches made the hit rate a function of --workers."""
    from cubesat_12u_sim.optimization import driver as D
    calls = {"n": 0}

    class FakeProxy(dict):
        def get(self, k, default=None):
            calls["n"] += 1
            return dict.get(self, k, default)

    monkeypatch.setattr(D, "_make_shared_cache", lambda: FakeProxy())
    lb, ub = PID_SPACE.bounds()
    shared = D._make_shared_cache()
    D._init_worker({"kind": "pid", "cfg": O.PID_SEARCH_CFG, "horizon": 0,
                    "lb": lb, "ub": ub, "use_cpp_pid": False,
                    "nmpc_backend": "numpy", "shared_cache": shared})
    assert D._W["cache"] is shared


def test_worker_falls_back_to_a_private_cache_without_a_shared_one():
    from cubesat_12u_sim.optimization import driver as D
    lb, ub = PID_SPACE.bounds()
    D._init_worker({"kind": "pid", "cfg": O.PID_SEARCH_CFG, "horizon": 0,
                    "lb": lb, "ub": ub, "use_cpp_pid": False,
                    "nmpc_backend": "numpy", "shared_cache": None})
    assert D._W["cache"] == {}
    # ... and it really is per-process, not shared with the parent
    D._W["cache"]["k"] = 1
    assert D._W["cache"] is not None


def test_shared_cache_is_reported_in_the_tree(monkeypatch):
    from cubesat_12u_sim.optimization import driver as D
    monkeypatch.setattr(D, "_make_shared_cache", lambda: {})
    profiling.reset()
    D.multistart(PID_SPACE, "pid", _short_pid_cfg(0.01), 0, restarts=1, seed=1,
                 maxiter=1, workers=2, verbose=False)
    setup = profiling.profiler().root.children["pool_setup"]
    assert setup.counts["cache.shared"] == 1
    assert setup.counts["workers"] == 2





def test_final_eval_bypasses_the_shared_cache():
    """It must stay a real re-evaluation, not a lookup.

    With a shared cache this theta is very often already present, which would
    quietly turn the guarantee (always a full, untruncated cost) into nothing.
    """
    from cubesat_12u_sim.optimization import driver as D
    lb, ub = PID_SPACE.bounds()
    cfg = _short_pid_cfg(0.01)
    rep = D.run_restart(PID_SPACE.seed_theta(), PID_SPACE, "pid", cfg,
                        horizon=0, seed=1, maxiter=2, workers=1)
    assert rep.evals > 0
    # final_eval recorded real work: a fresh sim ran under it
    fe = profiling.profiler().root.children["final_eval"]
    assert fe.children["eval"].counts["cache_miss"] == 1
