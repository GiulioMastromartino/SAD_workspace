"""Stage-1 verification: the shim against the pure-Python mock OmniSystem.

Checks (all numbers printed):

1. construction via the mock ``OmniSystem`` == construction via the hand-written
   ``heyoka_rw.build_rw_system()`` (structure, dim, order, tol, pars);
2. 600 s closed-loop propagation at dt = 0.1 (PD + wheel allocation + MRP
   shadow reset, mirroring ``heyoka_rw.run_rw_mirror``): max abs / max rel
   difference between ``OmniTaylorAdaptive`` and ``hy.taylor_adaptive``;
3. every heyoka property / method listed in DESIGN.md still works through the
   shim (and still *raises* where the real class raises);
4. ``state`` / ``pars`` writable, ``tol`` not;
5. symbolic == numeric accelerations over random states and random parameters.

Run:  PYTHONPATH=.:.. ../env/bin/python scratch_ws2/stage1_check.py
"""

from __future__ import annotations

import sys
import time
import traceback

import numpy as np
import heyoka as hy
from heyoka._core import taylor_adaptive_dbl

from omniheyoka import (
    DEFAULT_TOL,
    OmniTaylorAdaptive,
    native_available,
    rw_demo_heyoka,
    rw_demo_pars,
    rw_demo_state,
)
from omniheyoka.mock import mock_rw_demo_system, mock_rw_state, mock_rw_pars
from cubesat_12u_sim.backends.heyoka import heyoka_rw as hrw
from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff

DURATION = 600.0
DT = 0.1
RW_K, RW_P = hrw.RW_K, hrw.RW_P

RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def run_closed_loop(ta, duration=DURATION, dt=DT):
    """PD attitude loop + wheel allocation + MRP shadow reset (run_rw_mirror)."""
    n = int(round(duration / dt))
    ts = np.zeros(n + 1)
    Y = np.zeros((15, n + 1))
    U = np.zeros((3, n + 1))
    Y[:, 0] = np.asarray(ta.state, dtype=float)
    zero3 = np.zeros(3)
    for k in range(1, n + 1):
        st = Y[:, k - 1]
        e = mrp_diff(st[6:9], zero3)
        Lr = -RW_K * e - RW_P * st[9:12]
        u = hrw.rw_allocate(Lr)
        ta.pars[0], ta.pars[1], ta.pars[2] = map(float, u)
        ta.propagate_for(dt)
        snew = np.asarray(ta.state, dtype=float)
        s2 = float(snew[6:9] @ snew[6:9])
        if s2 > 1.0:  # MRP shadow-set reset (caller's job, not the ODE's)
            snew[6:9] = -snew[6:9] / s2
            ta.state[6:9] = snew[6:9]
        Y[:, k] = snew
        U[:, k] = u
        ts[k] = ta.time
    return ts, Y, U


def relerr(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    diff = np.abs(a - b)
    denom = np.abs(b).max(axis=1, keepdims=True) if b.ndim > 1 else np.abs(b).max()
    return float(diff.max()), float((diff / np.maximum(denom, 1e-300)).max())


def main() -> int:
    print("=" * 78)
    print("STAGE 1 -- shim vs mock OmniSystem (native module built? "
          f"{native_available()})")
    print("=" * 78)

    # ---------------------------------------------------------------- build
    y0 = rw_demo_state()
    pars = rw_demo_pars(np.zeros(3))
    assert np.allclose(y0, mock_rw_state()), "preset and mock states disagree"
    assert np.allclose(pars, mock_rw_pars()), "preset and mock pars disagree"

    sys_mock = mock_rw_demo_system()
    t0 = time.time()
    ta = OmniTaylorAdaptive(sys_mock, y0, pars=pars, tol=DEFAULT_TOL)
    t_build = time.time() - t0
    t0 = time.time()
    ref = rw_demo_heyoka(y0, np.zeros(3), tol=DEFAULT_TOL)
    t_ref = time.time() - t0

    check("constructor", isinstance(ta, taylor_adaptive_dbl),
          f"type={type(ta).__name__}, build {t_build:.3f}s (ref {t_ref:.3f}s)")
    check("isinstance(hy.taylor_adaptive == dbl class)", isinstance(ref, taylor_adaptive_dbl))
    check("ta.system attached", ta.system is sys_mock)
    check("dim == 15", ta.dim == 15, f"dim={ta.dim}")
    check("state_names", ta.state_names == tuple(hrw.STATE_NAMES),
          str(ta.state_names))
    check("n_states convenience", ta.n_states == ta.dim, f"{ta.n_states}")
    check("n_pars convenience", ta.n_pars == len(ta.pars) == 19, f"{ta.n_pars}")
    check("order matches ref", ta.order == ref.order, f"{ta.order} vs {ref.order}")
    check("tol matches", ta.tol == ref.tol == DEFAULT_TOL, f"{ta.tol}")
    check("sys equals ref sys", [str(r) for _, r in ta.sys] == [str(r) for _, r in ref.sys])
    check("initial state identical",
          np.array_equal(np.asarray(ta.state, float), np.asarray(ref.state, float)))

    # -------------------------------------------------------600 s closed loop
    ts_o, Y_o, U_o = run_closed_loop(ta)
    ts_r, Y_r, U_r = run_closed_loop(ref)
    absd, reld = relerr(Y_o, Y_r)
    check("final time", abs(ts_o[-1] - ts_r[-1]) < 1e-9,
          f"omni t={ts_o[-1]:.9f} ref t={ts_r[-1]:.9f}")
    check("|u| <= u_max", np.abs(U_o).max() <= hrw.RW_U_MAX + 1e-15,
          f"max|u|={np.abs(U_o).max():.4f}")
    check("motor torque identical", np.array_equal(U_o, U_r))
    max_abs_state = float(np.abs(Y_r).max())
    check("600 s @ dt=0.1 parity (abs)", absd < 1e-12,
          f"max|Δ|={absd:.3e}  (state scale {max_abs_state:.3e})")
    check("600 s @ dt=0.1 parity (rel)", reld < 1e-12,
          f"max rel|Δ|={reld:.3e}")
    sw_o = int(np.sum(np.sum(Y_o[6:9] ** 2, axis=0) > 1.0))
    sw_r = int(np.sum(np.sum(Y_r[6:9] ** 2, axis=0) > 1.0))
    check("MRP shadow switches match", sw_o == sw_r, f"{sw_o} vs {sw_r}")
    check("|sigma| <= 1 after reset", np.all(np.sum(Y_o[6:9] ** 2, axis=0) <= 1.0 + 1e-9))

    # -------------------------------------------heyoka surface through shim
    surface_ok = True
    notes = []
    props = ["compact_mode", "d_output", "decomposition", "dim", "dtime",
             "high_accuracy", "is_variational", "last_h", "llvm_state",
             "n_orig_sv", "order", "pars", "state", "sys", "tc", "time",
             "tol", "with_events"]
    for name in props:
        try:
            getattr(ta, name)
        except Exception as exc:  # noqa: BLE001
            surface_ok = False
            notes.append(f"{name}: {type(exc).__name__}: {exc}")
    methods = ["eval_taylor_map", "get_mindex", "get_vslice", "propagate_for",
               "propagate_grid", "propagate_until", "reset_cooldowns", "step",
               "step_backward", "update_d_output"]
    for name in methods:
        try:
            getattr(ta, name)
        except Exception as exc:  # noqa: BLE001
            surface_ok = False
            notes.append(f"{name}: {type(exc).__name__}: {exc}")
    check("all heyoka properties/methods present", surface_ok, "; ".join(notes))

    # properties that must still raise exactly like the real class
    for name in ["t_events", "nt_events", "te_cooldowns", "tstate", "vargs", "vorder"]:
        exc_shim = exc_ref = None
        try:
            getattr(ta, name)
        except Exception as exc:  # noqa: BLE001
            exc_shim = type(exc)
        try:
            getattr(ref, name)
        except Exception as exc:  # noqa: BLE001
            exc_ref = type(exc)
        check(f"{name} raises like ref ({exc_ref and exc_ref.__name__})",
              exc_shim is exc_ref is ValueError)

    # mutability contract
    old = float(ta.pars[0])
    ta.pars[0] = 0.123
    ok_write = float(ta.pars[0]) == 0.123
    ta.pars[0] = old
    check("pars writable", ok_write)
    saved_state = np.array(ta.state, dtype=float, copy=True)
    ta.state[0] += 1.0
    ok_state = abs(float(ta.state[0]) - (saved_state[0] + 1.0)) < 1e-12
    ta.state[:] = saved_state
    check("state writable", ok_state)
    try:
        ta.tol = 1e-10
        check("tol read-only", False, "assignment silently succeeded")
    except AttributeError:
        check("tol read-only", True, "AttributeError (expected)")
    check("tol unchanged", ta.tol == DEFAULT_TOL)

    # custom attribute survival
    ta.custom_marker = 42
    check("custom attribute attachable", ta.__dict__.get("custom_marker") == 42)

    # ------------------------------------------------- live step / step back
    before = float(ta.time)
    ta.step()
    check("step() advances", ta.time > before, f"{before:.6f} -> {ta.time:.6f}")
    ta2 = OmniTaylorAdaptive(sys_mock, y0, pars=pars, tol=DEFAULT_TOL)
    ta2.propagate_until(30.0)
    check("propagate_until()", abs(ta2.time - 30.0) < 1e-9, f"t={ta2.time:.6f}")
    ta2.step_backward()
    check("step_backward()", ta2.time < 30.0, f"t={ta2.time:.6f}")
    ta3 = OmniTaylorAdaptive(sys_mock, y0, pars=pars, tol=DEFAULT_TOL)
    grid = ta3.propagate_grid([0.0, 0.5, 1.0, 2.5])
    check("propagate_grid()", grid is not None and abs(ta3.time - 2.5) < 1e-9,
          f"3 points, t={ta3.time}")
    ta4 = OmniTaylorAdaptive(sys_mock, y0, pars=pars, tol=DEFAULT_TOL)
    out = ta4.propagate_for(1.0)
    check("propagate_for() 6-tuple", isinstance(out, tuple) and len(out) == 6,
          f"{len(out)} fields")
    idx = ta4.get_mindex(1)
    check("get_mindex(1)", idx is not None, str(idx))
    vs = ta4.get_vslice()
    check("get_vslice()", hasattr(vs, "start"), str(vs))
    tm = ta4.eval_taylor_map(3)
    check("eval_taylor_map(3)", tm is not None, str(getattr(tm, "shape", type(tm))))
    ta4.update_d_output(np.zeros(ta4.dim))
    check("update_d_output()", ta4.d_output is not None)
    ta4.reset_cooldowns()
    check("reset_cooldowns()", True)
    check("tc shape", np.asarray(ta4.tc).shape == (15, ta4.order + 1),
          str(np.asarray(ta4.tc).shape))
    check("decomposition", isinstance(ta4.decomposition, list) and len(ta4.decomposition) > 0,
          f"{len(ta4.decomposition)} nodes")
    check("llvm_state", ta4.llvm_state is not None)

    # kwargs forwarding
    ta5 = OmniTaylorAdaptive(sys_mock, y0, pars=pars, tol=1e-10,
                             high_accuracy=True, compact_mode=False, opt_level=3)
    check("kwargs forwarded (high_accuracy)", bool(ta5.high_accuracy) is True)
    check("kwargs forwarded (compact_mode)", bool(ta5.compact_mode) is False)
    check("kwargs forwarded (tol)", ta5.tol == 1e-10)
    try:
        OmniTaylorAdaptive(sys_mock, y0, pars=pars, fp_type="long double")
        check("fp_type rejected", False, "no error raised")
    except TypeError:
        check("fp_type rejected", True, "TypeError (expected)")

    # ------------------------------------------- symbolic vs numeric accel
    rng = np.random.default_rng(20261010)
    worst = 0.0
    worst_case = None
    for trial in range(25):
        st = np.array(rw_demo_state(
            r=rng.normal(0, 1e7, 3), v=rng.normal(0, 5e3, 3),
            sigma=rng.normal(0, 0.4, 3), omega=rng.normal(0, 0.05, 3),
            wheel_speeds=rng.normal(0, 300, 3)))
        u = np.clip(rng.normal(0, 0.15, 3), -hrw.RW_U_MAX, hrw.RW_U_MAX)
        ps = rw_demo_pars(u, 750.0, np.diag([900.0, 800.0, 600.0]))
        num = np.asarray(sys_mock.accelerations(list(map(float, st)),
                                                list(map(float, ps))), float)
        full = ta4.symbolic_rhs(st, ps)
        sym = np.concatenate([full[3:6], full[9:12], full[12:15]])
        scale = max(1.0, float(np.abs(sym).max()))
        rel = float(np.abs(num - sym).max() / scale)
        if rel > worst:
            worst, worst_case = rel, num - sym
    check("numeric accelerations == symbolic (random, 25 trials)", worst < 1e-13,
          f"max rel diff = {worst:.3e}")

    # --------------------------------------------------------------- summary
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print("-" * 78)
    print(f"{len(RESULTS) - n_fail}/{len(RESULTS)} checks passed")
    if n_fail:
        print("FAILURES:")
        for name, ok, detail in RESULTS:
            if not ok:
                print(f"  - {name}: {detail}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:  # pragma: no cover - debugging aid
        traceback.print_exc()
        sys.exit(2)
