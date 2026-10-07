"""Phase 3: batch + ensemble Heyoka Monte Carlo runner (Tier-1 screener).

Uses ``taylor_adaptive_batch`` (SIMD, batch = ``recommended_simd_size()``)
so one ``propagate_for(dt)`` advances a whole lane-group at once; the 50 Hz
control/allocation stays in Python per lane. Result dicts match
``monte_carlo.run_single_mission`` so statistics/plots/savers are reused.

Also provides :func:`openloop_ensemble` (Heyoka ``ensemble_propagate_for``)
for dynamics-only validation sweeps.
"""
from __future__ import annotations

import time
from dataclasses import asdict

import numpy as np
import heyoka as hy

from . import config as C
from .actuators import allocate_cached, allocation_matrices
from .heyoka_attitude import (
    N_PARS,
    build_mrp_system,
    fast_wrench_pars,
    gyro_noise_std,
    mrp_diff,
    mrp_diff_batch,
    mrp_to_dcm,
    mrp_to_dcm_batch,
    pack_mrp_state,
    pack_pars,
    precompute_sun_refs,
    sun_direction_at,
)
from Basilisk.utilities import RigidBodyKinematics as rk


def _default_batch() -> int:
    try:
        simd = int(hy.recommended_simd_size())
    except Exception:
        simd = 4
    # A few SIMD vectors per chunk amortizes per-chunk Python overhead while
    # lanes stay cheap; results are independent of B (same seeds -> same KPIs).
    return max(8, simd)


def _make_batch_template(B: int, mc_cfg,
                         compact_mode: bool = False) -> hy.taylor_adaptive_batch:
    """Build one batch integrator for B lanes (JIT-compiles once).

    Reused across all chunks via reset (set_time/state/pars), so the ~3 s
    cold LLVM compile (0.17 s warm from disk cache, 0.01 s from mem cache)
    is paid exactly once per process -- including for short trailing chunks,
    which are padded with dummy lanes instead of rebuilding for another B.
    """
    system, _ = build_mrp_system()
    state = np.zeros((12, B))
    pars = np.zeros((N_PARS, B))
    return hy.taylor_adaptive_batch(
        system, state, pars=pars, tol=float(C.HEYOKA_TOL),
        high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY),
        compact_mode=bool(compact_mode))


def _run_batch(seeds: list[int], mc_cfg, run_offset: int,
               quantize_thrusters: bool = True,
               ta: hy.taylor_adaptive_batch | None = None,
               ctrl_every: int = 1,
               use_fastloop: bool = True,
               sun_refs: np.ndarray | None = None) -> list[dict]:
    """Closed-loop batch for one lane-group; returns per-seed result dicts.

    If ``ta`` (sized for ``len(seeds)``, padded upstream if needed) is given
    it is reset in place instead of rebuilt, avoiding any re-JIT.
    Control (guidance/PD/allocation) runs every ``ctrl_every`` dynamics
    ticks with ZOH; sensing/observer stay per-tick.
    With ``use_fastloop`` (default) the per-tick lane loop runs in the C++
    extension (numpy fallback otherwise; results agree to ~1e-12 relative).
    Pass campaign-shared ``sun_refs`` to skip the per-chunk precompute.
    """
    from .actuators import allocate_cached, allocation_matrices
    from .heyoka_attitude import fast_wrench_pars, precompute_sun_refs
    from .monte_carlo import sample_dispersions

    B = len(seeds)
    disps = [sample_dispersions(int(s), mc_cfg) for s in seeds]
    dt = float(mc_cfg.task_step)
    dlog = float(mc_cfg.log_step)
    n_steps = int(round(mc_cfg.t_total / dt))
    log_every = max(1, int(round(dlog / dt)))

    system, _ = build_mrp_system()
    state = np.zeros((12, B))
    pars = np.zeros((N_PARS, B))
    for j, d in enumerate(disps):
        state[:, j] = pack_mrp_state(d["r0"], d["v0"], d["sigma0"], d["omega0"])
        pars[:, j] = pack_pars(np.zeros(3), np.zeros(3),
                               float(d["mass"]), np.asarray(d["inertia"], float))
    if ta is None:
        ta = hy.taylor_adaptive_batch(system, state, pars=pars,
                                      tol=float(C.HEYOKA_TOL),
                                      high_accuracy=bool(C.HEYOKA_HIGH_ACCURACY))
    else:
        ta.set_time(0.0)
        ta.state[:] = state
        ta.pars[:] = pars
    # Static pars rows (mass + inertias) never change mid-run; the hot loop
    # refreshes only the 6 wrench rows below.
    pars_static = pars[6:, :].copy()

    # Per-lane filter / accounting state
    rngs = [np.random.default_rng(int(s) + 999983) for s in seeds]
    # Pre-generate all gyro noise upfront: identical stream to per-tick
    # sampling (verified), but B calls instead of B*n_steps. Layout
    # (n_steps, B, 3) so each tick's slice is C-contiguous (no hidden copy
    # in the C++ fast path). Pre-scaled by the (rate-independent)
    # Basilisk-equivalent noise std, so the hot loop is a pure add.
    _arw_std = gyro_noise_std(np.array([d["gyro_arw"] for d in disps]))
    noise_all = np.ascontiguousarray(np.array(
        [rngs[j].normal(0.0, 1.0, (n_steps, 3)) for j in range(B)]
    ).transpose(1, 0, 2) * _arw_std[None, :, :])
    bias_true = np.array([d["gyro_bias"] for d in disps])
    bias_est = np.zeros((B, 3))
    est_rate = np.zeros((B, 3))
    est_init = np.zeros(B, dtype=bool)
    sigma_init = [np.asarray(d["sigma0"], float) for d in disps]
    roll_ref = np.asarray(mc_cfg.sun_roll_ref, float)
    masses = np.array([float(d["mass"]) for d in disps])
    isps = np.array([float(d["thruster_isp"]) for d in disps])
    on_time = np.zeros((B, len(C.THRUSTER_TMAX)))
    phase_imp = np.zeros((B, 3))   # detumble/slew/sunpoint
    phase_fuel = np.zeros((B, 3))

    sigmas = np.asarray([d["sigma0"] for d in disps], float)
    omegas = np.asarray([d["omega0"] for d in disps], float)
    log_t, log_om, log_sg, log_se = [], [], [], []

    # One-time per-chunk setup: shared nominal allocation matrix (controller
    # side is identical for all lanes), shared sun-ref timeline, per-lane
    # wrench constants (no inv/cross in the hot loop), ZOH holds.
    # Everything the tick loop touches is hoisted to locals (no attribute
    # lookups, float() casts, or len() calls inside the 15k iterations).
    P_nom = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)
    ce = int(ctrl_every)
    t_ctrl = float(dt * ce)
    sigma_slew = np.asarray(mc_cfg.sigma_slew, float)
    if sun_refs is None:
        sun_refs = precompute_sun_refs(mc_cfg, roll_ref, dt)
    # Full reference timeline (n_steps+1, B, 3): detumble/slew tiles + sun.
    k_det = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                float(mc_cfg.t_detumble), side="left"))
    k_slew = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                 float(mc_cfg.t_detumble + mc_cfg.t_slew),
                                 side="left"))
    REFS = np.empty((n_steps + 1, B, 3))
    REFS[:k_det] = np.array(sigma_init)[None, :, :]
    REFS[k_det:k_slew] = sigma_slew[None, None, :]
    REFS[k_slew:] = sun_refs[k_slew:][:, None, :]
    PH = np.zeros(n_steps + 1, dtype=np.int64)
    PH[k_det:k_slew] = 1
    PH[k_slew:] = 2
    K_ = float(C.ACS_K)
    P_ = float(C.ACS_P)
    OG = float(C.ACS_OBSERVER_GAIN)
    OBG = float(C.ACS_OBSERVER_BIAS_GAIN)
    SC = float(C.THRUSTER_CONTROL_SCALE)
    G0 = float(C.G0)
    N_TH = len(C.THRUSTER_TMAX)
    QF = bool(quantize_thrusters)
    LOGEVERY = log_every
    ce = int(ctrl_every)
    Z3 = np.zeros(3)
    from .heyoka_attitude import wrench_setup
    ws_all = [wrench_setup(d) for d in disps]
    tmax_arr = np.array([w["tmax"] for w in ws_all])
    I6_arr = np.array([w["I6"] for w in ws_all])
    invI6_arr = np.array([w["invI6"] for w in ws_all])
    F_hold = np.zeros((B, 3))
    T_hold = np.zeros((B, 3))
    # Stacked per-lane effectiveness for the C++ fast path.
    MF_all = np.stack([w["M_F"] for w in ws_all]).copy()
    MT_all = np.stack([w["M_T"] for w in ws_all]).copy()
    tmax_12 = np.stack([w["tmax"] for w in ws_all]).copy()
    I6_arr = np.array([w["I6"] for w in ws_all])
    invI6_arr = np.array([w["invI6"] for w in ws_all])
    try:
        from . import fastloop as _fl
        use_fastloop = bool(_fl.available) and bool(use_fastloop)
    except Exception:
        use_fastloop = False

    for k in range(n_steps + 1):
        if k % LOGEVERY == 0:
            log_t.append(k * dt)
            log_om.append(omegas.copy())
            log_sg.append(sigmas.copy())
            log_se.append(mrp_diff_batch(sigmas, REFS[k]))
        if k == n_steps:
            break
        # --- sensing + observer every tick for all lanes ---
        # (skipped on fastloop control ticks: the C++ kernel does observer
        # + control together there; on hold ticks the Python observer runs)
        do_ctrl = (k % ce == 0)
        if not (use_fastloop and do_ctrl):
            meas = omegas + bias_true + noise_all[k]
            new = ~est_init
            est_rate[new] = meas[new]
            est_init[new] = True
            old = ~new
            if np.any(old):
                resid = meas[old] - est_rate[old] - bias_est[old]
                est_rate[old] += OG * resid * dt
                bias_est[old] += OBG * resid * dt
            fb = meas - bias_est
        # --- decimated control with ZOH ---
        if do_ctrl:
            if use_fastloop:
                _fl.tick_all(
                    B, N_TH, sigmas, omegas, bias_true, noise_all[k], REFS[k],
                    est_rate, bias_est, est_init, P_nom, MF_all, MT_all,
                    tmax_12, isps, K_, P_, OG, OBG, SC, t_ctrl, dt,
                    G0, int(PH[k]), QF,
                    F_hold, T_hold, on_time, phase_imp, phase_fuel, pars)
                F_all = F_hold
                T_all = T_hold
            else:
                ref = REFS[k]
                for j in range(B):
                    e = mrp_diff(sigmas[j], ref[j])
                    tcmd = -K_ * e - P_ * fb[j]
                    u = allocate_cached(P_nom, Z3, SC * tcmd)
                    F_true, T_true = fast_wrench_pars(
                        u, ws_all[j], quantize_thrusters)
                    F_hold[j], T_hold[j] = F_true, T_true
                    on = np.clip(u, 0.0, 1.0) * t_ctrl
                    on_time[j] += on
                    tick_imp = float(np.sum(on * tmax_arr[j]))
                    tick_fuel = tick_imp / (isps[j] * G0)
                    phase_imp[j, PH[k]] += tick_imp
                    phase_fuel[j, PH[k]] += tick_fuel
        if not (use_fastloop and do_ctrl):
            # fallback path packs wrench rows itself (fast path did it in C++)
            F_all = F_hold
            T_all = T_hold
            pars[0:3, :] = F_all.T
            pars[3:6, :] = T_all.T
        ta.pars[:] = pars
        ta.propagate_for(dt)
        st = np.asarray(ta.state)[6:12, :].T.copy()
        sigmas = st[:, 0:3]
        omegas = st[:, 3:6]
        # shadow-set reset per lane (vectorized mask)
        n2 = np.einsum("ij,ij->i", sigmas, sigmas)
        over = n2 > 1.0
        if np.any(over):
            sigmas[over] /= -n2[over, None]
            ta.state[6:9, over] = sigmas[over].T

    # --- KPIs per lane (same logic as monte_carlo; vectorized pre-pass) ---
    times_a = np.asarray(log_t)
    n_log = len(times_a)
    OM = np.asarray(log_om).reshape(n_log, B, 3)      # (L,B,3)
    SG = np.asarray(log_sg).reshape(n_log, B, 3)
    SE = np.asarray(log_se).reshape(n_log, B, 3)
    se = np.linalg.norm(SE, axis=2).T                   # (B,L)
    onorm = np.degrees(np.linalg.norm(OM, axis=2)).T   # (B,L)
    # Sun angle for all (lane, tick) pairs in two batched calls.
    bore = np.asarray(mc_cfg.sun_pointing_axis_b, float)
    sun_dirs = np.array([sun_direction_at(float(tj)) for tj in times_a])
    dcm_all = mrp_to_dcm_batch(SG.reshape(n_log * B, 3)).reshape(n_log, B, 3, 3)
    # sun in body frame: sb[l,b,j] = sum_col dcm[l,b,j,col] * sun[l,col]
    sun_b = np.einsum("lbij,lj->lbi", dcm_all, sun_dirs)
    sang_all = np.degrees(np.arccos(np.clip(
        np.einsum("lbj,j->lb", sun_b, bore), -1.0, 1.0))).T  # (B,L)
    dm = times_a <= mc_cfg.t_detumble
    sm = (times_a > mc_cfg.t_detumble) & (times_a < mc_cfg.t_detumble + mc_cfg.t_slew)
    steady_m = times_a >= (mc_cfg.t_total - 40.0)
    return _results_from_arrays(
        times_a, onorm, se, sang_all, B, mc_cfg, disps, seeds, run_offset,
        on_time, phase_imp, phase_fuel, dm, sm, steady_m)


def _results_from_arrays(times_a, onorm, se, sang_all, B, mc_cfg, disps,
                         seeds, run_offset, on_time, phase_imp, phase_fuel,
                         dm, sm, steady_m):
    """Per-lane KPI thresholds + result-dict assembly (shared by backends).

    ``onorm``/``se``/``sang_all`` are (B,L) arrays; logic mirrors
    ``monte_carlo`` exactly.
    """
    results = []
    for j in range(B):
        onorm_j = onorm[j]
        se_j = se[j]
        sang = sang_all[j]
        dt_t, dt_r = times_a[dm], onorm_j[dm]
        ab = np.where(dt_r >= mc_cfg.rate_settle_thresh_deg_s)[0]
        d_set = float(dt_t[0]) if len(ab) == 0 else (
            float(mc_cfg.t_detumble) if ab[-1] == len(dt_r) - 1 else float(dt_t[ab[-1] + 1]))
        st_t, st_e = times_a[sm], se_j[sm]
        sang_e = np.degrees(4.0 * np.arctan(st_e))
        av = np.where(sang_e >= mc_cfg.slew_settle_thresh_deg)[0]
        s_set = 0.0 if len(av) == 0 else (
            float(mc_cfg.t_slew) if av[-1] == len(sang_e) - 1
            else float(st_t[av[-1] + 1] - mc_cfg.t_detumble))
        final_slew = float(sang_e[-1]) if len(sang_e) else 0.0
        steady = sang[steady_m]
        final_sun = float(sang[-1])
        ss_max = float(np.max(steady)) if len(steady) else final_sun
        inside = bool(ss_max <= mc_cfg.sun_cone_angle_deg)
        success = bool((dt_r[-1] < mc_cfg.rate_settle_thresh_deg_s * 2.0) and inside)
        d = disps[j]
        det_f, sl_f, sp_f = (phase_fuel[j, 0] * 1000.0, phase_fuel[j, 1] * 1000.0,
                             phase_fuel[j, 2] * 1000.0)
        tot_imp = float(phase_imp[j].sum())
        results.append({
            "run_idx": run_offset + j + 1, "seed": int(seeds[j]), "success": success,
            "dispersions": {
                "mass_kg": float(d["mass"]),
                "inertia_diag": [float(d["inertia"][0, 0]), float(d["inertia"][1, 1]),
                                 float(d["inertia"][2, 2])],
                "com_offset_mm": (np.asarray(d["com_offset"], float) * 1000.0).tolist(),
                "thruster_isp_s": float(d["thruster_isp"]),
                "omega0_deg_s": np.degrees(np.asarray(d["omega0"], float)).tolist(),
                "sigma0": np.asarray(d["sigma0"], float).tolist()},
            "kpis": {
                "detumble_settle_time_s": d_set,
                "final_detumble_rate_deg_s": float(dt_r[-1]),
                "slew_settle_time_s": s_set,
                "final_slew_error_deg": final_slew,
                "final_sun_angle_deg": final_sun,
                "steady_state_max_sun_angle_deg": ss_max,
                "inside_sun_cone": inside,
                "detumble_propellant_g": det_f, "slew_propellant_g": sl_f,
                "sunpoint_propellant_g": sp_f,
                "total_propellant_g": det_f + sl_f + sp_f,
                "total_impulse_ns": tot_imp,
                "delta_v_m_s": float(tot_imp / float(d["mass"])),
                "thruster_duty_cycles": (on_time[j] / mc_cfg.t_total).tolist()},
            "timeseries": {"time": times_a, "omega_norm_deg_s": onorm_j,
                           "sigma_err_norm": se_j, "sun_angle_deg": sang}})
    return results


def _run_batch_cxx(seeds: list[int], mc_cfg, run_offset: int,
                    quantize_thrusters: bool = True,
                    sun_refs=None) -> list[dict]:
    """Closed-loop batch chunk via the full C++ Heyoka driver.

    Same dispersions schema and result dicts as :func:`_run_batch`; gyro
    noise comes from C++ ``mt19937_64`` (different streams than numpy, so
    parity vs the batch backend is distributional: fuel means within ~2%).
    """
    from . import cxxdrive as _cxx
    from .heyoka_attitude import sun_direction_at, wrench_setup
    from .monte_carlo import sample_dispersions

    if not _cxx.available:
        raise RuntimeError("cxxdrive extension unavailable")
    B = len(seeds)
    disps = [sample_dispersions(int(s), mc_cfg) for s in seeds]
    dt = float(mc_cfg.task_step)
    n_steps = int(round(mc_cfg.t_total / dt))
    log_every = max(1, int(round(float(mc_cfg.log_step) / dt)))
    n_log = n_steps // log_every + 1
    if sun_refs is None:
        sun_refs = precompute_sun_refs(
            mc_cfg, np.asarray(mc_cfg.sun_roll_ref, float), dt)
    sigma_init = np.array([np.asarray(d["sigma0"], float) for d in disps])
    k_det = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                float(mc_cfg.t_detumble), side="left"))
    k_slew = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                 float(mc_cfg.t_detumble + mc_cfg.t_slew),
                                 side="left"))
    REFS = np.empty((n_steps + 1, B, 3))
    REFS[:k_det] = sigma_init[None, :, :]
    REFS[k_det:k_slew] = np.asarray(mc_cfg.sigma_slew, float)[None, None, :]
    REFS[k_slew:] = np.asarray(sun_refs[k_slew:])[:, None, :]
    PH = np.zeros(n_steps + 1, dtype=np.int64)
    PH[k_det:k_slew] = 1
    PH[k_slew:] = 2
    ws_all = [wrench_setup(d) for d in disps]
    P_nom = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR,
                                C.THRUSTER_TMAX)
    out = _cxx.run_chunk(
        np.array([d["r0"] for d in disps], float),
        np.array([d["v0"] for d in disps], float),
        np.array([d["sigma0"] for d in disps], float),
        np.array([d["omega0"] for d in disps], float),
        np.array([float(d["mass"]) for d in disps]),
        np.array([w["I6"] for w in ws_all]),
        np.array([w["invI6"] for w in ws_all]),
        np.stack([w["M_F"] for w in ws_all]),
        np.stack([w["M_T"] for w in ws_all]),
        np.stack([w["tmax"] for w in ws_all]),
        np.array([float(d["thruster_isp"]) for d in disps]),
        np.array([d["gyro_bias"] for d in disps], float),
        np.array([d["gyro_arw"] for d in disps], float),
        np.array([int(s) for s in seeds], dtype=np.uint64),
        P_nom, np.ascontiguousarray(REFS), PH,
        np.array([sun_direction_at(float((k) * dt))
                  for k in range(0, n_steps + 1, log_every)]),
        np.asarray(mc_cfg.sun_pointing_axis_b, float),
        float(C.ACS_K), float(C.ACS_P),
        float(C.ACS_OBSERVER_GAIN), float(C.ACS_OBSERVER_BIAS_GAIN),
        float(C.THRUSTER_CONTROL_SCALE), dt, n_steps, log_every,
        float(C.G0),
        float(mc_cfg.rate_settle_thresh_deg_s),
        float(mc_cfg.slew_settle_thresh_deg),
        float(mc_cfg.sun_cone_angle_deg),
        bool(quantize_thrusters), float(C.HEYOKA_TOL), False,
        float(C.MU_EARTH), bool(C.USE_GRAVITY_GRADIENT))
    times_a = np.arange(n_log) * (log_every * dt)
    onorm = np.degrees(np.asarray(out["omega_norm"], float)).T  # (B,L)
    se = np.asarray(out["sigma_err"], float).T
    sang_all = np.degrees(np.asarray(out["sun_angle"], float)).T
    on_time = np.asarray(out["on_time"], float)
    phase_imp = np.asarray(out["phase_imp"], float)
    phase_fuel = np.asarray(out["phase_fuel"], float)
    dm = times_a <= mc_cfg.t_detumble
    sm = ((times_a > mc_cfg.t_detumble)
          & (times_a < mc_cfg.t_detumble + mc_cfg.t_slew))
    steady_m = times_a >= (mc_cfg.t_total - 40.0)
    return _results_from_arrays(
        times_a, onorm, se, sang_all, B, mc_cfg, disps, seeds, run_offset,
        on_time, phase_imp, phase_fuel, dm, sm, steady_m)


def _run_subset(job: tuple) -> list[dict]:
    """Worker entry point for process-pool fan-out (must stay picklable).

    ``job`` = (chunks, mc_cfg, quantize_thrusters, ctrl_every, batch_size,
    compact_mode, use_fastloop, sun_refs, backend) where ``chunks`` is a list
    of ``(seeds, run_offset, real_n)`` with each seeds list already padded to
    ``batch_size`` upstream. Each worker builds its own template (0.17 s
    from the shared on-disk cache) and returns that subset's run dicts
    (dummy lanes sliced).
    """
    (chunks, mc_cfg, quantize_thrusters, ctrl_every,
     batch_size, compact_mode, use_fastloop, sun_refs, backend) = job
    runs: list[dict] = []
    if backend == "cxx":
        from . import cxxdrive as _cxx
        if not _cxx.available:
            raise RuntimeError("cxxdrive extension unavailable")
        for seeds, run_offset, real_n in chunks:
            batch_res = _run_batch_cxx(seeds, mc_cfg, run_offset,
                                       quantize_thrusters, sun_refs)
            runs.extend(batch_res[:real_n])
        return runs
    ta = _make_batch_template(batch_size, mc_cfg, compact_mode)
    for seeds, run_offset, real_n in chunks:
        batch_res = _run_batch(seeds, mc_cfg, run_offset,
                               quantize_thrusters, ta, ctrl_every,
                               use_fastloop, sun_refs)
        runs.extend(batch_res[:real_n])
    return runs


def run_heyoka_monte_carlo(num_runs: int = 30, base_seed: int = 100,
                           mc_cfg=None, batch_size: int | None = None,
                           verbose: bool = True,
                           quantize_thrusters: bool = True,
                           ctrl_every: int = 1,
                           n_jobs: int = 1,
                           compact_mode: bool = False,
                           use_fastloop: bool = True,
                           backend: str = "batch") -> dict:
    """Tier-1 batch screening campaign; reuses Basilisk stats/plots schema.

    ``quantize_thrusters`` mirrors the Basilisk effector staircase (default
    True for parity; False gives the ideal proportional-thruster model).
    ``ctrl_every`` decimates control to every Nth tick (default 1; use 5 for 10 Hz screening with --ideal-thrusters).
    ``use_fastloop`` runs the per-tick lane loop in the C++ extension
    (numpy fallback when unavailable; results agree to ~1e-12 relative).
    ``backend="cxx"`` drives the whole chunk (including Taylor stepping) in
    the C++ Heyoka driver; RNG streams differ from numpy, so parity vs the
    batch backend is distributional (fuel means within ~2%).
    """
    from .monte_carlo import MonteCarloConfig, compute_monte_carlo_statistics
    if mc_cfg is None:
        mc_cfg = MonteCarloConfig()
    B = int(batch_size or _default_batch())
    seeds = [base_seed + i for i in range(num_runs)]
    if verbose:
        print("=" * 72)
        print(f"Heyoka Tier-1 batch screening: {num_runs} runs, batch={B}, "
              f"jobs={n_jobs}, compact={compact_mode}, backend={backend}")
        print("=" * 72)
    t0 = time.time()
    # Sun refs depend only on mc_cfg/dt: compute once per campaign and share
    # across all chunks/workers (pickled with the job at negligible cost).
    from .heyoka_attitude import precompute_sun_refs as _pre
    sun_refs = _pre(mc_cfg, np.asarray(mc_cfg.sun_roll_ref, float),
                    float(mc_cfg.task_step))
    if int(n_jobs) <= 1:
        runs: list[dict] = []
        if backend == "cxx":
            from . import cxxdrive as _cxx
            if not _cxx.available:
                raise RuntimeError("cxxdrive extension unavailable")
            for off in range(0, num_runs, B):
                chunk = seeds[off:off + B]
                real_n = len(chunk)
                if real_n < B:
                    chunk = chunk + [chunk[0]] * (B - real_n)
                t1 = time.time()
                batch_res = _run_batch_cxx(chunk, mc_cfg, off,
                                           quantize_thrusters, sun_refs)
                runs.extend(batch_res[:real_n])
                if verbose:
                    for r in runs[off:]:
                        k = r["kpis"]
                        print(f'[Run {r["run_idx"]:2d}/{num_runs}] [{"PASS" if r["success"] else "FAIL"}] '
                              f'Fuel={k["total_propellant_g"]:.2f}g Sun={k["final_sun_angle_deg"]:.2f}deg '
                              f'Slew={k["final_slew_error_deg"]:.1f}deg ({time.time()-t1:.2f}s/batch)')
        else:
            # One template integrator for the whole campaign: trailing short
            # chunks are padded with a repeated seed (results sliced back) so
            # every chunk reuses the same compiled code -- no re-JIT per chunk
            # or batch size.
            ta = _make_batch_template(B, mc_cfg, compact_mode)
            if verbose:
                print(f"[heyoka] template compiled; memcache={hy.llvm_state.get_memcache_size()} bytes")
            for off in range(0, num_runs, B):
                chunk = seeds[off:off + B]
                real_n = len(chunk)
                if real_n < B:
                    chunk = chunk + [chunk[0]] * (B - real_n)
                t1 = time.time()
                batch_res = _run_batch(chunk, mc_cfg, off, quantize_thrusters, ta,
                                       ctrl_every, use_fastloop, sun_refs)
                runs.extend(batch_res[:real_n])  # drop padded dummy lanes
                if verbose:
                    for r in runs[off:]:
                        k = r["kpis"]
                        print(f'[Run {r["run_idx"]:2d}/{num_runs}] [{"PASS" if r["success"] else "FAIL"}] '
                              f'Fuel={k["total_propellant_g"]:.2f}g Sun={k["final_sun_angle_deg"]:.2f}deg '
                              f'Slew={k["final_slew_error_deg"]:.1f}deg ({time.time()-t1:.2f}s/batch)')
    else:
        # Process fan-out (GIL-free; disk cache shared, mem cache per worker).
        # Deal whole B-sized chunks (not individual seeds) so workers never
        # run extra dummy lanes beyond the single trailing partial chunk.
        import concurrent.futures
        all_chunks: list[tuple[list[int], int, int]] = []
        for off in range(0, len(seeds), B):
            chunk = seeds[off:off + B]
            real_n = len(chunk)
            if real_n < B:
                chunk = chunk + [chunk[0]] * (B - real_n)
            all_chunks.append((chunk, off, real_n))
        grouped: list[list[tuple[list[int], int, int]]] = [
            [] for _ in range(int(n_jobs))]
        for i, ch in enumerate(all_chunks):
            grouped[i % int(n_jobs)].append(ch)
        jobs = [(grp, mc_cfg, quantize_thrusters, ctrl_every, B,
                 compact_mode, use_fastloop, sun_refs, backend)
                for grp in grouped if grp]
        runs = []
        with concurrent.futures.ProcessPoolExecutor(max_workers=int(n_jobs)) as ex:
            for grp_runs in ex.map(_run_subset, jobs):
                runs.extend(grp_runs)
                if verbose:
                    for r in grp_runs:
                        k = r["kpis"]
                        print(f'[Run {r["run_idx"]:2d}/{num_runs}] [{"PASS" if r["success"] else "FAIL"}] '
                              f'Fuel={k["total_propellant_g"]:.2f}g Sun={k["final_sun_angle_deg"]:.2f}deg '
                              f'Slew={k["final_slew_error_deg"]:.1f}deg')
        runs.sort(key=lambda r: r["run_idx"])
    wall = time.time() - t0
    stats = compute_monte_carlo_statistics(runs)
    if verbose:
        print("-" * 72)
        print(f"Done {num_runs} runs in {wall:.1f}s ({wall/max(1,num_runs):.2f}s/run)")
        print(f'Success: {stats["summary"]["success_rate_pct"]:.1f}%  '
              f'Cone: {stats["summary"]["sun_cone_compliance_pct"]:.1f}%')
        print("=" * 72)
    return {"config": asdict(mc_cfg), "stats": stats, "runs": runs,
            "wall_time_s": wall,
            "backend": "heyoka-cxx" if backend == "cxx" else "heyoka-batch"}


def openloop_ensemble(num_runs: int = 8, t_end: float = 60.0) -> list:
    """Dynamics-only validation sweep via Heyoka ensemble (no control).

    Each ensemble member gets dispersed r0/v0/sigma0/omega0; thrust pars are
    zero. Useful to validate the MRP system against Basilisk free-fall before
    trusting closed-loop screening.
    """
    from .monte_carlo import MonteCarloConfig, sample_dispersions
    mc_cfg = MonteCarloConfig()
    system, _ = build_mrp_system()
    template = hy.taylor_adaptive(
        system, [0.0] * 12,
        pars=[0.0] * N_PARS, tol=float(C.HEYOKA_TOL))
    ics = []
    for i in range(num_runs):
        d = sample_dispersions(1000 + i, mc_cfg)
        ics.append((pack_mrp_state(d["r0"], d["v0"], d["sigma0"], d["omega0"]),
                    pack_pars(np.zeros(3), np.zeros(3), float(d["mass"]),
                              np.asarray(d["inertia"], float))))

    def gen(ta_copy, idx):
        y0, p = ics[idx]
        ta_copy.time = 0.0
        ta_copy.state[:] = y0
        ta_copy.pars[:] = p
        return ta_copy

    return hy.ensemble_propagate_for(template, float(t_end), num_runs, gen)


def main(argv=None) -> int:
    import argparse
    from pathlib import Path
    from .monte_carlo import MonteCarloConfig, plot_monte_carlo, save_monte_carlo_results
    ap = argparse.ArgumentParser(description="Heyoka Tier-1 batch MC screening")
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--batch", type=int, default=None)
    ap.add_argument("--save-dir", type=str, default="results_heyoka_mc")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--ctrl-every", type=int, default=1,
                    help="control decimation: every Nth dynamics tick (default: 1; 5 = 10 Hz, ideal plant only)")
    ap.add_argument("--ideal-thrusters", action="store_true",
                    help="disable effector quantizer emulation (proportional model)")
    ap.add_argument("--jobs", type=int, default=1,
                    help="process-pool workers over seed groups (default: 1)")
    ap.add_argument("--compact-mode", action="store_true",
                    help="heyoka compact codegen (faster cold JIT on large systems; ~neutral here)")
    ap.add_argument("--no-fastloop", action="store_true",
                    help="force the numpy control path instead of the C++ extension")
    ap.add_argument("--backend", choices=["batch", "cxx"], default="batch",
                    help="batch: Python loop + Heyoka batch stepping; cxx: full C++ driver")
    a = ap.parse_args(argv)
    res = run_heyoka_monte_carlo(a.runs, a.seed, MonteCarloConfig(),
                                 batch_size=a.batch or _default_batch(),
                                 quantize_thrusters=not a.ideal_thrusters,
                                 ctrl_every=a.ctrl_every,
                                 n_jobs=a.jobs,
                                 compact_mode=a.compact_mode,
                                 use_fastloop=not a.no_fastloop,
                                 backend=a.backend)
    out = Path(a.save_dir)
    save_monte_carlo_results(res, output_dir=out)
    print(f"[info] saved to {out.resolve()}")
    if not a.no_plot:
        for p in plot_monte_carlo(res, output_dir=out):
            print(f"  - {p}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
