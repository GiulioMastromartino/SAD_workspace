"""Wall-time dissection for Basilisk stock, Heyoka Python, and C++ cxxdrive.

Instruments every major component (JIT, setup, per-tick control, propagation,
logging, KPIs) and prints a breakdown table per backend.

Usage (from Python/):
    ./env/bin/python -m cubesat_12u_sim.tuning.run_dissect
"""
from __future__ import annotations

import sys
import time

import numpy as np


def _pct(t: float, total: float) -> str:
    return f"{100.0 * t / total:5.1f}%"


def _row(label: str, t: float, total: float) -> str:
    return f"  {label:<38} {t * 1000:9.2f} ms  {_pct(t, total)}"


# =====================================================================
# 1. Basilisk stock
# =====================================================================
def dissect_basilisk(duration: float = 600.0) -> dict:
    from Basilisk.architecture import messaging
    from Basilisk.fswAlgorithms import (
        attTrackingError, inertial3D, mrpFeedback, rwMotorTorque,
    )
    from Basilisk.simulation import (
        reactionWheelStateEffector, simpleNav, spacecraft,
    )
    from Basilisk.utilities import (
        SimulationBaseClass, fswSetupRW, macros, orbitalMotion,
        simHelpers, simIncludeGravBody, simIncludeRW,
    )

    T = {}

    # --- init ---
    t0 = time.perf_counter()
    task_name = "simTask"
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("simProcess")
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(0.1)))
    T["init_base"] = time.perf_counter() - t0

    # --- spacecraft ---
    t0 = time.perf_counter()
    I = [900.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 600.0]
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = 750.0
    sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I)
    sim.AddModelToTask(task_name, sc, 1)
    T["spacecraft"] = time.perf_counter() - t0

    # --- gravity ---
    t0 = time.perf_counter()
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    grav.addBodiesTo(sc)
    T["gravity"] = time.perf_counter() - t0

    # --- RW ---
    t0 = time.perf_counter()
    rw_factory = simIncludeRW.rwFactory()
    rw_factory.create("Honeywell_HR16", [1, 0, 0], maxMomentum=50.0, Omega=100.0)
    rw_factory.create("Honeywell_HR16", [0, 1, 0], maxMomentum=50.0, Omega=200.0)
    rw_factory.create("Honeywell_HR16", [0, 0, 1], maxMomentum=50.0, Omega=300.0,
                      rWB_B=[0.5, 0.5, 0.5])
    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = "RW_cluster"
    rw_factory.addToSpacecraft(rw_effector.ModelTag, rw_effector, sc)
    sim.AddModelToTask(task_name, rw_effector, 2)
    T["rw_setup"] = time.perf_counter() - t0

    # --- nav + ref + err + ctl + alloc ---
    t0 = time.perf_counter()
    nav = simpleNav.SimpleNav()
    sim.AddModelToTask(task_name, nav)
    ref = inertial3D.inertial3D()
    sim.AddModelToTask(task_name, ref)
    ref.sigma_R0N = [0.0, 0.0, 0.0]
    err = attTrackingError.attTrackingError()
    sim.AddModelToTask(task_name, err)
    ctl = mrpFeedback.mrpFeedback()
    sim.AddModelToTask(task_name, ctl)
    ctl.K = 3.5
    ctl.Ki = -1
    ctl.P = 30.0
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1
    alloc = rwMotorTorque.rwMotorTorque()
    sim.AddModelToTask(task_name, alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    T["fsw_setup"] = time.perf_counter() - t0

    # --- messages + ICs ---
    t0 = time.perf_counter()
    vc_msg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=I))
    fsw_msg = rw_factory.getConfigMessage()
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    sc.hub.r_CN_NInit = rN
    sc.hub.v_CN_NInit = vN
    sc.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    sc.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]
    nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
    err.attNavInMsg.subscribeTo(nav.attOutMsg)
    err.attRefInMsg.subscribeTo(ref.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
    ctl.vehConfigInMsg.subscribeTo(vc_msg)
    ctl.rwParamsInMsg.subscribeTo(fsw_msg)
    ctl.rwSpeedsInMsg.subscribeTo(rw_effector.rwSpeedOutMsg)
    alloc.rwParamsInMsg.subscribeTo(fsw_msg)
    alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
    rw_effector.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)
    T["messages_ics"] = time.perf_counter() - t0

    # --- recorders ---
    t0 = time.perf_counter()
    state_log = sc.scStateOutMsg.recorder()
    err_log = err.attGuidOutMsg.recorder()
    torque_log = ctl.cmdTorqueOutMsg.recorder()
    motor_log = alloc.rwMotorTorqueOutMsg.recorder()
    speed_log = rw_effector.rwSpeedOutMsg.recorder()
    for rec in (state_log, err_log, torque_log, motor_log, speed_log):
        sim.AddModelToTask(task_name, rec)
    T["recorders"] = time.perf_counter() - t0

    # --- initialize ---
    t0 = time.perf_counter()
    sim.InitializeSimulation()
    T["initialize"] = time.perf_counter() - t0

    # --- execute ---
    t0 = time.perf_counter()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    T["execute"] = time.perf_counter() - t0

    # --- extract ---
    t0 = time.perf_counter()
    ns = macros.NANO2SEC
    result = {
        "t": state_log.times() * ns,
        "sigma_BN": np.asarray(state_log.sigma_BN, float),
        "omega_BN_B": np.asarray(state_log.omega_BN_B, float),
        "r_CN_N": np.asarray(state_log.r_CN_N, float),
        "v_CN_N": np.asarray(state_log.v_CN_N, float),
        "sigma_BR": np.asarray(err_log.sigma_BR, float),
        "omega_BR_B": np.asarray(err_log.omega_BR_B, float),
        "torque_demand": np.asarray(torque_log.torqueRequestBody, float),
        "motor_torque": np.asarray(motor_log.motorTorque, float)[:, :3],
        "wheel_speeds": np.asarray(speed_log.wheelSpeeds, float)[:, :3],
    }
    T["extract"] = time.perf_counter() - t0

    T["total"] = sum(T.values())
    return result, T


# =====================================================================
# 2. Heyoka Python (RW mirror)
# =====================================================================
def dissect_heyoka_rw(duration: float = 600.0, y0: np.ndarray | None = None) -> tuple[dict, dict]:
    import heyoka as hy
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.backends.heyoka.heyoka_rw import (
        RW_K, RW_P, RW_U_MAX, RW_JS, pack_rw_state, rw_allocate,
        make_rw_integrator,
    )
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff

    T = {}

    # --- system build + JIT ---
    t0 = time.perf_counter()
    if y0 is None:
        raise ValueError("y0 required")
    ta = make_rw_integrator(y0, np.zeros(3), 750.0, np.diag([900.0, 800.0, 600.0]))
    T["jit_build"] = time.perf_counter() - t0

    # --- pre-loop setup ---
    t0 = time.perf_counter()
    n = int(round(duration / 0.1))
    ts = np.zeros(n + 1)
    Y = np.zeros((15, n + 1))
    U = np.zeros((3, n + 1))
    Y[:, 0] = np.asarray(y0, float)
    zero3 = np.zeros(3)
    T["setup"] = time.perf_counter() - t0

    # --- main loop ---
    t_mrp = 0.0
    t_pd = 0.0
    t_alloc = 0.0
    t_prop = 0.0
    t_shadow = 0.0

    for k in range(1, n + 1):
        st = Y[:, k - 1]

        ta0 = time.perf_counter()
        e = mrp_diff(st[6:9], zero3)
        ta1 = time.perf_counter()
        t_mrp += ta1 - ta0

        Lr = -RW_K * e - RW_P * st[9:12]
        ta2 = time.perf_counter()
        t_pd += ta2 - ta1

        u = rw_allocate(Lr)
        ta3 = time.perf_counter()
        t_alloc += ta3 - ta2

        ta.pars[0], ta.pars[1], ta.pars[2] = map(float, u)
        ta.propagate_for(0.1)
        ta4 = time.perf_counter()
        t_prop += ta4 - ta3

        snew = np.asarray(ta.state, float)
        if float(snew[6:9] @ snew[6:9]) > 1.0:
            snew[6:9] = -snew[6:9] / float(snew[6:9] @ snew[6:9])
            ta.state[6:9] = snew[6:9]
        ta5 = time.perf_counter()
        t_shadow += ta5 - ta4

        Y[:, k] = snew
        U[:, k] = u
        ts[k] = ta.time

    T["mrp_diff"] = t_mrp
    T["pd_ctrl"] = t_pd
    T["rw_alloc"] = t_alloc
    T["propagate"] = t_prop
    T["shadow_reset"] = t_shadow
    T["loop_total"] = t_mrp + t_pd + t_alloc + t_prop + t_shadow
    T["extract"] = 0.0  # already in Y
    T["total"] = T["jit_build"] + T["setup"] + T["loop_total"]

    result = {"t": ts, "Y": Y, "u_motor": U, "dt": 0.1}
    return result, T


# =====================================================================
# 3. C++ cxxdrive (thruster MC)
# =====================================================================
def dissect_cxxdrive(duration: float = 600.0, n_lanes: int = 8) -> tuple[dict, dict]:
    from cubesat_12u_sim.core import config as C
    from cubesat_12u_sim.native import cxxdrive as _cxx
    from cubesat_12u_sim.backends.heyoka.heyoka_attitude import precompute_sun_refs, sun_direction_at, wrench_setup
    from cubesat_12u_sim.uncertainty.monte_carlo import sample_dispersions, MonteCarloConfig
    from cubesat_12u_sim.core.actuators import allocation_matrices

    if not _cxx.available:
        raise RuntimeError("cxxdrive extension unavailable")

    T = {}

    # --- config + dispersions ---
    t0 = time.perf_counter()
    mc_cfg = MonteCarloConfig(
        task_step=0.1, log_step=1.0,
        t_detumble=duration * 0.4, t_slew=duration * 0.3,
        t_sunpoint=duration * 0.3,
        sigma_slew=(0.0, 0.0, 0.0),
        sun_roll_ref=(0.0, 0.0, 0.0),
        sun_pointing_axis_b=(0.0, 0.0, 1.0),
        rate_settle_thresh_deg_s=0.1,
        slew_settle_thresh_deg=1.0,
        sun_cone_angle_deg=10.0,
    )
    seeds = list(range(100, 100 + n_lanes))
    dt = float(mc_cfg.task_step)
    n_steps = int(round(mc_cfg.t_total / dt))
    log_every = max(1, int(round(float(mc_cfg.log_step) / dt)))
    disps = [sample_dispersions(int(s), mc_cfg) for s in seeds]
    T["config_dispersions"] = time.perf_counter() - t0

    # --- sun refs + REFS + PH ---
    t0 = time.perf_counter()
    sun_refs = precompute_sun_refs(mc_cfg, np.asarray(mc_cfg.sun_roll_ref, float), dt)
    sigma_init = np.array([np.asarray(d["sigma0"], float) for d in disps])
    k_det = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                float(mc_cfg.t_detumble), side="left"))
    k_slew = int(np.searchsorted(np.arange(n_steps + 1) * dt,
                                 float(mc_cfg.t_detumble + mc_cfg.t_slew),
                                 side="left"))
    REFS = np.empty((n_steps + 1, n_lanes, 3))
    REFS[:k_det] = sigma_init[None, :, :]
    REFS[k_det:k_slew] = np.asarray(mc_cfg.sigma_slew, float)[None, None, :]
    REFS[k_slew:] = np.asarray(sun_refs[k_slew:])[:, None, :]
    PH = np.zeros(n_steps + 1, dtype=np.int64)
    PH[k_det:k_slew] = 1
    PH[k_slew:] = 2
    T["refs_setup"] = time.perf_counter() - t0

    # --- wrench setup ---
    t0 = time.perf_counter()
    ws_all = [wrench_setup(d) for d in disps]
    P_nom = allocation_matrices(C.THRUSTER_POS, C.THRUSTER_DIR, C.THRUSTER_TMAX)
    T["wrench_setup"] = time.perf_counter() - t0

    # --- JIT build (first call includes LLVM compile) ---
    t0 = time.perf_counter()
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
        True, float(C.HEYOKA_TOL), False,
        float(C.MU_EARTH), bool(C.USE_GRAVITY_GRADIENT))
    T["jit_and_run"] = time.perf_counter() - t0

    # --- second call (warm JIT, pure run) ---
    t0 = time.perf_counter()
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
        True, float(C.HEYOKA_TOL), False,
        float(C.MU_EARTH), bool(C.USE_GRAVITY_GRADIENT))
    T["warm_run"] = time.perf_counter() - t0

    T["jit_cold"] = T["jit_and_run"] - T["warm_run"]
    T["total"] = T["config_dispersions"] + T["refs_setup"] + T["wrench_setup"] + T["jit_and_run"]

    result = {
        "omega_norm": np.asarray(out["omega_norm"], float),
        "sigma_err": np.asarray(out["sigma_err"], float),
        "sun_angle": np.asarray(out["sun_angle"], float),
        "on_time": np.asarray(out["on_time"], float),
        "phase_fuel": np.asarray(out["phase_fuel"], float),
    }
    return result, T


# =====================================================================
# Print
# =====================================================================
def print_dissect(name: str, T: dict) -> None:
    total = T["total"]
    print(f"\n{'=' * 70}")
    print(f"  {name}")
    print(f"{'=' * 70}")
    print(f"  {'Component':<38} {'Time':>12} {'% of total':>10}")
    print(f"  {'-' * 38} {'-' * 12} {'-' * 10}")
    for key, val in T.items():
        if key == "total":
            continue
        print(_row(key, val, total))
    print(f"  {'-' * 38} {'-' * 12} {'-' * 10}")
    print(f"  {'TOTAL':<38} {total * 1000:9.2f} ms  100.0%")


def main() -> int:
    duration = 600.0
    n_lanes = 8

    print("=" * 70)
    print("WALL-TIME DISSECTION")
    print("=" * 70)
    print(f"Scenario: {duration:.0f} s, dt=0.1 s ({int(duration / 0.1)} steps)")

    # --- Basilisk ---
    print("\n[1/3] Dissecting Basilisk stock...")
    try:
        bsk_result, bsk_T = dissect_basilisk(duration)
        print_dissect("Basilisk stock (RW demo)", bsk_T)
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        bsk_T = None

    # --- Heyoka Python ---
    print("\n[2/3] Dissecting Heyoka Python RW mirror...")
    try:
        from cubesat_12u_sim.backends.heyoka.heyoka_rw import pack_rw_state
        y0 = pack_rw_state(
            bsk_result["r_CN_N"][0], bsk_result["v_CN_N"][0],
            bsk_result["sigma_BN"][0], bsk_result["omega_BN_B"][0],
            bsk_result["wheel_speeds"][0]) if bsk_T else None
        hey_result, hey_T = dissect_heyoka_rw(duration, y0=y0)
        print_dissect("Heyoka Python (RW mirror)", hey_T)
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        hey_T = None

    # --- C++ cxxdrive ---
    print(f"\n[3/3] Dissecting C++ cxxdrive ({n_lanes} lanes)...")
    try:
        cxx_result, cxx_T = dissect_cxxdrive(duration, n_lanes=n_lanes)
        print_dissect("C++ cxxdrive (thruster MC)", cxx_T)
    except Exception as e:
        print(f"  FAILED: {e}")
        import traceback; traceback.print_exc()
        cxx_T = None

    # --- Summary ---
    print(f"\n{'=' * 70}")
    print("SUMMARY")
    print(f"{'=' * 70}")
    if bsk_T:
        print(f"  Basilisk stock:  {bsk_T['total'] * 1000:9.2f} ms")
    if hey_T:
        print(f"  Heyoka Python:   {hey_T['total'] * 1000:9.2f} ms")
    if cxx_T:
        print(f"  C++ cxxdrive:    {cxx_T['total'] * 1000:9.2f} ms")

    if bsk_T and hey_T:
        print(f"\n  Heyoka Python speedup: {bsk_T['total'] / hey_T['total']:.2f}x")
    if bsk_T and cxx_T:
        print(f"  C++ cxxdrive speedup:  {bsk_T['total'] / cxx_T['total']:.2f}x")

    return 0


if __name__ == "__main__":
    sys.exit(main())
