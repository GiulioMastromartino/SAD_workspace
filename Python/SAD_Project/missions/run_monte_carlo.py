"""Monte Carlo dispersed campaign (C++ modules)."""
import sys, os, json, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import sad_py
from SAD_Project.core import config as C
from SAD_Project.core import dynamics as D

def run_one(rng, Nsteps=None, tau_max=None):
    if tau_max is None:
        tau_max = C.ACS_TAU_MAX
    TS = C.TS
    TRACK_DUR = 2.0 * C.T_ORBIT
    MAX_TIME = TS + TRACK_DUR + 60.0
    # dispersions
    mass = C.MASS * (1 + rng.uniform(-0.05, 0.05))
    inertia = C.INERTIA.copy()
    for i in range(3):
        inertia[i,i] *= (1 + rng.uniform(-0.05, 0.05))
    # products (off-diagonal) small
    inertia[0,1] = inertia[1,0] = rng.uniform(-0.05, 0.05)*inertia[0,0]*0.1
    invI = np.linalg.inv(inertia)
    tip = np.deg2rad(rng.uniform(-1.5, 1.5, 3))
    q0 = np.array([0,0,0,1.0])
    # attitude error ~0.08 rad
    err = rng.normal(0, 0.08/3, 3)
    a = np.linalg.norm(err)
    if a > 1e-9:
        qerr = np.array([err[0]*np.sin(a/2)/a, err[1]*np.sin(a/2)/a, err[2]*np.sin(a/2)/a, np.cos(a/2)])
        q0 = D.quat_mul(qerr, q0)
        q0 = D.quat_normalize(q0)
    w0 = C.OMEGA0_BODY.copy() + tip
    dr = C.DISP_R_KM * 1000.0
    dv = C.DISP_V_MPS
    r0 = C.R0_ECI.copy() + np.array([rng.uniform(-dr, dr), rng.uniform(-dr, dr), rng.uniform(-dr, dr)])
    v0 = C.V0_ECI.copy() + np.array([rng.uniform(-dv, dv), rng.uniform(-dv, dv), rng.uniform(-dv, dv)])

    # thruster dispersion
    tmax = C.THRUSTER_TMAX.copy()
    tmax = tmax * (1 + rng.uniform(-0.05, 0.05, size=tmax.shape))
    # canting: perturb directions
    dirs = C.THRUSTER_DIR.copy().astype(float)
    for i in range(len(dirs)):
        dirs[i] = dirs[i] + rng.normal(0, np.deg2rad(1), 3)
        nrm = np.linalg.norm(dirs[i])
        if nrm > 0: dirs[i] = dirs[i]/nrm

    # star tracker sigma
    star_sigma = C.STAR_SIGMA * (1 + rng.uniform(-0.2, 0.2))

    # setup cpp
    cpp = sad_py.DynamicsParams()
    cpp.mass = mass
    cpp.mu = C.MU_EARTH
    cpp.gravity_gradient = True
    cpp.inertia = inertia
    cpp.inv_inertia = invI
    cpp.r0 = r0; cpp.v0 = v0; cpp.q0 = q0; cpp.w0 = w0
    st = sad_py.DynamicsState(); sad_py.dynamics_init(st, cpp)

    tp = sad_py.ThrusterParams()
    tp.n = len(C.THRUSTER_POS)
    tp.pos = [np.array(p, dtype=float) for p in C.THRUSTER_POS]
    tp.dir = [np.array(d, dtype=float) for d in C.THRUSTER_DIR]  # nominal geometry for robust allocation
    tp.tmax = list(tmax)
    tp.isp = C.ISP_JET; tp.g0 = C.G0
    ts = sad_py.ThrusterState(); sad_py.thruster_init(ts, tp)
    alloc = sad_py.ThrusterAllocator(); sad_py.thruster_allocator_init(alloc, tp)

    sp = sad_py.SunParams(); sp.distance = 1.496e11; sp.year = 365.25*86400; sp.phase0 = 0.5*np.pi
    sp.roll_ref = np.array([1.0, 0, 0])
    ss = sad_py.SunState(); sad_py.sun_init(ss, sp)

    qp = sad_py.QuatPDParams()
    qp.detumble_K = C.ACS_K_DETUMBLE; qp.detumble_P = C.ACS_P_DETUMBLE; qp.detumble_Ki = C.ACS_KI_DETUMBLE
    qp.slew_K = C.ACS_K_SLEW; qp.slew_P = C.ACS_P_SLEW; qp.slew_Ki = C.ACS_KI_SLEW
    qp.track_K = C.ACS_K_TRACK; qp.track_P = C.ACS_P_TRACK; qp.track_Ki = C.ACS_KI_TRACK
    qp.inertia = np.diag(C.INERTIA)
    qp.implicit_dt = bool(getattr(C, 'IMPLICIT_PD', True))
    qp.implicit_track_only = bool(getattr(C, 'IMPLICIT_TRACK_ONLY', True))
    qs = sad_py.QuatPDState(); sad_py.quatpd_init(qs, qp)

    mp = sad_py.MissionParams()
    ms = sad_py.MissionState(); sad_py.mission_init(ms, mp)

    stp = sad_py.StarTrackerParams(); stp.sigma = star_sigma; stp.rate_cap = C.STAR_RATE_CAP
    stp.dropout_prob = C.STAR_DROPOUT; stp.seed = int(rng.integers(0, 100000))
    sts = sad_py.StarTrackerState(); sad_py.startracker_init(sts, stp)

    # KPI tracking
    detumble_settle = None
    slew_settle = None
    _prev_phase = None
    slew_start_idx = None
    track_errors = []
    cone_violations = 0
    track_count = 0
    orbit_errs = []   # km radial deviation from design orbit (all phases)

    t = 0.0
    track_start = None
    failed = False
    _loop_guard = 0
    while _loop_guard < 200000:
        _loop_guard += 1
        if t >= TS + TRACK_DUR + 60.0:
            break
        dt = 0.1 if (ms.phase in (sad_py.MissionPhase.Detumble, sad_py.MissionPhase.Slew)) else C.TRACK_DT
        if ms.phase == sad_py.MissionPhase.Track:
            if track_start is None:
                track_start = t
            if (t - track_start) >= TRACK_DUR:
                break
        # abort if not in track by TS
        if (ms.phase != sad_py.MissionPhase.Track) and t > TS:
            failed = True
            break
        # sun: pin the free roll to the current body +Y so the target is the
        # nearest-roll sun attitude (avoids an unnecessary 180-deg roll)
        ss.params.roll_ref = D.quat_to_dcm(st.q).T @ np.array([0.0,1.0,0.0])
        si = sad_py.SunInput(); si.t = t
        so = sad_py.SunOutput()
        sad_py.sun_step(si, ss, so, dt)

        sti = sad_py.StarTrackerInput()
        sti.q_true = st.q.copy()
        sti.omega_body = st.w.copy()
        sti.sun_dir_b = np.zeros(3)
        sto = sad_py.StarTrackerOutput()
        sad_py.startracker_step(sti, sts, sto, dt)
        q_meas = sto.q_meas if sto.valid else st.q.copy()

        mi = sad_py.MissionInput(); mi.q = st.q.copy(); mi.omega = st.w.copy(); mi.q_sun = so.q_sun_pointing; mi.t = t
        mo = sad_py.MissionOutput()
        sad_py.mission_step(mi, ms, mo, dt)

        # record settle times on phase transitions
        if _prev_phase is not None and _prev_phase != mo.phase:
            if mo.phase == sad_py.MissionPhase.Slew and detumble_settle is None:
                detumble_settle = t
            if mo.phase == sad_py.MissionPhase.Track and slew_settle is None:
                slew_settle = t
        _prev_phase = mo.phase
        # track phase changes by recording when slew starts
        # (detumble->slew transition detected via mission output phase)

        qi = sad_py.QuatPDInput(); qi.q_current = st.q.copy(); qi.omega = st.w.copy()
        qi.q_desired = mo.q_desired; qi.mode = mo.ctrl_mode
        qo = sad_py.QuatPDOutput()
        sad_py.quatpd_step(qi, qs, qo, dt)
        tau = np.clip(qo.torque_b, -tau_max, tau_max)

        thr = sad_py.thruster_allocate_cached(alloc, tp, np.zeros(3), tau)
        throttle = [thr[i]/tmax[i] if tmax[i] > 0 else 0.0 for i in range(len(thr))]
        # clamp throttle
        throttle = [min(1.0, max(0.0, x)) for x in throttle]
        ti = sad_py.ThrusterInput(); ti.u = throttle; ti.is_on_time = False; ti.dt = dt
        to = sad_py.ThrusterOutput()
        sad_py.thruster_step(ti, ts, to, dt)

        ui = sad_py.DynamicsInput(); ui.force_b = to.force_b; ui.torque_b = to.torque_b
        uo = sad_py.DynamicsOutput()
        sad_py.dynamics_step(ui, st, uo, dt)
        t += dt
        if not (np.all(np.isfinite(st.r)) and np.all(np.isfinite(st.v)) and np.all(np.isfinite(st.q)) and np.all(np.isfinite(st.w))):
            failed = True
            break

        # KPIs
        rate = float(np.linalg.norm(st.w))
        if mo.phase == sad_py.MissionPhase.Slew:
            # angle error to slew target (90 deg about Z)
            qe = D.quat_mul(C.ACS_SLEW_QUAT, D.quat_conj(st.q))
            if qe[3] < 0: qe = -qe
            ang = 2*np.arctan2(np.linalg.norm(qe[:3]), abs(qe[3]))
            if slew_settle is None and ang < 0.05 and rate < 0.02:
                slew_settle = t
        elif mo.phase == sad_py.MissionPhase.Track:
            track_count += 1
            qe = D.quat_mul(mo.q_desired, D.quat_conj(st.q))
            if qe[3] < 0: qe = -qe
            ang = 2*np.arctan2(np.linalg.norm(qe[:3]), abs(qe[3]))
            track_errors.append(ang)
            # cone check: body +X within 15 deg of sun
            R = D.quat_to_dcm(st.q)
            sun_body = R @ so.sun_n
            cosang = float(sun_body[0])
            cosang = max(-1.0, min(1.0, cosang))
            ang_sun = np.degrees(np.arccos(cosang))
            if ang_sun > C.SUN_KEEP_IN_CONE_DEG:
                cone_violations += 1
            # orbit box: radial deviation from the design orbit (km)
            orbit_errs.append(abs(float(np.linalg.norm(st.r)) - C.R_ORBIT) / 1000.0)

    if failed:
        return {
            'detumble_settle': detumble_settle, 'slew_settle': slew_settle,
            'final_rate': None, 'mean_track_err': None, 'max_track_err': None,
            'track_count': track_count, 'cone_violations': cone_violations,
            'cone_compliance': ((1.0 - cone_violations / max(1, track_count))
                                if track_count > 0 else None),
            'fuel_used': float(ts.fuel_used),
            'reached_track': track_count > 0, 'failed': True,
            'orbit_err_km': (float(np.mean(orbit_errs)) if orbit_errs else None),
            'box_max_km': (float(np.max(orbit_errs)) if orbit_errs else None),
        }
    # residual rate at end:

    final_rate = float(np.linalg.norm(st.w))
    # slew residual error (if reached track, use last slew error else final)
    if track_errors:
        mean_track_err = float(np.mean(track_errors))
        max_track_err = float(np.max(track_errors))
    else:
        mean_track_err = None
        max_track_err = None
    cone_compliance = 1.0 - (cone_violations / max(1, track_count))
    orbit_err_mean = float(np.mean(orbit_errs)) if orbit_errs else None
    orbit_err_max = float(np.max(orbit_errs)) if orbit_errs else None
    return {
        'detumble_settle': detumble_settle,
        'slew_settle': slew_settle,
        'final_rate': final_rate,
        'mean_track_err': mean_track_err,
        'max_track_err': max_track_err,
        'track_count': track_count,
        'cone_violations': cone_violations,
        'cone_compliance': cone_compliance,
        'fuel_used': float(ts.fuel_used),
        'reached_track': track_count > 0,
        'orbit_err_km': orbit_err_mean,
        'box_max_km': orbit_err_max,
    }

def main():
    N = 30
    rng = np.random.default_rng(42)
    t0 = time.time()
    results = []
    for i in range(N):
        r = run_one(rng)
        results.append(r)
        if (i+1) % 5 == 0:
            print(f"  {i+1}/{N} done ({time.time()-t0:.1f}s)")
    # aggregate
    def agg(key):
        vals = [r[key] for r in results if r[key] is not None and not (isinstance(r[key],float) and np.isnan(r[key]))]
        if not vals: return None
        return {'mean': float(np.mean(vals)), 'std': float(np.std(vals)), 'min': float(np.min(vals)), 'max': float(np.max(vals)), 'n': len(vals)}
    summary = {
        'N': N,
        'elapsed_s': time.time()-t0,
        'reached_track_rate': float(np.mean([r['reached_track'] for r in results])),
        'detumble_settle': agg('detumble_settle'),
        'slew_settle': agg('slew_settle'),
        'final_rate': agg('final_rate'),
        'mean_track_err': agg('mean_track_err'),
        'max_track_err': agg('max_track_err'),
        'cone_compliance': agg('cone_compliance'),
        'fuel_used': agg('fuel_used'),
        'track_count': agg('track_count'),
        'orbit_err_km': agg('orbit_err_km'),
        'box_max_km': agg('box_max_km'),
    }
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'results_mc_cpp')
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'mc_summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    np.savez(os.path.join(out_dir, 'mc_results.npz'), results=np.array(results, dtype=object))
    # always generate cost map
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fuels = np.array([r['fuel_used'] for r in results])
        times = np.array([ (r['slew_settle'] if r['slew_settle'] is not None else (r['detumble_settle'] if r['detumble_settle'] is not None else 120.0)) for r in results])
        _costs = []
        for r in results:
            fr = r['final_rate']
            if fr is None or (isinstance(fr, float) and (np.isnan(fr) or np.isinf(fr))):
                _costs.append(50.0)
            else:
                _costs.append(float(fr) + (0.0 if r['reached_track'] else 10.0))
        costs = np.array(_costs)
        fig, ax = plt.subplots(figsize=(8,6))
        sc = ax.scatter(fuels, times, c=costs, cmap='viridis_r', s=60, edgecolors='k', linewidths=0.5)
        plt.colorbar(sc, ax=ax, label='Cost')
        ax.set_xlabel('Fuel consumption (kg)')
        ax.set_ylabel('Total time detumble+slew (s)')
        ax.set_title('Cost surface: fuel vs detumble+slew time')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, 'cost_map.png'), dpi=150)
        plt.close(fig)
        print('cost map saved:', os.path.join(out_dir, 'cost_map.png'))
    except Exception as e:
        print('cost map generation failed:', e)
    print(json.dumps(summary, indent=2))

if __name__ == '__main__':
    main()
