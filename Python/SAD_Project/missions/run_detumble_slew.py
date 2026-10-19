"""Detumble -> slew -> track mission using C++ modules."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import numpy as np
import sad_py
from SAD_Project.core import config as C
from SAD_Project.core import sun_model as SM
from SAD_Project.core import dynamics as D

def main():
    # params
    inertia = C.INERTIA
    invI = C.INV_INERTIA
    cpp_params = sad_py.DynamicsParams()
    cpp_params.mass = C.MASS
    cpp_params.mu = C.MU_EARTH
    cpp_params.gravity_gradient = True
    cpp_params.inertia = inertia
    cpp_params.inv_inertia = invI
    cpp_params.r0 = C.R0_ECI.copy()
    cpp_params.v0 = C.V0_ECI.copy()
    cpp_params.q0 = np.array([0,0,0,1.0])
    cpp_params.w0 = C.OMEGA0_BODY.copy()

    st = sad_py.DynamicsState(); sad_py.dynamics_init(st, cpp_params)

    # thruster
    tp = sad_py.ThrusterParams()
    tp.n = len(C.THRUSTER_POS)
    tp.pos = [np.array(p,dtype=float) for p in C.THRUSTER_POS]
    tp.dir = [np.array(d,dtype=float) for d in C.THRUSTER_DIR]
    tp.tmax = list(C.THRUSTER_TMAX)
    tp.isp = C.ISP_JET; tp.g0 = C.G0
    ts = sad_py.ThrusterState(); sad_py.thruster_init(ts, tp)
    alloc = sad_py.ThrusterAllocator(); sad_py.thruster_allocator_init(alloc, tp)

    # sun
    sp = sad_py.SunParams(); sp.distance = 1.496e11; sp.year = 365.25*86400; sp.phase0 = 0.5*np.pi
    sp.roll_ref = np.array([1.0,0,0])
    ss = sad_py.SunState(); sad_py.sun_init(ss, sp)

    # controller
    qp = sad_py.QuatPDParams()
    qp.detumble_K = C.ACS_K_DETUMBLE; qp.detumble_P = C.ACS_P_DETUMBLE; qp.detumble_Ki = C.ACS_KI_DETUMBLE
    qp.slew_K = C.ACS_K_SLEW; qp.slew_P = C.ACS_P_SLEW; qp.slew_Ki = C.ACS_KI_SLEW
    qp.track_K = C.ACS_K_TRACK; qp.track_P = C.ACS_P_TRACK; qp.track_Ki = C.ACS_KI_TRACK
    qp.inertia = np.diag(C.INERTIA)
    qp.implicit_dt = bool(getattr(C, 'IMPLICIT_PD', True))
    # detumble/slew use dt=0.1 below, so only the track phase needs scaling
    qp.implicit_track_only = True
    qs = sad_py.QuatPDState(); sad_py.quatpd_init(qs, qp)

    # mission
    mp = sad_py.MissionParams()
    ms = sad_py.MissionState(); sad_py.mission_init(ms, mp)

    # star tracker
    stp = sad_py.StarTrackerParams(); stp.sigma = C.STAR_SIGMA; stp.rate_cap = C.STAR_RATE_CAP
    stp.dropout_prob = C.STAR_DROPOUT; stp.seed = 7
    sts = sad_py.StarTrackerState(); sad_py.startracker_init(sts, stp)

    T = C.T_END
    phases = []
    t = 0.0
    while t < T:
        # detumble/slew at 0.1 s, track at the configured (larger) step
        dt = C.DT if ms.phase in (sad_py.MissionPhase.Detumble,
                                   sad_py.MissionPhase.Slew) else C.TRACK_DT
        # sun: nearest-roll sun target (body +Y projected onto the sun plane)
        ss.params.roll_ref = D.quat_to_dcm(st.q).T @ np.array([0.0,1.0,0.0])
        si = sad_py.SunInput(); si.t = t
        so = sad_py.SunOutput()
        sad_py.sun_step(si, ss, so, dt)

        # star tracker measurement (true q from state)
        sti = sad_py.StarTrackerInput()
        sti.q_true = st.q.copy()
        sti.omega_body = st.w.copy()
        sti.sun_dir_b = np.zeros(3)
        sto = sad_py.StarTrackerOutput()
        sad_py.startracker_step(sti, sts, sto, dt)
        q_meas = sto.q_meas if sto.valid else st.q.copy()

        # mission
        mi = sad_py.MissionInput(); mi.q = st.q.copy(); mi.omega = st.w.copy(); mi.q_sun = so.q_sun_pointing; mi.t = t
        mo = sad_py.MissionOutput()
        sad_py.mission_step(mi, ms, mo, dt)

        # controller
        qi = sad_py.QuatPDInput(); qi.q_current = st.q.copy(); qi.omega = st.w.copy()
        qi.q_desired = mo.q_desired; qi.mode = mo.ctrl_mode
        qo = sad_py.QuatPDOutput()
        sad_py.quatpd_step(qi, qs, qo, dt)
        tau = qo.torque_b

        # thruster allocation: convert torque to thruster commands via pinv for force=0
        thr = sad_py.thruster_allocate_cached(alloc, tp, np.zeros(3), tau)
        # throttle
        throttle = [thr[i]/C.THRUSTER_TMAX[i] if C.THRUSTER_TMAX[i]>0 else 0.0 for i in range(len(thr))]
        # step thruster
        ti = sad_py.ThrusterInput(); ti.u = throttle; ti.is_on_time = False; ti.dt = dt
        to = sad_py.ThrusterOutput()
        sad_py.thruster_step(ti, ts, to, dt)

        # dynamics step
        ui = sad_py.DynamicsInput(); ui.force_b = to.force_b; ui.torque_b = to.torque_b
        uo = sad_py.DynamicsOutput()
        sad_py.dynamics_step(ui, st, uo, dt)
        phases.append(int(mo.phase))
        t += dt

    # summary
    from collections import Counter
    print("Phase counts:", Counter(phases))
    print("Final phase:", phases[-1])
    print("Final rate norm:", float(np.linalg.norm(st.w)))
    print("Fuel used (kg):", ts.fuel_used)

if __name__ == '__main__':
    main()
