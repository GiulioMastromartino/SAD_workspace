#include "sad_mission_batch.h"
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#ifdef SAD_WITH_HEYOKA
#include "sad_dynamics_heyoka.h"
#include "sad_dynamics_heyoka_track.h"
#endif

namespace sad {

// Standard 12-thruster 6-DOF geometry (same layout as core/config.py):
// 4 on +/-X, 4 on +/-Z, 4 on +/-Y, at the face edges.  Rank 6, full +/- authority.
static const int NTHR = 12;
static const double THR_POS[12][3] = {
    { 0.05,  0.10, 0.0}, { 0.05, -0.10, 0.0},   // +X face, push -X
    {-0.05,  0.10, 0.0}, {-0.05, -0.10, 0.0},   // -X face, push +X
    { 0.05, 0.0,  0.15}, {-0.05, 0.0,  0.15},   // +Z face, push -Z
    { 0.05, 0.0, -0.15}, {-0.05, 0.0, -0.15},   // -Z face, push +Z
    {0.0,  0.10,  0.15}, {0.0,  0.10, -0.15},   // +Y face, push -Y
    {0.0, -0.10,  0.15}, {0.0, -0.10, -0.15},   // -Y face, push +Y
};
static const double THR_DIR[12][3] = {
    {-1,0,0},{-1,0,0},
    { 1,0,0},{ 1,0,0},
    {0,0,-1},{0,0,-1},
    {0,0, 1},{0,0, 1},
    {0,-1,0},{0,-1,0},
    {0, 1,0},{0, 1,0},
};
static const double THR_TMAX0 = 0.5;  // 6U bus; 12U reference uses 1.0 N

static inline int phase_index(MissionPhase p) {
    switch (p) {
        case MissionPhase::Detumble: return 0;
        case MissionPhase::Slew:     return 1;
        default:                     return 2; // Track
    }
}

void run_mission_batch(int N, unsigned seed, const BatchParams &bp, BatchResult &out) {
    out.final_rate.assign(N, 0.0);
    out.fuel_used.assign(N, 0.0);
    out.detumble_settle.assign(N, 0.0);
    out.slew_settle.assign(N, 0.0);
    out.orbit_err.assign(N, 0.0);
    out.box_max.assign(N, 0.0);
    out.track_ang_mean.assign(N, 0.0);
    out.track_ang_max.assign(N, 0.0);
    out.reached_track.assign(N, 0);
    out.failed.assign(N, 0);

    // fresh timing payload
    out.timing = BatchTiming();
    out.timing.enabled = bp.timing;
    out.timing.run_ms.clear();
    out.timing.run_setup_ms.clear();
    out.timing.run_dispersion_ms.clear();
    out.timing.run_steps.clear();
    out.timing.run_phase_steps.clear();

    const bool timing = bp.timing;
    g_timing.reset();
    g_timing.enabled = timing;
    double _batch_t0 = timing ? sad_now_ns() : 0.0;

    std::mt19937 rng(seed);
    const bool cached_alloc = bp.cached_alloc;

#ifdef SAD_WITH_HEYOKA
    // One compiled Taylor system is reused across all runs and optimizer calls.
    HeyokaDynamics *hey = nullptr;
    if (bp.use_heyoka) {
        hey = &heyoka_dynamics_shared(bp.heyoka_tol, bp.heyoka_compact);
    }
#else
    if (bp.use_heyoka) {
        throw std::runtime_error("Heyoka backend is not available in this build");
    }
#endif

    double T_ORBIT = 2.0 * M_PI * std::sqrt(bp.R_ORBIT*bp.R_ORBIT*bp.R_ORBIT / bp.MU);

    // nominal inertia for 6U
    double mass0 = bp.mass;
    double IX = mass0/12.0*(0.2*0.2+0.3*0.3);
    double IY = mass0/12.0*(0.1*0.1+0.3*0.3);
    double IZ = mass0/12.0*(0.1*0.1+0.2*0.2);
    Mat3 inertia0 = Mat3::Zero();
    inertia0(0,0)=IX; inertia0(1,1)=IY; inertia0(2,2)=IZ;
    Mat3 invI0 = inertia0.inverse();

#ifdef SAD_WITH_HEYOKA
    HeyokaTrack* hey_track = nullptr;
    if (bp.track_embedded) hey_track = &heyoka_track_shared(bp.heyoka_tol, bp.heyoka_compact);
#endif

    for (int run = 0; run < N; ++run) {
        double _run_t0 = timing ? sad_now_ns() : 0.0;

        // ---------------- dispersion ----------------
        double _disp_t0 = timing ? sad_now_ns() : 0.0;
        double mass = mass0;
        Mat3 inertia = inertia0;
        Mat3 invI = invI0;
        Vec3 tip = Vec3::Zero();
        Vec4 q0(0,0,0,1);
        Vec3 w0(0.017, 0.0087, -0.0087); // OMEGA0
        Vec3 r0(bp.R_ORBIT, 0, 0);
        Vec3 v0(0, std::sqrt(bp.MU/bp.R_ORBIT), 0);

        if (bp.disperse) {
            std::uniform_real_distribution<double> ud(-0.05, 0.05);
            mass = mass0 * (1 + ud(rng));
            for (int i=0;i<3;++i) inertia(i,i) *= (1 + ud(rng));
            invI = inertia.inverse();
            // tip-off
            std::uniform_real_distribution<double> ud2(-1.5*3.14159/180.0, 1.5*3.14159/180.0);
            tip = Vec3(ud2(rng), ud2(rng), ud2(rng));
            w0 = Vec3(0.017,0.0087,-0.0087) + tip;
            // attitude error
            std::normal_distribution<double> nd(0, 0.08/3.0);
            Vec3 err(nd(rng), nd(rng), nd(rng));
            double a = err.norm();
            if (a > 1e-9) {
                Vec4 qerr(err[0]*std::sin(a/2)/a, err[1]*std::sin(a/2)/a, err[2]*std::sin(a/2)/a, std::cos(a/2));
                q0 = quat_normalize(quat_mul(qerr, q0));
            }
            // orbit injection
            std::uniform_real_distribution<double> ud3(-bp.disp_r_m, bp.disp_r_m);
            r0 = Vec3(bp.R_ORBIT + ud3(rng), ud3(rng), ud3(rng));
            std::uniform_real_distribution<double> ud4(-bp.disp_v_mps, bp.disp_v_mps);
            v0 = Vec3(ud4(rng), std::sqrt(bp.MU/bp.R_ORBIT)+ud4(rng), ud4(rng));
        }
        if (timing) g_timing.add(TDispersion, sad_now_ns() - _disp_t0);

        // ---------------- per-run setup ----------------
        double _setup_t0 = timing ? sad_now_ns() : 0.0;
        // thruster params
        ThrusterParams tp;
        tp.n = NTHR;
        tp.pos.resize(NTHR); tp.dir.resize(NTHR); tp.tmax.resize(NTHR);
        std::vector<double> tmax(NTHR, THR_TMAX0);
        if (bp.disperse) {
            std::uniform_real_distribution<double> ud(-0.05, 0.05);
            for (int i=0;i<NTHR;++i) tmax[i] = THR_TMAX0 * (1 + ud(rng));
        }
        for (int i=0;i<NTHR;++i) {
            tp.pos[i] = Vec3(THR_POS[i][0], THR_POS[i][1], THR_POS[i][2]);
            tp.dir[i] = Vec3(THR_DIR[i][0], THR_DIR[i][1], THR_DIR[i][2]);
            tp.tmax[i] = tmax[i];
        }
        tp.isp = 220.0; tp.g0 = 9.80665;

        // The influence matrix depends only on the (dispersed) geometry, so the
        // pseudo-inverse is built once per run and reused for every control step.
        ThrusterAllocator alloc;
        thruster_allocator_init(alloc, tp);

        // star tracker
        StarTrackerParams stp;
        stp.sigma = 1e-4; stp.rate_cap = 0.1; stp.dropout_prob = 0.05; stp.seed = seed + run;
        if (bp.disperse) {
            std::uniform_real_distribution<double> ud(-0.2, 0.2);
            stp.sigma = 1e-4 * (1 + ud(rng));
        }

        // dynamics params
        DynamicsParams dp;
        dp.mass = mass; dp.mu = bp.MU; dp.gravity_gradient = true;
        dp.inertia = inertia; dp.inv_inertia = invI;
        dp.r0 = r0; dp.v0 = v0; dp.q0 = q0; dp.w0 = w0;

        // state
        DynamicsState ds; dynamics_init(ds, dp);
#ifdef SAD_WITH_HEYOKA
        if (hey) hey->reset(ds);
#endif
        ThrusterState ts; thruster_init(ts, tp);
        SunParams sp; sp.distance=1.496e11; sp.year=365.25*86400; sp.phase0=0.5*M_PI; sp.roll_ref=Vec3(1,0,0);
        SunState ss; sun_init(ss, sp);
        QuatPDParams qp;
        qp.detumble_K=bp.detumble_K; qp.detumble_P=bp.detumble_P; qp.detumble_Ki=bp.detumble_Ki;
        qp.slew_K=bp.slew_K; qp.slew_P=bp.slew_P; qp.slew_Ki=bp.slew_Ki;
        qp.track_K=bp.track_K; qp.track_P=bp.track_P; qp.track_Ki=bp.track_Ki;
        // nominal inertia for the step-aware PD (not the dispersed plant value)
        qp.inertia = Vec3(IX, IY, IZ);
        qp.implicit_dt = bp.implicit_pd;
        qp.implicit_track_only = bp.implicit_track_only;
        QuatPDState qs; quatpd_init(qs, qp);
        MissionParams mp; mp.detumble_rate_tol=0.05; mp.detumble_time_min=2.0; mp.slew_angle_tol=0.1; mp.slew_rate_tol=0.05;
        MissionState ms; mission_init(ms, mp);
        StarTrackerState sts; startracker_init(sts, stp);
        if (timing) g_timing.add(TPerRunSetup, sad_now_ns() - _setup_t0);

        Vec4 q_slew(0,0,std::sin(M_PI/4),std::cos(M_PI/4));

        double TS = bp.TS;
        double TRACK_DUR = 2.0 * T_ORBIT;
        double t = 0.0;
        double track_start = -1;
        double detumble_settle_t = 0.0;
        double slew_settle_t = 0.0;
        int track_count = 0;
        int cone_viol = 0;
        double track_ang_sum = 0.0;
        double track_ang_max = 0.0;
        std::vector<double> orbit_errs;
        bool failed = false;
        bool reached_track = false;
        bool track_embedded_done = false;

        std::array<int,3> run_phase_steps{{0,0,0}};
        int run_steps = 0;

        // hot-loop scratch, hoisted so the control step performs no heap traffic
        double thr_buf[NTHR];
        ThrusterInput ti; ti.u.assign(NTHR, 0.0); ti.is_on_time = false;
        ThrusterOutput to;

        while (true) {
            double _iter_t0 = timing ? sad_now_ns() : 0.0;
            if (t >= TS + TRACK_DUR + 60.0) { failed = true; break; }
            double dt = (ms.phase == MissionPhase::Detumble || ms.phase == MissionPhase::Slew) ? 0.1 : bp.track_dt;
            if (ms.phase == MissionPhase::Track) {
                if (track_start < 0) track_start = t;
#ifdef SAD_WITH_HEYOKA
                if (bp.track_embedded && hey_track != nullptr && !track_embedded_done) {
                    // Freeze the roll reference to the current body +Y (in inertial).
                    Mat3 Rbi_now = quat_to_dcm(ds.q);
                    Vec3 frozen_roll = Rbi_now.transpose() * Vec3(0, 1, 0);

                    HeyokaTrackInit init;
                    init.r = ds.r;
                    init.v = ds.v;
                    init.q = ds.q;
                    init.w = ds.w;
                    init.z = qs.integral;
                    init.fuel = ts.fuel_used;
                    init.mass = mass;
                    init.inertia = Vec3(inertia(0, 0), inertia(1, 1), inertia(2, 2));
                    init.inv_inertia = Vec3(invI(0, 0), invI(1, 1), invI(2, 2));
                    init.mu = bp.MU;
                    init.tau_max = bp.tau_max;
                    init.eps = 1e-6;
                    init.z_max = 0.5;
                    init.roll_ref = frozen_roll;

                    double dtc = (bp.track_dt > 0) ? bp.track_dt : 1.0;
                    double D0 = 1.0 + dtc * bp.track_P / IX + 0.5 * dtc * dtc * bp.track_K / IX;
                    double D1 = 1.0 + dtc * bp.track_P / IY + 0.5 * dtc * dtc * bp.track_K / IY;
                    double D2 = 1.0 + dtc * bp.track_P / IZ + 0.5 * dtc * dtc * bp.track_K / IZ;
                    init.Keff[0] = bp.track_K / D0;
                    init.Keff[1] = bp.track_K / D1;
                    init.Keff[2] = bp.track_K / D2;
                    init.Peff[0] = (bp.track_P + 0.5 * dtc * bp.track_K) / D0;
                    init.Peff[1] = (bp.track_P + 0.5 * dtc * bp.track_K) / D1;
                    init.Peff[2] = (bp.track_P + 0.5 * dtc * bp.track_K) / D2;
                    init.Kieff[0] = bp.track_Ki / D0;
                    init.Kieff[1] = bp.track_Ki / D1;
                    init.Kieff[2] = bp.track_Ki / D2;

                    hey_track->reset(init, t);
                    track_count = 0;
                    track_ang_sum = 0.0;
                    track_ang_max = 0.0;
                    cone_viol = 0;
                    orbit_errs.clear();

                    HeyokaTrackResult seg = hey_track->propagate(track_start + TRACK_DUR, bp.track_sample_dt,
                        [&](double tk, const std::array<double, 17>& st) {
                            Vec3 r(st[0], st[1], st[2]);
                            Vec4 q(st[6], st[7], st[8], st[9]);
                            q = quat_normalize(q);
                            Mat3 R = quat_to_dcm(q);

                            double sun_ang = 2.0 * M_PI * tk / (365.25 * 86400.0) + 0.5 * M_PI;
                            Vec3 sun_n(std::cos(sun_ang), std::sin(sun_ang), 0.0);
                            Vec3 sb = R * sun_n;
                            double cosang = sb[0];
                            if (cosang > 1.0) cosang = 1.0;
                            if (cosang < -1.0) cosang = -1.0;
                            double ang_sun = std::acos(cosang) * 180.0 / M_PI;

                            track_count++;
                            track_ang_sum += ang_sun;
                            if (ang_sun > track_ang_max) track_ang_max = ang_sun;
                            if (ang_sun > 15.0) cone_viol++;

                            double rnorm = r.norm();
                            orbit_errs.push_back(std::fabs(rnorm - bp.R_ORBIT) / 1000.0);
                        });

                    if (timing) {
                        out.timing.heyoka_track_steps += seg.steps;
                        if (out.timing.heyoka_track_min_step == 0.0 || seg.min_step < out.timing.heyoka_track_min_step)
                            out.timing.heyoka_track_min_step = seg.min_step;
                        if (seg.max_step > out.timing.heyoka_track_max_step)
                            out.timing.heyoka_track_max_step = seg.max_step;
                    }

                    // Final state.
                    auto& last = seg.final_state;
                    for (int i = 0; i < 3; ++i) ds.r[i] = last[i];
                    for (int i = 0; i < 3; ++i) ds.v[i] = last[3 + i];
                    Vec4 qf(last[6], last[7], last[8], last[9]);
                    qf = quat_normalize(qf);
                    for (int i = 0; i < 4; ++i) ds.q[i] = qf[i];
                    for (int i = 0; i < 3; ++i) ds.w[i] = last[10 + i];
                    ds.t = track_start + TRACK_DUR;
                    ts.fuel_used = last[16];

                    track_embedded_done = true;
                    t = track_start + TRACK_DUR;
                    break;
                }
#endif
                if ((t - track_start) >= TRACK_DUR) break;
            }
            if (ms.phase != MissionPhase::Track && t > TS) { failed = true; break; }

            // bookkeeping: which phase are we stepping in
            int ph = phase_index(ms.phase);
            if (timing) {
                g_timing.phase = ph;
                g_timing.steps++;
                g_timing.phase_steps[ph]++;
            }
            run_phase_steps[ph]++;
            run_steps++;

            // sun: pin the free roll to the current body +Y (in inertial) so
            // the sun target is the nearest-roll attitude, not a fixed 180-deg
            // roll away from the post-slew attitude.
            {
                Mat3 Rbi_now = quat_to_dcm(ds.q);
                ss.params.roll_ref = Rbi_now.transpose() * Vec3(0,1,0);
            }
            SunInput si; si.t = t; SunOutput so; sun_step(si, ss, so, dt);
            // star tracker
            StarTrackerInput sti; sti.q_true = ds.q; sti.omega_body = ds.w; sti.sun_dir_b = Vec3::Zero();
            StarTrackerOutput sto; startracker_step(sti, sts, sto, dt);
            Vec4 q_meas = sto.valid ? sto.q_meas : ds.q;

            // mission
            MissionInput mi; mi.q = ds.q; mi.omega = ds.w; mi.q_sun = so.q_sun_pointing; mi.t = t;
            MissionPhase ph_before = ms.phase;
            MissionOutput mo; mission_step(mi, ms, mo, dt);
            if (ph_before == MissionPhase::Detumble && ms.phase == MissionPhase::Slew)
                detumble_settle_t = t + dt;
            if (ph_before == MissionPhase::Slew && ms.phase == MissionPhase::Track)
                slew_settle_t = t + dt;

            // controller
            QuatPDInput qi; qi.q_current = ds.q; qi.omega = ds.w; qi.q_desired = mo.q_desired; qi.mode = mo.ctrl_mode;
            QuatPDOutput qo; quatpd_step(qi, qs, qo, dt);
            Vec3 tau = qo.torque_b;
            for (int i=0;i<3;++i) { if (tau[i] > bp.tau_max) tau[i]=bp.tau_max; if (tau[i] < -bp.tau_max) tau[i]=-bp.tau_max; }

            // thruster handling depends on thruster_mode.
            DynamicsInput ui;
            ui.force_b = Vec3::Zero();
            if (bp.thruster_mode == 1) {
                // Direct commanded torque: no thruster allocation, no fuel.
                ui.torque_b = tau;
            } else {
                // Mode 0: current pseudo-inverse + clamp.
                // Mode 2: smooth torque-preserving pair allocator.
                if (bp.thruster_mode == 2) {
                    const double eps = 1e-6;
                    for (int i = 0; i < NTHR; ++i) thr_buf[i] = 0.0;
                    // Z torque: positive pair (0,3), negative pair (1,2), arm 0.10 each.
                    {
                        double s = std::sqrt(tau[2]*tau[2] + eps*eps);
                        double fpos = (s + tau[2]) / (2.0 * 0.20);
                        double fneg = (s - tau[2]) / (2.0 * 0.20);
                        thr_buf[0] = fpos; thr_buf[3] = fpos;
                        thr_buf[1] = fneg; thr_buf[2] = fneg;
                    }
                    // Y torque: positive pair (4,7), negative pair (5,6), arm 0.05 each.
                    {
                        double s = std::sqrt(tau[1]*tau[1] + eps*eps);
                        double fpos = (s + tau[1]) / (2.0 * 0.10);
                        double fneg = (s - tau[1]) / (2.0 * 0.10);
                        thr_buf[4] = fpos; thr_buf[7] = fpos;
                        thr_buf[5] = fneg; thr_buf[6] = fneg;
                    }
                    // X torque: positive pair (8,11), negative pair (9,10), arm 0.15 each.
                    {
                        double s = std::sqrt(tau[0]*tau[0] + eps*eps);
                        double fpos = (s + tau[0]) / (2.0 * 0.30);
                        double fneg = (s - tau[0]) / (2.0 * 0.30);
                        thr_buf[8] = fpos; thr_buf[11] = fpos;
                        thr_buf[9] = fneg; thr_buf[10] = fneg;
                    }
                } else {
                    if (cached_alloc) {
                        thruster_allocate_cached_into(alloc, tp, Vec3::Zero(), tau, thr_buf);
                    } else {
                        std::vector<double> thr = thruster_allocate(tp, Vec3::Zero(), tau);
                        for (int i = 0; i < NTHR; ++i) thr_buf[i] = thr[i];
                    }
                }
                for (int i = 0; i < NTHR; ++i) {
                    double th = (tp.tmax[i] > 0) ? thr_buf[i] / tp.tmax[i] : 0.0;
                    if (th < 0) th = 0; if (th > 1) th = 1;
                    ti.u[i] = th;
                }
                ti.dt = dt;
                thruster_step(ti, ts, to, dt);
                ui.force_b = to.force_b;
                ui.torque_b = to.torque_b;
            }
            DynamicsOutput uo;
#ifdef SAD_WITH_HEYOKA
            if (hey) {
                hey->step(ui, ds, uo, dt);
            } else {
                dynamics_step(ui, ds, uo, dt);
            }
#else
            dynamics_step(ui, ds, uo, dt);
#endif

            // KPIs
            if (ms.phase == MissionPhase::Track) {
                SAD_TIME(TKpi);
                track_count++;
                // cone check
                Mat3 R = quat_to_dcm(ds.q);
                Vec3 sb = R * so.sun_n;
                double cosang = sb[0];
                if (cosang > 1) cosang = 1; if (cosang < -1) cosang = -1;
                double ang_sun = std::acos(cosang) * 180.0 / M_PI;
                track_ang_sum += ang_sun;
                if (ang_sun > track_ang_max) track_ang_max = ang_sun;
                if (ang_sun > 15.0) cone_viol++;
                // orbit err
                double rnorm = ds.r.norm();
                orbit_errs.push_back(std::fabs(rnorm - bp.R_ORBIT) / 1000.0);
            }
            // phase transitions
            // (handled by checking ms.phase after step; but ms already updated)
            t += dt;

            // nan check
            if (!std::isfinite(ds.r[0]) || !std::isfinite(ds.w[0]) || !std::isfinite(ds.q[0])) {
                failed = true; break;
            }
            if (timing) g_timing.add(TLoopOverhead, sad_now_ns() - _iter_t0);
        }

        double final_rate = ds.w.norm();
        double fuel = ts.fuel_used;
        double orbit_err_mean = 0.0;
        double box_max = 0.0;
        if (!orbit_errs.empty()) {
            double sum = 0; for (double v : orbit_errs) sum += v;
            orbit_err_mean = sum / orbit_errs.size();
            for (double v : orbit_errs) if (v > box_max) box_max = v;
        }
        bool reached = (track_count > 0);

        out.final_rate[run] = failed ? 0.0 : final_rate; // 0 placeholder, will use failed flag
        // store as nan if failed
        if (failed) { out.final_rate[run] = std::nan("1"); }
        out.fuel_used[run] = fuel;
        out.detumble_settle[run] = (detumble_settle_t > 0.0) ? detumble_settle_t : t;
        out.slew_settle[run] = reached ? slew_settle_t : t;
        out.orbit_err[run] = orbit_err_mean;
        out.box_max[run] = box_max;
        out.track_ang_mean[run] = (track_count > 0) ? (track_ang_sum / track_count) : 0.0;
        out.track_ang_max[run] = track_ang_max;
        out.reached_track[run] = reached ? 1 : 0;
        out.failed[run] = failed ? 1 : 0;
        if (reached) out.timing.reached_runs++;
        if (failed) out.timing.failed_runs++;

        if (timing) {
            double run_ms = (sad_now_ns() - _run_t0) / 1e6;
            out.timing.run_ms.push_back(run_ms);
            out.timing.run_steps.push_back(run_steps);
            out.timing.run_phase_steps.push_back(run_phase_steps);
        }
    }

    if (timing) {
        out.timing.batch_ms = (sad_now_ns() - _batch_t0) / 1e6;
        out.timing.steps = g_timing.steps;
        out.timing.phase_steps = g_timing.phase_steps;
        out.timing.ns = g_timing.ns;
        out.timing.calls = g_timing.calls;
        out.timing.phase_ns = g_timing.phase_ns;
        out.timing.phase_calls = g_timing.phase_calls;
    }
    g_timing.enabled = false;
}

bool heyoka_available() {
#ifdef SAD_WITH_HEYOKA
    return true;
#else
    return false;
#endif
}

} // namespace sad
