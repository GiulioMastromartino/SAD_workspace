#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "sad_dynamics.h"
#include "sad_actuator_thruster.h"
#include "sad_actuator_rw.h"
#include "sad_sensor_startracker.h"
#include "sad_sensor_imu.h"
#include "sad_env_sun.h"
#include "sad_controller_quat_pd.h"
#include "sad_mission_sequencer.h"
#include "sad_mission_batch.h"

namespace py = pybind11;
using namespace sad;

PYBIND11_MODULE(sad_py, m) {
    m.doc() = "SAD_Project C++ modules";

    // Dynamics
    py::class_<DynamicsParams>(m, "DynamicsParams")
        .def(py::init<>())
        .def_readwrite("mass", &DynamicsParams::mass)
        .def_readwrite("mu", &DynamicsParams::mu)
        .def_readwrite("gravity_gradient", &DynamicsParams::gravity_gradient)
        .def_readwrite("r0", &DynamicsParams::r0)
        .def_readwrite("v0", &DynamicsParams::v0)
        .def_readwrite("q0", &DynamicsParams::q0)
        .def_readwrite("w0", &DynamicsParams::w0)
        .def_readwrite("inertia", &DynamicsParams::inertia)
        .def_readwrite("inv_inertia", &DynamicsParams::inv_inertia);

    py::class_<DynamicsState>(m, "DynamicsState")
        .def(py::init<>())
        .def_readwrite("params", &DynamicsState::params)
        .def_readwrite("r", &DynamicsState::r)
        .def_readwrite("v", &DynamicsState::v)
        .def_readwrite("q", &DynamicsState::q)
        .def_readwrite("w", &DynamicsState::w)
        .def_readwrite("t", &DynamicsState::t);

    py::class_<DynamicsInput>(m, "DynamicsInput")
        .def(py::init<>())
        .def_readwrite("force_b", &DynamicsInput::force_b)
        .def_readwrite("torque_b", &DynamicsInput::torque_b);

    py::class_<DynamicsOutput>(m, "DynamicsOutput")
        .def(py::init<>())
        .def_readwrite("r", &DynamicsOutput::r)
        .def_readwrite("v", &DynamicsOutput::v)
        .def_readwrite("q", &DynamicsOutput::q)
        .def_readwrite("w", &DynamicsOutput::w);

    m.def("dynamics_init", &dynamics_init);
    m.def("dynamics_step", &dynamics_step);
    m.def("dynamics_reset", &dynamics_reset);

    // Thruster
    py::class_<ThrusterParams>(m, "ThrusterParams").def(py::init<>())
        .def_readwrite("n", &ThrusterParams::n)
        .def_readwrite("pos", &ThrusterParams::pos)
        .def_readwrite("dir", &ThrusterParams::dir)
        .def_readwrite("tmax", &ThrusterParams::tmax)
        .def_readwrite("isp", &ThrusterParams::isp)
        .def_readwrite("g0", &ThrusterParams::g0);
    py::class_<ThrusterState>(m, "ThrusterState").def(py::init<>())
        .def_readwrite("params", &ThrusterState::params)
        .def_readwrite("on_time_accum", &ThrusterState::on_time_accum)
        .def_readwrite("fuel_used", &ThrusterState::fuel_used);
    py::class_<ThrusterInput>(m, "ThrusterInput").def(py::init<>())
        .def_readwrite("u", &ThrusterInput::u)
        .def_readwrite("is_on_time", &ThrusterInput::is_on_time)
        .def_readwrite("dt", &ThrusterInput::dt);
    py::class_<ThrusterOutput>(m, "ThrusterOutput").def(py::init<>())
        .def_readwrite("force_b", &ThrusterOutput::force_b)
        .def_readwrite("torque_b", &ThrusterOutput::torque_b)
        .def_readwrite("fuel_rate", &ThrusterOutput::fuel_rate);
    m.def("thruster_init", &thruster_init);
    m.def("thruster_step", &thruster_step);
    m.def("thruster_reset", &thruster_reset);
    m.def("thruster_allocate", &thruster_allocate);
    py::class_<ThrusterAllocator>(m, "ThrusterAllocator")
        .def(py::init<>())
        .def_readonly("n", &ThrusterAllocator::n)
        .def_readonly("pinv", &ThrusterAllocator::pinv);
    m.def("thruster_allocator_init", &thruster_allocator_init);
    m.def("thruster_allocate_cached", &thruster_allocate_cached);

    // RW
    py::class_<RWParams>(m, "RWParams").def(py::init<>())
        .def_readwrite("n", &RWParams::n)
        .def_readwrite("axes", &RWParams::axes)
        .def_readwrite("max_torque", &RWParams::max_torque)
        .def_readwrite("max_momentum", &RWParams::max_momentum)
        .def_readwrite("tau", &RWParams::tau);
    py::class_<RWState>(m, "RWState").def(py::init<>())
        .def_readwrite("params", &RWState::params)
        .def_readwrite("h", &RWState::h)
        .def_readwrite("omega", &RWState::omega);
    py::class_<RWInput>(m, "RWInput").def(py::init<>())
        .def_readwrite("torque_cmd_b", &RWInput::torque_cmd_b);
    py::class_<RWOutput>(m, "RWOutput").def(py::init<>())
        .def_readwrite("torque_b", &RWOutput::torque_b);
    m.def("rw_init", &rw_init);
    m.def("rw_step", &rw_step);
    m.def("rw_reset", &rw_reset);

    // Star tracker
    py::class_<StarTrackerParams>(m, "StarTrackerParams").def(py::init<>())
        .def_readwrite("sigma", &StarTrackerParams::sigma)
        .def_readwrite("rate_cap", &StarTrackerParams::rate_cap)
        .def_readwrite("dropout_prob", &StarTrackerParams::dropout_prob)
        .def_readwrite("seed", &StarTrackerParams::seed);
    py::class_<StarTrackerState>(m, "StarTrackerState").def(py::init<>())
        .def_readwrite("params", &StarTrackerState::params)
        .def_readwrite("valid", &StarTrackerState::valid);
    py::class_<StarTrackerInput>(m, "StarTrackerInput").def(py::init<>())
        .def_readwrite("q_true", &StarTrackerInput::q_true)
        .def_readwrite("omega_body", &StarTrackerInput::omega_body)
        .def_readwrite("sun_dir_b", &StarTrackerInput::sun_dir_b);
    py::class_<StarTrackerOutput>(m, "StarTrackerOutput").def(py::init<>())
        .def_readwrite("valid", &StarTrackerOutput::valid)
        .def_readwrite("q_meas", &StarTrackerOutput::q_meas);
    m.def("startracker_init", &startracker_init);
    m.def("startracker_step", &startracker_step);
    m.def("startracker_reset", &startracker_reset);

    // IMU
    py::class_<IMUParams>(m, "IMUParams").def(py::init<>())
        .def_readwrite("bias_tau", &IMUParams::bias_tau)
        .def_readwrite("seed", &IMUParams::seed);
    py::class_<IMUState>(m, "IMUState").def(py::init<>())
        .def_readwrite("params", &IMUState::params)
        .def_readwrite("bias", &IMUState::bias);
    py::class_<IMUInput>(m, "IMUInput").def(py::init<>())
        .def_readwrite("omega_true", &IMUInput::omega_true);
    py::class_<IMUOutput>(m, "IMUOutput").def(py::init<>())
        .def_readwrite("omega_meas", &IMUOutput::omega_meas);
    m.def("imu_init", &imu_init);
    m.def("imu_step", &imu_step);
    m.def("imu_reset", &imu_reset);

    // Sun
    py::class_<SunParams>(m, "SunParams").def(py::init<>())
        .def_readwrite("distance", &SunParams::distance)
        .def_readwrite("year", &SunParams::year)
        .def_readwrite("phase0", &SunParams::phase0)
        .def_readwrite("roll_ref", &SunParams::roll_ref);
    py::class_<SunState>(m, "SunState").def(py::init<>())
        .def_readwrite("params", &SunState::params)
        .def_readwrite("t", &SunState::t);
    py::class_<SunInput>(m, "SunInput").def(py::init<>())
        .def_readwrite("t", &SunInput::t);
    py::class_<SunOutput>(m, "SunOutput").def(py::init<>())
        .def_readwrite("sun_n", &SunOutput::sun_n)
        .def_readwrite("q_sun_pointing", &SunOutput::q_sun_pointing);
    m.def("sun_init", &sun_init);
    m.def("sun_step", &sun_step);
    m.def("sun_reset", &sun_reset);
    m.def("sun_pointing_quaternion", &sun_pointing_quaternion);

    // QuatPD
    py::enum_<CtrlMode>(m, "CtrlMode")
        .value("Detumble", CtrlMode::Detumble)
        .value("Slew", CtrlMode::Slew)
        .value("Track", CtrlMode::Track);
    py::class_<QuatPDParams>(m, "QuatPDParams").def(py::init<>())
        .def_readwrite("detumble_K", &QuatPDParams::detumble_K)
        .def_readwrite("detumble_P", &QuatPDParams::detumble_P)
        .def_readwrite("detumble_Ki", &QuatPDParams::detumble_Ki)
        .def_readwrite("slew_K", &QuatPDParams::slew_K)
        .def_readwrite("slew_P", &QuatPDParams::slew_P)
        .def_readwrite("slew_Ki", &QuatPDParams::slew_Ki)
        .def_readwrite("track_K", &QuatPDParams::track_K)
        .def_readwrite("track_P", &QuatPDParams::track_P)
        .def_readwrite("track_Ki", &QuatPDParams::track_Ki)
        .def_readwrite("q_slew_target", &QuatPDParams::q_slew_target)
        .def_readwrite("inertia", &QuatPDParams::inertia)
        .def_readwrite("implicit_dt", &QuatPDParams::implicit_dt)
        .def_readwrite("implicit_track_only", &QuatPDParams::implicit_track_only);
    py::class_<QuatPDState>(m, "QuatPDState").def(py::init<>())
        .def_readwrite("params", &QuatPDState::params)
        .def_readwrite("mode", &QuatPDState::mode);
    py::class_<QuatPDInput>(m, "QuatPDInput").def(py::init<>())
        .def_readwrite("q_current", &QuatPDInput::q_current)
        .def_readwrite("omega", &QuatPDInput::omega)
        .def_readwrite("q_desired", &QuatPDInput::q_desired)
        .def_readwrite("mode", &QuatPDInput::mode);
    py::class_<QuatPDOutput>(m, "QuatPDOutput").def(py::init<>())
        .def_readwrite("torque_b", &QuatPDOutput::torque_b);
    m.def("quatpd_init", &quatpd_init);
    m.def("quatpd_step", &quatpd_step);
    m.def("quatpd_reset", &quatpd_reset);

    // Mission
    py::enum_<MissionPhase>(m, "MissionPhase")
        .value("Detumble", MissionPhase::Detumble)
        .value("Slew", MissionPhase::Slew)
        .value("Track", MissionPhase::Track);
    py::class_<MissionParams>(m, "MissionParams").def(py::init<>())
        .def_readwrite("detumble_rate_tol", &MissionParams::detumble_rate_tol)
        .def_readwrite("detumble_time_min", &MissionParams::detumble_time_min)
        .def_readwrite("slew_angle_tol", &MissionParams::slew_angle_tol)
        .def_readwrite("slew_rate_tol", &MissionParams::slew_rate_tol);
    py::class_<MissionState>(m, "MissionState").def(py::init<>())
        .def_readwrite("params", &MissionState::params)
        .def_readwrite("phase", &MissionState::phase)
        .def_readwrite("phase_time", &MissionState::phase_time)
        .def_readwrite("q_slew", &MissionState::q_slew);
    py::class_<MissionInput>(m, "MissionInput").def(py::init<>())
        .def_readwrite("q", &MissionInput::q)
        .def_readwrite("omega", &MissionInput::omega)
        .def_readwrite("q_sun", &MissionInput::q_sun)
        .def_readwrite("t", &MissionInput::t);
    py::class_<MissionOutput>(m, "MissionOutput").def(py::init<>())
        .def_readwrite("phase", &MissionOutput::phase)
        .def_readwrite("q_desired", &MissionOutput::q_desired)
        .def_readwrite("ctrl_mode", &MissionOutput::ctrl_mode);
    m.def("mission_init", &mission_init);
    m.def("mission_step", &mission_step);
    m.def("mission_reset", &mission_reset);

    // Batch mission runner (C++ kernel)
    py::class_<BatchParams>(m, "BatchParams")
        .def(py::init<>())
        .def_readwrite("detumble_K", &BatchParams::detumble_K)
        .def_readwrite("detumble_P", &BatchParams::detumble_P)
        .def_readwrite("detumble_Ki", &BatchParams::detumble_Ki)
        .def_readwrite("slew_K", &BatchParams::slew_K)
        .def_readwrite("slew_P", &BatchParams::slew_P)
        .def_readwrite("slew_Ki", &BatchParams::slew_Ki)
        .def_readwrite("track_K", &BatchParams::track_K)
        .def_readwrite("track_P", &BatchParams::track_P)
        .def_readwrite("track_Ki", &BatchParams::track_Ki)
        .def_readwrite("TS", &BatchParams::TS)
        .def_readwrite("R_ORBIT", &BatchParams::R_ORBIT)
        .def_readwrite("MU", &BatchParams::MU)
        .def_readwrite("mass", &BatchParams::mass)
        .def_readwrite("disperse", &BatchParams::disperse)
        .def_readwrite("cached_alloc", &BatchParams::cached_alloc)
        .def_readwrite("track_dt", &BatchParams::track_dt)
        .def_readwrite("implicit_pd", &BatchParams::implicit_pd)
        .def_readwrite("implicit_track_only", &BatchParams::implicit_track_only)
        .def_readwrite("tau_max", &BatchParams::tau_max)
        .def_readwrite("disp_r_m", &BatchParams::disp_r_m)
        .def_readwrite("disp_v_mps", &BatchParams::disp_v_mps)
        .def_readwrite("use_heyoka", &BatchParams::use_heyoka)
        .def_readwrite("heyoka_tol", &BatchParams::heyoka_tol)
        .def_readwrite("heyoka_compact", &BatchParams::heyoka_compact)
        .def_readwrite("thruster_mode", &BatchParams::thruster_mode)
        .def_readwrite("track_embedded", &BatchParams::track_embedded)
        .def_readwrite("track_sample_dt", &BatchParams::track_sample_dt)
        .def_readwrite("timing", &BatchParams::timing);
    m.def("heyoka_available", &heyoka_available);
    m.def("run_mission_batch", [](int N, unsigned seed, const BatchParams &bp) {
        BatchResult out;
        {
            py::gil_scoped_release release;
            run_mission_batch(N, seed, bp, out);
        }
        py::dict d;
        d["final_rate"] = out.final_rate;
        d["fuel_used"] = out.fuel_used;
        d["detumble_settle"] = out.detumble_settle;
        d["slew_settle"] = out.slew_settle;
        d["orbit_err"] = out.orbit_err;
        d["box_max"] = out.box_max;
        d["track_ang_mean"] = out.track_ang_mean;
        d["track_ang_max"] = out.track_ang_max;
        d["reached_track"] = out.reached_track;
        d["failed"] = out.failed;
        // ---- timing payload ----
        const BatchTiming &T = out.timing;
        py::dict t;
        t["enabled"] = T.enabled;
        t["batch_ms"] = T.batch_ms;
        t["steps"] = T.steps;
        t["phase_steps"] = T.phase_steps;      // [detumble, slew, track]
        t["heyoka_track_steps"] = T.heyoka_track_steps;
        t["heyoka_track_min_step"] = T.heyoka_track_min_step;
        t["heyoka_track_max_step"] = T.heyoka_track_max_step;
        t["failed_runs"] = T.failed_runs;
        t["reached_runs"] = T.reached_runs;
        t["run_ms"] = T.run_ms;
        t["run_steps"] = T.run_steps;
        // per-run phase steps -> list of 3-lists
        py::list rps;
        for (const auto &a : T.run_phase_steps) rps.append(py::cast(a));
        t["run_phase_steps"] = rps;
        // flat slot tables: name -> ns / calls, plus per-phase
        py::dict ns_map, calls_map, phase_ns_map, phase_calls_map;
        for (int s = 0; s < TCount; ++s) {
            std::string name = timing_slot_name(s);
            ns_map[py::str(name)] = T.ns[s];
            calls_map[py::str(name)] = T.calls[s];
            py::list pn, pc;
            for (int p = 0; p < 3; ++p) {
                pn.append(T.phase_ns[s * 3 + p]);
                pc.append(T.phase_calls[s * 3 + p]);
            }
            phase_ns_map[py::str(name)] = pn;
            phase_calls_map[py::str(name)] = pc;
        }
        t["ns"] = ns_map;
        t["calls"] = calls_map;
        t["phase_ns"] = phase_ns_map;      // name -> [detumble, slew, track] ns
        t["phase_calls"] = phase_calls_map;
        d["timing"] = t;
        return d;
    });

    // math helpers
    m.def("quat_normalize", &quat_normalize);
    m.def("quat_mul", &quat_mul);
    m.def("quat_error_vec", &quat_error_vec);
    m.def("quat_to_dcm", &quat_to_dcm);
}
