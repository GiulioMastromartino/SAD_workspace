#include "sad_mission_sequencer.h"
#include "sad_timing.h"
#include <cmath>

namespace sad {
// 90 deg about +Z: quaternion [0,0,sin45,cos45]
static Vec4 slew_quat() {
    double a = 3.141592653589793/2.0; // 90 deg
    return Vec4(0,0,std::sin(a/2), std::cos(a/2));
}

void mission_init(MissionState& s, const MissionParams& p) {
    s.params = p;
    s.phase = MissionPhase::Detumble;
    s.phase_time = 0.0;
    s.q_target = Vec4(0,0,0,1);
    s.q_slew = slew_quat();
}

void mission_step(const MissionInput& u, MissionState& s, MissionOutput& y, double dt) {
    SAD_TIME(TMission);
    s.phase_time += dt;
    const MissionParams& p = s.params;
    double rate = u.omega.norm();
    // angle error to slew target
    Vec4 qerr = quat_mul(s.q_slew, quat_conj(u.q));
    if (qerr[3] < 0) qerr = -qerr;
    double ang_err = 2.0*std::atan2(std::sqrt(qerr[0]*qerr[0]+qerr[1]*qerr[1]+qerr[2]*qerr[2]), std::fabs(qerr[3]));

    switch (s.phase) {
        case MissionPhase::Detumble:
            if (rate < p.detumble_rate_tol && s.phase_time > p.detumble_time_min) {
                s.phase = MissionPhase::Slew;
                s.phase_time = 0.0;
            }
            break;
        case MissionPhase::Slew:
            if (ang_err < p.slew_angle_tol && rate < p.slew_rate_tol) {
                s.phase = MissionPhase::Track;
                s.phase_time = 0.0;
            }
            break;
        case MissionPhase::Track:
            break;
    }

    s.phase = s.phase;
    y.phase = s.phase;
    switch (s.phase) {
        case MissionPhase::Detumble:
            y.q_desired = Vec4(0,0,0,1);
            y.ctrl_mode = CtrlMode::Detumble;
            break;
        case MissionPhase::Slew:
            y.q_desired = s.q_slew;
            y.ctrl_mode = CtrlMode::Slew;
            break;
        case MissionPhase::Track:
            y.q_desired = u.q_sun;
            y.ctrl_mode = CtrlMode::Track;
            break;
    }
}

void mission_reset(MissionState& s) {
    s.phase = MissionPhase::Detumble;
    s.phase_time = 0.0;
}
} // namespace sad
