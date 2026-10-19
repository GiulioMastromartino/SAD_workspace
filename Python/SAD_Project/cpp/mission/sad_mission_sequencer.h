#pragma once
#include "sad_types.h"
#include "sad_module.h"
#include "../control/sad_controller_quat_pd.h"

namespace sad {
enum class MissionPhase { Detumble, Slew, Track };

struct MissionParams {
    double detumble_rate_tol = 0.05; // rad/s
    double detumble_time_min = 2.0;
    double slew_angle_tol = 0.1; // rad
    double slew_rate_tol = 0.05;
};
struct MissionState {
    MissionParams params;
    MissionPhase phase = MissionPhase::Detumble;
    double phase_time = 0.0;
    Vec4 q_target = Vec4(0,0,0,1);
    Vec4 q_slew = Vec4(0,0,0,1); // 90 deg about +Z
};
struct MissionInput {
    Vec4 q = Vec4(0,0,0,1);
    Vec3 omega = Vec3::Zero();
    Vec4 q_sun = Vec4(0,0,0,1);
    double t = 0.0;
};
struct MissionOutput {
    MissionPhase phase = MissionPhase::Detumble;
    Vec4 q_desired = Vec4(0,0,0,1);
    CtrlMode ctrl_mode = CtrlMode::Detumble;
};

void mission_init(MissionState& s, const MissionParams& p);
void mission_step(const MissionInput& u, MissionState& s, MissionOutput& y, double dt);
void mission_reset(MissionState& s);
} // namespace sad
