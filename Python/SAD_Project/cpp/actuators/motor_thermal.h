#pragma once
/* Skeleton module: motor_thermal (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MOTOR_THERMALParams {
    std::vector<double> params;
};
struct MOTOR_THERMALState {
    MOTOR_THERMALParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MOTOR_THERMALInput {
    std::vector<double> u;
};
struct MOTOR_THERMALOutput {
    std::vector<double> y;
};

void motor_thermal_init(MOTOR_THERMALState& s, const MOTOR_THERMALParams& p);
void motor_thermal_step(const MOTOR_THERMALInput& u, MOTOR_THERMALState& s, MOTOR_THERMALOutput& y, double dt);
void motor_thermal_reset(MOTOR_THERMALState& s);
} // namespace sad
