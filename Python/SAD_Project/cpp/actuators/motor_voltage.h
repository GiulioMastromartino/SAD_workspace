#pragma once
/* Skeleton module: motor_voltage (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MOTOR_VOLTAGEParams {
    std::vector<double> params;
};
struct MOTOR_VOLTAGEState {
    MOTOR_VOLTAGEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MOTOR_VOLTAGEInput {
    std::vector<double> u;
};
struct MOTOR_VOLTAGEOutput {
    std::vector<double> y;
};

void motor_voltage_init(MOTOR_VOLTAGEState& s, const MOTOR_VOLTAGEParams& p);
void motor_voltage_step(const MOTOR_VOLTAGEInput& u, MOTOR_VOLTAGEState& s, MOTOR_VOLTAGEOutput& y, double dt);
void motor_voltage_reset(MOTOR_VOLTAGEState& s);
} // namespace sad
