#pragma once
#include "sad_types.h"
#include "sad_module.h"

namespace sad {
struct RWParams {
    int n = 3;
    std::vector<Vec3> axes;      // body-frame spin axes
    double max_torque = 0.05;    // N m
    double max_momentum = 0.5;   // N m s
    double tau = 0.2;            // first-order time constant [s]
};
struct RWState {
    RWParams params;
    std::vector<double> h; // momentum per wheel [N m s]
    std::vector<double> omega; // wheel speed [rad/s]
};
struct RWInput {
    Vec3 torque_cmd_b = Vec3::Zero(); // desired body torque
};
struct RWOutput {
    Vec3 torque_b = Vec3::Zero(); // actual torque delivered
};

void rw_init(RWState& s, const RWParams& p);
void rw_step(const RWInput& u, RWState& s, RWOutput& y, double dt);
void rw_reset(RWState& s);
} // namespace sad
