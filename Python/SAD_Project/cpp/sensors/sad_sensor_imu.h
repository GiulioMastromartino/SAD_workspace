#pragma once
#include "sad_types.h"
#include "sad_module.h"
#include <random>

namespace sad {
struct IMUParams {
    double gyro_bias0[3] = {0,0,0};
    double gyro_arw[3] = {1e-4,1e-4,1e-4};
    double gyro_bias_walk[3] = {1e-5,1e-5,1e-5};
    double bias_tau = 300.0;
    unsigned seed = 999;
};
struct IMUState {
    IMUParams params;
    std::mt19937 rng;
    Vec3 bias = Vec3::Zero();
};
struct IMUInput {
    Vec3 omega_true = Vec3::Zero();
};
struct IMUOutput {
    Vec3 omega_meas = Vec3::Zero();
};

void imu_init(IMUState& s, const IMUParams& p);
void imu_step(const IMUInput& u, IMUState& s, IMUOutput& y, double dt);
void imu_reset(IMUState& s);
} // namespace sad
