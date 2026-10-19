#pragma once
#include "sad_types.h"
#include "sad_module.h"
#include <random>

namespace sad {
struct StarTrackerParams {
    double sigma = 1e-4; // rad (attitude noise)
    double rate_cap = 0.1; // rad/s max body rate for valid measurement
    double dropout_prob = 0.05;
    unsigned seed = 12345;
};
struct StarTrackerState {
    StarTrackerParams params;
    std::mt19937 rng;
    bool valid = true;
};
struct StarTrackerInput {
    Vec4 q_true = Vec4(0,0,0,1); // true q_BN
    Vec3 omega_body = Vec3::Zero();
    Vec3 sun_dir_b = Vec3::Zero();
};
struct StarTrackerOutput {
    bool valid = false;
    Vec4 q_meas = Vec4(0,0,0,1);
};

void startracker_init(StarTrackerState& s, const StarTrackerParams& p);
void startracker_step(const StarTrackerInput& u, StarTrackerState& s, StarTrackerOutput& y, double dt);
void startracker_reset(StarTrackerState& s);
} // namespace sad
