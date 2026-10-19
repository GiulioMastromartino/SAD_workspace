#pragma once
/* Skeleton module: inertial_est (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct INERTIAL_ESTParams {
    std::vector<double> params;
};
struct INERTIAL_ESTState {
    INERTIAL_ESTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct INERTIAL_ESTInput {
    std::vector<double> u;
};
struct INERTIAL_ESTOutput {
    std::vector<double> y;
};

void inertial_est_init(INERTIAL_ESTState& s, const INERTIAL_ESTParams& p);
void inertial_est_step(const INERTIAL_ESTInput& u, INERTIAL_ESTState& s, INERTIAL_ESTOutput& y, double dt);
void inertial_est_reset(INERTIAL_ESTState& s);
} // namespace sad
