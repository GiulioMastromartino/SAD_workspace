#pragma once
/* Skeleton module: inertial_3d (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct INERTIAL_3DParams {
    std::vector<double> params;
};
struct INERTIAL_3DState {
    INERTIAL_3DParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct INERTIAL_3DInput {
    std::vector<double> u;
};
struct INERTIAL_3DOutput {
    std::vector<double> y;
};

void inertial_3d_init(INERTIAL_3DState& s, const INERTIAL_3DParams& p);
void inertial_3d_step(const INERTIAL_3DInput& u, INERTIAL_3DState& s, INERTIAL_3DOutput& y, double dt);
void inertial_3d_reset(INERTIAL_3DState& s);
} // namespace sad
