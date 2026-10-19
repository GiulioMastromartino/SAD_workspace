#pragma once
/* Skeleton module: scalar_joint (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SCALAR_JOINTParams {
    std::vector<double> params;
};
struct SCALAR_JOINTState {
    SCALAR_JOINTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SCALAR_JOINTInput {
    std::vector<double> u;
};
struct SCALAR_JOINTOutput {
    std::vector<double> y;
};

void scalar_joint_init(SCALAR_JOINTState& s, const SCALAR_JOINTParams& p);
void scalar_joint_step(const SCALAR_JOINTInput& u, SCALAR_JOINTState& s, SCALAR_JOINTOutput& y, double dt);
void scalar_joint_reset(SCALAR_JOINTState& s);
} // namespace sad
