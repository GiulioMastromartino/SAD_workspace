#pragma once
/* Skeleton module: prescribed_motion (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct PRESCRIBED_MOTIONParams {
    std::vector<double> params;
};
struct PRESCRIBED_MOTIONState {
    PRESCRIBED_MOTIONParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct PRESCRIBED_MOTIONInput {
    std::vector<double> u;
};
struct PRESCRIBED_MOTIONOutput {
    std::vector<double> y;
};

void prescribed_motion_init(PRESCRIBED_MOTIONState& s, const PRESCRIBED_MOTIONParams& p);
void prescribed_motion_step(const PRESCRIBED_MOTIONInput& u, PRESCRIBED_MOTIONState& s, PRESCRIBED_MOTIONOutput& y, double dt);
void prescribed_motion_reset(PRESCRIBED_MOTIONState& s);
} // namespace sad
