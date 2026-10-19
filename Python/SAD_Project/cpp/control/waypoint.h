#pragma once
/* Skeleton module: waypoint (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct WAYPOINTParams {
    std::vector<double> params;
};
struct WAYPOINTState {
    WAYPOINTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct WAYPOINTInput {
    std::vector<double> u;
};
struct WAYPOINTOutput {
    std::vector<double> y;
};

void waypoint_init(WAYPOINTState& s, const WAYPOINTParams& p);
void waypoint_step(const WAYPOINTInput& u, WAYPOINTState& s, WAYPOINTOutput& y, double dt);
void waypoint_reset(WAYPOINTState& s);
} // namespace sad
