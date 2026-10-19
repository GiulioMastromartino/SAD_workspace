#pragma once
/* Skeleton module: ground (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct GROUNDParams {
    std::vector<double> params;
};
struct GROUNDState {
    GROUNDParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct GROUNDInput {
    std::vector<double> u;
};
struct GROUNDOutput {
    std::vector<double> y;
};

void ground_init(GROUNDState& s, const GROUNDParams& p);
void ground_step(const GROUNDInput& u, GROUNDState& s, GROUNDOutput& y, double dt);
void ground_reset(GROUNDState& s);
} // namespace sad
