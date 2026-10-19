#pragma once
/* Skeleton module: pointing (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct POINTINGParams {
    std::vector<double> params;
};
struct POINTINGState {
    POINTINGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct POINTINGInput {
    std::vector<double> u;
};
struct POINTINGOutput {
    std::vector<double> y;
};

void pointing_init(POINTINGState& s, const POINTINGParams& p);
void pointing_step(const POINTINGInput& u, POINTINGState& s, POINTINGOutput& y, double dt);
void pointing_reset(POINTINGState& s);
} // namespace sad
