#pragma once
/* Skeleton module: celestial_point (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct CELESTIAL_POINTParams {
    std::vector<double> params;
};
struct CELESTIAL_POINTState {
    CELESTIAL_POINTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct CELESTIAL_POINTInput {
    std::vector<double> u;
};
struct CELESTIAL_POINTOutput {
    std::vector<double> y;
};

void celestial_point_init(CELESTIAL_POINTState& s, const CELESTIAL_POINTParams& p);
void celestial_point_step(const CELESTIAL_POINTInput& u, CELESTIAL_POINTState& s, CELESTIAL_POINTOutput& y, double dt);
void celestial_point_reset(CELESTIAL_POINTState& s);
} // namespace sad
