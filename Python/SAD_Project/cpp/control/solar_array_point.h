#pragma once
/* Skeleton module: solar_array_point (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SOLAR_ARRAY_POINTParams {
    std::vector<double> params;
};
struct SOLAR_ARRAY_POINTState {
    SOLAR_ARRAY_POINTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SOLAR_ARRAY_POINTInput {
    std::vector<double> u;
};
struct SOLAR_ARRAY_POINTOutput {
    std::vector<double> y;
};

void solar_array_point_init(SOLAR_ARRAY_POINTState& s, const SOLAR_ARRAY_POINTParams& p);
void solar_array_point_step(const SOLAR_ARRAY_POINTInput& u, SOLAR_ARRAY_POINTState& s, SOLAR_ARRAY_POINTOutput& y, double dt);
void solar_array_point_reset(SOLAR_ARRAY_POINTState& s);
} // namespace sad
