#pragma once
/* Skeleton module: solar_flux (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SOLAR_FLUXParams {
    std::vector<double> params;
};
struct SOLAR_FLUXState {
    SOLAR_FLUXParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SOLAR_FLUXInput {
    std::vector<double> u;
};
struct SOLAR_FLUXOutput {
    std::vector<double> y;
};

void solar_flux_init(SOLAR_FLUXState& s, const SOLAR_FLUXParams& p);
void solar_flux_step(const SOLAR_FLUXInput& u, SOLAR_FLUXState& s, SOLAR_FLUXOutput& y, double dt);
void solar_flux_reset(SOLAR_FLUXState& s);
} // namespace sad
