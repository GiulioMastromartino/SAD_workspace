#pragma once
/* Skeleton module: planet_ephem (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct PLANET_EPHEMParams {
    std::vector<double> params;
};
struct PLANET_EPHEMState {
    PLANET_EPHEMParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct PLANET_EPHEMInput {
    std::vector<double> u;
};
struct PLANET_EPHEMOutput {
    std::vector<double> y;
};

void planet_ephem_init(PLANET_EPHEMState& s, const PLANET_EPHEMParams& p);
void planet_ephem_step(const PLANET_EPHEMInput& u, PLANET_EPHEMState& s, PLANET_EPHEMOutput& y, double dt);
void planet_ephem_reset(PLANET_EPHEMState& s);
} // namespace sad
