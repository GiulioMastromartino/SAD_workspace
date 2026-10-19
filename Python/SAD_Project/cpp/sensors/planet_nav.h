#pragma once
/* Skeleton module: planet_nav (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct PLANET_NAVParams {
    std::vector<double> params;
};
struct PLANET_NAVState {
    PLANET_NAVParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct PLANET_NAVInput {
    std::vector<double> u;
};
struct PLANET_NAVOutput {
    std::vector<double> y;
};

void planet_nav_init(PLANET_NAVState& s, const PLANET_NAVParams& p);
void planet_nav_step(const PLANET_NAVInput& u, PLANET_NAVState& s, PLANET_NAVOutput& y, double dt);
void planet_nav_reset(PLANET_NAVState& s);
} // namespace sad
