#pragma once
/* Skeleton module: ephem_nav (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct EPHEM_NAVParams {
    std::vector<double> params;
};
struct EPHEM_NAVState {
    EPHEM_NAVParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct EPHEM_NAVInput {
    std::vector<double> u;
};
struct EPHEM_NAVOutput {
    std::vector<double> y;
};

void ephem_nav_init(EPHEM_NAVState& s, const EPHEM_NAVParams& p);
void ephem_nav_step(const EPHEM_NAVInput& u, EPHEM_NAVState& s, EPHEM_NAVOutput& y, double dt);
void ephem_nav_reset(EPHEM_NAVState& s);
} // namespace sad
