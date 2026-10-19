#pragma once
/* Skeleton module: fuel_slosh (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct FUEL_SLOSHParams {
    std::vector<double> params;
};
struct FUEL_SLOSHState {
    FUEL_SLOSHParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct FUEL_SLOSHInput {
    std::vector<double> u;
};
struct FUEL_SLOSHOutput {
    std::vector<double> y;
};

void fuel_slosh_init(FUEL_SLOSHState& s, const FUEL_SLOSHParams& p);
void fuel_slosh_step(const FUEL_SLOSHInput& u, FUEL_SLOSHState& s, FUEL_SLOSHOutput& y, double dt);
void fuel_slosh_reset(FUEL_SLOSHState& s);
} // namespace sad
