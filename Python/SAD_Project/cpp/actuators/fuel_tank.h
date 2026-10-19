#pragma once
/* Skeleton module: fuel_tank (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct FUEL_TANKParams {
    std::vector<double> params;
};
struct FUEL_TANKState {
    FUEL_TANKParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct FUEL_TANKInput {
    std::vector<double> u;
};
struct FUEL_TANKOutput {
    std::vector<double> y;
};

void fuel_tank_init(FUEL_TANKState& s, const FUEL_TANKParams& p);
void fuel_tank_step(const FUEL_TANKInput& u, FUEL_TANKState& s, FUEL_TANKOutput& y, double dt);
void fuel_tank_reset(FUEL_TANKState& s);
} // namespace sad
