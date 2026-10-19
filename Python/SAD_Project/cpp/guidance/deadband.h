#pragma once
/* Skeleton module: deadband (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct DEADBANDParams {
    std::vector<double> params;
};
struct DEADBANDState {
    DEADBANDParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct DEADBANDInput {
    std::vector<double> u;
};
struct DEADBANDOutput {
    std::vector<double> y;
};

void deadband_init(DEADBANDState& s, const DEADBANDParams& p);
void deadband_step(const DEADBANDInput& u, DEADBANDState& s, DEADBANDOutput& y, double dt);
void deadband_reset(DEADBANDState& s);
} // namespace sad
