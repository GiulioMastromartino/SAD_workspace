#pragma once
/* Skeleton module: thrust_cm (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct THRUST_CMParams {
    std::vector<double> params;
};
struct THRUST_CMState {
    THRUST_CMParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct THRUST_CMInput {
    std::vector<double> u;
};
struct THRUST_CMOutput {
    std::vector<double> y;
};

void thrust_cm_init(THRUST_CMState& s, const THRUST_CMParams& p);
void thrust_cm_step(const THRUST_CMInput& u, THRUST_CMState& s, THRUST_CMOutput& y, double dt);
void thrust_cm_reset(THRUST_CMState& s);
} // namespace sad
