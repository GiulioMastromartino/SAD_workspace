#pragma once
/* Skeleton module: constrained_maneuver (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct CONSTRAINED_MANEUVERParams {
    std::vector<double> params;
};
struct CONSTRAINED_MANEUVERState {
    CONSTRAINED_MANEUVERParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct CONSTRAINED_MANEUVERInput {
    std::vector<double> u;
};
struct CONSTRAINED_MANEUVEROutput {
    std::vector<double> y;
};

void constrained_maneuver_init(CONSTRAINED_MANEUVERState& s, const CONSTRAINED_MANEUVERParams& p);
void constrained_maneuver_step(const CONSTRAINED_MANEUVERInput& u, CONSTRAINED_MANEUVERState& s, CONSTRAINED_MANEUVEROutput& y, double dt);
void constrained_maneuver_reset(CONSTRAINED_MANEUVERState& s);
} // namespace sad
