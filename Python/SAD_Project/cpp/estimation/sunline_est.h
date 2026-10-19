#pragma once
/* Skeleton module: sunline_est (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SUNLINE_ESTParams {
    std::vector<double> params;
};
struct SUNLINE_ESTState {
    SUNLINE_ESTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SUNLINE_ESTInput {
    std::vector<double> u;
};
struct SUNLINE_ESTOutput {
    std::vector<double> y;
};

void sunline_est_init(SUNLINE_ESTState& s, const SUNLINE_ESTParams& p);
void sunline_est_step(const SUNLINE_ESTInput& u, SUNLINE_ESTState& s, SUNLINE_ESTOutput& y, double dt);
void sunline_est_reset(SUNLINE_ESTState& s);
} // namespace sad
