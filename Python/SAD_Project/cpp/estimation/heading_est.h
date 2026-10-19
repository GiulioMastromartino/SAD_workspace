#pragma once
/* Skeleton module: heading_est (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct HEADING_ESTParams {
    std::vector<double> params;
};
struct HEADING_ESTState {
    HEADING_ESTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct HEADING_ESTInput {
    std::vector<double> u;
};
struct HEADING_ESTOutput {
    std::vector<double> y;
};

void heading_est_init(HEADING_ESTState& s, const HEADING_ESTParams& p);
void heading_est_step(const HEADING_ESTInput& u, HEADING_ESTState& s, HEADING_ESTOutput& y, double dt);
void heading_est_reset(HEADING_ESTState& s);
} // namespace sad
