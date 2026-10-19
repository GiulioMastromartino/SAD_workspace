#pragma once
/* Skeleton module: hinged_rigid_body (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct HINGED_RIGID_BODYParams {
    std::vector<double> params;
};
struct HINGED_RIGID_BODYState {
    HINGED_RIGID_BODYParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct HINGED_RIGID_BODYInput {
    std::vector<double> u;
};
struct HINGED_RIGID_BODYOutput {
    std::vector<double> y;
};

void hinged_rigid_body_init(HINGED_RIGID_BODYState& s, const HINGED_RIGID_BODYParams& p);
void hinged_rigid_body_step(const HINGED_RIGID_BODYInput& u, HINGED_RIGID_BODYState& s, HINGED_RIGID_BODYOutput& y, double dt);
void hinged_rigid_body_reset(HINGED_RIGID_BODYState& s);
} // namespace sad
