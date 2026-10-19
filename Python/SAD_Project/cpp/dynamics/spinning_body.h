#pragma once
/* Skeleton module: spinning_body (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SPINNING_BODYParams {
    std::vector<double> params;
};
struct SPINNING_BODYState {
    SPINNING_BODYParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SPINNING_BODYInput {
    std::vector<double> u;
};
struct SPINNING_BODYOutput {
    std::vector<double> y;
};

void spinning_body_init(SPINNING_BODYState& s, const SPINNING_BODYParams& p);
void spinning_body_step(const SPINNING_BODYInput& u, SPINNING_BODYState& s, SPINNING_BODYOutput& y, double dt);
void spinning_body_reset(SPINNING_BODYState& s);
} // namespace sad
