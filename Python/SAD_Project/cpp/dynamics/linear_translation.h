#pragma once
/* Skeleton module: linear_translation (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct LINEAR_TRANSLATIONParams {
    std::vector<double> params;
};
struct LINEAR_TRANSLATIONState {
    LINEAR_TRANSLATIONParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct LINEAR_TRANSLATIONInput {
    std::vector<double> u;
};
struct LINEAR_TRANSLATIONOutput {
    std::vector<double> y;
};

void linear_translation_init(LINEAR_TRANSLATIONState& s, const LINEAR_TRANSLATIONParams& p);
void linear_translation_step(const LINEAR_TRANSLATIONInput& u, LINEAR_TRANSLATIONState& s, LINEAR_TRANSLATIONOutput& y, double dt);
void linear_translation_reset(LINEAR_TRANSLATIONState& s);
} // namespace sad
