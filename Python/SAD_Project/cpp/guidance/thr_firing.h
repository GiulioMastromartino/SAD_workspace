#pragma once
/* Skeleton module: thr_firing (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct THR_FIRINGParams {
    std::vector<double> params;
};
struct THR_FIRINGState {
    THR_FIRINGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct THR_FIRINGInput {
    std::vector<double> u;
};
struct THR_FIRINGOutput {
    std::vector<double> y;
};

void thr_firing_init(THR_FIRINGState& s, const THR_FIRINGParams& p);
void thr_firing_step(const THR_FIRINGInput& u, THR_FIRINGState& s, THR_FIRINGOutput& y, double dt);
void thr_firing_reset(THR_FIRINGState& s);
} // namespace sad
