#pragma once
/* Skeleton module: thr_force_mapping (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct THR_FORCE_MAPPINGParams {
    std::vector<double> params;
};
struct THR_FORCE_MAPPINGState {
    THR_FORCE_MAPPINGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct THR_FORCE_MAPPINGInput {
    std::vector<double> u;
};
struct THR_FORCE_MAPPINGOutput {
    std::vector<double> y;
};

void thr_force_mapping_init(THR_FORCE_MAPPINGState& s, const THR_FORCE_MAPPINGParams& p);
void thr_force_mapping_step(const THR_FORCE_MAPPINGInput& u, THR_FORCE_MAPPINGState& s, THR_FORCE_MAPPINGOutput& y, double dt);
void thr_force_mapping_reset(THR_FORCE_MAPPINGState& s);
} // namespace sad
