#pragma once
/* Skeleton module: mtb_momentum (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MTB_MOMENTUMParams {
    std::vector<double> params;
};
struct MTB_MOMENTUMState {
    MTB_MOMENTUMParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MTB_MOMENTUMInput {
    std::vector<double> u;
};
struct MTB_MOMENTUMOutput {
    std::vector<double> y;
};

void mtb_momentum_init(MTB_MOMENTUMState& s, const MTB_MOMENTUMParams& p);
void mtb_momentum_step(const MTB_MOMENTUMInput& u, MTB_MOMENTUMState& s, MTB_MOMENTUMOutput& y, double dt);
void mtb_momentum_reset(MTB_MOMENTUMState& s);
} // namespace sad
