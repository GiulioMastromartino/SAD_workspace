#pragma once
/* Skeleton module: att_ref (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct ATT_REFParams {
    std::vector<double> params;
};
struct ATT_REFState {
    ATT_REFParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct ATT_REFInput {
    std::vector<double> u;
};
struct ATT_REFOutput {
    std::vector<double> y;
};

void att_ref_init(ATT_REFState& s, const ATT_REFParams& p);
void att_ref_step(const ATT_REFInput& u, ATT_REFState& s, ATT_REFOutput& y, double dt);
void att_ref_reset(ATT_REFState& s);
} // namespace sad
