#pragma once
/* Skeleton module: vscmg_steering (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct VSCMG_STEERINGParams {
    std::vector<double> params;
};
struct VSCMG_STEERINGState {
    VSCMG_STEERINGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct VSCMG_STEERINGInput {
    std::vector<double> u;
};
struct VSCMG_STEERINGOutput {
    std::vector<double> y;
};

void vscmg_steering_init(VSCMG_STEERINGState& s, const VSCMG_STEERINGParams& p);
void vscmg_steering_step(const VSCMG_STEERINGInput& u, VSCMG_STEERINGState& s, VSCMG_STEERINGOutput& y, double dt);
void vscmg_steering_reset(VSCMG_STEERINGState& s);
} // namespace sad
