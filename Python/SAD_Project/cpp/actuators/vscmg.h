#pragma once
/* Skeleton module: vscmg (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct VSCMGParams {
    std::vector<double> params;
};
struct VSCMGState {
    VSCMGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct VSCMGInput {
    std::vector<double> u;
};
struct VSCMGOutput {
    std::vector<double> y;
};

void vscmg_init(VSCMGState& s, const VSCMGParams& p);
void vscmg_step(const VSCMGInput& u, VSCMGState& s, VSCMGOutput& y, double dt);
void vscmg_reset(VSCMGState& s);
} // namespace sad
