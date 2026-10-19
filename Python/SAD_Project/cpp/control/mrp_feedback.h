#pragma once
/* Skeleton module: mrp_feedback (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MRP_FEEDBACKParams {
    std::vector<double> params;
};
struct MRP_FEEDBACKState {
    MRP_FEEDBACKParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MRP_FEEDBACKInput {
    std::vector<double> u;
};
struct MRP_FEEDBACKOutput {
    std::vector<double> y;
};

void mrp_feedback_init(MRP_FEEDBACKState& s, const MRP_FEEDBACKParams& p);
void mrp_feedback_step(const MRP_FEEDBACKInput& u, MRP_FEEDBACKState& s, MRP_FEEDBACKOutput& y, double dt);
void mrp_feedback_reset(MRP_FEEDBACKState& s);
} // namespace sad
