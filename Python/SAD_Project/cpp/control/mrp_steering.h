#pragma once
/* Skeleton module: mrp_steering (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MRP_STEERINGParams {
    std::vector<double> params;
};
struct MRP_STEERINGState {
    MRP_STEERINGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MRP_STEERINGInput {
    std::vector<double> u;
};
struct MRP_STEERINGOutput {
    std::vector<double> y;
};

void mrp_steering_init(MRP_STEERINGState& s, const MRP_STEERINGParams& p);
void mrp_steering_step(const MRP_STEERINGInput& u, MRP_STEERINGState& s, MRP_STEERINGOutput& y, double dt);
void mrp_steering_reset(MRP_STEERINGState& s);
} // namespace sad
