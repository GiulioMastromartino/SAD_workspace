#pragma once
/* Skeleton module: mrp_pd (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MRP_PDParams {
    std::vector<double> params;
};
struct MRP_PDState {
    MRP_PDParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MRP_PDInput {
    std::vector<double> u;
};
struct MRP_PDOutput {
    std::vector<double> y;
};

void mrp_pd_init(MRP_PDState& s, const MRP_PDParams& p);
void mrp_pd_step(const MRP_PDInput& u, MRP_PDState& s, MRP_PDOutput& y, double dt);
void mrp_pd_reset(MRP_PDState& s);
} // namespace sad
