#pragma once
/* Skeleton module: dv_guidance (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct DV_GUIDANCEParams {
    std::vector<double> params;
};
struct DV_GUIDANCEState {
    DV_GUIDANCEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct DV_GUIDANCEInput {
    std::vector<double> u;
};
struct DV_GUIDANCEOutput {
    std::vector<double> y;
};

void dv_guidance_init(DV_GUIDANCEState& s, const DV_GUIDANCEParams& p);
void dv_guidance_step(const DV_GUIDANCEInput& u, DV_GUIDANCEState& s, DV_GUIDANCEOutput& y, double dt);
void dv_guidance_reset(DV_GUIDANCEState& s);
} // namespace sad
