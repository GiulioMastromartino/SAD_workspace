#pragma once
/* Skeleton module: nav_aggregate (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct NAV_AGGREGATEParams {
    std::vector<double> params;
};
struct NAV_AGGREGATEState {
    NAV_AGGREGATEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct NAV_AGGREGATEInput {
    std::vector<double> u;
};
struct NAV_AGGREGATEOutput {
    std::vector<double> y;
};

void nav_aggregate_init(NAV_AGGREGATEState& s, const NAV_AGGREGATEParams& p);
void nav_aggregate_step(const NAV_AGGREGATEInput& u, NAV_AGGREGATEState& s, NAV_AGGREGATEOutput& y, double dt);
void nav_aggregate_reset(NAV_AGGREGATEState& s);
} // namespace sad
