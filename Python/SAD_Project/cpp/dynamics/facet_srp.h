#pragma once
/* Skeleton module: facet_srp (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct FACET_SRPParams {
    std::vector<double> params;
};
struct FACET_SRPState {
    FACET_SRPParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct FACET_SRPInput {
    std::vector<double> u;
};
struct FACET_SRPOutput {
    std::vector<double> y;
};

void facet_srp_init(FACET_SRPState& s, const FACET_SRPParams& p);
void facet_srp_step(const FACET_SRPInput& u, FACET_SRPState& s, FACET_SRPOutput& y, double dt);
void facet_srp_reset(FACET_SRPState& s);
} // namespace sad
