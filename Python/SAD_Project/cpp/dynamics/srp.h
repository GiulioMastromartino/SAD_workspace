#pragma once
/* Skeleton module: srp (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SRPParams {
    std::vector<double> params;
};
struct SRPState {
    SRPParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SRPInput {
    std::vector<double> u;
};
struct SRPOutput {
    std::vector<double> y;
};

void srp_init(SRPState& s, const SRPParams& p);
void srp_step(const SRPInput& u, SRPState& s, SRPOutput& y, double dt);
void srp_reset(SRPState& s);
} // namespace sad
