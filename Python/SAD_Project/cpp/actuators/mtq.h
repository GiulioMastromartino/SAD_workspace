#pragma once
/* Skeleton module: mtq (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MTQParams {
    std::vector<double> params;
};
struct MTQState {
    MTQParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MTQInput {
    std::vector<double> u;
};
struct MTQOutput {
    std::vector<double> y;
};

void mtq_init(MTQState& s, const MTQParams& p);
void mtq_step(const MTQInput& u, MTQState& s, MTQOutput& y, double dt);
void mtq_reset(MTQState& s);
} // namespace sad
