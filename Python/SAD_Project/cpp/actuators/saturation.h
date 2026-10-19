#pragma once
/* Skeleton module: saturation (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SATURATIONParams {
    std::vector<double> params;
};
struct SATURATIONState {
    SATURATIONParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SATURATIONInput {
    std::vector<double> u;
};
struct SATURATIONOutput {
    std::vector<double> y;
};

void saturation_init(SATURATIONState& s, const SATURATIONParams& p);
void saturation_step(const SATURATIONInput& u, SATURATIONState& s, SATURATIONOutput& y, double dt);
void saturation_reset(SATURATIONState& s);
} // namespace sad
