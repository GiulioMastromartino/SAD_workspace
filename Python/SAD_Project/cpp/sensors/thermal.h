#pragma once
/* Skeleton module: thermal (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct THERMALParams {
    std::vector<double> params;
};
struct THERMALState {
    THERMALParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct THERMALInput {
    std::vector<double> u;
};
struct THERMALOutput {
    std::vector<double> y;
};

void thermal_init(THERMALState& s, const THERMALParams& p);
void thermal_step(const THERMALInput& u, THERMALState& s, THERMALOutput& y, double dt);
void thermal_reset(THERMALState& s);
} // namespace sad
