#pragma once
/* Skeleton module: hill (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct HILLParams {
    std::vector<double> params;
};
struct HILLState {
    HILLParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct HILLInput {
    std::vector<double> u;
};
struct HILLOutput {
    std::vector<double> y;
};

void hill_init(HILLState& s, const HILLParams& p);
void hill_step(const HILLInput& u, HILLState& s, HILLOutput& y, double dt);
void hill_reset(HILLState& s);
} // namespace sad
