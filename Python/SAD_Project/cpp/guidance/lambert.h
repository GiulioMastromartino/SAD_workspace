#pragma once
/* Skeleton module: lambert (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct LAMBERTParams {
    std::vector<double> params;
};
struct LAMBERTState {
    LAMBERTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct LAMBERTInput {
    std::vector<double> u;
};
struct LAMBERTOutput {
    std::vector<double> y;
};

void lambert_init(LAMBERTState& s, const LAMBERTParams& p);
void lambert_step(const LAMBERTInput& u, LAMBERTState& s, LAMBERTOutput& y, double dt);
void lambert_reset(LAMBERTState& s);
} // namespace sad
