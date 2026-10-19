#pragma once
/* Skeleton module: rw_allocation (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct RW_ALLOCATIONParams {
    std::vector<double> params;
};
struct RW_ALLOCATIONState {
    RW_ALLOCATIONParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct RW_ALLOCATIONInput {
    std::vector<double> u;
};
struct RW_ALLOCATIONOutput {
    std::vector<double> y;
};

void rw_allocation_init(RW_ALLOCATIONState& s, const RW_ALLOCATIONParams& p);
void rw_allocation_step(const RW_ALLOCATIONInput& u, RW_ALLOCATIONState& s, RW_ALLOCATIONOutput& y, double dt);
void rw_allocation_reset(RW_ALLOCATIONState& s);
} // namespace sad
