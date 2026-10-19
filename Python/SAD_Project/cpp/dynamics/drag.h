#pragma once
/* Skeleton module: drag (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct DRAGParams {
    std::vector<double> params;
};
struct DRAGState {
    DRAGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct DRAGInput {
    std::vector<double> u;
};
struct DRAGOutput {
    std::vector<double> y;
};

void drag_init(DRAGState& s, const DRAGParams& p);
void drag_step(const DRAGInput& u, DRAGState& s, DRAGOutput& y, double dt);
void drag_reset(DRAGState& s);
} // namespace sad
