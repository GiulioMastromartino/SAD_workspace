#pragma once
/* Skeleton module: facet_drag (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct FACET_DRAGParams {
    std::vector<double> params;
};
struct FACET_DRAGState {
    FACET_DRAGParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct FACET_DRAGInput {
    std::vector<double> u;
};
struct FACET_DRAGOutput {
    std::vector<double> y;
};

void facet_drag_init(FACET_DRAGState& s, const FACET_DRAGParams& p);
void facet_drag_step(const FACET_DRAGInput& u, FACET_DRAGState& s, FACET_DRAGOutput& y, double dt);
void facet_drag_reset(FACET_DRAGState& s);
} // namespace sad
