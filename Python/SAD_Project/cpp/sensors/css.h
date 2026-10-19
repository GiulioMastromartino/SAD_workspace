#pragma once
/* Skeleton module: css (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct CSSParams {
    std::vector<double> params;
};
struct CSSState {
    CSSParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct CSSInput {
    std::vector<double> u;
};
struct CSSOutput {
    std::vector<double> y;
};

void css_init(CSSState& s, const CSSParams& p);
void css_step(const CSSInput& u, CSSState& s, CSSOutput& y, double dt);
void css_reset(CSSState& s);
} // namespace sad
