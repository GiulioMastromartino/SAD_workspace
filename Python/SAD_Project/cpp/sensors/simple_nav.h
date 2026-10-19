#pragma once
/* Skeleton module: simple_nav (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SIMPLE_NAVParams {
    std::vector<double> params;
};
struct SIMPLE_NAVState {
    SIMPLE_NAVParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SIMPLE_NAVInput {
    std::vector<double> u;
};
struct SIMPLE_NAVOutput {
    std::vector<double> y;
};

void simple_nav_init(SIMPLE_NAVState& s, const SIMPLE_NAVParams& p);
void simple_nav_step(const SIMPLE_NAVInput& u, SIMPLE_NAVState& s, SIMPLE_NAVOutput& y, double dt);
void simple_nav_reset(SIMPLE_NAVState& s);
} // namespace sad
