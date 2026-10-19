#pragma once
/* Skeleton module: small_body_nav (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SMALL_BODY_NAVParams {
    std::vector<double> params;
};
struct SMALL_BODY_NAVState {
    SMALL_BODY_NAVParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SMALL_BODY_NAVInput {
    std::vector<double> u;
};
struct SMALL_BODY_NAVOutput {
    std::vector<double> y;
};

void small_body_nav_init(SMALL_BODY_NAVState& s, const SMALL_BODY_NAVParams& p);
void small_body_nav_step(const SMALL_BODY_NAVInput& u, SMALL_BODY_NAVState& s, SMALL_BODY_NAVOutput& y, double dt);
void small_body_nav_reset(SMALL_BODY_NAVState& s);
} // namespace sad
