#pragma once
/* Skeleton module: sun_safe (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SUN_SAFEParams {
    std::vector<double> params;
};
struct SUN_SAFEState {
    SUN_SAFEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SUN_SAFEInput {
    std::vector<double> u;
};
struct SUN_SAFEOutput {
    std::vector<double> y;
};

void sun_safe_init(SUN_SAFEState& s, const SUN_SAFEParams& p);
void sun_safe_step(const SUN_SAFEInput& u, SUN_SAFEState& s, SUN_SAFEOutput& y, double dt);
void sun_safe_reset(SUN_SAFEState& s);
} // namespace sad
