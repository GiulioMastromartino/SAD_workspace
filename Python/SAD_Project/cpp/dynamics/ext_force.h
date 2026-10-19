#pragma once
/* Skeleton module: ext_force (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct EXT_FORCEParams {
    std::vector<double> params;
};
struct EXT_FORCEState {
    EXT_FORCEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct EXT_FORCEInput {
    std::vector<double> u;
};
struct EXT_FORCEOutput {
    std::vector<double> y;
};

void ext_force_init(EXT_FORCEState& s, const EXT_FORCEParams& p);
void ext_force_step(const EXT_FORCEInput& u, EXT_FORCEState& s, EXT_FORCEOutput& y, double dt);
void ext_force_reset(EXT_FORCEState& s);
} // namespace sad
