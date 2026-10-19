#pragma once
/* Skeleton module: ext_forcetorque (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct EXT_FORCETORQUEParams {
    std::vector<double> params;
};
struct EXT_FORCETORQUEState {
    EXT_FORCETORQUEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct EXT_FORCETORQUEInput {
    std::vector<double> u;
};
struct EXT_FORCETORQUEOutput {
    std::vector<double> y;
};

void ext_forcetorque_init(EXT_FORCETORQUEState& s, const EXT_FORCETORQUEParams& p);
void ext_forcetorque_step(const EXT_FORCETORQUEInput& u, EXT_FORCETORQUEState& s, EXT_FORCETORQUEOutput& y, double dt);
void ext_forcetorque_reset(EXT_FORCETORQUEState& s);
} // namespace sad
