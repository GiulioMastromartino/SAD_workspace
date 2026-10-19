#pragma once
/* Skeleton module: hinged_motor (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct HINGED_MOTORParams {
    std::vector<double> params;
};
struct HINGED_MOTORState {
    HINGED_MOTORParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct HINGED_MOTORInput {
    std::vector<double> u;
};
struct HINGED_MOTOROutput {
    std::vector<double> y;
};

void hinged_motor_init(HINGED_MOTORState& s, const HINGED_MOTORParams& p);
void hinged_motor_step(const HINGED_MOTORInput& u, HINGED_MOTORState& s, HINGED_MOTOROutput& y, double dt);
void hinged_motor_reset(HINGED_MOTORState& s);
} // namespace sad
