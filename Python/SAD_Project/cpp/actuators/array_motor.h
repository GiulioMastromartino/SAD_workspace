#pragma once
/* Skeleton module: array_motor (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct ARRAY_MOTORParams {
    std::vector<double> params;
};
struct ARRAY_MOTORState {
    ARRAY_MOTORParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct ARRAY_MOTORInput {
    std::vector<double> u;
};
struct ARRAY_MOTOROutput {
    std::vector<double> y;
};

void array_motor_init(ARRAY_MOTORState& s, const ARRAY_MOTORParams& p);
void array_motor_step(const ARRAY_MOTORInput& u, ARRAY_MOTORState& s, ARRAY_MOTOROutput& y, double dt);
void array_motor_reset(ARRAY_MOTORState& s);
} // namespace sad
