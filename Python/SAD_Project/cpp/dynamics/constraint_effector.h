#pragma once
/* Skeleton module: constraint_effector (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct CONSTRAINT_EFFECTORParams {
    std::vector<double> params;
};
struct CONSTRAINT_EFFECTORState {
    CONSTRAINT_EFFECTORParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct CONSTRAINT_EFFECTORInput {
    std::vector<double> u;
};
struct CONSTRAINT_EFFECTOROutput {
    std::vector<double> y;
};

void constraint_effector_init(CONSTRAINT_EFFECTORState& s, const CONSTRAINT_EFFECTORParams& p);
void constraint_effector_step(const CONSTRAINT_EFFECTORInput& u, CONSTRAINT_EFFECTORState& s, CONSTRAINT_EFFECTOROutput& y, double dt);
void constraint_effector_reset(CONSTRAINT_EFFECTORState& s);
} // namespace sad
