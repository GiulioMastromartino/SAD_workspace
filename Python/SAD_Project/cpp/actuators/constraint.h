#pragma once
/* Skeleton module: constraint (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct CONSTRAINTParams {
    std::vector<double> params;
};
struct CONSTRAINTState {
    CONSTRAINTParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct CONSTRAINTInput {
    std::vector<double> u;
};
struct CONSTRAINTOutput {
    std::vector<double> y;
};

void constraint_init(CONSTRAINTState& s, const CONSTRAINTParams& p);
void constraint_step(const CONSTRAINTInput& u, CONSTRAINTState& s, CONSTRAINTOutput& y, double dt);
void constraint_reset(CONSTRAINTState& s);
} // namespace sad
