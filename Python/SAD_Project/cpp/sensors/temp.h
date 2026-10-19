#pragma once
/* Skeleton module: temp (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct TEMPParams {
    std::vector<double> params;
};
struct TEMPState {
    TEMPParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct TEMPInput {
    std::vector<double> u;
};
struct TEMPOutput {
    std::vector<double> y;
};

void temp_init(TEMPState& s, const TEMPParams& p);
void temp_step(const TEMPInput& u, TEMPState& s, TEMPOutput& y, double dt);
void temp_reset(TEMPState& s);
} // namespace sad
