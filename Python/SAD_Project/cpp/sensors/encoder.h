#pragma once
/* Skeleton module: encoder (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct ENCODERParams {
    std::vector<double> params;
};
struct ENCODERState {
    ENCODERParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct ENCODERInput {
    std::vector<double> u;
};
struct ENCODEROutput {
    std::vector<double> y;
};

void encoder_init(ENCODERState& s, const ENCODERParams& p);
void encoder_step(const ENCODERInput& u, ENCODERState& s, ENCODEROutput& y, double dt);
void encoder_reset(ENCODERState& s);
} // namespace sad
