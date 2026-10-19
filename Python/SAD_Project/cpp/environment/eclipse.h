#pragma once
/* Skeleton module: eclipse (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct ECLIPSEParams {
    std::vector<double> params;
};
struct ECLIPSEState {
    ECLIPSEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct ECLIPSEInput {
    std::vector<double> u;
};
struct ECLIPSEOutput {
    std::vector<double> y;
};

void eclipse_init(ECLIPSEState& s, const ECLIPSEParams& p);
void eclipse_step(const ECLIPSEInput& u, ECLIPSEState& s, ECLIPSEOutput& y, double dt);
void eclipse_reset(ECLIPSEState& s);
} // namespace sad
