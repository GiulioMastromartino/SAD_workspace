#pragma once
/* Skeleton module: fdir (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct FDIRParams {
    std::vector<double> params;
};
struct FDIRState {
    FDIRParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct FDIRInput {
    std::vector<double> u;
};
struct FDIROutput {
    std::vector<double> y;
};

void fdir_init(FDIRState& s, const FDIRParams& p);
void fdir_step(const FDIRInput& u, FDIRState& s, FDIROutput& y, double dt);
void fdir_reset(FDIRState& s);
} // namespace sad
