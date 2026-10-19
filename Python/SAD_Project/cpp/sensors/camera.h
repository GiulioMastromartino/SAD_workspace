#pragma once
/* Skeleton module: camera (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct CAMERAParams {
    std::vector<double> params;
};
struct CAMERAState {
    CAMERAParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct CAMERAInput {
    std::vector<double> u;
};
struct CAMERAOutput {
    std::vector<double> y;
};

void camera_init(CAMERAState& s, const CAMERAParams& p);
void camera_step(const CAMERAInput& u, CAMERAState& s, CAMERAOutput& y, double dt);
void camera_reset(CAMERAState& s);
} // namespace sad
