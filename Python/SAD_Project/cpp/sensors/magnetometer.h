#pragma once
/* Skeleton module: magnetometer (sensors). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MAGNETOMETERParams {
    std::vector<double> params;
};
struct MAGNETOMETERState {
    MAGNETOMETERParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MAGNETOMETERInput {
    std::vector<double> u;
};
struct MAGNETOMETEROutput {
    std::vector<double> y;
};

void magnetometer_init(MAGNETOMETERState& s, const MAGNETOMETERParams& p);
void magnetometer_step(const MAGNETOMETERInput& u, MAGNETOMETERState& s, MAGNETOMETEROutput& y, double dt);
void magnetometer_reset(MAGNETOMETERState& s);
} // namespace sad
