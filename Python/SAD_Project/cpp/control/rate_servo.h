#pragma once
/* Skeleton module: rate_servo (control). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct RATE_SERVOParams {
    std::vector<double> params;
};
struct RATE_SERVOState {
    RATE_SERVOParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct RATE_SERVOInput {
    std::vector<double> u;
};
struct RATE_SERVOOutput {
    std::vector<double> y;
};

void rate_servo_init(RATE_SERVOState& s, const RATE_SERVOParams& p);
void rate_servo_step(const RATE_SERVOInput& u, RATE_SERVOState& s, RATE_SERVOOutput& y, double dt);
void rate_servo_reset(RATE_SERVOState& s);
} // namespace sad
