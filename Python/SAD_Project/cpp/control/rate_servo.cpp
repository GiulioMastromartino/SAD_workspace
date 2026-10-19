#include "rate_servo.h"
#include <vector>

namespace sad {
void rate_servo_init(RATE_SERVOState& s, const RATE_SERVOParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void rate_servo_step(const RATE_SERVOInput& u, RATE_SERVOState& s, RATE_SERVOOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void rate_servo_reset(RATE_SERVOState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
