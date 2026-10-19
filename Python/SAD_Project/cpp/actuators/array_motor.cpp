#include "array_motor.h"
#include <vector>

namespace sad {
void array_motor_init(ARRAY_MOTORState& s, const ARRAY_MOTORParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void array_motor_step(const ARRAY_MOTORInput& u, ARRAY_MOTORState& s, ARRAY_MOTOROutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void array_motor_reset(ARRAY_MOTORState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
