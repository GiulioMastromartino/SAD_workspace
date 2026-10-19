#include "hinged_motor.h"
#include <vector>

namespace sad {
void hinged_motor_init(HINGED_MOTORState& s, const HINGED_MOTORParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void hinged_motor_step(const HINGED_MOTORInput& u, HINGED_MOTORState& s, HINGED_MOTOROutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void hinged_motor_reset(HINGED_MOTORState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
