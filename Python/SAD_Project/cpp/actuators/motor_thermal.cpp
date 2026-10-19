#include "motor_thermal.h"
#include <vector>

namespace sad {
void motor_thermal_init(MOTOR_THERMALState& s, const MOTOR_THERMALParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void motor_thermal_step(const MOTOR_THERMALInput& u, MOTOR_THERMALState& s, MOTOR_THERMALOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void motor_thermal_reset(MOTOR_THERMALState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
