#include "motor_voltage.h"
#include <vector>

namespace sad {
void motor_voltage_init(MOTOR_VOLTAGEState& s, const MOTOR_VOLTAGEParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void motor_voltage_step(const MOTOR_VOLTAGEInput& u, MOTOR_VOLTAGEState& s, MOTOR_VOLTAGEOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void motor_voltage_reset(MOTOR_VOLTAGEState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
