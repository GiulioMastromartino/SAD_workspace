#include "torque2dipole.h"
#include <vector>

namespace sad {
void torque2dipole_init(TORQUE2DIPOLEState& s, const TORQUE2DIPOLEParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void torque2dipole_step(const TORQUE2DIPOLEInput& u, TORQUE2DIPOLEState& s, TORQUE2DIPOLEOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void torque2dipole_reset(TORQUE2DIPOLEState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
