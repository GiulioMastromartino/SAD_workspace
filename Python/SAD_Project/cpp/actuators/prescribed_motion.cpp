#include "prescribed_motion.h"
#include <vector>

namespace sad {
void prescribed_motion_init(PRESCRIBED_MOTIONState& s, const PRESCRIBED_MOTIONParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void prescribed_motion_step(const PRESCRIBED_MOTIONInput& u, PRESCRIBED_MOTIONState& s, PRESCRIBED_MOTIONOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void prescribed_motion_reset(PRESCRIBED_MOTIONState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
