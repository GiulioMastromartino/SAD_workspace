#include "scalar_joint.h"
#include <vector>

namespace sad {
void scalar_joint_init(SCALAR_JOINTState& s, const SCALAR_JOINTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void scalar_joint_step(const SCALAR_JOINTInput& u, SCALAR_JOINTState& s, SCALAR_JOINTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void scalar_joint_reset(SCALAR_JOINTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
