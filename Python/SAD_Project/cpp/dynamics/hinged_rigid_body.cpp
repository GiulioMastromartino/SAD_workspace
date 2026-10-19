#include "hinged_rigid_body.h"
#include <vector>

namespace sad {
void hinged_rigid_body_init(HINGED_RIGID_BODYState& s, const HINGED_RIGID_BODYParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void hinged_rigid_body_step(const HINGED_RIGID_BODYInput& u, HINGED_RIGID_BODYState& s, HINGED_RIGID_BODYOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void hinged_rigid_body_reset(HINGED_RIGID_BODYState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
