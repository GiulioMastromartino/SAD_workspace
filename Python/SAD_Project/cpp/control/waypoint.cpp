#include "waypoint.h"
#include <vector>

namespace sad {
void waypoint_init(WAYPOINTState& s, const WAYPOINTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void waypoint_step(const WAYPOINTInput& u, WAYPOINTState& s, WAYPOINTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void waypoint_reset(WAYPOINTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
