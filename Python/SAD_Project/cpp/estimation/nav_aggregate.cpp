#include "nav_aggregate.h"
#include <vector>

namespace sad {
void nav_aggregate_init(NAV_AGGREGATEState& s, const NAV_AGGREGATEParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void nav_aggregate_step(const NAV_AGGREGATEInput& u, NAV_AGGREGATEState& s, NAV_AGGREGATEOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void nav_aggregate_reset(NAV_AGGREGATEState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
