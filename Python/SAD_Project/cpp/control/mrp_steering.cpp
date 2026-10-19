#include "mrp_steering.h"
#include <vector>

namespace sad {
void mrp_steering_init(MRP_STEERINGState& s, const MRP_STEERINGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void mrp_steering_step(const MRP_STEERINGInput& u, MRP_STEERINGState& s, MRP_STEERINGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void mrp_steering_reset(MRP_STEERINGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
