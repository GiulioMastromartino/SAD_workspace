#include "mrp_feedback.h"
#include <vector>

namespace sad {
void mrp_feedback_init(MRP_FEEDBACKState& s, const MRP_FEEDBACKParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void mrp_feedback_step(const MRP_FEEDBACKInput& u, MRP_FEEDBACKState& s, MRP_FEEDBACKOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void mrp_feedback_reset(MRP_FEEDBACKState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
