#include "vscmg_steering.h"
#include <vector>

namespace sad {
void vscmg_steering_init(VSCMG_STEERINGState& s, const VSCMG_STEERINGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void vscmg_steering_step(const VSCMG_STEERINGInput& u, VSCMG_STEERINGState& s, VSCMG_STEERINGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void vscmg_steering_reset(VSCMG_STEERINGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
