#include "vscmg.h"
#include <vector>

namespace sad {
void vscmg_init(VSCMGState& s, const VSCMGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void vscmg_step(const VSCMGInput& u, VSCMGState& s, VSCMGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void vscmg_reset(VSCMGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
