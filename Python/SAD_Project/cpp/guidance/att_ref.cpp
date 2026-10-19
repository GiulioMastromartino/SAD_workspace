#include "att_ref.h"
#include <vector>

namespace sad {
void att_ref_init(ATT_REFState& s, const ATT_REFParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void att_ref_step(const ATT_REFInput& u, ATT_REFState& s, ATT_REFOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void att_ref_reset(ATT_REFState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
