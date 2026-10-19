#include "thr_firing.h"
#include <vector>

namespace sad {
void thr_firing_init(THR_FIRINGState& s, const THR_FIRINGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void thr_firing_step(const THR_FIRINGInput& u, THR_FIRINGState& s, THR_FIRINGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void thr_firing_reset(THR_FIRINGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
