#include "thr_force_mapping.h"
#include <vector>

namespace sad {
void thr_force_mapping_init(THR_FORCE_MAPPINGState& s, const THR_FORCE_MAPPINGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void thr_force_mapping_step(const THR_FORCE_MAPPINGInput& u, THR_FORCE_MAPPINGState& s, THR_FORCE_MAPPINGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void thr_force_mapping_reset(THR_FORCE_MAPPINGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
