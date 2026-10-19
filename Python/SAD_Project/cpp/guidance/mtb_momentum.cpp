#include "mtb_momentum.h"
#include <vector>

namespace sad {
void mtb_momentum_init(MTB_MOMENTUMState& s, const MTB_MOMENTUMParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void mtb_momentum_step(const MTB_MOMENTUMInput& u, MTB_MOMENTUMState& s, MTB_MOMENTUMOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void mtb_momentum_reset(MTB_MOMENTUMState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
