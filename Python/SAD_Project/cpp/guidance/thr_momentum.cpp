#include "thr_momentum.h"
#include <vector>

namespace sad {
void thr_momentum_init(THR_MOMENTUMState& s, const THR_MOMENTUMParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void thr_momentum_step(const THR_MOMENTUMInput& u, THR_MOMENTUMState& s, THR_MOMENTUMOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void thr_momentum_reset(THR_MOMENTUMState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
