#include "mtq.h"
#include <vector>

namespace sad {
void mtq_init(MTQState& s, const MTQParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void mtq_step(const MTQInput& u, MTQState& s, MTQOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void mtq_reset(MTQState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
