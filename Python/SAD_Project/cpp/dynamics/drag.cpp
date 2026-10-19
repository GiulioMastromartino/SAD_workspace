#include "drag.h"
#include <vector>

namespace sad {
void drag_init(DRAGState& s, const DRAGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void drag_step(const DRAGInput& u, DRAGState& s, DRAGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void drag_reset(DRAGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
