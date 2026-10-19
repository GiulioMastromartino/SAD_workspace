#include "facet_drag.h"
#include <vector>

namespace sad {
void facet_drag_init(FACET_DRAGState& s, const FACET_DRAGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void facet_drag_step(const FACET_DRAGInput& u, FACET_DRAGState& s, FACET_DRAGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void facet_drag_reset(FACET_DRAGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
