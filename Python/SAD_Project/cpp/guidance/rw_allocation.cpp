#include "rw_allocation.h"
#include <vector>

namespace sad {
void rw_allocation_init(RW_ALLOCATIONState& s, const RW_ALLOCATIONParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void rw_allocation_step(const RW_ALLOCATIONInput& u, RW_ALLOCATIONState& s, RW_ALLOCATIONOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void rw_allocation_reset(RW_ALLOCATIONState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
