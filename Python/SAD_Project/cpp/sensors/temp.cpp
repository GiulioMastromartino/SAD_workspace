#include "temp.h"
#include <vector>

namespace sad {
void temp_init(TEMPState& s, const TEMPParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void temp_step(const TEMPInput& u, TEMPState& s, TEMPOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void temp_reset(TEMPState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
