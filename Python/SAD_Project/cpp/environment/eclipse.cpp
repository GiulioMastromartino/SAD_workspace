#include "eclipse.h"
#include <vector>

namespace sad {
void eclipse_init(ECLIPSEState& s, const ECLIPSEParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void eclipse_step(const ECLIPSEInput& u, ECLIPSEState& s, ECLIPSEOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void eclipse_reset(ECLIPSEState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
