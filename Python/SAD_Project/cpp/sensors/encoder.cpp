#include "encoder.h"
#include <vector>

namespace sad {
void encoder_init(ENCODERState& s, const ENCODERParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void encoder_step(const ENCODERInput& u, ENCODERState& s, ENCODEROutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void encoder_reset(ENCODERState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
