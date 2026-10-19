#include "opnav.h"
#include <vector>

namespace sad {
void opnav_init(OPNAVState& s, const OPNAVParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void opnav_step(const OPNAVInput& u, OPNAVState& s, OPNAVOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void opnav_reset(OPNAVState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
