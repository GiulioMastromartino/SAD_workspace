#include "sc_location.h"
#include <vector>

namespace sad {
void sc_location_init(SC_LOCATIONState& s, const SC_LOCATIONParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void sc_location_step(const SC_LOCATIONInput& u, SC_LOCATIONState& s, SC_LOCATIONOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void sc_location_reset(SC_LOCATIONState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
