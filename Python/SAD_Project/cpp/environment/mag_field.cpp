#include "mag_field.h"
#include <vector>

namespace sad {
void mag_field_init(MAG_FIELDState& s, const MAG_FIELDParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void mag_field_step(const MAG_FIELDInput& u, MAG_FIELDState& s, MAG_FIELDOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void mag_field_reset(MAG_FIELDState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
