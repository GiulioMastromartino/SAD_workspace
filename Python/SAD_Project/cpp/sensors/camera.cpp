#include "camera.h"
#include <vector>

namespace sad {
void camera_init(CAMERAState& s, const CAMERAParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void camera_step(const CAMERAInput& u, CAMERAState& s, CAMERAOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void camera_reset(CAMERAState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
