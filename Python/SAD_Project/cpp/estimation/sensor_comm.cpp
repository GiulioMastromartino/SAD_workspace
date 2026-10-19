#include "sensor_comm.h"
#include <vector>

namespace sad {
void sensor_comm_init(SENSOR_COMMState& s, const SENSOR_COMMParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void sensor_comm_step(const SENSOR_COMMInput& u, SENSOR_COMMState& s, SENSOR_COMMOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void sensor_comm_reset(SENSOR_COMMState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
