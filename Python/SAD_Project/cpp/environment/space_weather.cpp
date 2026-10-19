#include "space_weather.h"
#include <vector>

namespace sad {
void space_weather_init(SPACE_WEATHERState& s, const SPACE_WEATHERParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void space_weather_step(const SPACE_WEATHERInput& u, SPACE_WEATHERState& s, SPACE_WEATHEROutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void space_weather_reset(SPACE_WEATHERState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
