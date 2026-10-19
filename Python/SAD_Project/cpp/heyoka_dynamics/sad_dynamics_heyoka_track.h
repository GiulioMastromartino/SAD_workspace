#pragma once

#include "sad_types.h"

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace sad {

struct HeyokaTrackInit {
    Vec3 r = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Vec4 q = Vec4(0, 0, 0, 1);
    Vec3 w = Vec3::Zero();
    Vec3 z = Vec3::Zero();
    double fuel = 0.0;

    double mass = 8.0;
    Vec3 inertia = Vec3(1, 1, 1);
    Vec3 inv_inertia = Vec3(1, 1, 1);
    double mu = 3.986004418e14;

    double tau_max = 0.1;
    double eps = 1e-6;
    double z_max = 0.5;

    std::array<double, 3> Keff{0.007, 0.005, 0.002};
    std::array<double, 3> Peff{0.086, 0.066, 0.033};
    std::array<double, 3> Kieff{1e-7, 1e-7, 1e-7};

    Vec3 roll_ref = Vec3(1, 0, 0);
};

struct HeyokaTrackResult {
    int steps = 0;
    double min_step = 0.0;
    double max_step = 0.0;
    std::array<double, 17> final_state{};
};

class HeyokaTrack {
public:
    explicit HeyokaTrack(double tol = 1e-12, bool compact = false);
    ~HeyokaTrack();

    HeyokaTrack(const HeyokaTrack&) = delete;
    HeyokaTrack& operator=(const HeyokaTrack&) = delete;

    void reset(const HeyokaTrackInit& init, double t0);
    HeyokaTrackResult propagate(double t_end, double sample_dt,
                               const std::function<void(double, const std::array<double, 17>&)>& sample_cb = nullptr);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Process/thread-local cached integrator (avoids repeated LLVM JIT and
// serialization across optimizer workers).
HeyokaTrack& heyoka_track_shared(double tol = 1e-12, bool compact = false);

} // namespace sad
