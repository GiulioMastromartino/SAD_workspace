#include "sad_sensor_startracker.h"
#include "sad_timing.h"
#include <cmath>

namespace sad {
void startracker_init(StarTrackerState& s, const StarTrackerParams& p) {
    s.params = p;
    s.rng.seed(p.seed);
    s.valid = true;
}
void startracker_step(const StarTrackerInput& u, StarTrackerState& s, StarTrackerOutput& y, double dt) {
    SAD_TIME(TStarTracker);
    (void)dt;
    const StarTrackerParams& p = s.params;
    y.valid = true;
    // rate cap
    if (u.omega_body.norm() > p.rate_cap) y.valid = false;
    // sun flare: if sun in FOV (sun_dir_b roughly along boresight +X), dropout
    if (u.sun_dir_b.norm() > 1e-9) {
        Vec3 sd = u.sun_dir_b.normalized();
        if (sd[0] > 0.9) y.valid = false; // sun near boresight
    }
    // random dropout
    std::uniform_real_distribution<double> ud(0.0,1.0);
    if (ud(s.rng) < p.dropout_prob) y.valid = false;

    if (!y.valid) {
        y.q_meas = u.q_true;
        return;
    }
    // add small rotation noise: convert quat to angle-axis perturbation
    std::normal_distribution<double> nd(0.0, p.sigma);
    Vec3 err(nd(s.rng), nd(s.rng), nd(s.rng));
    // apply as quaternion multiplication: q_meas = q_err * q_true
    double a = err.norm();
    Vec4 qerr;
    if (a < 1e-12) qerr = Vec4(0,0,0,1);
    else {
        double s_ = std::sin(a/2)/a;
        qerr = Vec4(err[0]*s_, err[1]*s_, err[2]*s_, std::cos(a/2));
    }
    y.q_meas = quat_normalize(quat_mul(qerr, u.q_true));
}
void startracker_reset(StarTrackerState& s) {
    s.rng.seed(s.params.seed);
    s.valid = true;
}
} // namespace sad
