#include "sad_env_sun.h"
#include "sad_timing.h"
#include <cmath>

namespace sad {
static const double PI = 3.14159265358979323846;

Vec4 sun_pointing_quaternion(const Vec3& sun_dir_n, const Vec3& roll_ref_n) {
    // body +X aligns with sun_dir_n; roll pinned by projecting roll_ref onto plane perpendicular to sun
    Vec3 x = sun_dir_n.normalized();
    Vec3 r = roll_ref_n - (roll_ref_n.dot(x))*x;
    if (r.norm() < 1e-9) { r = Vec3(0,0,1) - (Vec3(0,0,1).dot(x))*x; }
    if (r.norm() < 1e-9) r = Vec3(0,1,0);
    r.normalize();
    Vec3 z = x.cross(r).normalized(); // body Z? choose z perpendicular
    // body frame: x=sun, z = x cross roll (so y = z cross x)
    Vec3 y = z.cross(x).normalized();
    // DCM body-from-inertial: columns are body axes expressed in inertial? R_bi has rows body axes.
    // We want R such that R * sun_dir = [1,0,0]. Build rotation matrix whose first row = x^T.
    Mat3 R;
    R.row(0) = x.transpose();
    R.row(1) = y.transpose();
    R.row(2) = z.transpose();
    return dcm_to_quat(R);
}

void sun_init(SunState& s, const SunParams& p) { s.params = p; s.t = 0.0; }
void sun_step(const SunInput& u, SunState& s, SunOutput& y, double dt) {
    SAD_TIME(TSun);
    (void)dt;
    s.t = u.t;
    const SunParams& p = s.params;
    double ang = 2*PI*u.t / p.year + p.phase0;
    // sun sweeps circle in inertial XY plane at distance
    Vec3 sun(std::cos(ang), std::sin(ang), 0.0);
    sun *= p.distance; // position
    Vec3 dir = sun.normalized(); // direction from origin to sun ~ from sc? approximate
    y.sun_n = dir;
    { SAD_TIME(TSunPointQuat); y.q_sun_pointing = sun_pointing_quaternion(dir, p.roll_ref); }
}
void sun_reset(SunState& s) { s.t = 0.0; }
} // namespace sad
