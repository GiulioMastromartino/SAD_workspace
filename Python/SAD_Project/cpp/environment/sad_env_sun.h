#pragma once
#include "sad_types.h"
#include "sad_module.h"

namespace sad {
struct SunParams {
    double distance = 1.496e11; // m
    double year = 365.25*86400.0;
    double phase0 = 0.5*3.141592653589793;
    Vec3 roll_ref = Vec3(1,0,0);
};
struct SunState { SunParams params; double t = 0.0; };
struct SunInput { double t = 0.0; };
struct SunOutput {
    Vec3 sun_n = Vec3(0,1,0); // sun direction in inertial
    Vec4 q_sun_pointing = Vec4(0,0,0,1); // target attitude to point body +X at sun
};

void sun_init(SunState& s, const SunParams& p);
void sun_step(const SunInput& u, SunState& s, SunOutput& y, double dt);
void sun_reset(SunState& s);

// helper: quaternion that rotates body +X to align with sun direction (inertial), with roll ref
Vec4 sun_pointing_quaternion(const Vec3& sun_dir_n, const Vec3& roll_ref_n);
} // namespace sad
