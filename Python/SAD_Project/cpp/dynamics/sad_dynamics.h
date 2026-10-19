#pragma once
#include "sad_types.h"
#include "sad_module.h"

namespace sad {
struct DynamicsParams {
    double mass = 8.0;
    Mat3 inertia = Mat3::Identity();
    Mat3 inv_inertia = Mat3::Identity();
    double mu = 3.986004418e14;
    Vec3 r0 = Vec3::Zero();
    Vec3 v0 = Vec3::Zero();
    Vec4 q0 = Vec4(0,0,0,1);
    Vec3 w0 = Vec3::Zero();
    bool gravity_gradient = true;
};
struct DynamicsState {
    DynamicsParams params;
    Vec3 r = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Vec4 q = Vec4(0,0,0,1);
    Vec3 w = Vec3::Zero();
    double t = 0.0;
};
struct DynamicsInput {
    Vec3 force_b = Vec3::Zero();
    Vec3 torque_b = Vec3::Zero();
};
struct DynamicsOutput {
    Vec3 r = Vec3::Zero();
    Vec3 v = Vec3::Zero();
    Vec4 q = Vec4(0,0,0,1);
    Vec3 w = Vec3::Zero();
};

void dynamics_init(DynamicsState& s, const DynamicsParams& p);
void dynamics_step(const DynamicsInput& u, DynamicsState& s, DynamicsOutput& y, double dt);
void dynamics_reset(DynamicsState& s);
} // namespace sad
