#include "sad_sensor_imu.h"
#include <cmath>

namespace sad {
void imu_init(IMUState& s, const IMUParams& p) {
    s.params = p;
    s.rng.seed(p.seed);
    s.bias = Vec3(p.gyro_bias0[0], p.gyro_bias0[1], p.gyro_bias0[2]);
}
void imu_step(const IMUInput& u, IMUState& s, IMUOutput& y, double dt) {
    const IMUParams& p = s.params;
    std::normal_distribution<double> nd(0.0,1.0);
    Vec3 noise;
    for (int i=0;i<3;++i) noise[i] = nd(s.rng)*p.gyro_arw[i]/std::sqrt(dt>1e-9?dt:1e-9);
    // bias random walk
    for (int i=0;i<3;++i) {
        double dw = nd(s.rng)*p.gyro_bias_walk[i]/std::sqrt(dt>1e-9?dt:1e-9);
        s.bias[i] += dw*dt;
        // first-order correlation
        double alpha = dt/(p.bias_tau+dt);
        s.bias[i] = (1-alpha)*s.bias[i] + alpha*p.gyro_bias0[i];
    }
    y.omega_meas = u.omega_true + s.bias + noise;
}
void imu_reset(IMUState& s) {
    s.rng.seed(s.params.seed);
    s.bias = Vec3(s.params.gyro_bias0[0], s.params.gyro_bias0[1], s.params.gyro_bias0[2]);
}
} // namespace sad
