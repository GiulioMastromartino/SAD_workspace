// ISC License - see native_rw/LICENSE.
//
// Templated MRP attitude parameterisation.
//
// The DCM formula below reproduces EXACTLY the one in
// cubesat_12u_sim/backends/heyoka/heyoka_attitude.py::build_mrp_system,
// which is the validated reference:
//
//   sn2 = |s|^2 ; d = (1+sn2)^2 ; c = 4(1-sn2)/d ; k8 = 8/d
//   R = I + k8 * S2 + c * <skew terms>
//
// with the S2 entries (S^2 of the skew matrix S(s)):
//   s11 = -s2^2-s3^2, s12 = s1*s2, s13 = s1*s3, ...
//
// R is body-from-inertial: v_B = R * v_N ; F_N = R^T * F_B.
//
// BSK's Eigen::MRPd is a CONCRETE double type, so its toRotationMatrix()
// cannot be instantiated for heyoka::expression. Hence this hand-written
// version (design constraint 2 from the Option-A prototype).
#ifndef OMNI_MRP_T_HPP
#define OMNI_MRP_T_HPP

#include "omni/linear3.hpp"

namespace omni {

template <typename S>
struct MRPt {
    Vector3<S> sigma;

    MRPt() { sigma = zero_vec3<S>(); }

    explicit MRPt(const Vector3<S>& s) : sigma(s) {}

    // Body-from-inertial DCM. Uses only + - * / -- safe for expressions.
    Matrix3<S> toRotationMatrix() const {
        const S& s1 = sigma[0];
        const S& s2 = sigma[1];
        const S& s3 = sigma[2];

        const S one = make_scalar<S>(1.0);
        const S sn2 = s1 * s1 + s2 * s2 + s3 * s3;
        const S one_plus = one + sn2;
        const S d = one_plus * one_plus;
        const S c = make_scalar<S>(4.0) * (one - sn2) / d;
        const S k8 = make_scalar<S>(8.0) / d;

        // S^2 entries
        const S s11 = make_scalar<S>(0.0) - s2 * s2 - s3 * s3;
        const S s12 = s1 * s2;
        const S s13 = s1 * s3;
        const S s21 = s1 * s2;
        const S s22 = make_scalar<S>(0.0) - s1 * s1 - s3 * s3;
        const S s23 = s2 * s3;
        const S s31 = s1 * s3;
        const S s32 = s2 * s3;
        const S s33 = make_scalar<S>(0.0) - s1 * s1 - s2 * s2;

        Matrix3<S> R;
        R(0, 0) = one + k8 * s11;
        R(0, 1) = k8 * s12 + c * s3;
        R(0, 2) = k8 * s13 - c * s2;
        R(1, 0) = k8 * s21 - c * s3;
        R(1, 1) = one + k8 * s22;
        R(1, 2) = k8 * s23 + c * s1;
        R(2, 0) = k8 * s31 + c * s2;
        R(2, 1) = k8 * s32 - c * s1;
        R(2, 2) = one + k8 * s33;
        return R;
    }
};

// MRP kinematics: sdot = 0.25 * B(s) * omega.
// B entries reproduce heyoka_rw.py exactly:
//   B11 = 1-sn2+2 s1^2 ; B12 = 2(s1 s2 - s3) ; B13 = 2(s1 s3 + s2) ; ...
template <typename S>
inline Vector3<S> mrp_kinematics(const Vector3<S>& sigma, const Vector3<S>& omega) {
    const S& s1 = sigma[0];
    const S& s2 = sigma[1];
    const S& s3 = sigma[2];
    const S& w1 = omega[0];
    const S& w2 = omega[1];
    const S& w3 = omega[2];

    const S one = make_scalar<S>(1.0);
    const S two = make_scalar<S>(2.0);
    const S quarter = make_scalar<S>(0.25);
    const S sn2 = s1 * s1 + s2 * s2 + s3 * s3;

    const S B11 = one - sn2 + two * s1 * s1;
    const S B12 = two * (s1 * s2 - s3);
    const S B13 = two * (s1 * s3 + s2);
    const S B21 = two * (s2 * s1 + s3);
    const S B22 = one - sn2 + two * s2 * s2;
    const S B23 = two * (s2 * s3 - s1);
    const S B31 = two * (s3 * s1 - s2);
    const S B32 = two * (s3 * s2 + s1);
    const S B33 = one - sn2 + two * s3 * s3;

    Vector3<S> out;
    out[0] = quarter * (B11 * w1 + B12 * w2 + B13 * w3);
    out[1] = quarter * (B21 * w1 + B22 * w2 + B23 * w3);
    out[2] = quarter * (B31 * w1 + B32 * w2 + B33 * w3);
    return out;
}

// Shadow-set reset predicate (numeric only by nature -- the caller owns it).
inline bool mrp_needs_reset(double s1, double s2, double s3) {
    return (s1 * s1 + s2 * s2 + s3 * s3) > 1.0;
}

}  // namespace omni

#endif  // OMNI_MRP_T_HPP
