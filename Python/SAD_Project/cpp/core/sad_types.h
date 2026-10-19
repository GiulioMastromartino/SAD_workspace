#pragma once
// SAD_Project core types: quaternions (Hamiltonian, scalar-last), vectors, matrices.
// All modules include this. Uses Eigen for linear algebra.

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <vector>

namespace sad {

using Vec3  = Eigen::Vector3d;
using Vec4  = Eigen::Vector4d;   // [x,y,z,w] (scalar-last quaternion)
using Mat3  = Eigen::Matrix3d;
using Mat34 = Eigen::Matrix<double,3,4>;
using Mat4  = Eigen::Matrix4d;

// Quaternion helpers (Hamiltonian, scalar-last: q = [qx,qy,qz,qw])
inline Vec4 quat_mul(const Vec4& a, const Vec4& b) {
    // a * b
    Vec4 r;
    r[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    r[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    r[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    r[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
    return r;
}
inline Vec4 quat_conj(const Vec4& q){ return Vec4(-q[0],-q[1],-q[2],q[3]); }
inline double quat_norm(const Vec4& q){ return q.norm(); }
inline Vec4 quat_normalize(const Vec4& q){ double n=q.norm(); if(n<1e-15) return Vec4(0,0,0,1); return q/n; }
inline Vec3 quat_rotate(const Vec4& q, const Vec3& v){
    // rotate vector v by quaternion q (body<-inertial convention: v_b = q* v * q^-1)
    Vec4 qv(v[0],v[1],v[2],0.0);
    Vec4 r = quat_mul(quat_mul(q, qv), quat_conj(q));
    return Vec3(r[0],r[1],r[2]);
}
// Body-from-inertial DCM (passive rotation), matching quat_derivative():
// q_dot = 0.5 * q (x) [omega, 0], i.e. omega is the body rate.
inline Mat3 quat_to_dcm(const Vec4& q){
    double x=q[0],y=q[1],z=q[2],w=q[3];
    Mat3 R;
    R << 1-2*(y*y+z*z), 2*(x*y + z*w), 2*(x*z - y*w),
         2*(x*y - z*w), 1-2*(x*x+z*z), 2*(y*z + x*w),
         2*(x*z + y*w), 2*(y*z - x*w), 1-2*(x*x+y*y);
    return R;
}
// Inverse of the body-from-inertial quat_to_dcm() above: q such that
// quat_to_dcm(q) == R.
inline Vec4 dcm_to_quat(const Mat3& R){
    // The closed-form below is for the active (inertial-from-body) matrix, so
    // transpose first to keep it the exact inverse of quat_to_dcm().
    Mat3 S = R.transpose();
    double tr = S.trace();
    Vec4 q;
    if (tr > 0) {
        double s = 2.0*std::sqrt(tr+1.0);
        q[3]=0.25*s; q[0]=(S(2,1)-S(1,2))/s; q[1]=(S(0,2)-S(2,0))/s; q[2]=(S(1,0)-S(0,1))/s;
    } else if (S(0,0)>S(1,1) && S(0,0)>S(2,2)) {
        double s = 2.0*std::sqrt(1.0+S(0,0)-S(1,1)-S(2,2));
        q[0]=0.25*s; q[1]=(S(0,1)+S(1,0))/s; q[2]=(S(0,2)+S(2,0))/s; q[3]=(S(2,1)-S(1,2))/s;
    } else if (S(1,1)>S(2,2)) {
        double s = 2.0*std::sqrt(1.0+S(1,1)-S(0,0)-S(2,2));
        q[0]=(S(0,1)+S(1,0))/s; q[1]=0.25*s; q[2]=(S(1,2)+S(2,1))/s; q[3]=(S(0,2)-S(2,0))/s;
    } else {
        double s = 2.0*std::sqrt(1.0+S(2,2)-S(0,0)-S(1,1));
        q[0]=(S(0,2)+S(2,0))/s; q[1]=(S(1,2)+S(2,1))/s; q[2]=0.25*s; q[3]=(S(1,0)-S(0,1))/s;
    }
    return quat_normalize(q);
}
inline Vec3 cross(const Vec3& a, const Vec3& b){ return a.cross(b); }
// (placeholder removed)

// Quaternion kinematics: q_dot = 0.5 * Xi(q) * omega, where Xi maps omega to q_dot
inline Vec4 quat_derivative(const Vec4& q, const Vec3& omega){
    // q = [qx,qy,qz,qw]
    double x=q[0],y=q[1],z=q[2],w=q[3];
    Vec4 dq;
    dq[0] = 0.5*( w*omega[0] + y*omega[2] - z*omega[1]);
    dq[1] = 0.5*( w*omega[1] - x*omega[2] + z*omega[0]);
    dq[2] = 0.5*( w*omega[2] + x*omega[1] - y*omega[0]);
    dq[3] = 0.5*(-x*omega[0] - y*omega[1] - z*omega[2]);
    return dq;
}

// quaternion error q_err = q_current * q_desired^{-1}; returns vector part (for PD)
// (current relative to desired) so that tau = -K*e - P*w is stabilizing.
inline Vec3 quat_error_vec(const Vec4& qd, const Vec4& qc){
    Vec4 qe = quat_mul(qc, quat_conj(qd));
    // shortest path: if qw<0 flip
    if (qe[3] < 0) { qe = -qe; }
    return Vec3(qe[0], qe[1], qe[2]);
}

// Euler (small) angle from quat vector part: 4*atan(|q_vec|) approximate angle
inline double quat_angle(const Vec4& q){
    Vec4 qn = quat_normalize(q);
    double s = std::sqrt(qn[0]*qn[0]+qn[1]*qn[1]+qn[2]*qn[2]);
    if (s < 1e-12) return 0.0;
    return 2.0*std::atan2(s, std::fabs(qn[3]));
}

} // namespace sad
