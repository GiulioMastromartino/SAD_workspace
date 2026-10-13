// ISC License - see native_rw/LICENSE.
//
// Templated back-substitution blocks + Schur-complement solve.
//
// Mirrors Basilisk's BackSubMatrices struct (stateEffector.h:31-38), but
// templated on the scalar type so the SAME arithmetic runs numerically
// (S = double) and symbolically (S = heyoka::expression).
//
// Solve (standard hub/effector back-substitution):
//   S      = A - B * D^-1 * C
//   rDDot  = S^-1 * (vecTrans - B * D^-1 * vecRot)
//   wDot   = D^-1 * (vecRot - C * rDDot)
//
// All inverses are the explicit 3x3 adjugate/determinant from linear3.hpp.
// No Eigen kernels, no redux, no Scalar(0) construction.
#ifndef OMNI_BACKSUB_T_HPP
#define OMNI_BACKSUB_T_HPP

#include "omni/force_torque.hpp"
#include "omni/linear3.hpp"

namespace omni {

template <typename S>
struct BackSubMatricesT {
    Matrix3<S> A, B, C, D;
    Vector3<S> vecTrans, vecRot;

    BackSubMatricesT() {
        set_zero(A); set_zero(B); set_zero(C); set_zero(D);
        vecTrans = zero_vec3<S>();
        vecRot = zero_vec3<S>();
    }
};

// Solve the coupled hub/effector system into translational and angular
// accelerations. All arithmetic is + - * / only.
// Solve the coupled hub/effector system into translational and angular
// accelerations, with any DynamicEffector force/torque folded in first.
//
//   vecTrans += forceExternal_N                (m*a = m*g_N + F_N)
//   vecRot   += torqueExternalPntB_B           (I*wdot = -(w x I w) + T_B)
//
// then the standard Schur-complement back-substitution. All arithmetic is
// + - * / only (design A3: no clamps anywhere in the symbolic graph).
template <typename S>
inline void solve_backsub(BackSubMatricesT<S>& m,
                          Vector3<S>& rDDot_BN_N,
                          Vector3<S>& omegaDot_BN_B,
                          const ForceTorqueT<S>* ft = nullptr) {
    if (ft != nullptr) {
        for (int i = 0; i < 3; ++i) {
            m.vecTrans[i] = m.vecTrans[i] + ft->forceExternal_N[i];
            m.vecRot[i] = m.vecRot[i] + ft->torqueExternalPntB_B[i];
        }
    }

    // WP25-C2 M4 posture: no det(D)==0 guard — D is the positive-definite hub block, so det(D)>0 by construction; a singular build Infs/NaNs loudly via inv3, never silently.
    const Matrix3<S> Dinv = inv3(m.D);
    // B*Dinv reused twice below.
    const Matrix3<S> BDinv = matmul3(m.B, Dinv);
    // S = A - B * D^-1 * C   (Schur complement; 'Sch' avoids shadowing S)
    const Matrix3<S> Sch = matsub3(m.A, matmul3(BDinv, m.C));
    // WP25-C2 M4 posture: no det(Sch)==0 guard — same loud-Inf/NaN posture as D; a degenerate coupling has no silent path.
    const Matrix3<S> Schinv = inv3(Sch);
    // rDDot = Sch^-1 * (vecTrans - B * D^-1 * vecRot)
    const Vector3<S> rhs_t = vecsub3(m.vecTrans, matvec3(BDinv, m.vecRot));
    rDDot_BN_N = matvec3(Schinv, rhs_t);
    // wDot = D^-1 * (vecRot - C * rDDot)
    const Vector3<S> rhs_r = vecsub3(m.vecRot, matvec3(m.C, rDDot_BN_N));
    omegaDot_BN_B = matvec3(Dinv, rhs_r);
}

}  // namespace omni

#endif  // OMNI_BACKSUB_T_HPP
