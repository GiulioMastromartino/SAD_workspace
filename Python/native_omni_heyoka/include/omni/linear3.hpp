// ISC License - see native_rw/LICENSE.
//
// Hand-written 3x3 / 3-vector linear algebra over an arbitrary scalar type.
//
// WHY THIS EXISTS, AND WHY IT IS NOT JUST "avoid Eigen":
// Eigen's dense kernels cannot carry heyoka::expression as a scalar type.
// Two hard blockers, both reproduced at compile time:
//   1. Eigen's redux path (sum/trace/norm) constructs Scalar(0), which is
//      ambiguous -- heyoka::expression has several explicit constructors that
//      accept integer/float arguments.
//   2. Even the plain mat-vec A*v routes through an inner-product that calls
//      .sum() (ProductEvaluators.h, the gemv evaluator).
// So Eigen is used purely as STORAGE (fixed-size Matrix<Scalar,3,3>), and every
// operation is written explicitly. This is also faster than the generic path
// and matches the hand-written adjugate/determinant the design mandates.
//
// Every function here uses only + - * /, so the expression instantiation builds
// a clean Taylor-friendly graph (design constraint A3: no min/max/abs/piecewise).
#ifndef OMNI_LINEAR3_HPP
#define OMNI_LINEAR3_HPP

#include <cmath>

#include "omni/scalar_traits.hpp"

namespace omni {

// --- construction ---------------------------------------------------------
template <typename S>
inline void set_zero(Matrix3<S>& M) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) M(i, j) = make_scalar<S>(0.0);
}

template <typename S>
inline void set_identity(Matrix3<S>& M) {
    set_zero(M);
    for (int i = 0; i < 3; ++i) M(i, i) = make_scalar<S>(1.0);
}

template <typename S>
inline void set_diag(Matrix3<S>& M, S a, S b, S c) {
    set_zero(M);
    M(0, 0) = a;
    M(1, 1) = b;
    M(2, 2) = c;
}

template <typename S>
inline Vector3<S> zero_vec3() {
    Vector3<S> v;
    for (int i = 0; i < 3; ++i) v[i] = make_scalar<S>(0.0);
    return v;
}

// --- elementwise add / sub (avoid Eigen's CwiseBinaryOp with expressions) --
template <typename S>
inline Matrix3<S> matadd3(const Matrix3<S>& A, const Matrix3<S>& B) {
    Matrix3<S> out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out(i, j) = A(i, j) + B(i, j);
    return out;
}

template <typename S>
inline Matrix3<S> matsub3(const Matrix3<S>& A, const Matrix3<S>& B) {
    Matrix3<S> out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out(i, j) = A(i, j) - B(i, j);
    return out;
}

template <typename S>
inline Vector3<S> vecadd3(const Vector3<S>& a, const Vector3<S>& b) {
    Vector3<S> out;
    for (int i = 0; i < 3; ++i) out[i] = a[i] + b[i];
    return out;
}

template <typename S>
inline Vector3<S> vecsub3(const Vector3<S>& a, const Vector3<S>& b) {
    Vector3<S> out;
    for (int i = 0; i < 3; ++i) out[i] = a[i] - b[i];
    return out;
}

// --- scale by a scalar (avoids Eigen's matrix*scalar operator) -------------
template <typename S>
inline Matrix3<S> scale_mat3(const Matrix3<S>& A, const S& s) {
    Matrix3<S> out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out(i, j) = A(i, j) * s;
    return out;
}

template <typename S>
inline Vector3<S> scale_vec3(const Vector3<S>& v, const S& s) {
    Vector3<S> out;
    for (int i = 0; i < 3; ++i) out[i] = v[i] * s;
    return out;
}

template <typename S>
inline Vector3<S> negate_vec3(const Vector3<S>& v) {
    Vector3<S> out;
    for (int i = 0; i < 3; ++i) out[i] = make_scalar<S>(0.0) - v[i];
    return out;
}

// --- matrix products ------------------------------------------------------
template <typename S>
inline Matrix3<S> matmul3(const Matrix3<S>& A, const Matrix3<S>& B) {
    Matrix3<S> out;
    set_zero(out);
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) {
            // Structural-zero skipping only makes sense numerically; an
            // expression cannot be compared, so the branch compiles away.
            if constexpr (is_expression_v<S>) {
                for (int j = 0; j < 3; ++j) out(i, j) = out(i, j) + A(i, k) * B(k, j);
            } else {
                const S aik = A(i, k);
                if (aik != static_cast<S>(0)) {
                    for (int j = 0; j < 3; ++j) out(i, j) = out(i, j) + aik * B(k, j);
                }
            }
        }
    return out;
}

// C = A*B + C  (accumulate in place)
template <typename S>
inline void matmul3_madd(Matrix3<S>& C, const Matrix3<S>& A, const Matrix3<S>& B) {
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) {
            if constexpr (is_expression_v<S>) {
                for (int j = 0; j < 3; ++j) C(i, j) = C(i, j) + A(i, k) * B(k, j);
            } else {
                const S aik = A(i, k);
                if (aik != static_cast<S>(0)) {
                    for (int j = 0; j < 3; ++j) C(i, j) = C(i, j) + A(i, k) * B(k, j);
                }
            }
        }
}

// --- matrix-vector --------------------------------------------------------
template <typename S>
inline Vector3<S> matvec3(const Matrix3<S>& A, const Vector3<S>& v) {
    Vector3<S> out = zero_vec3<S>();
    for (int i = 0; i < 3; ++i) {
        S acc = make_scalar<S>(0.0);
        for (int k = 0; k < 3; ++k) acc = acc + A(i, k) * v[k];
        out[i] = acc;
    }
    return out;
}

// --- determinant / adjugate / inverse (explicit 3x3) ----------------------
template <typename S>
inline S det3(const Matrix3<S>& M) {
    return M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1))
         - M(0, 1) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0))
         + M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0));
}

// adjugate = transpose of the matrix of cofactors
template <typename S>
inline Matrix3<S> adjugate3(const Matrix3<S>& M) {
    Matrix3<S> A;
    A(0, 0) = M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1);
    A(0, 1) = make_scalar<S>(0.0) - (M(0, 1) * M(2, 2) - M(0, 2) * M(2, 1));
    A(0, 2) = M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1);
    A(1, 0) = make_scalar<S>(0.0) - (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0));
    A(1, 1) = M(0, 0) * M(2, 2) - M(0, 2) * M(2, 0);
    A(1, 2) = make_scalar<S>(0.0) - (M(0, 0) * M(1, 2) - M(0, 2) * M(1, 0));
    A(2, 0) = M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0);
    A(2, 1) = make_scalar<S>(0.0) - (M(0, 0) * M(2, 1) - M(0, 1) * M(2, 0));
    A(2, 2) = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
    return A;
}

// Explicit 3x3 inverse. Works for both double and heyoka::expression.
// WP25-C2 M4 NaN posture (deliberate): no det==0 guard — 1/det on a singular
// input yields Inf/NaN loudly in the graph, never a silent wrong inverse;
// callers guarantee det>0 (positive-definite inertia blocks, nonzero radii).
template <typename S>
inline Matrix3<S> inv3(const Matrix3<S>& M) {
    const S s = make_scalar<S>(1.0) / det3(M);
    const Matrix3<S> a = adjugate3(M);
    Matrix3<S> out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out(i, j) = a(i, j) * s;
    return out;
}

// --- cross / dot products -------------------------------------------------
template <typename S>
inline Vector3<S> cross3(const Vector3<S>& a, const Vector3<S>& b) {
    Vector3<S> out;
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
    return out;
}

template <typename S>
inline S dot3(const Vector3<S>& a, const Vector3<S>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template <typename S>
inline S sqrt_scalar(const S& x) {
    if constexpr (is_expression_v<S>) {
        return heyoka::sqrt(x);
    } else {
        using std::sqrt;
        return sqrt(x);
    }
}

template <typename S>
inline S norm3(const Vector3<S>& v) {
    return sqrt_scalar(dot3(v, v));
}

}  // namespace omni

#endif  // OMNI_LINEAR3_HPP
