// ISC License - see native_rw/LICENSE.
//
// Scalar-type plumbing for omni. The whole design rests on writing the
// contribution arithmetic ONCE, templated on a scalar type, and instantiating
// it twice:
//
//   * S = double             -> numeric cross-check / trajectory output
//   * S = heyoka::expression -> symbolic build, from which the EOM falls out
//
// Eigen does not know about heyoka::expression, so NumTraits must be supplied.
//
// NOTE ON DESIGN CONSTRAINT A3/A6: the symbolic graph must use only
// + - * / and sqrt. No min/max/abs/if/tanh/piecewise. Motor-torque saturation
// is applied by the caller before the value reaches par[0..2] -- exactly as
// native_rw does in C++ -- so the graph never needs a clamp.
#ifndef OMNI_SCALAR_TRAITS_HPP
#define OMNI_SCALAR_TRAITS_HPP

#include <heyoka/heyoka.hpp>
#include <heyoka/expression.hpp>

// Pull in Eigen BEFORE specialising NumTraits, otherwise the partial
// specialisation may not be visible where Eigen instantiates it.
#include <Eigen/Dense>

#include <type_traits>

namespace omni {

// ---------------------------------------------------------------------------
// Trait: is this the symbolic scalar type?
// ---------------------------------------------------------------------------
template <typename S>
struct is_expression : std::false_type {};

template <>
struct is_expression<heyoka::expression> : std::true_type {};

template <typename S>
inline constexpr bool is_expression_v = is_expression<S>::value;

// ---------------------------------------------------------------------------
// make_scalar: promote a plain literal to the working scalar type.
// ---------------------------------------------------------------------------
template <typename S>
inline S make_scalar(double v) {
    return static_cast<S>(v);
}

template <>
inline heyoka::expression make_scalar<heyoka::expression>(double v) {
    return heyoka::expression{v};
}

// Convenience: the scalar type corresponding to an Eigen expression is just
// itself (Matrix<double,3,3> and Matrix<expression,3,3> both work).
template <typename S>
using Matrix3 = Eigen::Matrix<S, 3, 3>;

template <typename S>
using Matrix3X = Eigen::Matrix<S, 3, Eigen::Dynamic>;

template <typename S>
using Vector3 = Eigen::Matrix<S, 3, 1>;

// ---------------------------------------------------------------------------
// Tests that the trait hooks actually compose with Eigen.
// ---------------------------------------------------------------------------
static_assert(is_expression_v<double> == false);
static_assert(is_expression_v<heyoka::expression> == true);

}  // namespace omni

// ---------------------------------------------------------------------------
// Eigen support for heyoka::expression.
//
// Eigen requires NumTraits<T> to expose a handful of static members. An
// expression has no ordering, no epsilon, no numeric limits -- so the limit
// members return deliberately meaningless dummy values. They must never be
// used by the arithmetic we run; if one of them is ever called it indicates
// that some Eigen code path is comparing or rounding, which our contribution
// arithmetic must not do (and does not).
// ---------------------------------------------------------------------------
template <>
struct Eigen::NumTraits<heyoka::expression> {
    using Real = heyoka::expression;
    // Eigen 3.4 distinguishes "floating point" from integer scalars.
    using FloatingPoint = heyoka::expression;
    using NonInteger = heyoka::expression;
    using Nested = heyoka::expression;
    // Used by Eigen's promote_scalar_arg machinery. Aliased to the expression
    // type itself: there is no simpler literal type that an expression is
    // unambiguously convertible from (heyoka::expression has several
    // constructors that all accept integer/float arguments).
    using Literal = heyoka::expression;

    enum {
        IsComplex = 0,
        IsInteger = 0,
        IsSigned = 1,
        RequireInitialization = 0,
        ReadCost = 1,
        AddCost = 1,
        MulCost = 1
    };

    // Deliberate dummies. An expression cannot answer these meaningfully.
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression epsilon() {
        return heyoka::expression{0.0};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression dummy_precision() {
        return heyoka::expression{1.0e-12};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression dummy_precision(
        const heyoka::expression&) {
        return heyoka::expression{1.0e-12};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression highest() {
        return heyoka::expression{1.0};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression lowest() {
        return heyoka::expression{-1.0};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression smallest() {
        return heyoka::expression{0.0};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression smallest_normal() {
        return heyoka::expression{1.0e-300};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression digits10() {
        return heyoka::expression{0.0};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression min_exponent() {
        return heyoka::expression{0.0};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE heyoka::expression max_exponent() {
        return heyoka::expression{0.0};
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool equal(const heyoka::expression& a,
                                                           const heyoka::expression& b) {
        // Fall back on pointer/value comparison of the expression. Not used by
        // any of our arithmetic.
        return &a == &b;
    }
    static EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE bool not_equal(const heyoka::expression& a,
                                                               const heyoka::expression& b) {
        return &a != &b;
    }
};

#endif  // OMNI_SCALAR_TRAITS_HPP
