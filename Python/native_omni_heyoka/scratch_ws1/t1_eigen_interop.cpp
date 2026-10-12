// Stage-1 smoke test: Eigen as STORAGE for heyoka::expression + our hand-written
// 3x3 algebra. Avoids every Eigen kernel (redux, nullary, scalar-product) that
// constructs Scalar(0) and hits expression's ambiguous constructors.
#include "omni/linear3.hpp"

#include <iostream>
#include <typeinfo>

using omni::Matrix3;
using omni::Vector3;

// ONE function body, TWO scalar types -- the point of the whole design.
template <typename S>
static Vector3<S> kinematic_accel(const Matrix3<S>& A, const Vector3<S>& v) {
    // A*v + (-(v x v)) == A*v ; the cross term is structurally zero but keeps
    // cross3 instantiated in both scalar types.
    return omni::vecadd3(omni::matvec3(A, v),
                         omni::negate_vec3(omni::cross3(v, v)));
}

int main() {
    // --- 1. plain double path -------------------------------------------
    Matrix3<double> Ad;
    Ad << 1.0, 2.0, 3.0,
        0.0, 1.0, 4.0,
        5.0, 6.0, 0.0;
    Vector3<double> vd;
    vd << 0.7, -0.2, 0.4;
    Vector3<double> rd = kinematic_accel(Ad, vd);
    std::cout << "double  : " << rd[0] << " " << rd[1] << " " << rd[2] << "\n";

    // --- 2. expression path ---------------------------------------------
    auto rx = heyoka::make_vars("rx");
    auto ry = heyoka::make_vars("ry");
    auto rz = heyoka::make_vars("rz");

    Matrix3<heyoka::expression> Ae;
    Ae(0, 0) = rx * rx; Ae(0, 1) = ry; Ae(0, 2) = rz;
    Ae(1, 0) = ry; Ae(1, 1) = rz * rz; Ae(1, 2) = rx;
    Ae(2, 0) = rz; Ae(2, 1) = rx; Ae(2, 2) = rx * ry + heyoka::expression(1.0);

    Vector3<heyoka::expression> ve;
    ve[0] = rx; ve[1] = ry; ve[2] = rz;

    Vector3<heyoka::expression> re = kinematic_accel(Ae, ve);
    std::cout << "express : graph built OK\n";

    std::cout << "double type   : " << typeid(rd).name() << "\n";
    std::cout << "express type  : " << typeid(re).name() << "\n";

    // --- 3. mixed arithmetic via our own helpers -------------------------
    Matrix3<heyoka::expression> I3;
    omni::set_identity(I3);
    Matrix3<heyoka::expression> Ae2 =
        omni::matadd3(omni::scale_mat3(Ae, heyoka::expression(2.0)), I3);
    Vector3<heyoka::expression> r2 = kinematic_accel(Ae2, ve);
    std::cout << "mixed    : OK\n";

    // --- 4. hand-written 3x3 inverse vs Eigen, on plain doubles ----------
    Eigen::Matrix<double, 3, 3> R;
    R << 1.0, 2.0, 3.0,
        0.0, 1.0, 4.0,
        5.0, 6.0, 0.0;
    Eigen::Matrix<double, 3, 3> want = R.inverse();
    Eigen::Matrix<double, 3, 3> got = omni::inv3(R);
    double err = (got - want).cwiseAbs().maxCoeff();
    std::cout << "inv3 vs Eigen: max|diff| = " << err << "\n";
    if (!(err < 1e-12)) { std::cout << "FAIL\n"; return 1; }

    // --- 5. heyoka JIT accepts a graph built through our helpers ---------
    auto xx = heyoka::make_vars("xx");
    heyoka::taylor_adaptive<double> ta(
        std::vector<std::pair<heyoka::expression, heyoka::expression>>{
            {xx, xx * xx + heyoka::expression(0.5)}},
        std::vector<double>{0.5}, heyoka::kw::tol = 1e-12);
    std::cout << "heyoka JIT accepted our expression graph: state=" << ta.get_state()[0]
              << " order=" << ta.get_order() << "\n";

    std::cout << "\nSTAGE 1 (Eigen storage + hand-rolled 3x3 algebra): PASS\n";
    return 0;
}
