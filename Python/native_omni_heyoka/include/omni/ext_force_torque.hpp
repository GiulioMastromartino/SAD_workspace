// ISC License - see native_rw/LICENSE.
//
// ExtForceTorque contributor: the DynamicEffector channel for an
// external force and torque applied to the hub (stock extForceTorque).
//
// Stock's ExtForceTorque exposes three independent body/inertial inputs:
//   extForce_B (body) , extForce_N (inertial) , extTorquePntB_B (body)
// plus three message-driven inputs (cmdForceBody, cmdForceInertial,
// cmdTorqueBody) that ADD (never replace) to the constants. No state, no
// mass, no blocks -- DynamicEffector channel only.
//
// This contributor holds 9 input slots (3+3+3), all module-written per tick
// (A3: the graph reads them verbatim; all composition/branches live
// module-side). It ACCUMULATES onto ForceTorqueT (env pass), so it composes
// with thrust/drag/SRP/mag rather than overwriting.
#ifndef OMNI_EXT_FORCE_TORQUE_HPP
#define OMNI_EXT_FORCE_TORQUE_HPP

#include "omni/contributors.hpp"
#include "omni/force_torque.hpp"
#include "omni/linear3.hpp"

namespace omni {

template <typename S>
class ExtForceTorqueContributorT : public ContributorT<S> {
public:
    ExtForceTorqueContributorT() {}

    std::string kind() const override { return "extft"; }
    size_t n_states() const override { return 0; }
    // Body force, body torque, inertial force -- in that order, 3 each.
    size_t n_inputs() const override { return 9; }
    size_t n_pars() const override { return 0; }
    std::vector<std::string> state_names() const override { return {}; }
    std::vector<std::string> input_names() const override {
        return {"extFxB", "extFyB", "extFzB", "extTxB", "extTyB", "extTzB",
                "extFxN", "extFyN", "extFzN"};
    }
    std::vector<std::string> par_names() const override { return {}; }
    std::vector<double> par_defaults() const override { return {}; }
    bool has_force_torque() const override { return true; }

    void updateContributions(double, BackSubMatricesT<S>&, const Vector3<S>&,
                            const EvalCtx<S>&) const override {}

    std::vector<S> state_derivatives(const Vector3<S>&, const Vector3<S>&,
                                      const EvalCtx<S>&) const override {
        return {};
    }

    void computeForceTorque(double, double, ForceTorqueT<S>& ft,
                            const EvalCtx<S>& ctx) const override {
        const std::vector<S>& P = ctx.pars;
        Vector3<S> fB = zero_vec3<S>();
        Vector3<S> tB = zero_vec3<S>();
        Vector3<S> fNinertial = zero_vec3<S>();
        const size_t o = this->input_offset;
        for (int i = 0; i < 3; ++i) {
            fB[i] = P[o + size_t(i)];
            tB[i] = P[o + size_t(3 + i)];
            fNinertial[i] = P[o + size_t(6 + i)];
        }
        // Body force -> inertial: F_N contribution = R^T F_B (stock semantics).
        const MRPt<S> mrp(ctx.sigma());
        const Matrix3<S> R = mrp.toRotationMatrix();
        Matrix3<S> Rt;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) Rt(i, j) = R(j, i);
        const Vector3<S> fNfromB = matvec3(Rt, fB);
        const Vector3<S> fN = vecadd3(fNfromB, fNinertial);
        ft.forceExternal_B = vecadd3(ft.forceExternal_B, fB);
        ft.torqueExternalPntB_B = vecadd3(ft.torqueExternalPntB_B, tB);
        ft.forceExternal_N = vecadd3(ft.forceExternal_N, fN);
    }
};

}  // namespace omni

#endif  // OMNI_EXT_FORCE_TORQUE_HPP