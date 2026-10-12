// ISC License - see native_rw/LICENSE.
//
// ForceTorqueT: the DynamicEffector contribution channel.
//
// Stock Basilisk splits effectors in two -- StateEffector (which writes
// BackSubMatrices blocks) and DynamicEffector (which writes three flat
// 3-vectors).  Thrusters, drag and SRP use the second path.  This header holds
// only that struct so both backsub_t.hpp (which folds it into the solve) and
// contributors.hpp (which declares it) can include it without a cycle.
#ifndef OMNI_FORCE_TORQUE_HPP
#define OMNI_FORCE_TORQUE_HPP

#include "omni/linear3.hpp"

namespace omni {

template <typename S>
struct ForceTorqueT {
    Vector3<S> forceExternal_N;       //!< inertial force [N]
    Vector3<S> forceExternal_B;       //!< body-frame force [N]
    Vector3<S> torqueExternalPntB_B;  //!< body torque about CoM [Nm]

    ForceTorqueT() {
        forceExternal_N = zero_vec3<S>();
        forceExternal_B = zero_vec3<S>();
        torqueExternalPntB_B = zero_vec3<S>();
    }
};

}  // namespace omni

#endif  // OMNI_FORCE_TORQUE_HPP
