/* ISC License - see native_rw/LICENSE.
 *
 * HeyokaRWDynamics: native Basilisk SysModel replacing spacecraft +
 * reactionWheelStateEffector for the scenarioAttitudeFeedbackRW baseline.
 *
 * Coupled 15-state hub+RW dynamics [r(3), v(3), sigma(3), omega(3), W(3)]
 * propagated by a persistent Heyoka Taylor integrator owned by the module.
 * UpdateState runs fully in C++: read motor torques, saturate at +-uMax
 * (stock effector behavior), propagate one scheduler step, write SCStates
 * + RWSpeed messages. Zero Python per tick.
 *
 * Conventions (validated against bsk_rw_stock + heyoka_rw):
 * - wheel:  Js_i * dW_i = u_i  (W relative to hub, rad/s)
 * - hub:    I*dw = -w x (I*w + h) - u,  h_i = Js_i*(W_i + w_i)
 * - translation: point-mass Kepler (no thrust force in the RW demo)
 * - MRP shadow-set reset when |sigma|^2 > 1 (counted in MRPSwitchCount)
 * - first UpdateState publishes ICs (no propagation: no motor msg exists yet)
 * - body origin == CoM: r_BN_N/v_BN_N mirror r_CN_N/v_CN_N; omegaDot not
 *   published (zero); wheelThetas not integrated (no jitter model)
 */
#ifndef HEYOKA_RW_DYNAMICS_H
#define HEYOKA_RW_DYNAMICS_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include <cstdint>
#include <memory>

// NOTE: heyoka types are deliberately absent here (PIMPL below), so the
// SWIG wrapper TU never parses heyoka headers and no forward declaration
// can collide with heyoka's inline-namespace declarations.

/*! @brief native Heyoka-propagated hub+RW dynamics (drop-in for the RW demo) */
class HeyokaRWDynamics : public SysModel {
public:
    HeyokaRWDynamics();
    ~HeyokaRWDynamics() override;

    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    ReadFunctor<ArrayMotorTorqueMsgPayload> rwMotorCmdInMsg;  //!< motor torque cmds
    Message<SCStatesMsgPayload> scStateOutMsg;                 //!< hub state out
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;                  //!< wheel speeds out

    BSKLogger bskLogger;  //!< BSK logging

    // --- configuration setters (ICs) ---
    void setR_CN_NInit(double x, double y, double z);
    void setV_CN_NInit(double x, double y, double z);
    void setSigmaBNInit(double s1, double s2, double s3);
    void setOmegaBNInit(double x, double y, double z);
    void setWheelSpeedsInitRPM(double w1, double w2, double w3);
    // --- configuration setters (properties) ---
    void setMass(double m);
    void setInertia(double Ixx, double Iyy, double Izz,
                    double Ixy, double Ixz, double Iyz);
    void setRotorInertia(double J1, double J2, double J3);
    void setUMax(double uMax);
    void setTol(double tol);
    void setMu(double mu);
    // --- state getters (verification) ---
    double getSigmaBN(int i);
    double getOmegaBN(int i);
    double getWheelSpeed(int i);
    double getMu() const { return this->mu; }

private:
    double rNInit[3] = {};
    double vNInit[3] = {};
    double sigmaInit[3] = {};
    double omegaInit[3] = {};
    double wsInitRPM[3] = {};
    double mass = 750.0;
    double Ixx = 900.0, Iyy = 800.0, Izz = 600.0;
    double Ixy = 0.0, Ixz = 0.0, Iyz = 0.0;
    double J1 = 0.07957747154594767, J2 = 0.07957747154594767,
           J3 = 0.07957747154594767;
    double uMax = 0.2;
    double tol = 1e-12;
    double mu = 3.986004418e14;
    // runtime
    bool initDone = false;
    bool configDirty = true;  // property setters re-arm; rebuilt on next use
    uint64_t prevNanos = 0;
    uint64_t mrpSwitches = 0;
    double st[15] = {};
    double invI6[6] = {};  // packed inverse inertia, computed at build
    struct TaHolder;  // defined in the .cpp; owns the heyoka integrator
    std::unique_ptr<TaHolder> ta;
    void buildIntegrator();  // validate config, (re)build ta + invI6
    void writeMsgs(uint64_t CurrentSimNanos);  // publish st to both outputs
};

#endif
