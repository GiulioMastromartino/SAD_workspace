/* ISC License - see native_rw/LICENSE.
 *
 * HeyokaMCDynamics: native Basilisk SysModel replacing spacecraft for the
 * thruster (RCS-jet) pipeline, mirroring heyoka_dynamics.HeyokaSpacecraft.
 *
 * Coupled 12-state hub dynamics [r(3), v(3), sigma(3), omega(3)] with body
 * wrench [F(3), T(3)] as pars, propagated by a persistent Heyoka Taylor
 * integrator owned by the module. Per tick (fully in C++): read thruster
 * on-time commands, convert to throttle duty, apply the measured 3-level
 * effector staircase ({0,1/6,5/6,1}), form the wrench via the configured
 * effectiveness matrices, propagate one scheduler step with the PREVIOUS
 * tick's wrench (Basilisk mailbox/ZOH semantics, as in heyoka_dynamics),
 * write SCStatesMsg. Zero Python per tick.
 *
 * Conventions (validated against heyoka_dynamics + cxxdrive/campaign.cpp):
 * - translation: Kepler + body thrust rotated to ECI (F_N = R^T F_B),
 *   optional gravity-gradient torque with full inertia
 * - MRP shadow-set reset when |sigma|^2 > 1 (counted in MRPSwitchCount)
 * - first UpdateState publishes ICs (no on-time message exists yet)
 * - body origin == CoM: r_BN_N/v_BN_N mirror r_CN_N/v_CN_N; omegaDot not
 *   published (zero)
 */
#ifndef HEYOKA_MC_DYNAMICS_H
#define HEYOKA_MC_DYNAMICS_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

#include <cstdint>
#include <memory>
#include <vector>

// NOTE: heyoka types are deliberately absent here (PIMPL below), so the
// SWIG wrapper TU never parses heyoka headers.

/*! @brief native Heyoka-propagated hub dynamics for the thruster pipeline */
class HeyokaMCDynamics : public SysModel {
public:
    HeyokaMCDynamics();
    ~HeyokaMCDynamics() override;

    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    ReadFunctor<THRArrayOnTimeCmdMsgPayload> thrOnTimeInMsg;  //!< thruster on-times
    Message<SCStatesMsgPayload> scStateOutMsg;                //!< hub state out

    BSKLogger bskLogger;  //!< BSK logging

    // --- configuration setters (ICs) ---
    void setR_CN_NInit(double x, double y, double z);
    void setV_CN_NInit(double x, double y, double z);
    void setSigmaBNInit(double s1, double s2, double s3);
    void setOmegaBNInit(double x, double y, double z);
    // --- configuration setters (properties) ---
    void setMass(double m);
    void setInertia(double Ixx, double Iyy, double Izz,
                    double Ixy, double Ixz, double Iyz);
    // wrench effectiveness, row-major (3, NTHR): F/T per unit duty
    void setForceMatrix(const std::vector<double> &MF);
    void setTorqueMatrix(const std::vector<double> &MT);
    void setQuantize(bool quantize);
    void setUseGravityGradient(bool useGG);
    void setTol(double tol);
    void setMu(double mu);
    // --- state getters (verification) ---
    double getSigmaBN(int i);
    double getOmegaBN(int i);
    double getMu() const { return this->mu; }

    static constexpr int NTHR = 12;  // demo thruster count (fixed layout)

private:
    // config
    double rNInit[3] = {};
    double vNInit[3] = {};
    double sigmaInit[3] = {};
    double omegaInit[3] = {};
    double mass = 20.0;
    double Ixx = 0.0, Iyy = 0.0, Izz = 0.0;
    double Ixy = 0.0, Ixz = 0.0, Iyz = 0.0;
    std::vector<double> MF = std::vector<double>(3 * NTHR, 0.0);
    std::vector<double> MT = std::vector<double>(3 * NTHR, 0.0);
    bool quantize = true;
    bool useGG = true;
    double tol = 1e-12;
    double mu = 3.986004418e14;
    // runtime
    bool initDone = false;
    bool configDirty = true;  // property setters re-arm; rebuilt on next use
    uint64_t prevNanos = 0;
    uint64_t mrpSwitches = 0;
    double st[12] = {};
    double prevF[3] = {};
    double prevT[3] = {};
    double invI6[6] = {};  // packed inverse inertia, computed at build
    struct TaHolder;  // defined in the .cpp; owns the heyoka integrator
    std::unique_ptr<TaHolder> ta;
    void buildIntegrator();  // validate config, (re)build ta + invI6
    void writeMsgs(uint64_t CurrentSimNanos);  // publish st to outputs
};

#endif
