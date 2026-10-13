/* ISC License - see native_rw/LICENSE.
 *
 * OmniVecIntegrator: a stock-compatible StateVecIntegrator backed by heyoka.
 *
 * Drop-in recipe (stock sim script, RW + central-gravity envelope):
 *
 *     from omniheyoka.omni_heyoka import OmniVecIntegrator
 *     from omniheyoka.swap import mirror_stock_config
 *     oi = OmniVecIntegrator(sc)
 *     mirror_stock_config(sc, oi, motorMsg)  # hub/RW/gravity -> records
 *     sc.setIntegrator(oi)
 *
 * Division of labour (load-bearing, follows the SDK boundary):
 * - Python (swap.py) sees stock OBJECTS: hub record, wheel gs/Js/limits,
 *   central mu, and REJECTS anything unmirrorable (dynamic effectors,
 *   friction, jitter, multi-body). The C++ side cannot see these (no
 *   spacecraft.h in bsk_sdk; `states`/`dynEffectors` are Spacecraft members,
 *   not DynamicObject members).
 * - C++ sees stock STATES by name (DynParamManager, a DynamicObject member):
 *   census via ExtendedStateVector, airtight claim check (anything beyond
 *   hubPosition/Velocity/Sigma/Omega + reactionWheelOmegas is rejected WITH
 *   its registered name), gather -> propagate_for(dt) -> scatter.
 * - Per-tick RW motor torques come from the SAME ArrayMotorTorque message
 *   the stock effector reads, through an EXACT replica of stock
 *   ConfigureRWRequests saturation (u_max clamp, u_min deadband, P_max
 *   foldback, Omega_max direction-aware cut; upstream v2.12.0 source).
 *   Identical inputs by construction; no task-ordering dependence.
 *
 * Deliberately NOT owned here (stock postIntegration keeps them): MRP shadow
 * switching, DV accumulators, energy/momentum, telemetry. The integrator is
 * purely gather -> propagate -> scatter.
 *
 * SWIG note: `class Spacecraft;` is only forward-declared. It unifies with
 * the stock proxy BY CLASS NAME (cSysModel precedent), so
 * OmniVecIntegrator(sc) accepts a stock Spacecraft. The .cpp round-trips the
 * pointer through void* (valid under stock's single-inheritance layout:
 * Spacecraft : DynamicObject : SysModel) to reach DynamicObject members.
 */
#ifndef OMNI_VEC_INTEGRATOR_HPP
#define OMNI_VEC_INTEGRATOR_HPP

#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/msgPayloadDefC/MagneticFieldMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/WindMsgPayload.h"

#include <cstddef>
#include <cstdint>
#include <array>
#include <memory>
#include <string>
#include <vector>

class Spacecraft;  // forward declaration only (see note above)

class OmniVecIntegrator : public StateVecIntegrator {
public:
    explicit OmniVecIntegrator(Spacecraft* sc);
    ~OmniVecIntegrator() override;

    void integrate(double currentTime, double timeStep) override;

    // --- setup-time records (mirrored from stock BEFORE first integrate) ----
    void setMass(double m);
    void setInertia(double Ixx, double Iyy, double Izz,
                    double Ixy = 0.0, double Ixz = 0.0, double Iyz = 0.0);
    void setCoMOffset(double x, double y, double z);
    void setMu(double mu);
    void setUseGravityGradient(bool useGG);
    // --- WP5 environment (stage 3, opt-in explicit config) -------------------
    // Stock env effectors in tasks are REJECTED by the helper; env effects
    // are configured here explicitly (same validated module-side laws as
    // OmniSpacecraft) with message hijack for fields (density, wind,
    // eclipse, sun, magnetic). Any configured effect registers the single
    // 'e' cluster; per-tick refresh + per-arc hook mirror the module.
    void addEnvironmentBody(double mu, double ox, double oy, double oz);
    void setJ2(double J2);
    void setDragParams(double CdA, double cx, double cy, double cz);
    void addSRPFacet(double area, double nx, double ny, double nz,
                     double copx, double copy, double copz,
                     double c_diffuse, double c_specular);
    void setSRPCannonball(double area, double cr);
    // cr is the SPECULAR FRACTION r in [0,1] (F = (1+r)*P*A, same fraction
    // convention as facet cd/cs): stock's cannonball coefficientReflection
    // is the TOTAL multiplier C_R in [1,2], so pass cr = C_R - 1
    // (C_R = 1.5 -> 0.5). Values outside [0,1] are rejected.
    void setSunState(double px, double py, double pz);
    void setMagneticMoment(double mx, double my, double mz);
    void setMagneticDipole(double g10, double g11, double h11);
    void setMaxMtbDipole(double dmax);
    // --- Phase-2 swap: fuel depletion (single tank) --------------------------
    // fuelStateName is the STOCK registered state name (e.g. "fuelMass" or
    // "fuelTankMass2"); the graph's canonical state is always "fuelMass"
    // (last). rTB = tank center in B [m] (point-mass K about B); leak =
    // constant drain [kg/s]. Thruster Isps (one per mirrored thruster, [s])
    // feed the mdot law mdot = sum duty*Tmax/(Isp*g0) + leak.
    void setFuelStateName(const std::string& name);
    void setFuelTank(double rTBx, double rTBy, double rTBz, double leak);
    void setThrusterIsps(const std::vector<double>& isp);
    // Fuel mass/I feedback toggle (default ON). OFF = bookkeeping-only: the
    // fuel state still integrates (telemetry exact) but hub mass/I pars stay
    // at build values (WP6-style). Useful to isolate feedback effects.
    // Thruster cluster (stage 2, staircase only): one call per thruster,
    // stock order (must match the THRArrayConfigMsg order). Geometry is
    // structural; duties enter per-step via the staged on-time mailbox.
    void addThruster(double dx, double dy, double dz,
                     double px, double py, double pz, double tmax);
    // Pre-init on-time snapshot (write-once path): fired once on tick 0.
    // Snapshot the command message at MIRROR time (pre-init state); the
    // helper does this. Empty/zeros = fill path.
    void setInitialThrusterCmd(const std::vector<double>& onTimes);
    // One call per wheel, stock order (structural constants only; limits
    // live stock-side in ConfigureRWRequests).
    void addWheel(double gsx, double gsy, double gsz, double Js);
    // Phase-2 friction (Stribeck, module-side, per wheel i): mirrors the
    // live RWConfigPayload set (fCoulomb/fStatic/betaStatic/cViscous/
    // omegaLimitCycle). Enabled flag + coefficients; the integrator keeps
    // per-wheel frictionStribeck/omegaBefore state exactly like stock
    // (upstream reactionWheelStateEffector.cpp). Jitter stays rejected.
    void setFrictionEnabled(bool on);
    void setFrictionParams(size_t i, double fCoulomb, double fStatic,
                           double betaStatic, double cViscous,
                           double omegaLimitCycle);
    // Integrator options.
    void setTol(double tol);
    void setCompactMode(bool compact);

    // --- wheel-log inputs: u_current, timing-identical by construction ----
    // Link i-th wheel to the stock effector's i-th rwOutMsgs element:
    // linkWheelLog(i, rw.rwOutMsgs[i]). The log is published by the stock
    // RW UpdateState from the same u_current the stock integrator consumes,
    // so task order cannot skew us (a direct motor-message read could lead
    // or lag stock by a tick depending on task order -- rejected design).
    // Unlinked slots read 0, exactly like a never-configured stock wheel.
    std::vector<ReadFunctor<RWConfigLogMsgPayload>> rwLogInMsgs;
    void linkWheelLog(size_t i, Message<RWConfigLogMsgPayload>* src);
    // Thruster channel (stage 2): on-time command mailbox. Geometry arrives
    // via addThruster records (joined Python-side from factory + effector);
    // the config message is deliberately NOT consumed (its functor
    // subscription does not link in this build -- documented dead end).
    ReadFunctor<THRArrayOnTimeCmdMsgPayload> thrOnTimeInMsg;
    // --- WP5 environment message hijack (field inputs, stage 3) --------------
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;
    ReadFunctor<WindMsgPayload> windVelInMsg;
    ReadFunctor<EclipseMsgPayload> eclipseInMsg;
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;
    ReadFunctor<SpicePlanetStateMsgPayload> moonStateInMsg;
    ReadFunctor<MagneticFieldMsgPayload> magFieldInMsg;

    // --- introspection --------------------------------------------------------
    size_t nStates() const;
    size_t nPars() const;
    size_t nWheels() const;
    double getBuildSeconds() const;
    // Claimed registered-state names, stock order (census proof for tests).
    std::vector<std::string> claimedNames() const;

    mutable BSKLogger bskLogger;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
    DynamicObject* dyn_;  // borrowed: owned by the Python sim

    // Staged records (replayed into a fresh OmniSystem at build).
    double mass_ = 750.0;
    double Ixx_ = 900.0, Iyy_ = 800.0, Izz_ = 600.0;
    double Ixy_ = 0.0, Ixz_ = 0.0, Iyz_ = 0.0;
    double rBcB_[3]{0.0, 0.0, 0.0};
    double mu_ = 3.986004418e14;
    bool useGG_ = true;
    struct WheelRec {
        double gs[3];
        double Js;
    };
    std::vector<WheelRec> wheelRecs_;
    // Phase-2 friction records (parallel to wheelRecs_) + per-wheel state.
    bool frictionOn_ = false;
    struct FricRec {
        double fCoulomb = 0.0, fStatic = 0.0, betaStatic = -1.0;
        double cViscous = 0.0, omegaLimitCycle = 0.0001;
    };
    std::vector<FricRec> fricRecs_;
    struct FricState {
        int stribeck = 0;
        double omegaBefore = 0.0;
    };
    std::vector<FricState> fricState_;
    // Thruster geometry records, stock order (stage 2).
    std::vector<std::array<double, 3>> thrDirs_;
    std::vector<std::array<double, 3>> thrPos_;
    std::vector<double> thrTmax_;
    // Staged on-time mailbox (stock edge-triggered semantics). Advance rule:
    // fire the CURRENT message iff first-sight or timestamp strictly
    // advanced (frozen ticks idle); the tick-0 fill is covered by the
    // pre-staged snapshot below (Reset-equivalent staging).
    int64_t lastThrStamp_ = -1;
    // Pre-init command snapshot (write-once path): staged at build, fired
    // once on tick 0, then consumed. Empty = fresh/fill path (idle tick 0).
    std::vector<double> preStaged_;
    bool havePreStaged_ = false;
    // --- WP5 env config (staged until build; mirrors OmniSpacecraft) ---------
    std::vector<double> envBodyMu_;
    std::vector<double> envBodyOff0_;
    double envJ2_ = 0.0;
    bool j2On_ = false;
    bool dragOn_ = false;
    double envCdA_ = 0.0;
    double envComOff_[3]{0.0, 0.0, 0.0};
    struct SRPFacetSpec {
        double area = 0;
        double nHat[3]{1, 0, 0};
        double rCop[3]{0, 0, 0};
        double cd = 0, cs = 0;
    };
    std::vector<SRPFacetSpec> srpFacets_;
    bool cannonballOn_ = false;
    double cbArea_ = 0.0, cbCr_ = 0.0;
    double sunState_[3]{0.0, 0.0, 0.0};
    bool hasSun_ = false;
    bool magOn_ = false;
    double magM_[3]{0.0, 0.0, 0.0};
    double magG10_ = 0.0, magG11_ = 0.0, magH11_ = 0.0;
    double maxMtbDipole_ = 1.0;
    bool envConfigured_ = false;
    // --- Phase-2 fuel records -------------------------------------------------
    std::string fuelStateName_;
    bool hasFuel_ = false;
    double fuelRTB_[3]{0.0, 0.0, 0.0};
    double fuelLeak_ = 0.0;
    std::vector<double> fuelIsps_;
    double tol_ = 1e-12;
    bool compactMode_ = false;

    bool built_ = false;
    bool configDirty_ = true;
    double buildSeconds_ = 0.0;

    void buildOnce();  // census + claim + mirror + symbolic build
    // WP5 per-tick (/per-arc) env refresh into the par slots. y = gathered
    // omni-order state; P = live par vector. Module-side doubles only (A3).
    void updateEnvInputs(const double* y, std::vector<double>& P,
                         const double cLive[3]);
};

#endif  // OMNI_VEC_INTEGRATOR_HPP
