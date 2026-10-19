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
 * WP25-A4 multi-spacecraft (sync path): ONE integrator instance serves every
 * entry of dynPtrs (stock appends followers via
 * DynamicObject::syncDynamicsIntegration; the leader's integrateState then
 * runs preIntegration[all] -> integrate -> postIntegration[all] while each
 * follower's integrateState returns early -- upstream dynamicObject.cpp,
 * v2.12.0). Each object owns a per-object config slot (ObjCfg) and a
 * per-object runtime (Impl): per-object census/claim/gather/scatter with
 * name-derived slots resolved per object. Config-dirty is PER OBJECT: an
 * indexed setter rebuilds only its own slot, and a slot that was never
 * mirrored is a LOUD build error (object B never silently inherits object
 * A's graph). The symbolic graph is SHARED (one heyoka build, stepped
 * sequentially per object) iff every slot's config fingerprint is identical;
 * otherwise each object builds its own graph. tol/compactMode stay global
 * (JIT options, not graph structure). isDynamicsSynced is honored as: every
 * dynPtr is stepped exactly once per integrate() (the leader's call covers
 * synced followers -- stock never calls us twice per tick); a primary that
 * is itself synced, or more than one unsynced object sharing this
 * integrator, are loud build errors (both would silently double-step or
 * stall). Per-object step counters back the no-double-step audit.
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
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdForceInertialMsgPayload.h"
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
    // Legacy single-object surface: slot-0 shorthand. Each delegates to its
    // ...ForObject(0, ...) twin (identical semantics by construction); the
    // single-object stepping path reads slot 0 through the same per-object
    // helpers as the sync path.
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
    void setThrusterLag(bool lag, double cutoff);
    // --- ExtForceTorque (external force/torque, DynamicEffector channel) -------
    void setExtForceTorque(double fx, double fy, double fz,
                           double tx, double ty, double tz);
    void setExtForceInertial(double fx, double fy, double fz);
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
    // Integrator options (GLOBAL: JIT/stepping options, not graph structure;
    // shared-graph objects always step under one tol).
    void setTol(double tol);
    void setCompactMode(bool compact);

    // --- WP25-A4 per-object mirror (sync path) --------------------------------
    // Indexed twins of every setter above: slot `obj` is created on first
    // use (auto-grow) and that slot alone is marked mirrored+dirty, so
    // object B can never silently inherit object A's graph, and a setter
    // rebuilds only its own object. buildOnce() requires slots == dynPtrs
    // AND every slot mirrored, loudly otherwise.
    void setMassForObject(size_t obj, double m);
    void setInertiaForObject(size_t obj, double Ixx, double Iyy, double Izz,
                             double Ixy = 0.0, double Ixz = 0.0,
                             double Iyz = 0.0);
    void setCoMOffsetForObject(size_t obj, double x, double y, double z);
    void setMuForObject(size_t obj, double mu);
    void setUseGravityGradientForObject(size_t obj, bool useGG);
    void addEnvironmentBodyForObject(size_t obj, double mu,
                                     double ox, double oy, double oz);
    void setJ2ForObject(size_t obj, double J2);
    void setDragParamsForObject(size_t obj, double CdA,
                                double cx, double cy, double cz);
    void addSRPFacetForObject(size_t obj, double area,
                              double nx, double ny, double nz,
                              double copx, double copy, double copz,
                              double c_diffuse, double c_specular);
    void setSRPCannonballForObject(size_t obj, double area, double cr);
    void setSunStateForObject(size_t obj, double px, double py, double pz);
    void setMagneticMomentForObject(size_t obj, double mx, double my,
                                    double mz);
    void setMagneticDipoleForObject(size_t obj, double g10, double g11,
                                    double h11);
    void setMaxMtbDipoleForObject(size_t obj, double dmax);
    void setThrusterLagForObject(size_t obj, bool lag, double cutoff);
    void setExtForceTorqueForObject(size_t obj, double fx, double fy, double fz,
                                    double tx, double ty, double tz);
    void setExtForceInertialForObject(size_t obj, double fx, double fy, double fz);
    void setFuelStateNameForObject(size_t obj, const std::string& name);
    void setFuelTankForObject(size_t obj, double rTBx, double rTBy,
                              double rTBz, double leak);
    void setThrusterIspsForObject(size_t obj, const std::vector<double>& isp);
    void addThrusterForObject(size_t obj, double dx, double dy, double dz,
                              double px, double py, double pz, double tmax);
    void setInitialThrusterCmdForObject(size_t obj,
                                        const std::vector<double>& onTimes);
    void addWheelForObject(size_t obj, double gsx, double gsy, double gsz,
                           double Js);
    void setFrictionEnabledForObject(size_t obj, bool on);
    void setFrictionParamsForObject(size_t obj, size_t i, double fCoulomb,
                                    double fStatic, double betaStatic,
                                    double cViscous, double omegaLimitCycle);
    // Per-object message links (reads only: never mark dirty, never mark
    // mirrored -- a forgotten config mirror must still fail loudly at
    // build even if its messages were linked).
    void linkWheelLogForObject(size_t obj, size_t i,
                               Message<RWConfigLogMsgPayload>* src);
    void subscribeThrCmdForObject(size_t obj,
                                  Message<THRArrayOnTimeCmdMsgPayload>* src);
    void subscribeAtmoForObject(size_t obj, Message<AtmoPropsMsgPayload>* src);
    void subscribeWindForObject(size_t obj, Message<WindMsgPayload>* src);
    void subscribeEclipseForObject(size_t obj,
                                   Message<EclipseMsgPayload>* src);
    void subscribeSunForObject(size_t obj,
                               Message<SpicePlanetStateMsgPayload>* src);
    void subscribeMoonForObject(size_t obj,
                                Message<SpicePlanetStateMsgPayload>* src);
    void subscribeMagForObject(size_t obj,
                               Message<MagneticFieldMsgPayload>* src);
    void subscribeExtTorqueForObject(size_t obj,
                                     Message<CmdTorqueBodyMsgPayload>* src);
    void subscribeExtForceBodyForObject(size_t obj,
                                        Message<CmdForceBodyMsgPayload>* src);
    void subscribeExtForceInertialForObject(size_t obj,
                                            Message<CmdForceInertialMsgPayload>* src);

    // --- wheel-log inputs: u_current, timing-identical by construction ----
    // Link i-th wheel to the stock effector's i-th rwOutMsgs element:
    // linkWheelLog(i, rw.rwOutMsgs[i]). The log is published by the stock
    // RW UpdateState from the same u_current the stock integrator consumes,
    // so task order cannot skew us (a direct motor-message read could lead
    // or lag stock by a tick depending on task order -- rejected design).
    // Unlinked slots read 0, exactly like a never-configured stock wheel.
    // (Slot-0 storage; slots >= 1 live in extraMsgs_ via the ForObject
    // linkers above. Untouched by the A4 refactor.)
    std::vector<ReadFunctor<RWConfigLogMsgPayload>> rwLogInMsgs;
    void linkWheelLog(size_t i, Message<RWConfigLogMsgPayload>* src);
    // Thruster channel (stage 2): on-time command mailbox. Geometry arrives
    // via addThruster records (joined Python-side from factory + effector);
    // the config message is deliberately NOT consumed (its functor
    // subscription does not link in this build -- documented dead end).
    // (Slot-0 storage; see above.)
    ReadFunctor<THRArrayOnTimeCmdMsgPayload> thrOnTimeInMsg;
    // --- WP5 environment message hijack (field inputs, stage 3) --------------
    // (Slot-0 storage; see above.)
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;
    ReadFunctor<WindMsgPayload> windVelInMsg;
    ReadFunctor<EclipseMsgPayload> eclipseInMsg;
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;
    ReadFunctor<SpicePlanetStateMsgPayload> moonStateInMsg;
    ReadFunctor<MagneticFieldMsgPayload> magFieldInMsg;
    // --- ExtForceTorque message hijack (external force/torque channel) --------
    ReadFunctor<CmdTorqueBodyMsgPayload> extTorqueInMsg;   //!< torqueRequestBody [Nm, body]
    ReadFunctor<CmdForceBodyMsgPayload> extForceBodyInMsg;  //!< forceRequestBody [N, body]
    ReadFunctor<CmdForceInertialMsgPayload> extForceInertialInMsg;  //!< forceRequestInertial [N, inertial]

    // --- introspection --------------------------------------------------------
    size_t nStates() const;
    size_t nPars() const;
    size_t nWheels() const;
    double getBuildSeconds() const;
    // WP25-B2.5 vec MRP switch count (Phase-3 FSW dependency). Stock owns the
    // shadow-set switch (postIntegration flips the scattered B-point state);
    // the integrator only COUNTS crossings per tick (|sigma|^2 > 1
    // post-propagation, same threshold as the SysModel path). Cumulative
    // since construction (rebuilds do not reset it). Polled per tick from
    // Python (swap helper/test builds the tick trace); leg-total gate vs
    // stock is equality +-1 (tick-exact equality NOT required). The SysModel
    // mrpSwitches member is untouched (reference implementation).
    uint64_t getMRPSwitchCount() const { return mrpSwitchCount_; }
    // Claimed registered-state names, stock order (census proof for tests).
    std::vector<std::string> claimedNames() const;
    // --- WP25-A4 multi introspection ------------------------------------------
    // Config slots (slot 0 pre-exists; ForObject setters auto-grow).
    size_t nObjects() const;
    // Cumulative propagate count for one object (survives rebuilds). The
    // no-double-step audit asserts steps[i] == ticks for every object.
    uint64_t getStepCountForObject(size_t obj) const;
    // integrate() invocations since construction. Healthy sync runs satisfy
    // calls == ticks (stock calls the leader once per tick) and
    // steps[i] == calls (each object stepped exactly once per call).
    uint64_t getIntegrateCallCount() const { return integrateCalls_; }
    uint64_t getMRPSwitchCountForObject(size_t obj) const;
    double getBuildSecondsForObject(size_t obj) const;
    // True when every slot's config fingerprint matched and one heyoka
    // build serves all objects (probe-first on build cost: per-object
    // build seconds are recorded either way; JIT tiers still bind).
    bool sharedGraphUsed() const { return sharedGraph_; }
    // Canonical config fingerprint (graph-affecting fields only: commands,
    // message links, tol, and the stock state NAME are excluded). Equal
    // fingerprints share one graph; any difference builds per-object
    // graphs. Never silent: an unmirrored slot fails the build loudly.
    std::string configFingerprintForObject(size_t obj) const;
    std::vector<std::string> claimedNamesForObject(size_t obj) const;

    mutable BSKLogger bskLogger;

private:
    struct Impl;
    // Per-object runtime (build artifacts only): sys + heyoka + live pars +
    // cached StateData sources + name-derived offsets. Replaced wholesale on
    // that object's rebuild; counters/friction-state/mailbox-edge live in
    // ObjCfg so rebuilds never reset them (batch-2 semantics preserved).
    std::vector<std::unique_ptr<Impl>> impls_;
    DynamicObject* dyn_;  // borrowed: owned by the Python sim

    struct WheelRec {
        double gs[3];
        double Js;
    };
    // Phase-2 friction records (parallel to wheels) + per-wheel state.
    struct FricRec {
        double fCoulomb = 0.0, fStatic = 0.0, betaStatic = -1.0;
        double cViscous = 0.0, omegaLimitCycle = 0.0001;
    };
    struct FricState {
        int stribeck = 0;
        double omegaBefore = 0.0;
    };
    struct SRPFacetSpec {
        double area = 0;
        double nHat[3]{1, 0, 0};
        double rCop[3]{0, 0, 0};
        double cd = 0, cs = 0;
    };
    // WP25-A4 per-object staged config + lifecycle. Slot 0 is the legacy
    // single-object record (legacy setters write here via the ForObject
    // twins); slots >= 1 serve synced followers. Message links are NOT
    // stored here: slot 0 uses the legacy public members, slots >= 1 use
    // extraMsgs_ (routed uniformly through msgsFor()).
    struct ObjCfg {
        // Staged records (replayed into a fresh OmniSystem at build).
        double mass = 750.0;
        double Ixx = 900.0, Iyy = 800.0, Izz = 600.0;
        double Ixy = 0.0, Ixz = 0.0, Iyz = 0.0;
        double rBcB[3]{0.0, 0.0, 0.0};
        double mu = 3.986004418e14;
        bool useGG = true;
        std::vector<WheelRec> wheels;
        bool frictionOn = false;
        std::vector<FricRec> fricRecs;
        // Runtime Stribeck state (survives rebuilds, as before the refactor).
        std::vector<FricState> fricState;
        // Thruster geometry records, stock order (stage 2).
        std::vector<std::array<double, 3>> thrDirs;
        std::vector<std::array<double, 3>> thrPos;
        std::vector<double> thrTmax;
        // Thruster lag (state-effector emulation): opt-in.
        bool thrLag = false;
        double thrLagCutoff = 10.0;
        // Staged on-time mailbox edge (survives rebuilds, as before).
        int64_t lastThrStamp = -1;
        // Pre-init command snapshot (write-once path).
        std::vector<double> preStaged;
        bool havePreStaged = false;
        // --- WP5 env config (staged until build; mirrors OmniSpacecraft) -----
        std::vector<double> envBodyMu;
        std::vector<double> envBodyOff0;
        double envJ2 = 0.0;
        bool j2On = false;
        bool dragOn = false;
        double envCdA = 0.0;
        double envComOff[3]{0.0, 0.0, 0.0};
        std::vector<SRPFacetSpec> srpFacets;
        bool cannonballOn = false;
        double cbArea = 0.0, cbCr = 0.0;
        double sunState[3]{0.0, 0.0, 0.0};
        bool hasSun = false;
        bool magOn = false;
        double magM[3]{0.0, 0.0, 0.0};
        double magG10 = 0.0, magG11 = 0.0, magH11 = 0.0;
        double maxMtbDipole = 1.0;
        bool envConfigured = false;
        // --- ExtForceTorque records -------------------------------------------
        bool hasExtft = false;
        double extForceB[3]{0.0, 0.0, 0.0};
        double extTorqueB[3]{0.0, 0.0, 0.0};
        double extForceN[3]{0.0, 0.0, 0.0};
        // --- Phase-2 fuel records ---------------------------------------------
        std::string fuelStateName;
        bool hasFuel = false;
        double fuelRTB[3]{0.0, 0.0, 0.0};
        double fuelLeak = 0.0;
        std::vector<double> fuelIsps;
        // --- WP25-A4 lifecycle -------------------------------------------------
        // mirrored: any set*/add* twin ran for this slot. Links/subscribes
        // never set it (a forgotten config mirror must fail loudly even if
        // its messages were linked). built/dirty drive per-object rebuilds.
        bool mirrored = false;
        bool built = false;
        bool dirty = true;
        double buildSeconds = 0.0;
        uint64_t stepCount = 0;
        uint64_t mrpCount = 0;
    };
    // Owned message links for slots >= 1 (slot 0 = legacy public members).
    struct ObjMsgSet {
        std::vector<ReadFunctor<RWConfigLogMsgPayload>> rwLogs;
        ReadFunctor<THRArrayOnTimeCmdMsgPayload> thrCmd;
        ReadFunctor<AtmoPropsMsgPayload> atmo;
        ReadFunctor<WindMsgPayload> wind;
        ReadFunctor<EclipseMsgPayload> eclipse;
        ReadFunctor<SpicePlanetStateMsgPayload> sun;
        ReadFunctor<SpicePlanetStateMsgPayload> moon;
        ReadFunctor<MagneticFieldMsgPayload> mag;
        ReadFunctor<CmdTorqueBodyMsgPayload> extTorque;
        ReadFunctor<CmdForceBodyMsgPayload> extForceBody;
        ReadFunctor<CmdForceInertialMsgPayload> extForceInertial;
    };
    // Uniform per-object message view (pointers; slot 0 aliases the legacy
    // public members so the stepping path is single-sourced).
    struct ObjMsgs {
        std::vector<ReadFunctor<RWConfigLogMsgPayload>>* rwLogs = nullptr;
        ReadFunctor<THRArrayOnTimeCmdMsgPayload>* thrCmd = nullptr;
        ReadFunctor<AtmoPropsMsgPayload>* atmo = nullptr;
        ReadFunctor<WindMsgPayload>* wind = nullptr;
        ReadFunctor<EclipseMsgPayload>* eclipse = nullptr;
        ReadFunctor<SpicePlanetStateMsgPayload>* sun = nullptr;
        ReadFunctor<SpicePlanetStateMsgPayload>* moon = nullptr;
        ReadFunctor<MagneticFieldMsgPayload>* mag = nullptr;
        ReadFunctor<CmdTorqueBodyMsgPayload>* extTorque = nullptr;
        ReadFunctor<CmdForceBodyMsgPayload>* extForceBody = nullptr;
        ReadFunctor<CmdForceInertialMsgPayload>* extForceInertial = nullptr;
    };
    std::vector<ObjCfg> objCfgs_{1};  // slot 0 pre-exists (legacy surface)
    std::vector<ObjMsgSet> extraMsgs_;  // slot i>=1 <-> extraMsgs_[i-1]
    double tol_ = 1e-12;
    bool compactMode_ = false;

    bool sharedGraph_ = false;
    uint64_t integrateCalls_ = 0;
    double buildSeconds_ = 0.0;  // slot-0 build time (legacy accessor compat)
    // WP25-B2.5: cumulative shadow-set crossing count (see getter above).
    uint64_t mrpSwitchCount_ = 0;

    // Slot lifecycle: auto-grow (never shrinks; a stale extra slot fails the
    // build loudly via the slots==dynPtrs check, never silently idles).
    void ensureSlot(size_t obj);
    ObjCfg& cfgFor(size_t obj);
    const ObjCfg& cfgAt(size_t obj) const;
    ObjMsgs msgsFor(size_t obj);
    // Canonical graph-affecting fingerprint for the shared-graph rule.
    std::string calcFingerprint(size_t obj) const;
    bool needsBuild() const;
    // Per-object (index-keyed) census: ExtendedStateVector keys are
    // (dynIndex, name) upstream, so same-named states on two objects never
    // collide here (the old index-blind census would have merged them).
    std::vector<std::vector<std::pair<std::string, size_t>>> takeCensus() const;
    void checkSyncGuards();
    void buildOnce();  // guards + shared-graph decision + per-slot builds
    void claimAndGrab(
        size_t obj, const std::vector<std::pair<std::string, size_t>>& census,
        size_t& nW, size_t& nF, size_t& nK);
    void buildObjectFull(
        size_t obj, const std::vector<std::pair<std::string, size_t>>& census);
    void buildObjectShared(
        size_t obj, const std::vector<std::pair<std::string, size_t>>& census);
    void freezeObject(
        size_t obj, const std::vector<std::pair<std::string, size_t>>& census);
    void stepObject(size_t obj, double timeStep);
    // WP5 per-tick (/per-arc) env refresh into the par slots. y = gathered
    // omni-order state; P = live par vector. Module-side doubles only (A3).
    void updateEnvInputs(size_t obj, const double* y, std::vector<double>& P,
                         const double cLive[3]);
    void updateExtInputs(size_t obj, std::vector<double>& P);
};

#endif  // OMNI_VEC_INTEGRATOR_HPP
