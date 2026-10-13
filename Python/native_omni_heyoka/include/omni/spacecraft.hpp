/* ISC License - see native_rw/LICENSE.
 *
 * OmniSpacecraft: native Basilisk SysModel with runtime-COMPOSED dynamics.
 *
 * Unlike native_rw's HeyokaRWDynamics (whose 15-state EOM is hand-written and
 * baked at compile time), this module carries NO closed-form EOM.  It owns an
 * omni::OmniSystem -- a set of contributors (hub, reaction wheels, central
 * gravity) registered at configuration time -- and derives the equations of
 * motion SYMBOLICALLY at Reset() by carrying the back-substitution arithmetic
 * in heyoka::expression.  Adding a wheel, tilting its spin axis, or attaching a
 * future effector needs no change to this class.
 *
 * Per tick, entirely in C++: read the motor-torque message, saturate at +-uMax
 * (stock effector behaviour), push state + pars into the Taylor integrator,
 * propagate one scheduler step, apply the MRP shadow-set reset, write
 * SCStatesMsg + RWSpeedMsg.  Zero Python per tick.
 *
 * Envelope matches native_rw's (so parity is comparable) but WITHOUT the
 * body-axis-aligned restriction: wheel spin axes are arbitrary.
 */
#ifndef OMNI_SPACECRAFT_HPP
#define OMNI_SPACECRAFT_HPP

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/THRArrayCmdForceMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefC/FuelTankMsgPayload.h"
#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/EclipseMsgPayload.h"
#include "architecture/msgPayloadDefC/MagneticFieldMsgPayload.h"
#include "architecture/msgPayloadDefC/EarthRadiationMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/WindMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

// NOTE: heyoka and the omni core types are deliberately absent here (PIMPL
// below) so the SWIG wrapper TU never parses them -- same discipline as
// native_rw, and it keeps the wrapper compile fast.

/*! @brief native SysModel with Heyoka dynamics composed at runtime */
class OmniSpacecraft : public SysModel {
public:
    OmniSpacecraft();
    ~OmniSpacecraft() override;

    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    ReadFunctor<ArrayMotorTorqueMsgPayload> rwMotorCmdInMsg;
    ReadFunctor<THRArrayOnTimeCmdMsgPayload> thrOnTimeInMsg;  //!< thruster on-times
    ReadFunctor<THRArrayCmdForceMsgPayload> thrForceInMsg;  //!< WP3: force-path guard ONLY (never subscribed; bskError if linked)
    Message<SCStatesMsgPayload> scStateOutMsg;
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;
    // --- WP3: thruster telemetry ---
    // One publisher per thruster (stock thrusterOutMsgs parity, 9/9 fields).
    // Grown in addThruster (recorder-ready pre-Reset) with a lazy backstop in
    // writeMsgs; index i == thruster index. Steady-tick semantics: end-of-step
    // (post-propagation) values; tick0 (pre-fire fill) publishes zeros +
    // geometry echo.
    std::vector<Message<THROutputMsgPayload>> thrusterOutMsgs;
    // --- WP6: telemetry publishers (value-vector, not stock pointer-vector) ---
    // Stock holds `vector<Message<RWConfigLogMsgPayload>*>` (heap ptrs). Omni uses
    // values resized in Reset(): no new/delete, deterministic teardown.
    std::vector<Message<RWConfigLogMsgPayload>> rwOutMsgs;
    Message<RWConfigLogMsgPayload>* rwLogOutMsg(size_t i) { return i < rwOutMsgs.size() ? &rwOutMsgs[i] : nullptr; }
    // FuelTankMsg-shaped bookkeeping publisher (NON-PARITY label).
    Message<FuelTankMsgPayload> fuelTankOutMsg;
    // --- WP5 environment inputs (message-hijack boundary) ---------------------
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;  //!< neutralDensity; required when drag on
    ReadFunctor<WindMsgPayload> windVelInMsg;  //!< v_air_N full air velocity; optional (unlinked -> inertial fallback)
    ReadFunctor<EclipseMsgPayload> eclipseInMsg;  //!< shadowFactor (ONE field); optional (unlinked -> 1.0 full-sun ON)
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;  //!< full sun state; required when SRP on
    ReadFunctor<SpicePlanetStateMsgPayload> moonStateInMsg;  //!< optional body-1 per-tick refresh
    ReadFunctor<MagneticFieldMsgPayload> magFieldInMsg;  //!< magField_N inertial N; required when mag on
    ReadFunctor<EarthRadiationMsgPayload> erpInMsg;  //!< field-only (message equality); NEVER force

    mutable BSKLogger bskLogger;  // mutable so the const WP0 guard can raise

    // --- configuration: hub -------------------------------------------------
    void setMass(double m);
    void setInertia(double Ixx, double Iyy, double Izz,
                    double Ixy = 0.0, double Ixz = 0.0, double Iyz = 0.0);
    void setMu(double mu);
    void setUseGravityGradient(bool useGG);
    void setCoMOffset(double x, double y, double z);  // WP4: aggregate c_B record

    // --- configuration: contributors (the generalisation point) -------------
    // Arbitrary spin axis; normalised internally.  Call once per wheel, in the
    // order the wheel states should appear.  uMax is per wheel (stock
    // saturates each effector from its own factory config); default is the
    // HR16 demo value.  Wheels are RECORDED here and
    // registered into a freshly-built OmniSystem at Reset() -- because Reset()
    // replaces the whole Impl, the registrations must be replayable.
    void addWheel(double gsx, double gsy, double gsz, double Js,
                  double uMax = 0.2);
    // Stock-style rated constructor (WP1): derives Js = maxMomentum /
    // omegaMaxRad with omegaMaxRPM -> rad/s conversion. HR16 path:
    // addRatedWheel(gsx, gsy, gsz, 50.0, 6000.0, 0.2). `custom` (addWheel)
    // never inherits rated defaults: its cap stays OFF.
    void addRatedWheel(double gsx, double gsy, double gsz, double maxMomentum,
                       double omegaMaxRPM, double uMax);
    // Setter-only Omega_max (no config message carries it): omegaMaxRPM <= 0
    // selects OFF (stored as -1.0, stock custom convention).
    void setWheelOmegaMax(size_t i, double omegaMaxRPM);

    // Thruster cluster (DynamicEffector channel).  Structural geometry is
    // baked in, like native_rw's wheel axes; only the duty levels are
    // parameters.  Call once per thruster, or once per cluster via
    // setThrusters().  Thrusters add NO state.
    void addThruster(double dx, double dy, double dz,
                     double px, double py, double pz, double tmax);
    void setThrusters(const std::vector<double>& dirs,
                      const std::vector<double>& pos,
                      const std::vector<double>& tmax);
    // --- WP5 environment (per-effect flags only; deliberately NO master envOn_) --
    void addEnvironmentBody(double mu, double ox, double oy, double oz);
    void setJ2(double J2);  // enables j2On_ (WP0: default off = USE_J2=False parity)
    void setDragParams(double CdA, double cx, double cy, double cz);
    void addSRPFacet(double area, double nx, double ny, double nz,
                     double copx, double copy, double copz,
                     double c_diffuse, double c_specular);
    void setSRPCannonball(double area, double cr);  // cannonball preset (staged first)
    void setSunState(double px, double py, double pz);  // static sun (used when sun msg unlinked)
    void setMagneticMoment(double mx, double my, double mz);  // MTB dipole [Am^2], body
    void setMagneticDipole(double g10, double g11, double h11);  // centered-dipole preset (recorded)
    void setMaxMtbDipole(double dmax);  // identity-Gt clamp preset
    // --- WP3: thruster timing models (all default OFF) -------
    // Each setter flips its WP0 guard flag and installs the model; all are
    // runtime (tick-rate) state -- none rebuilds the integrator.
    void setMinPulseTime(bool enable, double minOnTime = 0.02);
    void setThrusterRamps(const std::vector<double>& onRamp,
                          const std::vector<double>& offRamp,
                          bool linearShape = true);
    void setCutoffFrequency(double cutoffFrequency);
    void setBlowdownCoeffs(const std::vector<double>& thrustCoeffs);
    void setBlowdownCoeffs(const std::vector<double>& thrustCoeffs,
                           const std::vector<double>& ispCoeffs);
    void setMaxSwirlTorque(double maxSwirlTorque);

    // --- configuration: integrator / saturation -----------------------------
    void setUMax(double uMax);
    void setTol(double tol);
    void setCompactMode(bool compact);
    // --- WP2: friction / jitter / voltage --------------------
    // Friction (Stribeck iter-1, module-side; live RWConfigPayload address:
    // fCoulomb/fStatic/betaStatic/cViscous/omegaLimitCycle).
    void setFriction(double fCoulomb, double fStatic, double betaStatic,
                     double cViscous, double omegaLimitCycle);
    void setFrictionEnabled(bool on);
    void setFrictionHubSign(double s);  // default +1 (F4 react-on-hub)
    // JitterSimple only (distributional parity, never bit-exact).
    void setJitterEnabled(bool on);
    void setJitterSeed(uint64_t seed);
    void setJitterSigma(double sigmaNm);
    void setRWModelJitterCoupled();     // ALWAYS bskError (no theta states)
    // Voltage: informational only; wiring is stock MotorVoltageInterface
    // motorTorqueOutMsg -> rwMotorCmdInMsg, zero omni change.
    void setVoltageEnabled(bool on);

    // --- configuration: ICs -------------------------------------------------
    void setR_CN_NInit(double x, double y, double z);
    void setV_CN_NInit(double x, double y, double z);
    void setSigmaBNInit(double s1, double s2, double s3);
    void setOmegaBNInit(double x, double y, double z);
    void setWheelSpeedsInitRPM(const std::vector<double>& rpm);
    void setFuelParams(double m0, double mMax, double isp);  // WP6 bookkeeping

    // --- introspection (verification) ---------------------------------------
    size_t nStates() const;
    size_t nPars() const;
    size_t nWheels() const;
    size_t nThrusters() const;
    double getSigmaBN(int i);
    double getOmegaBN(int i);
    double getWheelSpeed(int i);
    double getMu() const { return this->mu; }
    uint64_t getMRPSwitchCount() const { return this->mrpSwitches; }
    // Symbolic build cost, in seconds (0 before the first Reset).
    double getBuildSeconds() const { return this->buildSeconds; }

private:
    struct Impl;  // defined in the .cpp; owns omni::OmniSystem + the integrator
    std::unique_ptr<Impl> impl;

    // configuration (staged until Reset)
    std::vector<double> rNInit{0.0, 0.0, 0.0};
    std::vector<double> vNInit{0.0, 0.0, 0.0};
    std::vector<double> sigmaInit{0.0, 0.0, 0.0};
    std::vector<double> omegaInit{0.0, 0.0, 0.0};
    // recorded spin axes + rotor inertias, in registration order.  Replayed
    // into a fresh OmniSystem on every Reset().
    struct WheelSpec {
        double gs[3];
        double Js;
        // WP1 momentum-saturation cap [rad/s]: -1.0 = OFF (stock `custom`
        // default Omega_max=-1). Positive = direction-aware cut at |W| >= cap.
        double omegaMaxRad = -1.0;
    };
    std::vector<WheelSpec> wheels;
    std::vector<double> wsInitRPM;
    // Per-wheel motor-torque saturation [Nm], parallel to wheels (stock
    // saturates each effector from its own factory config; A3: the caller
    // clamps before writing the input slots).
    std::vector<double> wheelUMax;

    // Thruster geometry, recorded like the wheel specs so Reset() can replay
    // them into a fresh OmniSystem.
    std::vector<std::array<double, 3>> thrDirs;
    std::vector<std::array<double, 3>> thrPos;
    std::vector<double> thrTmax;
    // --- WP3: timing-model options (all default OFF = pre-WP3 envelope) ------
    // WP0 guard flags use EXACT stipulated names; WP0 owns the Reset() guard
    // call site. Setters above flip these; models live in the staging layer.
    // (Flags themselves are declared once in the WP0 block below.)
    double minOnTime_ = 0.02;  // [s] stock MinOnTime default
    std::vector<double> onRamp_;  // on-ramp knots [s] since valve opened
    std::vector<double> offRamp_;  // off-ramp knots [s] since valve closed
    bool rampLinear_ = true;
    double cutoffFreq_ = 10.0;  // [rad/s] stock default; accepted but INERT
    // without ramps (WP25-A2: recorded, never modeled -- 10 vs 1000 identical)
    std::vector<double> blowdownThrustCoeffs_;  // descending poly; no tank => 1.0 + warn
    std::vector<double> blowdownIspCoeffs_;
    double maxSwirlTorque_ = 0.0;  // [N-m] about the thrust axis
    // WP5 env config, replayed into a fresh OmniSystem at Reset(). Per-effect
    // flags ONLY (dragOn_/cannonballOn_/srpFacets/magOn_/j2On_); NO master envOn_.
    std::vector<double> envBodyMu;
    std::vector<double> envBodyOff0;  // 3 per body: static/initial (refreshed per tick from msgs when linked)
    double envJ2 = 0.0;
    bool dragOn_ = false;
    double envCdA = 0.0;
    std::array<double, 3> envComOff{0.0, 0.0, 0.0};
    double envDensityCorr = 0.0;
    struct SRPFacetSpec { double area = 0; double nHat[3]{1, 0, 0}; double rCop[3]{0, 0, 0}; double cd = 0, cs = 0; };
    std::vector<SRPFacetSpec> srpFacets;
    bool cannonballOn_ = false;
    double cbArea = 0.0, cbCr = 0.0;
    std::array<double, 3> sunState_{0.0, 0.0, 0.0};
    bool hasSun_ = false;
    bool magOn_ = false;
    std::array<double, 3> magM_{0.0, 0.0, 0.0};
    double magG10 = 0.0, magG11 = 0.0, magH11 = 0.0;
    double maxMtbDipole_ = 1.0;
    bool envConfigured = false;  // any env setter called -> registers 'e' cluster
    double mass = 750.0;
    double Ixx = 900.0, Iyy = 800.0, Izz = 600.0;
    double Ixy = 0.0, Ixz = 0.0, Iyz = 0.0;
    std::array<double, 3> rBcB_{0.0, 0.0, 0.0};  // WP4 aggregate c_B record.
    // MEASURED B/C convention (WP25-A1, stock-pinned): r_BN = r_CN - R^T c_B,
    // v_BN = v_CN - R^T (w x c_B), R = C_BN(sigma) (r_BN - r_CN = -c exactly).
    // Evidence: audit_bundle/results/swap_p1_parity.json:thr_offset (offset 0.1,
    // omni-vs-stock deltas r 2.24e-08; wrong sign would be ~0.2 m). The output
    // transform is module-side in writeMsgs (A3); c = 0 stays an exact no-op.
    double mu = 3.986004418e14;
    bool useGG = true;
    double uMax = 0.2;
    double tol = 1e-12;
    bool compactMode = false;
    // WP0 loud-reject flags -- EXACT member names; other WPs' setters flip
    // these when they observe the request. checkUnsupported() raises while
    // any is true (WP0 models none of them).
    bool frictionOn_ = false;   // WP2
    bool jitterOn_ = false;     // WP2
    bool voltageOn_ = false;    // WP2 (informational; reuse needs no code)
    bool minPulseOn_ = false;   // WP3b (modeled behind default-off setter)
    bool rampsOn_ = false;      // WP3b (modeled behind default-off setter)
    bool cutoffLagOn_ = false;  // WP25-A2: NEVER armed -- cutoff accepted,
    // never modeled (inert without ramps); re-entry only composed WITH ramps (M2)
    bool blowdownOn_ = false;   // WP3b/WP6 (1.0 + warn without tank)
    bool swirlOn_ = false;      // WP3b (modeled behind default-off setter)
    bool forceMsgOn_ = true;    // WP3 guard-only: force path has no model
    bool j2On_ = false;         // WP5 (off unless explicitly set)
    bool offsetOn_ = false;     // WP4 (modeled behind setCoMOffset)
    // WP2 state (all default OFF = tauF identically 0).
    double fCoulomb_ = 0.0, fStatic_ = 0.0, betaStatic_ = 0.0;
    double cViscous_ = 0.0, omegaLimitCycle_ = 1.0;
    double frictionHubSign_ = 1.0;
    uint64_t jitterSeed_ = 0;
    double jitterSigma_ = 0.0;

    // runtime
    bool initDone = false;
    bool configDirty = true;
    uint64_t prevNanos = 0;
    // Thruster command staging (stock mailbox semantics): the on-time
    // message is STAGED on first sight / timestamp advance and consumed by
    // the NEXT propagation (one-shot). This reproduces stock exactly:
    // fresh commands idle the fill step then fire every step; write-once
    // commands fire once then idle; same-timestamp rewrites never re-fire.
    // Timestamps: last staged message's (init -1 = never staged).
    std::vector<double> thrStaged;
    int64_t lastThrStamp = -1;
    uint64_t mrpSwitches = 0;
    double buildSeconds = 0.0;
    std::vector<double> st;  // state, mirrored for the getters
    // WP0/WP7 layout hygiene (A1/A2): hub/wheel state-block offsets resolved
    // BY NAME from OmniSystem::state_names() in buildIntegrator(); use sites
    // must use these, never numeric literals. Defaults = canonical hub block.
    size_t rOff_ = 0;
    size_t vOff_ = 3;
    size_t sigmaOff_ = 6;
    size_t omegaOff_ = 9;
    size_t wheelOff_ = 12;
    // --- WP6: SC gap-fill + fuel state (WP4 owns B/C lines + lastWDot source) ---
    double accumDV_CN_N_[3] = {0.0, 0.0, 0.0};
    double accumDV_BN_B_[3] = {0.0, 0.0, 0.0};
    double accumDVBdy_[3] = {0.0, 0.0, 0.0};
    double nonConsAccelB_[3] = {0.0, 0.0, 0.0};
    // Fuel bookkeeping (caller-side, NO mass feedback): set via setFuelParams().
    double fuelMass_ = 10.0, fuelMax_ = 10.0, fuelIsp_ = 250.0;
    static constexpr double WP6_G0 = 9.80665;

    void buildIntegrator();  // register contributors + symbolic build
    void writeMsgs(uint64_t CurrentSimNanos);
    // Stage latest on-time message iff new (first sight / ts advance).
    void stageThrusters();
    // WP0 loud-reject guard: bskErrors listing every active-but-unmodeled
    // feature. Called at the top of Reset() and UpdateState(). (Non-const:
    // ReadFunctor::isLinked() is not const-qualified.)
    void checkUnsupported();
    // WP3: ramp profile factor at event time t (module-side, doubles only).
    double rampFactor(double t, const std::vector<double>& knots, bool isOn) const;
    // WP5 per-tick (/per-arc) env refresh: body offsets (per-tick, never
    // frozen) + drag/SRP/mag wrenches from state + config + msgs.
    // Module-side doubles only (A3).
    void updateEnvInputs(const double* y);
};

#endif  // OMNI_SPACECRAFT_HPP
