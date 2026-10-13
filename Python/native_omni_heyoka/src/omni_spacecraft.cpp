// ISC License - see native_rw/LICENSE.  See omni/spacecraft.hpp for the contract.
#include "omni/spacecraft.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>

#include "architecture/msgPayloadDefC/THRArrayOnTimeCmdMsgPayload.h"

#include "omni/system.hpp"
#include "omni/environment.hpp"

// ---------------------------------------------------------------------------
// PIMPL: keeps omni::OmniSystem and the heyoka integrator out of the header
// (and therefore out of the SWIG wrapper translation unit).
// ---------------------------------------------------------------------------
struct OmniSpacecraft::Impl {
    omni::OmniSystem sys;
    // optional: taylor_adaptive is not default-constructible, and the symbolic
    // system does not exist until the contributors are registered.
    std::optional<heyoka::taylor_adaptive<double>> ta;
    std::vector<double> state;
    std::vector<double> pars;
    // RW motor-torque input slots in wheel order (layout is dynamic, A2).
    std::vector<size_t> wheelIn;
    // Truncated par count owned by ta (suffix-drop; push bound, heap safety).
    size_t ta_npars = 0;
    // WP2: per-wheel friction-torque (tauF) second slots, same order.
    std::vector<size_t> fricIn;
    // WP2 jitter RNG: one stream per wheel (seed ^ splitmix64(index+1)).
    std::vector<std::mt19937_64> jitterRng_;
    uint64_t jitterInitSeed_ = 0;
    bool jitterRngInit_ = false;
    // --- WP3: per-thruster timing states (staging layer) --------------------
    struct MinPulseState {
        int64_t lastFireStep = -1;
        double carryover = 0.0;
        long fireCounter = 0;
    };
    std::vector<MinPulseState> minPulseState_;
    std::vector<double> lagState_;
    std::vector<double> rampClock_;
    std::vector<double> lastFilt_;
    std::vector<double> lastQ_;
    int64_t thrStepCount_ = 0;
    bool swirlOffCached_ = false;
    size_t swirlOff_ = 0;
    bool blowdownWarned_ = false;
    // WP4: published omegaDot_BN_B source (steady-tick numeric value).
    double lastWDot[3]{0.0, 0.0, 0.0};
    // WP5 env slot bases + body count (valid after buildIntegrator).
    size_t envBody = 0, envDrag = 0, envSrp = 0, envMag = 0, envNBodies = 0;
    bool hasEnv = false;

    Impl() = default;
};

OmniSpacecraft::OmniSpacecraft() = default;
OmniSpacecraft::~OmniSpacecraft() = default;

// ---------------------------------------------------------------------------
// Configuration setters mark the build dirty; nothing is compiled until Reset.
// ---------------------------------------------------------------------------
void OmniSpacecraft::setMass(double m) {
    if (!(m > 0.0) || !std::isfinite(m))
        this->bskLogger.bskError("OmniSpacecraft: mass must be finite positive.");
    this->mass = m;
    this->configDirty = true;
}

void OmniSpacecraft::setInertia(double a, double b, double c,
                                double d, double e, double f) {
    this->Ixx = a; this->Iyy = b; this->Izz = c;
    this->Ixy = d; this->Ixz = e; this->Iyz = f;
    this->configDirty = true;
}

void OmniSpacecraft::setMu(double mu) {
    if (!(mu > 0.0) || !std::isfinite(mu))
        this->bskLogger.bskError("OmniSpacecraft: mu must be finite positive.");
    this->mu = mu;
    this->configDirty = true;
}

void OmniSpacecraft::setUseGravityGradient(bool v) { this->useGG = v; this->configDirty = true; }

void OmniSpacecraft::setCoMOffset(double x, double y, double z) {
    this->rBcB_ = {x, y, z};
    this->offsetOn_ = (x != 0.0 || y != 0.0 || z != 0.0);
    this->configDirty = true;
}

void OmniSpacecraft::addWheel(double gsx, double gsy, double gsz, double Js,
                               double uMax) {
    const double n = std::sqrt(gsx * gsx + gsy * gsy + gsz * gsz);
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniSpacecraft: wheel spin axis must be nonzero.");
    if (!(Js > 0.0) || !std::isfinite(Js))
        this->bskLogger.bskError("OmniSpacecraft: Js must be finite positive.");
    if (!(uMax >= 0.0) || !std::isfinite(uMax))
        this->bskLogger.bskError("OmniSpacecraft: wheel uMax must be finite non-negative.");
    WheelSpec w{};
    w.gs[0] = gsx / n;
    w.gs[1] = gsy / n;
    w.gs[2] = gsz / n;
    w.Js = Js;
    w.omegaMaxRad = -1.0;  // WP1: custom path, saturation OFF (stock custom
                           // default Omega_max=-1; preserves pre-WP1 behavior).
    this->wheels.push_back(w);
    this->wheelUMax.push_back(uMax);
    this->rwOutMsgs.emplace_back();  // WP6: recorder-ready pre-Reset (matches thrusterOutMsgs)
    this->wsInitRPM.resize(this->wheels.size(), 0.0);
    this->configDirty = true;
}

void OmniSpacecraft::addRatedWheel(double gsx, double gsy, double gsz,
                                   double maxMomentum, double omegaMaxRPM,
                                   double uMax) {
    // Stock-style rated wheel (rwFactory parity): Js is DERIVED, never set
    // directly (stock hard-errors when both Js and the rated pair are given).
    // HR16 demo path: maxMomentum=50.0 Nms, omegaMaxRPM=6000.0 RPM, uMax=0.2.
    // Units (triple-unit trap): omegaMaxRPM kwarg in RPM, stored omegaMaxRad
    // in rad/s (matches RWConfigPayload.Omega_max units).
    constexpr double RPM_TO_RADS = 2.0 * 3.141592653589793 / 60.0;
    if (!(maxMomentum > 0.0) || !std::isfinite(maxMomentum))
        this->bskLogger.bskError("OmniSpacecraft: rated maxMomentum must be finite positive.");
    if (!(omegaMaxRPM > 0.0) || !std::isfinite(omegaMaxRPM))
        this->bskLogger.bskError("OmniSpacecraft: rated omegaMaxRPM must be finite positive.");
    if (!(uMax >= 0.0) || !std::isfinite(uMax))
        this->bskLogger.bskError("OmniSpacecraft: wheel uMax must be finite non-negative.");
    const double n = std::sqrt(gsx * gsx + gsy * gsy + gsz * gsz);
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniSpacecraft: wheel spin axis must be nonzero.");
    WheelSpec w{};
    w.gs[0] = gsx / n;
    w.gs[1] = gsy / n;
    w.gs[2] = gsz / n;
    w.omegaMaxRad = omegaMaxRPM * RPM_TO_RADS;
    w.Js = maxMomentum / w.omegaMaxRad;
    this->wheels.push_back(w);
    this->wheelUMax.push_back(uMax);
    this->rwOutMsgs.emplace_back();  // WP6: recorder-ready pre-Reset (matches thrusterOutMsgs)
    this->wsInitRPM.resize(this->wheels.size(), 0.0);
    this->configDirty = true;
}

void OmniSpacecraft::setWheelOmegaMax(size_t i, double omegaMaxRPM) {
    // Setter-only Omega_max (WP1): RWArrayConfigMsg carries uMax/JsList/
    // GsMatrix_B only, and RWConfigLog.Omega_max is output-only/circular
    // once stock is replaced. omegaMaxRPM <= 0 selects OFF (stored -1.0).
    if (i >= this->wheels.size())
        this->bskLogger.bskError("OmniSpacecraft: setWheelOmegaMax index out of range.");
    if (omegaMaxRPM <= 0.0) {
        this->wheels[i].omegaMaxRad = -1.0;
    } else {
        if (!std::isfinite(omegaMaxRPM))
            this->bskLogger.bskError("OmniSpacecraft: omegaMaxRPM must be finite or <= 0 (OFF).");
        constexpr double RPM_TO_RADS = 2.0 * 3.141592653589793 / 60.0;
        this->wheels[i].omegaMaxRad = omegaMaxRPM * RPM_TO_RADS;
    }
    this->configDirty = true;
}

void OmniSpacecraft::addThruster(double dx, double dy, double dz,
                                 double px, double py, double pz, double tmax) {
    const double n = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniSpacecraft: thruster direction must be nonzero.");
    if (!(tmax > 0.0) || !std::isfinite(tmax))
        this->bskLogger.bskError("OmniSpacecraft: thruster Tmax must be positive.");
    const double s = 1.0 / n;
    this->thrDirs.push_back({dx * s, dy * s, dz * s});
    this->thrPos.push_back({px, py, pz});
    this->thrTmax.push_back(tmax);
    this->thrusterOutMsgs.emplace_back();  // WP3: publisher per thruster (recorder-ready pre-Reset)
    this->configDirty = true;
}

void OmniSpacecraft::setThrusters(const std::vector<double>& dirs,
                                  const std::vector<double>& pos,
                                  const std::vector<double>& tmax) {
    if (dirs.size() != pos.size() || dirs.size() % 3 != 0 || tmax.size() * 3 != dirs.size())
        this->bskLogger.bskError(
            "OmniSpacecraft: setThrusters needs dirs/pos of length 3N and tmax of length N.");
    this->thrDirs.clear();
    this->thrPos.clear();
    this->thrTmax.clear();
    this->thrusterOutMsgs.clear();  // WP3: rebuilt by the addThruster loop below
    for (size_t i = 0; i < tmax.size(); ++i) {
        this->addThruster(dirs[3 * i], dirs[3 * i + 1], dirs[3 * i + 2],
                          pos[3 * i], pos[3 * i + 1], pos[3 * i + 2], tmax[i]);
    }
}

// --- WP3b timing-model setters (runtime config; never rebuilds) --------------
void OmniSpacecraft::setMinPulseTime(bool enable, double minOnTime) {
    if (enable && (!(minOnTime > 0.0) || !std::isfinite(minOnTime)))
        this->bskLogger.bskError("OmniSpacecraft: minOnTime must be finite positive.");
    this->minPulseOn_ = enable;
    if (enable) this->minOnTime_ = minOnTime;
}

void OmniSpacecraft::setThrusterRamps(const std::vector<double>& onRamp,
                                      const std::vector<double>& offRamp,
                                      bool linearShape) {
    auto check = [&](const std::vector<double>& v, const char* what) {
        for (double t : v) {
            if (!(t >= 0.0) || !std::isfinite(t))
                this->bskLogger.bskError("OmniSpacecraft: %s ramp knots must be finite non-negative.", what);
        }
        for (size_t i = 1; i < v.size(); ++i) {
            if (!(v[i] > v[i - 1]))
                this->bskLogger.bskError("OmniSpacecraft: %s ramp knots must be strictly ascending.", what);
        }
    };
    check(onRamp, "on");
    check(offRamp, "off");
    this->onRamp_ = onRamp;
    this->offRamp_ = offRamp;
    this->rampLinear_ = linearShape;
    this->rampsOn_ = !onRamp.empty() || !offRamp.empty();
}

void OmniSpacecraft::setCutoffFrequency(double cutoffFrequency) {
    if (!(cutoffFrequency > 0.0) || !std::isfinite(cutoffFrequency))
        this->bskLogger.bskError("OmniSpacecraft: cutoffFrequency must be finite positive.");
    this->cutoffFreq_ = cutoffFrequency;
    this->cutoffLagOn_ = true;
}

void OmniSpacecraft::setBlowdownCoeffs(const std::vector<double>& thrustCoeffs) {
    this->setBlowdownCoeffs(thrustCoeffs, std::vector<double>());
}

void OmniSpacecraft::setBlowdownCoeffs(const std::vector<double>& thrustCoeffs,
                                       const std::vector<double>& ispCoeffs) {
    for (double c : thrustCoeffs) {
        if (!std::isfinite(c))
            this->bskLogger.bskError("OmniSpacecraft: blowdown thrust coeffs must be finite.");
    }
    for (double c : ispCoeffs) {
        if (!std::isfinite(c))
            this->bskLogger.bskError("OmniSpacecraft: blowdown isp coeffs must be finite.");
    }
    this->blowdownThrustCoeffs_ = thrustCoeffs;
    this->blowdownIspCoeffs_ = ispCoeffs;
    this->blowdownOn_ = !thrustCoeffs.empty() || !ispCoeffs.empty();
}

void OmniSpacecraft::setMaxSwirlTorque(double maxSwirlTorque) {
    if (!(maxSwirlTorque >= 0.0) || !std::isfinite(maxSwirlTorque))
        this->bskLogger.bskError("OmniSpacecraft: maxSwirlTorque must be finite non-negative.");
    this->maxSwirlTorque_ = maxSwirlTorque;
    this->swirlOn_ = (maxSwirlTorque != 0.0);
}

// --- WP5 environment setters (runtime config; replayed at Reset) --------------
void OmniSpacecraft::addEnvironmentBody(double mu, double ox, double oy, double oz) {
    if (!(mu > 0.0) || !std::isfinite(mu))
        this->bskLogger.bskError("OmniSpacecraft: env body mu must be finite positive.");
    this->envBodyMu.push_back(mu);
    this->envBodyOff0.insert(this->envBodyOff0.end(), {ox, oy, oz});
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::setJ2(double J2) {
    if (!std::isfinite(J2) || J2 < 0.0)
        this->bskLogger.bskError("OmniSpacecraft: J2 must be finite non-negative.");
    this->envJ2 = J2;
    this->j2On_ = true;
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::setDragParams(double CdA, double cx, double cy, double cz) {
    if (!(CdA >= 0.0) || !std::isfinite(CdA))
        this->bskLogger.bskError("OmniSpacecraft: CdA must be finite non-negative.");
    this->envCdA = CdA;
    this->envComOff = {cx, cy, cz};
    this->dragOn_ = true;
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::addSRPFacet(double area, double nx, double ny, double nz,
                                 double copx, double copy, double copz,
                                 double c_diffuse, double c_specular) {
    const double n = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!(area > 0.0) || !std::isfinite(area))
        this->bskLogger.bskError("OmniSpacecraft: SRP facet area must be finite positive.");
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniSpacecraft: SRP facet normal must be nonzero.");
    SRPFacetSpec f;
    f.area = area;
    f.nHat[0] = nx / n; f.nHat[1] = ny / n; f.nHat[2] = nz / n;
    f.rCop[0] = copx; f.rCop[1] = copy; f.rCop[2] = copz;
    f.cd = c_diffuse; f.cs = c_specular;
    this->srpFacets.push_back(f);
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::setSRPCannonball(double area, double cr) {
    if (!(area > 0.0) || !std::isfinite(area))
        this->bskLogger.bskError("OmniSpacecraft: cannonball area must be finite positive.");
    if (!std::isfinite(cr) || cr < 0.0)
        this->bskLogger.bskError("OmniSpacecraft: cr must be finite non-negative.");
    this->cbArea = area; this->cbCr = cr;
    this->cannonballOn_ = true;
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::setSunState(double px, double py, double pz) {
    this->sunState_ = {px, py, pz};
    this->hasSun_ = true;
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::setMagneticMoment(double mx, double my, double mz) {
    this->magM_ = {mx, my, mz};
    this->magOn_ = true;
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::setMagneticDipole(double g10, double g11, double h11) {
    this->magG10 = g10; this->magG11 = g11; this->magH11 = h11;
    this->envConfigured = true;
    this->configDirty = true;
}

void OmniSpacecraft::setMaxMtbDipole(double dmax) {
    if (!(dmax > 0.0) || !std::isfinite(dmax))
        this->bskLogger.bskError("OmniSpacecraft: maxMtbDipole must be finite positive.");
    this->maxMtbDipole_ = dmax;
}

// --- WP2 friction / jitter / voltage setters (runtime; no rebuild) ------------
void OmniSpacecraft::setFriction(double fCoulomb, double fStatic, double betaStatic,
                                 double cViscous, double omegaLimitCycle) {
    // Live RWConfigPayload address (fCoulomb/fStatic/betaStatic/cViscous/
    // omegaLimitCycle). Stores params; setFrictionEnabled() activates.
    // Iter-1 model uses fCoulomb/cViscous/omegaLimitCycle; fStatic/betaStatic
    // shape iter-2 (documented hook, accepted-but-inert until then).
    for (double v : {fCoulomb, fStatic, betaStatic, cViscous, omegaLimitCycle}) {
        if (!std::isfinite(v) || v < 0.0)
            this->bskLogger.bskError("OmniSpacecraft: friction params must be finite non-negative.");
    }
    this->fCoulomb_ = fCoulomb; this->fStatic_ = fStatic;
    this->betaStatic_ = betaStatic; this->cViscous_ = cViscous;
    this->omegaLimitCycle_ = (omegaLimitCycle > 0.0) ? omegaLimitCycle : 1.0;
}

void OmniSpacecraft::setFrictionEnabled(bool on) { this->frictionOn_ = on; }

void OmniSpacecraft::setFrictionHubSign(double s) {
    if (s != 1.0 && s != -1.0)
        this->bskLogger.bskError("OmniSpacecraft: friction hub sign must be +1 or -1 (F4).");
    this->frictionHubSign_ = s;
}

void OmniSpacecraft::setJitterEnabled(bool on) { this->jitterOn_ = on; }

void OmniSpacecraft::setJitterSeed(uint64_t seed) {
    this->jitterSeed_ = seed;
    if (this->impl) this->impl->jitterRngInit_ = false;  // force re-split
}

void OmniSpacecraft::setJitterSigma(double sigmaNm) {
    if (!(sigmaNm >= 0.0) || !std::isfinite(sigmaNm))
        this->bskLogger.bskError("OmniSpacecraft: jitter sigma must be finite non-negative.");
    this->jitterSigma_ = sigmaNm;
}

void OmniSpacecraft::setRWModelJitterCoupled() {
    // ALWAYS fatal: JitterFullyCoupled needs theta states + aOmega/bOmega/
    // cOmega back-substitution terms omni does not register.
    this->bskLogger.bskError("OmniSpacecraft: JitterFullyCoupled is not modeled (no theta states); use JitterSimple or BalancedWheels.");
}

void OmniSpacecraft::setVoltageEnabled(bool on) {
    // Informational only: wiring is stock MotorVoltageInterface
    // motorTorqueOutMsg -> rwMotorCmdInMsg; zero omni change.
    this->voltageOn_ = on;
}

void OmniSpacecraft::setFuelParams(double m0, double mMax, double isp) {
    // WP6 caller-side bookkeeping (NON-PARITY): no mass feedback into dynamics.
    if (!(m0 >= 0.0) || !std::isfinite(m0))
        this->bskLogger.bskError("OmniSpacecraft: fuel m0 must be finite non-negative.");
    if (!(mMax > 0.0) || !std::isfinite(mMax))
        this->bskLogger.bskError("OmniSpacecraft: fuel mMax must be finite positive.");
    if (!(isp > 0.0) || !std::isfinite(isp))
        this->bskLogger.bskError("OmniSpacecraft: fuel isp must be finite positive.");
    this->fuelMass_ = m0; this->fuelMax_ = mMax; this->fuelIsp_ = isp;
}

void OmniSpacecraft::setUMax(double uMax) {
    if (!(uMax >= 0.0) || !std::isfinite(uMax))
        this->bskLogger.bskError("OmniSpacecraft: uMax must be finite non-negative.");
    this->uMax = uMax;
}

void OmniSpacecraft::setTol(double t) {
    if (!(t > 0.0) || !std::isfinite(t))
        this->bskLogger.bskError("OmniSpacecraft: tol must be finite positive.");
    this->tol = t;
    this->configDirty = true;
}

void OmniSpacecraft::setCompactMode(bool v) { this->compactMode = v; this->configDirty = true; }

void OmniSpacecraft::setR_CN_NInit(double x, double y, double z) {
    this->rNInit = {x, y, z};
}
void OmniSpacecraft::setV_CN_NInit(double x, double y, double z) {
    this->vNInit = {x, y, z};
}
void OmniSpacecraft::setSigmaBNInit(double s1, double s2, double s3) {
    this->sigmaInit = {s1, s2, s3};
}
void OmniSpacecraft::setOmegaBNInit(double x, double y, double z) {
    this->omegaInit = {x, y, z};
}
void OmniSpacecraft::setWheelSpeedsInitRPM(const std::vector<double>& rpm) {
    this->wsInitRPM = rpm;
}

// ---------------------------------------------------------------------------
// Introspection
// ---------------------------------------------------------------------------
size_t OmniSpacecraft::nStates() const {
    return this->impl ? this->impl->sys.n_states() : 0;
}
size_t OmniSpacecraft::nPars() const {
    return this->impl ? this->impl->sys.n_pars() : 0;
}
size_t OmniSpacecraft::nWheels() const { return this->wheels.size(); }
size_t OmniSpacecraft::nThrusters() const { return this->thrDirs.size(); }

double OmniSpacecraft::getSigmaBN(int i) {
    if (i < 0 || i > 2) {
        this->bskLogger.bskError("OmniSpacecraft: getSigmaBN index out of range.");
        return NAN;
    }
    return this->st[this->sigmaOff_ + static_cast<size_t>(i)];
}
double OmniSpacecraft::getOmegaBN(int i) {
    if (i < 0 || i > 2) {
        this->bskLogger.bskError("OmniSpacecraft: getOmegaBN index out of range.");
        return NAN;
    }
    return this->st[this->omegaOff_ + static_cast<size_t>(i)];
}
double OmniSpacecraft::getWheelSpeed(int i) {
    if (i < 0 || int(this->wheels.size()) <= i) {
        this->bskLogger.bskError("OmniSpacecraft: getWheelSpeed index out of range.");
        return NAN;
    }
    return this->st[this->wheelOff_ + static_cast<size_t>(i)];
}

// ---------------------------------------------------------------------------
// Symbolic build: register contributors ONCE, derive the EOM, JIT the stepping.
// This is the whole point of the module -- the EOM is never written by hand.
// ---------------------------------------------------------------------------
void OmniSpacecraft::buildIntegrator() {
    const auto t0 = std::chrono::steady_clock::now();

    this->impl->sys.set_gravity(this->mu);
    this->impl->sys.set_use_gravity_gradient(this->useGG);
    this->impl->sys.set_u_max(this->uMax);

    this->impl->sys.add_hub(this->mass,
                            {this->Ixx, this->Ixy, this->Ixz,
                             this->Ixy, this->Iyy, this->Iyz,
                             this->Ixz, this->Iyz, this->Izz},
                            {this->rBcB_[0], this->rBcB_[1], this->rBcB_[2]});

    // Replay the recorded contributors into the freshly-built OmniSystem.
    // THIS is the generalisation mechanism: nothing here knows what a wheel is
    // beyond "a contributor with a spin axis and a Js".
    for (const auto& w : this->wheels) {
        this->impl->sys.add_reaction_wheel({w.gs[0], w.gs[1], w.gs[2]}, w.Js, 0.0);
    }
    if (!this->thrDirs.empty()) {
        std::vector<std::array<double, 3>> d = this->thrDirs;
        std::vector<std::array<double, 3>> p = this->thrPos;
        this->impl->sys.add_thrusters(std::move(d), std::move(p), this->thrTmax);
    }
    // WP5 env replay: bodies + J2 feed the graph (g_N channel); drag/SRP/mag
    // geometry lives module-side and enters via slots (A3). Any configured
    // effect registers the single 'e' cluster.
    if (this->envConfigured) {
        for (size_t i = 0; i < this->envBodyMu.size(); ++i) {
            this->impl->sys.addEnvironmentBody(this->envBodyMu[i],
                                               this->envBodyOff0[3 * i],
                                               this->envBodyOff0[3 * i + 1],
                                               this->envBodyOff0[3 * i + 2]);
        }
        if (this->j2On_) this->impl->sys.setJ2(this->envJ2);
        if (this->dragOn_)
            this->impl->sys.setDragParams(this->envCdA, this->envComOff[0],
                                          this->envComOff[1], this->envComOff[2]);
        for (const auto& f : this->srpFacets)
            this->impl->sys.addSRPFacet(f.area, f.nHat[0], f.nHat[1], f.nHat[2],
                                        f.rCop[0], f.rCop[1], f.rCop[2], f.cd, f.cs);
        if (this->cannonballOn_) this->impl->sys.setSRPCannonball(this->cbArea, this->cbCr);
        if (this->hasSun_)
            this->impl->sys.setSunState(this->sunState_[0], this->sunState_[1],
                                        this->sunState_[2]);
        if (this->magOn_)
            this->impl->sys.setMagneticMoment(this->magM_[0], this->magM_[1],
                                              this->magM_[2]);
        this->impl->sys.setMagneticDipole(this->magG10, this->magG11, this->magH11);
    }

    // Initial state: hub ICs + wheel speeds converted RPM -> rad/s.
    const size_t n = this->impl->sys.n_states();
    std::vector<double> y(n, 0.0);
    for (int i = 0; i < 3; ++i) {
        const size_t k = static_cast<size_t>(i);
        y[this->rOff_ + k] = this->rNInit[k];
        y[this->vOff_ + k] = this->vNInit[k];
        y[this->sigmaOff_ + k] = this->sigmaInit[k];
        y[this->omegaOff_ + k] = this->omegaInit[k];
    }
    const size_t nw = this->wheels.size();
    if (this->wsInitRPM.size() < nw) this->wsInitRPM.resize(nw, 0.0);
    for (size_t i = 0; i < nw; ++i) {
        y[this->wheelOff_ + i] = this->wsInitRPM[i] * 2.0 * 3.141592653589793 / 60.0;
    }

    const auto sysrows = this->impl->sys.build_symbolic();
    this->impl->state = y;
    this->impl->pars = this->impl->sys.default_pars();
    this->impl->wheelIn = this->impl->sys.wheel_input_offsets();
    this->impl->fricIn = this->impl->sys.friction_input_offsets();
    {
        const auto es = this->impl->sys.env_slot_offsets();
        this->impl->hasEnv = es.has;
        this->impl->envBody = es.body;
        this->impl->envDrag = es.drag;
        this->impl->envSrp = es.srp;
        this->impl->envMag = es.mag;
        this->impl->envNBodies = es.nBodies;
    }
    // WP0/WP7 layout hygiene (A1/A2): resolve state-block offsets by NAME
    // from the derived layout; keep the canonical default for any absent
    // name (e.g. W1 when no wheels are registered).
    {
        const std::vector<std::string> sn = this->impl->sys.state_names();
        const size_t none = static_cast<size_t>(-1);
        auto posOf = [&](const char* nm) -> size_t {
            for (size_t i = 0; i < sn.size(); ++i)
                if (sn[i] == nm) return i;
            return none;
        };
        size_t p = posOf("rx"); if (p != none) this->rOff_ = p;
        p = posOf("vx"); if (p != none) this->vOff_ = p;
        p = posOf("s1"); if (p != none) this->sigmaOff_ = p;
        p = posOf("omx"); if (p != none) this->omegaOff_ = p;
        p = posOf("W1"); if (p != none) this->wheelOff_ = p;
    }
    this->impl->ta_npars = omni::OmniSystem::checked_par_count(
        sysrows, this->impl->pars.size());
    std::vector<double> pTrunc(this->impl->pars.begin(),
                               this->impl->pars.begin() + this->impl->ta_npars);
    this->impl->ta.emplace(sysrows, y,
                           heyoka::kw::pars = pTrunc,
                           heyoka::kw::tol = this->tol,
                           heyoka::kw::compact_mode = this->compactMode,
                           heyoka::kw::high_accuracy = false);
    this->st = y;

    const auto t1 = std::chrono::steady_clock::now();
    this->buildSeconds =
        std::chrono::duration<double>(t1 - t0).count();
    this->configDirty = false;

    this->bskLogger.bskLog(BSK_INFORMATION,
                           "OmniSpacecraft: %zu states, %zu pars, %zu wheels; "
                           "symbolic build %.2f ms, tol %g.",
                           n, this->impl->pars.size(), nw,
                           this->buildSeconds * 1e3, this->tol);
}

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------
void OmniSpacecraft::writeMsgs(uint64_t CurrentSimNanos) {
    SCStatesMsgPayload sc = this->scStateOutMsg.zeroMsgPayload;
    // WP4 B/C split: integrator state is C-point (r_CN/v_CN); B-point derived.
    // c_B structural (rBcB_), R=C_BN(sigma). Zero offset => identity (regression).
    // Telemetry thrustTorquePntB_B (WP3/WP6) stays about-B UNSHIFTED elsewhere.
    double Rbn[3][3];
    {
        const double s0 = this->st[this->sigmaOff_],
                     s1 = this->st[this->sigmaOff_ + 1],
                     s2 = this->st[this->sigmaOff_ + 2];
        const double s2n = s0 * s0 + s1 * s1 + s2 * s2;
        const double d = (1.0 + s2n) * (1.0 + s2n);
        const double S[3][3] = {{0.0, -s2, s1}, {s2, 0.0, -s0}, {-s1, s0, 0.0}};
        double S2[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) S2[i][j] += S[i][k] * S[k][j];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Rbn[i][j] = (i == j ? 1.0 : 0.0)
                    + (8.0 * S2[i][j] - 4.0 * (1.0 - s2n) * S[i][j]) / d;
    }
    const double cx = this->rBcB_[0], cy = this->rBcB_[1], cz = this->rBcB_[2];
    const double wx = this->st[this->omegaOff_], wy = this->st[this->omegaOff_ + 1],
                 wz = this->st[this->omegaOff_ + 2];
    const double wxc[3] = {wy * cz - wz * cy, wz * cx - wx * cz, wx * cy - wy * cx};
    for (int i = 0; i < 3; ++i) {
        const size_t k = static_cast<size_t>(i);
        sc.r_CN_N[i] = this->st[this->rOff_ + k];
        sc.v_CN_N[i] = this->st[this->vOff_ + k];
        sc.r_BN_N[i] = this->st[this->rOff_ + k] + Rbn[0][i] * cx + Rbn[1][i] * cy + Rbn[2][i] * cz;
        sc.v_BN_N[i] = this->st[this->vOff_ + k] + Rbn[0][i] * wxc[0] + Rbn[1][i] * wxc[1] + Rbn[2][i] * wxc[2];
        sc.sigma_BN[i] = this->st[this->sigmaOff_ + k];
        sc.omega_BN_B[i] = this->st[this->omegaOff_ + k];
        sc.omegaDot_BN_B[i] = this->impl ? this->impl->lastWDot[i] : 0.0;
    }
    sc.MRPSwitchCount = this->mrpSwitches;
    // --- WP6: SC gap-fill (DV bookkeeping, LABELED non-parity vs stock
    // substep integration; B/C lines above owned by WP4, untouched here) ---
    for (int i = 0; i < 3; ++i) {
        sc.TotalAccumDV_CN_N[i] = this->accumDV_CN_N_[i];
        sc.TotalAccumDV_BN_B[i] = this->accumDV_BN_B_[i];
        sc.TotalAccumDVBdy[i] = this->accumDVBdy_[i];
        sc.nonConservativeAccelpntB_B[i] = this->nonConsAccelB_[i];
    }
    this->scStateOutMsg.write(&sc, this->moduleID, CurrentSimNanos);

    const size_t nw = this->wheels.size();
    if (nw > 0) {
        RWSpeedMsgPayload rw = this->rwSpeedOutMsg.zeroMsgPayload;
        constexpr size_t NAVAIL =
            sizeof(rw.wheelSpeeds) / sizeof(rw.wheelSpeeds[0]);
        for (size_t i = 0; i < nw && i < NAVAIL; ++i) rw.wheelSpeeds[i] = this->st[this->wheelOff_ + i];
        this->rwSpeedOutMsg.write(&rw, this->moduleID, CurrentSimNanos);
    }
    // --- WP6: RWConfigLog publish (full 23-field shape, per wheel per tick) ---
    // u_current = post-saturation/cut torque actually applied: read BACK from
    // the input slot P[wheelIn[i]] (equality by construction; never re-derive).
    if (this->rwOutMsgs.size() != nw) this->rwOutMsgs.resize(nw);
    for (size_t i = 0; i < nw; ++i) {
        RWConfigLogMsgPayload lg = this->rwOutMsgs[i].zeroMsgPayload;
        const double Om = (this->wheelOff_ + i < this->st.size()) ? this->st[this->wheelOff_ + i] : 0.0;
        double uCur = 0.0;
        if (this->impl && i < this->impl->wheelIn.size() && this->impl->wheelIn[i] < this->impl->pars.size())
            uCur = this->impl->pars[this->impl->wheelIn[i]];
        const double Js = (i < this->wheels.size()) ? this->wheels[i].Js : 0.0;
        const double um = (i < this->wheelUMax.size()) ? this->wheelUMax[i] : this->uMax;
        lg.Omega = Om; lg.Js = Js; lg.u_current = uCur; lg.u_max = um;
        lg.u_min = 1e-6; lg.Omega_max = 6000.0 * 2.0 * 3.141592653589793 / 60.0;
        lg.P_max = 200.0; lg.frictionTorque = 0.0;
        lg.theta = 0.0; lg.U_s = 0.0; lg.U_d = 0.0; lg.d = 0.0; lg.J13 = 0.0;
        lg.Jt = 0.5 * Js; lg.Jg = 0.0; lg.u_f = 0.0; lg.linearFrictionRatio = 0.0;
        lg.RWModel = static_cast<RWModels>(0);  // stock default model slot (BalancedWheels-class); echo only
        lg.mass = 0.0;
        if (i < this->wheels.size()) { lg.gsHat_B[0]=this->wheels[i].gs[0]; lg.gsHat_B[1]=this->wheels[i].gs[1]; lg.gsHat_B[2]=this->wheels[i].gs[2]; }
        lg.rWB_B[0]=lg.rWB_B[1]=lg.rWB_B[2]=0.0;  // fenced: balanced-only negligible
        this->rwOutMsgs[i].write(&lg, this->moduleID, CurrentSimNanos);
    }

    // --- WP3: thruster telemetry ---
    // One THROutputMsgPayload per thruster (stock thrusterOutMsgs parity, 9/9
    // fields; torque about B UNSHIFTED -- WP4 owns any CoM shift, telemetry
    // stays about-B by design, P-WP4.4). Steady-tick semantics: end-of-step
    // (post-propagation) staged duties; tick0 (pre-fire fill) publishes zeros
    // + geometry echo (T1 dump note). Fuel-mass depletion stays OUT: blowdown
    // factors are 1.0 without a tank (M6) -- labeled, not parity.
    {
        const size_t nt = this->nThrusters();
        while (this->thrusterOutMsgs.size() < nt) this->thrusterOutMsgs.emplace_back();
        const bool haveCache = this->impl && this->impl->lastFilt_.size() == nt
            && this->impl->lastQ_.size() == nt && nt > 0;
        for (size_t i = 0; i < nt; ++i) {
            THROutputMsgPayload out = this->thrusterOutMsgs[i].zeroMsgPayload;
            const double tmax = this->thrTmax[i];
            const auto& d = this->thrDirs[i];
            const auto& r = this->thrPos[i];
            const double bT = 1.0, bI = 1.0;  // no tank => 1.0 (M6; warn tick-side)
            const double q = haveCache ? this->impl->lastFilt_[i] : 0.0;
            const double F = q * tmax * bT;
            out.thrustFactor = q;
            out.thrustForce = F;
            out.thrustForce_B[0] = F * d[0];
            out.thrustForce_B[1] = F * d[1];
            out.thrustForce_B[2] = F * d[2];
            double tx = r[1] * out.thrustForce_B[2] - r[2] * out.thrustForce_B[1];
            double ty = r[2] * out.thrustForce_B[0] - r[0] * out.thrustForce_B[2];
            double tz = r[0] * out.thrustForce_B[1] - r[1] * out.thrustForce_B[0];
            if (this->swirlOn_) {
                const double s = q * this->maxSwirlTorque_;
                tx += s * d[0]; ty += s * d[1]; tz += s * d[2];
            }
            out.thrustTorquePntB_B[0] = tx;
            out.thrustTorquePntB_B[1] = ty;
            out.thrustTorquePntB_B[2] = tz;
            out.thrustBlowDownFactor = bT;
            out.ispBlowDownFactor = bI;
            out.maxThrust = tmax;
            out.thrusterLocation[0] = r[0];
            out.thrusterLocation[1] = r[1];
            out.thrusterLocation[2] = r[2];
            out.thrusterDirection[0] = d[0];
            out.thrusterDirection[1] = d[1];
            out.thrusterDirection[2] = d[2];
            this->thrusterOutMsgs[i].write(&out, this->moduleID, CurrentSimNanos);
        }
    }
    // --- WP6: fuel bookkeeping publish (FuelTankMsg-shaped, NON-PARITY labeled) ---
    // mdot=-F_ach/(isp*g0) with staircase-ACHIEVED F. 3 divergences: (1)
    // commanded!=achieved (closed 1st-order only), (2) no blowdown/ISP scaling
    // (factors echoed 1.0 until WP3), (3) NO mass feedback (omni mass fixed).
    {
        FuelTankMsgPayload fl = this->fuelTankOutMsg.zeroMsgPayload;
        fl.fuelMass = this->fuelMass_; fl.maxFuelMass = this->fuelMax_;
        fl.fuelMassDot = 0.0;
        this->fuelTankOutMsg.write(&fl, this->moduleID, CurrentSimNanos);
    }
}

// WP0 loud-reject guard: raises ONLY for features with no model. Every other
// trap flag below is now MODELED behind its default-off setter (WP1 saturation,
// WP2 friction/jitter, WP3b timing, WP4 offset, WP5 env) or informational
// (voltage reuse needs no code), so raising on them would reject the very runs
// the WPs implement. The single remaining fatal is the force-command path,
// which has no subscriber model by design. Wheels/thrusters merely being
// present is fine (no flag).
void OmniSpacecraft::checkUnsupported() {
    std::string active;
    // Modeled/informational (never fatal): frictionOn_, jitterOn_, voltageOn_,
    // minPulseOn_, rampsOn_, cutoffLagOn_, blowdownOn_, swirlOn_, j2On_,
    // offsetOn_ -- each WP owns its model + tests. Only the unmodeled force
    // path raises (guard-only by design, M6).
    if (this->forceMsgOn_ && this->thrForceInMsg.isLinked()) active += " forceMsg[forceMsgOn_]";
    if (!active.empty())
        this->bskLogger.bskError(
            "OmniSpacecraft: unmodeled feature(s) active:%s.", active.c_str());
}

void OmniSpacecraft::Reset(uint64_t CurrentSimNanos) {
    this->checkUnsupported();
    this->initDone = false;
    this->prevNanos = 0;
    this->thrStaged.clear();
    this->lastThrStamp = -1;
    this->mrpSwitches = 0;
    this->impl = std::make_unique<Impl>();
    // Fresh OmniSystem: buildIntegrator() replays the recorded wheel specs.
    this->buildIntegrator();
    if (this->rwOutMsgs.size() != this->wheels.size())  // WP6: size publishers
        this->rwOutMsgs.resize(this->wheels.size());

    SCStatesMsgPayload scZero = this->scStateOutMsg.zeroMsgPayload;
    this->scStateOutMsg.write(&scZero, this->moduleID, CurrentSimNanos);
}

void OmniSpacecraft::UpdateState(uint64_t CurrentSimNanos) {
    this->checkUnsupported();
    if (this->configDirty) this->buildIntegrator();

    if (!this->initDone) {
        // First tick: publish the ICs (no propagation) but STAGE the
        // current command so the first propagation uses it (stock parity:
        // write-once commands fire the first step; fresh commands idle it).
        this->stageThrusters();
        this->writeMsgs(CurrentSimNanos);
        this->prevNanos = CurrentSimNanos;
        this->initDone = true;
        return;
    }

    const double dt = static_cast<double>(CurrentSimNanos - this->prevNanos) * 1e-9;
    this->prevNanos = CurrentSimNanos;
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        this->bskLogger.bskLog(BSK_WARNING,
                               "OmniSpacecraft: non-positive dt, republishing state.");
        this->writeMsgs(CurrentSimNanos);
        return;
    }

    // --- inputs, per contributor (DESIGN Q2: contributor owns conversion) ---
    std::vector<double>& P = this->impl->pars;

    // Motor torques (RW path): read, apply WP1 momentum saturation then
    // saturate per wheel at its own uMax (stock saturates each effector from
    // its factory config), write into the input slots. Slots come from the
    // system (dynamic layout, A2) — never assumed to start at par 0.
    // Constant over the step, including across arcs below.
    //
    // WP1 saturation (ledger wp1_sat @ dt=0.1: overshoot 0.40 RPM, plateau
    // 6000.40 RPM): direction-aware cut,
    //   saturated = (W_i * u_i > 0) && (|W_i| >= omegaMax_i) -> u_i = 0,
    // else pass through (then +/-uMax clamp). Silent, instant resume.
    // Evaluated at STEP START ONLY from the step-opening state (same
    // tick-detected overshoot class as stock; tolerance overshoot <= 0.5
    // RPM @ dt=0.1, plateau +/-0.05 RPM). omegaMaxRad <= 0 means OFF (stock
    // `custom` Omega_max=-1 convention). Graph purity (A3): module-side only.
    {
        const size_t nw = this->wheels.size();
        if (this->wheelUMax.size() < nw) this->wheelUMax.resize(nw, this->uMax);
        const std::vector<size_t>& off = this->impl->wheelIn;
        auto zero_u = [&]() {
            for (size_t i = 0; i < nw && i < off.size(); ++i)
                if (off[i] < P.size()) P[off[i]] = 0.0;
        };
        if (this->rwMotorCmdInMsg.isLinked()) {
            const ArrayMotorTorqueMsgPayload msg = this->rwMotorCmdInMsg();
            constexpr size_t NAVAIL =
                sizeof(msg.motorTorque) / sizeof(msg.motorTorque[0]);
            for (size_t i = 0; i < nw && i < off.size() && i < NAVAIL; ++i) {
                if (off[i] >= P.size()) continue;
                double v = msg.motorTorque[i];
                // WP1 momentum cap (step-start, direction-aware).
                const double cap = this->wheels[i].omegaMaxRad;
                if (cap > 0.0 && this->wheelOff_ + i < this->st.size()) {
                    const double Wi = this->st[this->wheelOff_ + i];
                    if (Wi * v > 0.0 && std::fabs(Wi) >= cap) v = 0.0;
                }
                const double um = this->wheelUMax[i];
                P[off[i]] = std::max(-um, std::min(um, v));
            }
        } else {
            zero_u();
        }
    }
    // WP2 friction + JitterSimple (module-side doubles; A3-clean): AFTER the
    // WP1 saturation cut above, for each wheel read W_i, compute signed tauF
    // opposing W (iter-1: -fC*tanh(W/k) - cV*W, k = omegaLimitCycle_; tanh
    // HERE is fine, never in the graph), add JitterSimple noise (own
    // per-wheel mt19937_64 streams, seed ^ splitmix64(index+1),
    // normal_distribution(0, jitterSigma_)) when on, write P[fricIn[i]].
    // Off (frictionOn_ false): tauF slot identically 0 (deviation from
    // factory-off documented in tests/test_wp2_friction_jitter.py). Jitter
    // with friction off still writes pure noise; both off writes 0.0.
    // NOTE: JitterFullyCoupled/theta states are rejected at the setter.
    {
        const size_t nw = this->wheels.size();
        const std::vector<size_t>& foff = this->impl->fricIn;
        if (this->jitterOn_ && this->jitterSigma_ > 0.0) {
            if (!this->impl->jitterRngInit_ || this->impl->jitterInitSeed_ != this->jitterSeed_) {
                this->impl->jitterRng_.clear();
                for (size_t i = 0; i < nw; ++i) {
                    uint64_t z = this->jitterSeed_ + 0x9E3779B97F4A7C15ULL + (i + 1);
                    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;  // splitmix64
                    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
                    z = z ^ (z >> 31);
                    this->impl->jitterRng_.emplace_back(z);
                }
                this->impl->jitterInitSeed_ = this->jitterSeed_;
                this->impl->jitterRngInit_ = true;
            }
        }
        std::normal_distribution<double> jitterDist(0.0, this->jitterSigma_);
        const double k = (this->omegaLimitCycle_ > 0.0) ? this->omegaLimitCycle_ : 1.0;
        for (size_t i = 0; i < nw && i < foff.size(); ++i) {
            if (foff[i] >= P.size()) continue;
            double tauF = 0.0;
            if (this->frictionOn_ && this->wheelOff_ + i < this->st.size()) {
                const double Wi = this->st[this->wheelOff_ + i];
                tauF = -this->fCoulomb_ * std::tanh(Wi / k) - this->cViscous_ * Wi;
            }
            if (this->jitterOn_ && this->jitterSigma_ > 0.0 && i < this->impl->jitterRng_.size())
                tauF += jitterDist(this->impl->jitterRng_[i]);
            P[foff[i]] = tauF;
        }
    }

    // Thruster on-times (DynamicEffector path): staged mailbox semantics,
    // evaluated SAME-TICK (stage, propagate staged, one-shot consume).
    // The latest message is (re)staged iff new (first sight ever or
    // timestamp advance); same-timestamp rewrites never re-stage.
    // Reproduces stock exactly: fresh commands idle the fill step then fire
    // every step; write-once commands fire once then idle; frozen rewrites
    // never fire. (Contributor owns duty -> quantized on-time; the module
    // owns stage timing. RW inputs above stay continuous.)
    // WP5 per-tick env refresh: body offsets (per-tick, never frozen) +
    // drag/SRP/mag wrenches from state + config + ReadFunctors (P1 truth
    // table enforced inside updateEnvInputs); zeroes slots when off.
    this->updateEnvInputs(this->st.data());
    this->stageThrusters();
    // WP3: per-tick timing-state sizing (lazy; Impl is fresh on Reset) +
    // one-time guards. RW input block above is untouched (WP1/WP2).
    {
        const size_t nt = this->nThrusters();
        auto& im = *(this->impl);
        if (im.minPulseState_.size() != nt) im.minPulseState_.resize(nt);
        if (im.lagState_.size() != nt) im.lagState_.assign(nt, 0.0);
        if (im.rampClock_.size() != nt) im.rampClock_.assign(nt, 0.0);
        if (im.lastFilt_.size() != nt) im.lastFilt_.assign(nt, 0.0);
        if (im.lastQ_.size() != nt) im.lastQ_.assign(nt, 0.0);
        if (this->blowdownOn_ && !im.blowdownWarned_) {
            im.blowdownWarned_ = true;
            this->bskLogger.bskLog(BSK_WARNING,
                                   "OmniSpacecraft: blowdown coeffs set with no fuel tank "
                                   "attached; factors held at 1.0 (M6; depletion OUT, labeled).");
        }
        if (nt > 0 && (!im.swirlOffCached_ || im.swirlOff_ >= P.size())) {
            try {
                im.swirlOff_ = im.sys.par_index("swirlScale");
            } catch (const std::exception& e) {
                this->bskLogger.bskError(
                    "OmniSpacecraft: swirlScale slot missing (%s); apply the WP3 contributor spec.",
                    e.what());
            }
            im.swirlOffCached_ = true;
        }
    }
    std::vector<double> openT;
    {
        const size_t nt = this->nThrusters();
        if (!this->thrStaged.empty()) {
            // WP3 min-pulse gate (M3): sub-MinOnTime requests suppress to 0.
            for (size_t i = 0; i < this->thrStaged.size() && i < nt; ++i) {
                Impl::MinPulseState& st8 = this->impl->minPulseState_[i];
                if (!this->minPulseOn_) {
                    if (this->thrStaged[i] > 0.0) {
                        st8.lastFireStep = this->impl->thrStepCount_;
                        st8.fireCounter++;
                    }
                    continue;
                }
                const double gated = omni::ThrusterContributorT<double>::minPulseGate(
                    this->thrStaged[i], this->thrStaged[i], this->minOnTime_);
                if (gated == 0.0 && this->thrStaged[i] > 0.0) {
                    st8.carryover += this->thrStaged[i];
                    this->thrStaged[i] = 0.0;
                } else if (this->thrStaged[i] > 0.0) {
                    st8.lastFireStep = this->impl->thrStepCount_;
                    st8.fireCounter++;
                    st8.carryover = 0.0;
                }
            }
            openT = this->impl->sys.thruster_on_times(dt, this->thrStaged.data(),
                                                      this->thrStaged.size());
        } else {
            openT.assign(nt, 0.0);
        }
        // Cache gated+quantized step duties for telemetry (steady-tick).
        for (size_t i = 0; i < nt; ++i) {
            const double qi = (i < openT.size() && dt > 0.0) ? openT[i] / dt : 0.0;
            this->impl->lastQ_[i] = qi;
            this->impl->rampClock_[i] = 0.0;
            if (qi <= 0.0) this->impl->lastFilt_[i] = 0.0;
        }
        this->thrStaged.clear();  // one-shot consume
        this->impl->thrStepCount_++;
    }

    // Push BOTH state and pars into the propagator, then advance arc by arc.
    std::vector<double> y = this->st;
    heyoka::taylor_adaptive<double>& ta = *(this->impl->ta);
    double* pd = ta.get_pars_data();
    // WP3: constant-per-step swirl scale (contributor "swirlScale" slot, A2).
    if (this->nThrusters() > 0 && this->impl->swirlOffCached_
        && this->impl->swirlOff_ < P.size()) {
        P[this->impl->swirlOff_] = this->swirlOn_ ? this->maxSwirlTorque_ : 0.0;
    }
    for (size_t i = 0; i < P.size() && i < this->impl->ta_npars; ++i) pd[i] = P[i];
    {
        // Switch list: 0, interior quantized open-times (epsilon-merged),
        // dt. Thruster j is open during arc k iff openT[j] > sw[k].
        // WP3b: ramp knots subdivide open arcs (piecewise duty(t), ZOH per
        // sub-arc); the sw switch-list structure is unchanged.
        std::vector<double> sw = {0.0};
        const double eps = 1e-9 * dt;
        auto addCut = [&](double t) {
            if (!(t > eps && t < dt - eps)) return;
            for (double s : sw) {
                if (std::fabs(s - t) <= eps) return;
            }
            sw.push_back(t);
        };
        for (double t : openT) addCut(t);
        if (this->rampsOn_) {
            for (size_t j = 0; j < openT.size(); ++j) {
                if (openT[j] <= eps) continue;
                for (double ko : this->onRamp_) addCut(ko);
                for (double kc : this->offRamp_) addCut(openT[j] + kc);
            }
        }
        std::sort(sw.begin(), sw.end());
        sw.push_back(dt);
        std::vector<double> eff(openT.size(), 0.0);
        double* sd = ta.get_state_data();
        for (size_t k = 0; k + 1 < sw.size(); ++k) {
            const double seg = sw[k + 1] - sw[k];
            if (!(seg > 0.0)) continue;
            const double tArc = sw[k];  // per-arc ZOH evaluation point
            for (size_t j = 0; j < openT.size(); ++j) {
                const bool open = (openT[j] > tArc);
                // WP3 effective duty: ramp(t) shaped, lag filtered, blowdown
                // scaled -- all module-side on doubles (A3); the graph sees
                // only the q-slot values (the thruster q-slot pattern).
                double target;
                if (!this->rampsOn_) {
                    target = open ? 1.0 : 0.0;
                } else if (open) {
                    target = this->rampFactor(this->impl->rampClock_[j], this->onRamp_, true);
                } else if (openT[j] > eps) {
                    target = this->rampFactor(tArc - openT[j], this->offRamp_, false);
                } else {
                    target = 0.0;
                }
                double filt = target;
                if (this->cutoffLagOn_) {
                    const double alpha = 1.0 - std::exp(-seg * this->cutoffFreq_);
                    double& lag = this->impl->lagState_[j];
                    lag += (target - lag) * alpha;
                    filt = lag;
                }
                eff[j] = filt;  // * blowdown(=1.0, no tank; depletion OUT, M6)
                if (open) this->impl->lastFilt_[j] = this->impl->lastQ_[j] * filt;
            }
            // WP5-arc-hook: refresh fast-varying env wrenches (drag/SRP/mag)
            // from the CURRENT arc state y. Writes env slots ONLY -- never
            // touches openT/sw/eff/fill_thruster_inputs (WP3-compatible).
            if (this->impl->hasEnv) this->updateEnvInputs(y.data());
            if (!openT.empty())
                this->impl->sys.fill_thruster_inputs(dt, eff.data(), eff.size(), P);
            for (size_t i = 0; i < P.size() && i < this->impl->ta_npars; ++i) pd[i] = P[i];
            for (size_t i = 0; i < y.size(); ++i) sd[i] = y[i];
            ta.set_time(0.0);
            try {
                ta.propagate_for(seg);
            } catch (const std::exception& e) {
                this->bskLogger.bskError("OmniSpacecraft: propagation failed: %s.", e.what());
            }
            for (size_t j = 0; j < openT.size(); ++j) {
                if (openT[j] > tArc) this->impl->rampClock_[j] += seg;
            }
            y = ta.get_state();
        }
    }
    // WP4: published wDot from numeric accelerations at end state
    // (steady-tick value; arcs untouched).
    {
        const std::vector<double> acc = this->impl->sys.accelerations(y, P);
        if (acc.size() >= 6) {
            this->impl->lastWDot[0] = acc[3];
            this->impl->lastWDot[1] = acc[4];
            this->impl->lastWDot[2] = acc[5];
        }
    }
    this->st = y;
    // WP6: per-tick DV bookkeeping (LABELED non-parity vs stock substep
    // integration): accumulate F*dt/m from the achieved end-of-step wrench.
    // O(dt) across staircase edges by contract.
    {
        const double m = this->mass;
        if (m > 0.0) {
            double Fx = 0.0, Fy = 0.0, Fz = 0.0;
            for (size_t j = 0; j < this->nThrusters() && j < this->impl->lastFilt_.size()
                 && j < this->thrTmax.size() && j < this->thrDirs.size(); ++j) {
                const double F = this->impl->lastFilt_[j] * this->thrTmax[j];
                Fx += F * this->thrDirs[j][0];
                Fy += F * this->thrDirs[j][1];
                Fz += F * this->thrDirs[j][2];
            }
            const double k = dt / m;
            this->accumDVBdy_[0] += Fx * k;
            this->accumDVBdy_[1] += Fy * k;
            this->accumDVBdy_[2] += Fz * k;
            this->accumDV_CN_N_[0] += Fx * k;
            this->accumDV_CN_N_[1] += Fy * k;
            this->accumDV_CN_N_[2] += Fz * k;
            this->accumDV_BN_B_[0] += Fx * k;
            this->accumDV_BN_B_[1] += Fy * k;
            this->accumDV_BN_B_[2] += Fz * k;
            this->nonConsAccelB_[0] = Fx / m;
            this->nonConsAccelB_[1] = Fy / m;
            this->nonConsAccelB_[2] = Fz / m;
        }
    }

    // MRP shadow-set reset, exactly as stock does.
    const double n2 = this->st[this->sigmaOff_] * this->st[this->sigmaOff_]
                    + this->st[this->sigmaOff_ + 1] * this->st[this->sigmaOff_ + 1]
                    + this->st[this->sigmaOff_ + 2] * this->st[this->sigmaOff_ + 2];
    if (n2 > 1.0) {
        this->st[this->sigmaOff_] /= -n2;
        this->st[this->sigmaOff_ + 1] /= -n2;
        this->st[this->sigmaOff_ + 2] /= -n2;
        ++this->mrpSwitches;
    }

    this->writeMsgs(CurrentSimNanos);
}

// Stage the latest on-time message iff it is new (first sight ever, or its
// timestamp strictly advances past the staged one). Same-timestamp rewrites
// never re-stage; write-once commands stage exactly once. Staged content is
// consumed one-shot by propagation (see UpdateState); stagedTs persists
// across consumes so re-sights stay idle.
void OmniSpacecraft::stageThrusters() {
    // WP3 guard-only path (M6): the force-command message has no subscriber
    // model; a linked THRArrayCmdForceMsg must raise loudly, never run silent.
    if (this->forceMsgOn_ && this->thrForceInMsg.isLinked()) {
        this->bskLogger.bskError(
            "OmniSpacecraft: THRArrayCmdForceMsg path is not modeled "
            "(omni subscribes OnTime only); unlink it or use OnTime commands.");
    }
    const size_t nt = this->nThrusters();
    if (!this->thrOnTimeInMsg.isLinked() || nt == 0) {
        this->thrStaged.clear();
        return;
    }
    const int64_t stamp =
        static_cast<int64_t>(this->thrOnTimeInMsg.timeWritten());
    if (stamp <= this->lastThrStamp) return;
    const THRArrayOnTimeCmdMsgPayload msg = this->thrOnTimeInMsg();
    constexpr size_t NAVAIL =
        sizeof(msg.OnTimeRequest) / sizeof(msg.OnTimeRequest[0]);
    this->thrStaged.assign(std::min(nt, NAVAIL), 0.0);
    for (size_t i = 0; i < this->thrStaged.size(); ++i)
        this->thrStaged[i] = msg.OnTimeRequest[i];
    this->lastThrStamp = stamp;
}

// WP3: ramp profile factor at event time t. Empty knots = no ramp. Linear
// interpolates 0->1 across [0, span] (interior knots place sub-arcs only);
// piecewise-hold steps through the knots (stock THRTimePair shape, factors
// implicit and even -- documented deviation: stock carries explicit
// ThrustFactor/IspFactor per pair).
double OmniSpacecraft::rampFactor(double t, const std::vector<double>& knots, bool isOn) const {
    if (knots.empty()) return isOn ? 1.0 : 0.0;
    const double span = knots.back();
    if (!(span > 0.0)) return isOn ? 1.0 : 0.0;
    if (t <= 0.0) return isOn ? 0.0 : 1.0;
    if (t >= span) return isOn ? 1.0 : 0.0;
    if (this->rampLinear_) {
        const double f = t / span;
        return isOn ? f : (1.0 - f);
    }
    size_t n = 0;
    while (n < knots.size() && knots[n] <= t) ++n;
    const double f = static_cast<double>(n) / static_cast<double>(knots.size());
    return isOn ? f : (1.0 - f);
}

// WP5 per-tick (/per-arc) env refresh. y = hub-first state (A1: r[0..2],
// v[3..5], sigma[6..8]). Module-side doubles only (A3); TRUE laws from
// omni::env. P1 truth table: density/sun/MTB -> bskError; eclipse -> 1.0;
// wind -> inertial. ERP intentionally untouched (field-only, no slots).
void OmniSpacecraft::updateEnvInputs(const double* y) {
    if (y == nullptr || !this->impl || !this->impl->hasEnv) return;
    std::vector<double>& P = this->impl->pars;
    const size_t bO = this->impl->envBody, dO = this->impl->envDrag;
    const size_t sO = this->impl->envSrp, mO = this->impl->envMag;
    const size_t nB = this->impl->envNBodies;
    const size_t nEnv = 3 * nB + 15;
    for (size_t k = 0; k < nEnv; ++k)
        if (bO + k < P.size()) P[bO + k] = 0.0;
    // Hub-first-12 state layout (A1: r/v/sigma blocks; resolved by NAME in
    // buildIntegrator, used here via members -- never numeric literals).
    const size_t rO = this->rOff_, vO = this->vOff_, sgO = this->sigmaOff_;
    omni::env::Vec3 r{y[rO], y[rO + 1], y[rO + 2]};
    omni::env::Vec3 v{y[vO], y[vO + 1], y[vO + 2]};
    omni::env::Vec3 sig{y[sgO], y[sgO + 1], y[sgO + 2]};
    const omni::env::Mat3 R = omni::env::dcmBN_from_sigma(sig);

    // N-body offsets: refreshed PER TICK from msgs when linked (never frozen),
    // else the recorded static/initial offset (bench/fixture path).
    for (size_t i = 0; i < nB && i < this->envBodyMu.size(); ++i) {
        omni::env::Vec3 off{this->envBodyOff0[3 * i], this->envBodyOff0[3 * i + 1],
                            this->envBodyOff0[3 * i + 2]};
        if (i == 0 && this->sunStateInMsg.isLinked()) {
            const SpicePlanetStateMsgPayload sun = this->sunStateInMsg();
            off = {sun.PositionVector[0], sun.PositionVector[1], sun.PositionVector[2]};
        }
        if (i == 1 && this->moonStateInMsg.isLinked()) {
            const SpicePlanetStateMsgPayload moon = this->moonStateInMsg();
            off = {moon.PositionVector[0], moon.PositionVector[1], moon.PositionVector[2]};
        }
        if (bO + 3 * i + 2 < P.size()) {
            P[bO + 3 * i] = off[0]; P[bO + 3 * i + 1] = off[1]; P[bO + 3 * i + 2] = off[2];
        }
    }

    if (this->dragOn_) {
        if (!this->atmoDensInMsg.isLinked()) {
            this->bskLogger.bskError(
                "OmniSpacecraft: drag on with atmoDensInMsg unlinked (stock bskError).");
        } else {
            const AtmoPropsMsgPayload atmo = this->atmoDensInMsg();
            omni::env::Vec3 vAir{0.0, 0.0, 0.0};  // wind unlinked -> inertial fallback
            if (this->windVelInMsg.isLinked()) {
                const WindMsgPayload wind = this->windVelInMsg();
                vAir = {wind.v_air_N[0], wind.v_air_N[1], wind.v_air_N[2]};
            }
            const omni::env::Vec3 vRelB = omni::env::matvec_BN(
                R, omni::env::vec_sub(v, vAir));
            // WP4-combo MEASURED (combo probe drag-offset leg, firing-proven):
            // stock drag torque does NOT shift by -c_B (offset-vs-no-offset
            // delta ~1e-17 vs 1.3e-5 shifted prediction). Pass the effector
            // comOffset VERBATIM. (SRP facets DO shift -- separate path below;
            // magnetic/gravity have no arms.)
            const omni::env::Vec3 dragArm{this->envComOff[0],
                                          this->envComOff[1],
                                          this->envComOff[2]};
            const omni::env::WrenchB w = omni::env::dragWrench(
                atmo.neutralDensity, this->envCdA, vRelB, dragArm,
                this->envDensityCorr);
            if (dO + 5 < P.size()) {
                P[dO] = w.F[0]; P[dO + 1] = w.F[1]; P[dO + 2] = w.F[2];
                P[dO + 3] = w.T[0]; P[dO + 4] = w.T[1]; P[dO + 5] = w.T[2];
            }
        }
    }

    const bool srpOn = this->cannonballOn_ || !this->srpFacets.empty();
    if (srpOn) {
        const bool sunLinked = this->sunStateInMsg.isLinked();
        if (!sunLinked && !this->hasSun_) {
            this->bskLogger.bskError(
                "OmniSpacecraft: SRP on with sun state unlinked (stock bskError).");
        } else {
            omni::env::Vec3 sunPos = this->sunState_;
            if (sunLinked) {
                const SpicePlanetStateMsgPayload sun = this->sunStateInMsg();
                sunPos = {sun.PositionVector[0], sun.PositionVector[1], sun.PositionVector[2]};
            }
            const omni::env::Vec3 rSB = omni::env::vec_sub(sunPos, r);
            const double d = omni::env::vec_norm(rSB);
            if (d > 0.0 && sO + 5 < P.size()) {
                const double Pflx = omni::env_const::P_SRP_1AU *
                    (omni::env_const::AU_M / d) * (omni::env_const::AU_M / d);
                const omni::env::Vec3 sHatB = omni::env::matvec_BN(
                    R, omni::env::vec_scale(rSB, 1.0 / d));
                // Eclipse: ONE field shadowFactor; unlinked -> 1.0 (full sun ON).
                double shadow = 1.0;
                if (this->eclipseInMsg.isLinked())
                    shadow = this->eclipseInMsg().shadowFactor;
                omni::env::Vec3 F{0.0, 0.0, 0.0}, T{0.0, 0.0, 0.0};
                if (this->cannonballOn_) {
                    F = omni::env::vec_add(
                        F, omni::env::srpCannonball(this->cbArea, this->cbCr, Pflx,
                                                   shadow, sHatB));
                    // Cannonball acts at CoM: torque identically zero.
                }
                for (const auto& f : this->srpFacets) {
                    // WP4-combo (SRP side of the measured split): facet
                    // torque arm shifts by -c_B (about CoM) under hub offset.
                    // NOTE drag does NOT shift -- see dragArm above; the
                    // split is measured, not assumed.
                    const omni::env::WrenchB w = omni::env::srpFacet(
                        f.area, {f.nHat[0], f.nHat[1], f.nHat[2]},
                        {f.rCop[0] - this->rBcB_[0],
                         f.rCop[1] - this->rBcB_[1],
                         f.rCop[2] - this->rBcB_[2]}, f.cd, f.cs, Pflx,
                        shadow, sHatB);
                    F = omni::env::vec_add(F, w.F);
                    T = omni::env::vec_add(T, w.T);
                }
                P[sO] = F[0]; P[sO + 1] = F[1]; P[sO + 2] = F[2];
                P[sO + 3] = T[0]; P[sO + 4] = T[1]; P[sO + 5] = T[2];
            }
        }
    }

    if (this->magOn_) {
        const bool mZero = (this->magM_[0] == 0.0 && this->magM_[1] == 0.0 &&
                            this->magM_[2] == 0.0);
        if (!this->magFieldInMsg.isLinked()) {
            this->bskLogger.bskError(
                "OmniSpacecraft: mag on with magField msg unlinked (stock bskError).");
        } else if (mZero) {
            this->bskLogger.bskError(
                "OmniSpacecraft: mag on with zero dipole (MTB cmd/params unlinked).");
        } else {
            const MagneticFieldMsgPayload mag = this->magFieldInMsg();
            const omni::env::Vec3 BN{mag.magField_N[0], mag.magField_N[1], mag.magField_N[2]};
            const omni::env::Vec3 BB = omni::env::matvec_BN(R, BN);
            omni::env::Vec3 m{this->magM_[0], this->magM_[1], this->magM_[2]};
            // Identity-Gt clamp preset: |m| <= maxMtbDipole_.
            const double mn = omni::env::vec_norm(m);
            if (mn > this->maxMtbDipole_ && mn > 0.0)
                m = omni::env::vec_scale(m, this->maxMtbDipole_ / mn);
            const omni::env::Vec3 tau = omni::env::magTorque(m, BB);
            if (mO + 2 < P.size()) { P[mO] = tau[0]; P[mO + 1] = tau[1]; P[mO + 2] = tau[2]; }
        }
    }
    // ERP: read-only field (message equality in the bench); NO slots, NO force.
}
