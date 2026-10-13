// ISC License - see native_rw/LICENSE.  See omni/vec_integrator.hpp.
#include "omni/vec_integrator.hpp"

#include <algorithm>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "omni/system.hpp"
#include "omni/environment.hpp"
#include "simulation/dynamics/_GeneralModuleFiles/extendedStateVector.h"

// ---------------------------------------------------------------------------
// PIMPL: heyoka + OmniSystem stay out of the header (SWIG discipline).
// ---------------------------------------------------------------------------
struct OmniVecIntegrator::Impl {
    omni::OmniSystem sys;
    std::optional<heyoka::taylor_adaptive<double>> ta;
    std::vector<double> parsBase;
    std::vector<size_t> wheelIn;
    std::vector<size_t> fricIn;
    size_t ta_npars = 0;  // truncated par count owned by ta (push bound)
    static constexpr size_t NPOS = static_cast<size_t>(-1);
    std::vector<std::string> claimed;  // registered names, stock order
    std::vector<size_t> claimedRows;  // registered rows, same order (B2.4 freeze)
    // Cached live StateData sources (fixed after InitializeSimulation).
    StateData* hubR = nullptr;
    StateData* hubV = nullptr;
    StateData* hubS = nullptr;
    StateData* hubO = nullptr;
    StateData* omegas = nullptr;
    StateData* passSrc[2] = {nullptr, nullptr};
    size_t nStates = 0;
    size_t nWheels = 0;
    // WP0/WP7 layout hygiene (A1/A2): omni-canonical block offsets resolved
    // BY NAME from state_names() at build; gather/scatter use these, never
    // numeric literals (CI-enforced).
    size_t rOff = 0, vOff = 3, sOff = 6, oOff = 9, wOff = 12;
    // WP5 env slot bases + body count (valid after buildOnce).
    size_t envBody = 0, envDrag = 0, envSrp = 0, envMag = 0, envNBodies = 0;
    bool hasEnv = false;
    // Phase-2 fuel (valid after buildOnce): live stock source, canonical
    // position (always last), mdot + hub mass/I par slots, live aggregate
    // c (I_C + arms + B/C boundary, refreshed per tick; no rebuilds).
    StateData* fuelSrc = nullptr;
    size_t fuelPos = 0;
    size_t mdotOff = 0, massOff = 0;
    size_t IO[6]{0, 0, 0, 0, 0, 0};
    size_t cOff[3]{NPOS, NPOS, NPOS};  // thruster cB slots (NPOS = masked out)
};

OmniVecIntegrator::OmniVecIntegrator(Spacecraft* sc)
    : StateVecIntegrator(static_cast<DynamicObject*>(static_cast<void*>(sc))),
      dyn_(static_cast<DynamicObject*>(static_cast<void*>(sc))) {
    // The void* round-trip is valid under stock's single-inheritance layout
    // (Spacecraft : DynamicObject : SysModel); the base ctor registers
    // dynPtrs[0] = this object, which setIntegrator() requires.
    if (dyn_ == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null Spacecraft pointer.");
}

OmniVecIntegrator::~OmniVecIntegrator() = default;

// --- legacy single-object setters: slot-0 shorthand (delegate; identical
// semantics by construction) ------------------------------------------------
void OmniVecIntegrator::setMass(double m) { this->setMassForObject(0, m); }

void OmniVecIntegrator::setInertia(double a, double b, double c,
                                   double d, double e, double f) {
    this->setInertiaForObject(0, a, b, c, d, e, f);
}

void OmniVecIntegrator::setCoMOffset(double x, double y, double z) {
    this->setCoMOffsetForObject(0, x, y, z);
}

void OmniVecIntegrator::setMu(double mu) { this->setMuForObject(0, mu); }

void OmniVecIntegrator::setUseGravityGradient(bool v) {
    this->setUseGravityGradientForObject(0, v);
}

void OmniVecIntegrator::addEnvironmentBody(double mu, double ox, double oy, double oz) {
    this->addEnvironmentBodyForObject(0, mu, ox, oy, oz);
}

void OmniVecIntegrator::setJ2(double J2) { this->setJ2ForObject(0, J2); }

void OmniVecIntegrator::setDragParams(double CdA, double cx, double cy, double cz) {
    this->setDragParamsForObject(0, CdA, cx, cy, cz);
}

void OmniVecIntegrator::addSRPFacet(double area, double nx, double ny, double nz,
                                    double copx, double copy, double copz,
                                    double c_diffuse, double c_specular) {
    this->addSRPFacetForObject(0, area, nx, ny, nz, copx, copy, copz,
                               c_diffuse, c_specular);
}

void OmniVecIntegrator::setSRPCannonball(double area, double cr) {
    this->setSRPCannonballForObject(0, area, cr);
}

void OmniVecIntegrator::setSunState(double px, double py, double pz) {
    this->setSunStateForObject(0, px, py, pz);
}

void OmniVecIntegrator::setMagneticMoment(double mx, double my, double mz) {
    this->setMagneticMomentForObject(0, mx, my, mz);
}

void OmniVecIntegrator::setMagneticDipole(double g10, double g11, double h11) {
    this->setMagneticDipoleForObject(0, g10, g11, h11);
}

void OmniVecIntegrator::setMaxMtbDipole(double dmax) {
    this->setMaxMtbDipoleForObject(0, dmax);
}

void OmniVecIntegrator::setFuelStateName(const std::string& name) {
    this->setFuelStateNameForObject(0, name);
}

void OmniVecIntegrator::setFuelTank(double rTBx, double rTBy, double rTBz, double leak) {
    this->setFuelTankForObject(0, rTBx, rTBy, rTBz, leak);
}

void OmniVecIntegrator::setThrusterIsps(const std::vector<double>& isp) {
    this->setThrusterIspsForObject(0, isp);
}

void OmniVecIntegrator::addThruster(double dx, double dy, double dz,
                                    double px, double py, double pz, double tmax) {
    this->addThrusterForObject(0, dx, dy, dz, px, py, pz, tmax);
}

void OmniVecIntegrator::setInitialThrusterCmd(const std::vector<double>& onTimes) {
    this->setInitialThrusterCmdForObject(0, onTimes);
}

void OmniVecIntegrator::addWheel(double gsx, double gsy, double gsz, double Js) {
    this->addWheelForObject(0, gsx, gsy, gsz, Js);
}

void OmniVecIntegrator::linkWheelLog(size_t i, Message<RWConfigLogMsgPayload>* src) {
    this->linkWheelLogForObject(0, i, src);
}

void OmniVecIntegrator::setFrictionEnabled(bool on) {
    this->setFrictionEnabledForObject(0, on);
}

void OmniVecIntegrator::setFrictionParams(size_t i, double fCoulomb, double fStatic,
                                          double betaStatic, double cViscous,
                                          double omegaLimitCycle) {
    this->setFrictionParamsForObject(0, i, fCoulomb, fStatic, betaStatic,
                                     cViscous, omegaLimitCycle);
}

void OmniVecIntegrator::setTol(double tol) {
    if (!(tol > 0.0) || !std::isfinite(tol))
        this->bskLogger.bskError("OmniVecIntegrator: tol must be finite positive.");
    this->tol_ = tol;
    for (auto& c : this->objCfgs_) c.dirty = true;
}

void OmniVecIntegrator::setCompactMode(bool compact) {
    this->compactMode_ = compact;
    for (auto& c : this->objCfgs_) c.dirty = true;
}

// --- WP25-A4 per-object setters (slot auto-grow; own slot mirrored+dirty) ---
void OmniVecIntegrator::setMassForObject(size_t obj, double m) {
    if (!(m > 0.0) || !std::isfinite(m))
        this->bskLogger.bskError("OmniVecIntegrator: mass must be finite positive.");
    ObjCfg& c = this->cfgFor(obj);
    c.mass = m;
    c.mirrored = true;
    c.dirty = true;
}

void OmniVecIntegrator::setInertiaForObject(size_t obj, double a, double b, double c,
                                            double d, double e, double f) {
    ObjCfg& C = this->cfgFor(obj);
    C.Ixx = a; C.Iyy = b; C.Izz = c;
    C.Ixy = d; C.Ixz = e; C.Iyz = f;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setCoMOffsetForObject(size_t obj, double x, double y, double z) {
    ObjCfg& C = this->cfgFor(obj);
    C.rBcB[0] = x; C.rBcB[1] = y; C.rBcB[2] = z;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setMuForObject(size_t obj, double mu) {
    if (!(mu > 0.0) || !std::isfinite(mu))
        this->bskLogger.bskError("OmniVecIntegrator: mu must be finite positive.");
    ObjCfg& C = this->cfgFor(obj);
    C.mu = mu;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setUseGravityGradientForObject(size_t obj, bool v) {
    ObjCfg& C = this->cfgFor(obj);
    C.useGG = v;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::addEnvironmentBodyForObject(size_t obj, double mu,
                                                    double ox, double oy,
                                                    double oz) {
    if (!(mu > 0.0) || !std::isfinite(mu))
        this->bskLogger.bskError("OmniVecIntegrator: env body mu must be finite positive.");
    ObjCfg& C = this->cfgFor(obj);
    C.envBodyMu.push_back(mu);
    C.envBodyOff0.push_back(ox);
    C.envBodyOff0.push_back(oy);
    C.envBodyOff0.push_back(oz);
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setJ2ForObject(size_t obj, double J2) {
    if (!std::isfinite(J2) || J2 < 0.0)
        this->bskLogger.bskError("OmniVecIntegrator: J2 must be finite non-negative.");
    ObjCfg& C = this->cfgFor(obj);
    C.envJ2 = J2;
    C.j2On = true;
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setDragParamsForObject(size_t obj, double CdA,
                                               double cx, double cy,
                                               double cz) {
    if (!(CdA >= 0.0) || !std::isfinite(CdA))
        this->bskLogger.bskError("OmniVecIntegrator: CdA must be finite non-negative.");
    ObjCfg& C = this->cfgFor(obj);
    C.envCdA = CdA;
    C.envComOff[0] = cx; C.envComOff[1] = cy; C.envComOff[2] = cz;
    C.dragOn = true;
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::addSRPFacetForObject(size_t obj, double area,
                                             double nx, double ny, double nz,
                                             double copx, double copy,
                                             double copz, double c_diffuse,
                                             double c_specular) {
    const double n = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!(area > 0.0))
        this->bskLogger.bskError("OmniVecIntegrator: facet area must be positive.");
    if (!(n > 0.0))
        this->bskLogger.bskError("OmniVecIntegrator: facet normal must be nonzero.");
    SRPFacetSpec f;
    f.area = area;
    f.nHat[0] = nx / n; f.nHat[1] = ny / n; f.nHat[2] = nz / n;
    f.rCop[0] = copx; f.rCop[1] = copy; f.rCop[2] = copz;
    f.cd = c_diffuse; f.cs = c_specular;
    ObjCfg& C = this->cfgFor(obj);
    C.srpFacets.push_back(f);
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setSRPCannonballForObject(size_t obj, double area, double cr) {
    if (!(area > 0.0))
        this->bskLogger.bskError("OmniVecIntegrator: cannonball area must be positive.");
    if (!(cr >= 0.0) || !(cr <= 1.0))
        this->bskLogger.bskError(
            "OmniVecIntegrator: cr must be the specular fraction in [0,1] "
            "(stock C_R total multiplier maps as cr = C_R - 1).");
    ObjCfg& C = this->cfgFor(obj);
    C.cbArea = area; C.cbCr = cr;
    C.cannonballOn = true;
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setSunStateForObject(size_t obj, double px, double py,
                                             double pz) {
    ObjCfg& C = this->cfgFor(obj);
    C.sunState[0] = px; C.sunState[1] = py; C.sunState[2] = pz;
    C.hasSun = true;
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setMagneticMomentForObject(size_t obj, double mx, double my,
                                                   double mz) {
    ObjCfg& C = this->cfgFor(obj);
    C.magM[0] = mx; C.magM[1] = my; C.magM[2] = mz;
    C.magOn = true;
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setMagneticDipoleForObject(size_t obj, double g10, double g11,
                                                   double h11) {
    ObjCfg& C = this->cfgFor(obj);
    C.magG10 = g10; C.magG11 = g11; C.magH11 = h11;
    C.envConfigured = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setMaxMtbDipoleForObject(size_t obj, double dmax) {
    if (!(dmax > 0.0) || !std::isfinite(dmax))
        this->bskLogger.bskError("OmniVecIntegrator: maxMtbDipole must be finite positive.");
    ObjCfg& C = this->cfgFor(obj);
    C.maxMtbDipole = dmax;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setFuelStateNameForObject(size_t obj, const std::string& name) {
    if (name.empty())
        this->bskLogger.bskError("OmniVecIntegrator: fuel state name must be nonempty.");
    ObjCfg& C = this->cfgFor(obj);
    C.fuelStateName = name;
    C.hasFuel = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setFuelTankForObject(size_t obj, double rTBx, double rTBy,
                                             double rTBz, double leak) {
    if (!std::isfinite(rTBx + rTBy + rTBz) || !std::isfinite(leak) || leak < 0.0)
        this->bskLogger.bskError("OmniVecIntegrator: fuel tank record invalid.");
    ObjCfg& C = this->cfgFor(obj);
    C.fuelRTB[0] = rTBx; C.fuelRTB[1] = rTBy; C.fuelRTB[2] = rTBz;
    C.fuelLeak = leak;
    C.hasFuel = true;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setThrusterIspsForObject(size_t obj,
                                                 const std::vector<double>& isp) {
    for (double v : isp) {
        if (!(v > 0.0) || !std::isfinite(v))
            this->bskLogger.bskError("OmniVecIntegrator: thruster Isp must be finite positive.");
    }
    ObjCfg& C = this->cfgFor(obj);
    C.fuelIsps = isp;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::addThrusterForObject(size_t obj, double dx, double dy, double dz,
                                             double px, double py, double pz,
                                             double tmax) {
    const double n = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniVecIntegrator: thruster direction must be nonzero.");
    if (!(tmax > 0.0) || !std::isfinite(tmax))
        this->bskLogger.bskError("OmniVecIntegrator: thruster Tmax must be finite positive.");
    ObjCfg& C = this->cfgFor(obj);
    C.thrDirs.push_back({dx / n, dy / n, dz / n});
    C.thrPos.push_back({px, py, pz});
    C.thrTmax.push_back(tmax);
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setInitialThrusterCmdForObject(
    size_t obj, const std::vector<double>& onTimes) {
    ObjCfg& C = this->cfgFor(obj);
    C.preStaged = onTimes;
    C.havePreStaged = !onTimes.empty();
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::addWheelForObject(size_t obj, double gsx, double gsy,
                                          double gsz, double Js) {
    const double n = std::sqrt(gsx * gsx + gsy * gsy + gsz * gsz);
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniVecIntegrator: wheel spin axis must be nonzero.");
    if (!(Js > 0.0) || !std::isfinite(Js))
        this->bskLogger.bskError("OmniVecIntegrator: Js must be finite positive.");
    WheelRec w{};
    w.gs[0] = gsx / n; w.gs[1] = gsy / n; w.gs[2] = gsz / n;
    w.Js = Js;
    ObjCfg& C = this->cfgFor(obj);
    C.wheels.push_back(w);
    C.fricRecs.push_back(FricRec{});
    C.fricState.push_back(FricState{});
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::linkWheelLogForObject(size_t obj, size_t i,
                                              Message<RWConfigLogMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null wheel-log source.");
    if (obj == 0) {
        // Slot-0 storage is the legacy public member (SWIG-visible, untouched).
        if (this->rwLogInMsgs.size() <= i) this->rwLogInMsgs.resize(i + 1);
        this->rwLogInMsgs[i].subscribeTo(src);
        return;
    }
    this->ensureSlot(obj);  // auto-grow (links alone never mark mirrored/dirty)
    std::vector<ReadFunctor<RWConfigLogMsgPayload>>& logs =
        this->extraMsgs_[obj - 1].rwLogs;
    if (logs.size() <= i) logs.resize(i + 1);
    logs[i].subscribeTo(src);
}

void OmniVecIntegrator::setFrictionEnabledForObject(size_t obj, bool on) {
    ObjCfg& C = this->cfgFor(obj);
    C.frictionOn = on;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::setFrictionParamsForObject(size_t obj, size_t i,
                                                   double fCoulomb, double fStatic,
                                                   double betaStatic, double cViscous,
                                                   double omegaLimitCycle) {
    ObjCfg& C = this->cfgFor(obj);
    if (i >= C.fricRecs.size())
        this->bskLogger.bskError("OmniVecIntegrator: wheel %zu not mirrored.", i);
    if (!std::isfinite(fCoulomb + fStatic + betaStatic + cViscous + omegaLimitCycle))
        this->bskLogger.bskError("OmniVecIntegrator: friction params must be finite.");
    FricRec& r = C.fricRecs[i];
    r.fCoulomb = fCoulomb; r.fStatic = fStatic; r.betaStatic = betaStatic;
    r.cViscous = cViscous; r.omegaLimitCycle = omegaLimitCycle;
    C.mirrored = true;
    C.dirty = true;
}

void OmniVecIntegrator::subscribeThrCmdForObject(
    size_t obj, Message<THRArrayOnTimeCmdMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null thruster-cmd source.");
    ObjMsgs M = this->msgsFor(obj);
    M.thrCmd->subscribeTo(src);
}

void OmniVecIntegrator::subscribeAtmoForObject(size_t obj,
                                               Message<AtmoPropsMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null atmo source.");
    ObjMsgs M = this->msgsFor(obj);
    M.atmo->subscribeTo(src);
}

void OmniVecIntegrator::subscribeWindForObject(size_t obj,
                                               Message<WindMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null wind source.");
    ObjMsgs M = this->msgsFor(obj);
    M.wind->subscribeTo(src);
}

void OmniVecIntegrator::subscribeEclipseForObject(size_t obj,
                                                  Message<EclipseMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null eclipse source.");
    ObjMsgs M = this->msgsFor(obj);
    M.eclipse->subscribeTo(src);
}

void OmniVecIntegrator::subscribeSunForObject(
    size_t obj, Message<SpicePlanetStateMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null sun-state source.");
    ObjMsgs M = this->msgsFor(obj);
    M.sun->subscribeTo(src);
}

void OmniVecIntegrator::subscribeMoonForObject(
    size_t obj, Message<SpicePlanetStateMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null moon-state source.");
    ObjMsgs M = this->msgsFor(obj);
    M.moon->subscribeTo(src);
}

void OmniVecIntegrator::subscribeMagForObject(
    size_t obj, Message<MagneticFieldMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null mag-field source.");
    ObjMsgs M = this->msgsFor(obj);
    M.mag->subscribeTo(src);
}

size_t OmniVecIntegrator::nStates() const {
    return this->impls_.empty() || !this->impls_[0] ? 0 : this->impls_[0]->nStates;
}
size_t OmniVecIntegrator::nPars() const {
    return this->impls_.empty() || !this->impls_[0]
               ? 0
               : this->impls_[0]->parsBase.size();
}
size_t OmniVecIntegrator::nWheels() const {
    return this->impls_.empty() || !this->impls_[0] ? 0 : this->impls_[0]->nWheels;
}
double OmniVecIntegrator::getBuildSeconds() const { return this->buildSeconds_; }
std::vector<std::string> OmniVecIntegrator::claimedNames() const {
    return this->claimedNamesForObject(0);
}
std::vector<std::string> OmniVecIntegrator::claimedNamesForObject(size_t obj) const {
    if (obj >= this->impls_.size() || !this->impls_[obj]) return {};
    return this->impls_[obj]->claimed;
}

size_t OmniVecIntegrator::nObjects() const { return this->objCfgs_.size(); }

uint64_t OmniVecIntegrator::getStepCountForObject(size_t obj) const {
    return this->cfgAt(obj).stepCount;
}

uint64_t OmniVecIntegrator::getMRPSwitchCountForObject(size_t obj) const {
    return this->cfgAt(obj).mrpCount;
}

double OmniVecIntegrator::getBuildSecondsForObject(size_t obj) const {
    return this->cfgAt(obj).buildSeconds;
}

std::string OmniVecIntegrator::configFingerprintForObject(size_t obj) const {
    return this->calcFingerprint(obj);
}

// --- WP25-A4 slot lifecycle ---------------------------------------------------
void OmniVecIntegrator::ensureSlot(size_t obj) {
    if (this->objCfgs_.empty()) this->objCfgs_.resize(1);
    while (this->objCfgs_.size() <= obj) this->objCfgs_.emplace_back();
    // extraMsgs_[i-1] serves slot i>=1: keep the invariant
    // extraMsgs_.size() == max(0, objCfgs_.size()-1).
    while (this->extraMsgs_.size() + 1 < this->objCfgs_.size())
        this->extraMsgs_.emplace_back();
}

OmniVecIntegrator::ObjCfg& OmniVecIntegrator::cfgFor(size_t obj) {
    this->ensureSlot(obj);
    return this->objCfgs_[obj];
}

const OmniVecIntegrator::ObjCfg& OmniVecIntegrator::cfgAt(size_t obj) const {
    if (obj >= this->objCfgs_.size())
        this->bskLogger.bskError(
            "OmniVecIntegrator: object %zu out of range (%zu slot(s) mirrored).",
            obj, this->objCfgs_.size());
    return this->objCfgs_[obj];
}

OmniVecIntegrator::ObjMsgs OmniVecIntegrator::msgsFor(size_t obj) {
    ObjMsgs M;
    if (obj == 0) {
        M.rwLogs = &this->rwLogInMsgs;
        M.thrCmd = &this->thrOnTimeInMsg;
        M.atmo = &this->atmoDensInMsg;
        M.wind = &this->windVelInMsg;
        M.eclipse = &this->eclipseInMsg;
        M.sun = &this->sunStateInMsg;
        M.moon = &this->moonStateInMsg;
        M.mag = &this->magFieldInMsg;
        return M;
    }
    this->ensureSlot(obj);
    ObjMsgSet& S = this->extraMsgs_[obj - 1];
    M.rwLogs = &S.rwLogs;
    M.thrCmd = &S.thrCmd;
    M.atmo = &S.atmo;
    M.wind = &S.wind;
    M.eclipse = &S.eclipse;
    M.sun = &S.sun;
    M.moon = &S.moon;
    M.mag = &S.mag;
    return M;
}

// Canonical graph-affecting fingerprint for the shared-graph rule. Streams
// every field the symbolic replay reads (fixed order, full double
// precision): equal fingerprints share one heyoka build; ANY structural
// difference (mass, inertia, wheels, friction, thrusters, fuel, env) forces
// per-object graphs. Deliberately EXCLUDED (never graph structure):
// commands (preStaged), message links, tol/compactMode (global JIT/stepping
// options), the stock fuel STATE NAME (name mapping only; the graph's
// canonical state is always "fuelMass" last), and lifecycle/counters.
std::string OmniVecIntegrator::calcFingerprint(size_t obj) const {
    const ObjCfg& C = this->cfgAt(obj);
    std::ostringstream oss;
    oss << std::setprecision(17) << std::scientific;
    oss << "m=" << C.mass << ";I=" << C.Ixx << "," << C.Iyy << "," << C.Izz
        << "," << C.Ixy << "," << C.Ixz << "," << C.Iyz << ";c="
        << C.rBcB[0] << "," << C.rBcB[1] << "," << C.rBcB[2] << ";mu=" << C.mu
        << ";gg=" << (C.useGG ? 1 : 0) << ";nw=" << C.wheels.size() << ";";
    for (const auto& w : C.wheels)
        oss << "w=" << w.gs[0] << "," << w.gs[1] << "," << w.gs[2] << ","
            << w.Js << ";";
    oss << "fric=" << (C.frictionOn ? 1 : 0) << ";";
    for (const auto& f : C.fricRecs)
        oss << "fr=" << f.fCoulomb << "," << f.fStatic << "," << f.betaStatic
            << "," << f.cViscous << "," << f.omegaLimitCycle << ";";
    oss << "nt=" << C.thrDirs.size() << ";";
    for (size_t k = 0; k < C.thrDirs.size(); ++k)
        oss << "t=" << C.thrDirs[k][0] << "," << C.thrDirs[k][1] << ","
            << C.thrDirs[k][2] << "," << C.thrPos[k][0] << ","
            << C.thrPos[k][1] << "," << C.thrPos[k][2] << ","
            << C.thrTmax[k] << ";";
    oss << "fuel=" << (C.hasFuel ? 1 : 0) << ";";
    if (C.hasFuel) {
        oss << "ft=" << C.fuelRTB[0] << "," << C.fuelRTB[1] << ","
            << C.fuelRTB[2] << "," << C.fuelLeak << ";isp=";
        for (double v : C.fuelIsps) oss << v << ",";
        oss << ";";
    }
    oss << "env=" << (C.envConfigured ? 1 : 0) << ";nb=" << C.envBodyMu.size()
        << ";";
    for (size_t k = 0; k < C.envBodyMu.size(); ++k)
        oss << "b=" << C.envBodyMu[k] << "," << C.envBodyOff0[3 * k] << ","
            << C.envBodyOff0[3 * k + 1] << "," << C.envBodyOff0[3 * k + 2]
            << ";";
    oss << "j2=" << (C.j2On ? 1 : 0) << "," << C.envJ2 << ";drag="
        << (C.dragOn ? 1 : 0) << "," << C.envCdA << "," << C.envComOff[0]
        << "," << C.envComOff[1] << "," << C.envComOff[2] << ";nf="
        << C.srpFacets.size() << ";";
    for (const auto& f : C.srpFacets)
        oss << "f=" << f.area << "," << f.nHat[0] << "," << f.nHat[1] << ","
            << f.nHat[2] << "," << f.rCop[0] << "," << f.rCop[1] << ","
            << f.rCop[2] << "," << f.cd << "," << f.cs << ";";
    oss << "cb=" << (C.cannonballOn ? 1 : 0) << "," << C.cbArea << ","
        << C.cbCr << ";sun=" << (C.hasSun ? 1 : 0) << "," << C.sunState[0]
        << "," << C.sunState[1] << "," << C.sunState[2] << ";mag="
        << (C.magOn ? 1 : 0) << "," << C.magM[0] << "," << C.magM[1] << ","
        << C.magM[2] << "," << C.magG10 << "," << C.magG11 << ","
        << C.magH11 << "," << C.maxMtbDipole << ";";
    return oss.str();
}

bool OmniVecIntegrator::needsBuild() const {
    if (this->impls_.size() != this->dynPtrs.size()) return true;
    for (size_t i = 0; i < this->dynPtrs.size(); ++i) {
        if (i >= this->objCfgs_.size()) return true;
        if (!this->impls_[i] || !this->objCfgs_[i].built || this->objCfgs_[i].dirty)
            return true;
    }
    return false;
}

std::vector<std::vector<std::pair<std::string, size_t>>>
OmniVecIntegrator::takeCensus() const {
    std::vector<std::vector<std::pair<std::string, size_t>>> per;
    per.resize(this->dynPtrs.size());
    {
        ExtendedStateVector ev = ExtendedStateVector::fromStates(this->dynPtrs);
        ev.apply([&](const size_t& idx, const std::string& nm,
                     const Eigen::MatrixXd& m) {
            if (idx < per.size())
                per[idx].emplace_back(nm, static_cast<size_t>(m.rows()));
        });
    }
    return per;
}

// --- census + claim + mirror + symbolic build (per object) -------------------
void OmniVecIntegrator::checkSyncGuards() {
    const size_t N = this->dynPtrs.size();
    // WP25-A4: the stage-1 `dynPtrs != 1` hard reject is LIFTED. One object
    // keeps the exact single path (slot 0 through the per-object helpers);
    // every extra dynPtr is a synced follower stepped once per integrate().
    // Loud rejects are preserved for genuinely unmirrorable configs (and
    // gained for sync misconfigurations that would double-step or stall).
    if (N == 0)
        this->bskLogger.bskError(
            "OmniVecIntegrator: no dynamics objects registered; the "
            "integrator was created without a DynamicObject.");
    // Unmirrored-slot check FIRST (per dynPtr): a forgotten follower must
    // name its object, never silently inherit another object's graph. The
    // stale-extra-slot count check follows.
    for (size_t i = 0; i < N; ++i) {
        if (i >= this->objCfgs_.size() || !this->objCfgs_[i].mirrored)
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu has no mirrored config (no "
                "set*/add* call ran for it); mirror it explicitly -- it must "
                "never silently inherit another object's graph.", i);
    }
    if (this->objCfgs_.size() != N)
        this->bskLogger.bskError(
            "OmniVecIntegrator: %zu mirrored config slot(s) for %zu dynamics "
            "object(s); drop the stale extra slot (rebuild the integrator "
            "after the sim's sync set changed) -- slots and dynPtrs must "
            "match exactly.",
            this->objCfgs_.size(), N);
    // Honor isDynamicsSynced (Phase-0 ordering record,
    // swap_p0_ordering.py:47,73): our integrate() runs only via an UNSYNCED
    // owner's integrateState (preIntegration[all] -> integrate ->
    // postIntegration[all]; skipped if isDynamicsSynced). A synced primary
    // would never run (silent stall); more than one unsynced object sharing
    // this integrator would step every object twice per tick (double-step).
    if (this->dyn_ != nullptr && this->dyn_->isDynamicsSynced)
        this->bskLogger.bskError(
            "OmniVecIntegrator: its own DynamicObject is synced to another "
            "integrator (isDynamicsSynced); integrate() would never run -- "
            "install this integrator on the sync PRIMARY instead.");
    {
        size_t nUnsynced = 0;
        for (const auto* d : this->dynPtrs)
            if (d != nullptr && !d->isDynamicsSynced) ++nUnsynced;
        if (nUnsynced > 1)
            this->bskLogger.bskError(
                "OmniVecIntegrator: %zu unsynced objects share one "
                "integrator; each owner's integrateState would step ALL "
                "objects again (double-step). Sync followers via "
                "syncDynamicsIntegration so exactly one owner stays "
                "unsynced.", nUnsynced);
    }
}

void OmniVecIntegrator::buildOnce() {
    // Guards already enforced per-tick by checkSyncGuards(); single caller
    // is integrate().
    const size_t N = this->dynPtrs.size();
    if (this->impls_.size() != N) this->impls_.resize(N);
    const auto census = this->takeCensus();
    // Shared-graph rule: one heyoka build iff every slot's config
    // fingerprint matches; otherwise per-object graphs (probe-first on
    // build cost: per-object build seconds are recorded either way, and
    // the JIT tiers still bind). A decision change dirties every slot
    // (graphs are rebuilt, never aliased across differing configs); a
    // rebuilt slot 0 re-seeds the shared slots' pars template.
    std::vector<std::string> fp(N);
    for (size_t i = 0; i < N; ++i) fp[i] = this->calcFingerprint(i);
    bool sharedNow = (N > 1);
    for (size_t i = 1; sharedNow && i < N; ++i)
        sharedNow = (fp[i] == fp[0]);
    if (sharedNow != this->sharedGraph_) {
        for (auto& c : this->objCfgs_) c.dirty = true;
        this->sharedGraph_ = sharedNow;
    }
    if (this->sharedGraph_ && this->objCfgs_[0].dirty) {
        for (size_t i = 1; i < N; ++i) this->objCfgs_[i].dirty = true;
    }
    if (this->sharedGraph_) {
        if (!this->objCfgs_[0].built || this->objCfgs_[0].dirty)
            this->buildObjectFull(0, census[0]);
        for (size_t i = 1; i < N; ++i) {
            if (!this->objCfgs_[i].built || this->objCfgs_[i].dirty)
                this->buildObjectShared(i, census[i]);
        }
    } else {
        for (size_t i = 0; i < N; ++i) {
            if (!this->objCfgs_[i].built || this->objCfgs_[i].dirty)
                this->buildObjectFull(i, census[i]);
        }
    }
    this->buildSeconds_ = this->objCfgs_[0].buildSeconds;
}

// Per-object census lookup + airtight claim check + live-source grab +
// stochastic loud-reject. Shared by the full and shared-graph builds so a
// genuinely unmirrorable config is loud on every object in both modes.
void OmniVecIntegrator::claimAndGrab(
    size_t obj, const std::vector<std::pair<std::string, size_t>>& census,
    size_t& nW, size_t& nF) {
    ObjCfg& C = this->cfgFor(obj);
    Impl* im = this->impls_[obj].get();
    auto rows_of = [&](const char* nm) -> size_t {
        for (const auto& e : census)
            if (e.first == nm) return e.second;
        return 0;
    };
    // Claim check: hub 4 x 3 (+ optional reactionWheelOmegas Nx1). Anything
    // else present (thetas, fuel mass, ...) is LOUDLY rejected with its name.
    const char* hubNames[4] = {"hubPosition", "hubVelocity", "hubSigma", "hubOmega"};
    for (const char* nm : hubNames) {
        if (rows_of(nm) != 3)
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: required hub state '%s' "
                "missing or not 3x1 (point-mass translational mode is out "
                "of envelope).", obj, nm);
    }
    nW = rows_of("reactionWheelOmegas");
    // Phase-2 fuel: the mirrored stock state name is claimed here (exact
    // match); anything else unclaimed is rejected as before.
    nF = (C.hasFuel && !C.fuelStateName.empty()
          && rows_of(C.fuelStateName.c_str()) == 1)
             ? 1
             : 0;
    if (C.hasFuel && nF == 0)
        this->bskLogger.bskError(
            "OmniVecIntegrator: object %zu: mirrored fuel state '%s' not "
            "registered.", obj, C.fuelStateName.c_str());
    // Gravity DV-reference states (stock pre/postIntegration bookkeeping,
    // refreshed every tick by matchGravitytoVelocityState, zero dynamics):
    // carried through UNCHANGED, never part of the symbolic system.
    const char* passNames[2] = {"hubGravVelocity", "BcGravVelocity"};
    for (const char* nm : passNames) {
        if (rows_of(nm) != 0 && rows_of(nm) != 3)
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: passthrough state '%s' is "
                "not 3x1.", obj, nm);
    }
    for (const auto& e : census) {
        const bool known = (e.first == "hubPosition" || e.first == "hubVelocity"
                            || e.first == "hubSigma" || e.first == "hubOmega"
                            || e.first == "reactionWheelOmegas"
                            || e.first == "hubGravVelocity"
                            || e.first == "BcGravVelocity"
                            || (nF == 1 && e.first == C.fuelStateName));
        if (!known)
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: registered state '%s' (%zu "
                "rows) has no contributor; envelope is hub + balanced "
                "wheels + staircase thrusters + fuel (when mirrored).",
                obj, e.first.c_str(), e.second);
    }
    im->claimed.clear();
    im->claimedRows.clear();
    for (const auto& e : census) {
        im->claimed.push_back(e.first);
        im->claimedRows.push_back(e.second);
    }

    // Cross-check: mirrored wheel records (Python side, stock order) must
    // match the registered omegas rows exactly.
    if (C.wheels.size() != nW)
        this->bskLogger.bskError(
            "OmniVecIntegrator: object %zu: %zu mirrored wheels but "
            "reactionWheelOmegas has %zu rows; mirror_stock_config missed "
            "wheels (or none attached).",
            obj, C.wheels.size(), nW);
    im->nWheels = nW;

    // Cache live StateData sources (fixed after InitializeSimulation).
    DynamicObject* dyn = this->dynPtrs[obj];
    auto grab = [&](const char* nm) -> StateData* {
        StateData* s = dyn->dynManager.getStateObject(std::string(nm));
        if (s == nullptr)
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: state '%s' vanished.", obj, nm);
        return s;
    };
    im->hubR = grab("hubPosition");
    im->hubV = grab("hubVelocity");
    im->hubS = grab("hubSigma");
    im->hubO = grab("hubOmega");
    im->omegas = (nW > 0) ? grab("reactionWheelOmegas") : nullptr;
    im->fuelSrc = (nF > 0) ? grab(C.fuelStateName.c_str()) : nullptr;
    // Passthrough sources (absent without gravity bodies; census-gated so
    // no spurious missing-name warnings).
    {
        const auto has = [&](const char* nm) {
            for (const auto& e : census)
                if (e.first == nm) return true;
            return false;
        };
        im->passSrc[0] = has("hubGravVelocity") ? grab("hubGravVelocity") : nullptr;
        im->passSrc[1] = has("BcGravVelocity") ? grab("BcGravVelocity") : nullptr;
    }

    // WP25-B2.4: stochastic/diffusion loud-reject at setup. The state-NAME
    // claim check above is insufficient by construction (diffusion attaches
    // to the SAME names via StateData::setNumNoiseSources), so inspect
    // enablement directly on every cached source: any noise source means SDE
    // dynamics the Taylor propagator cannot follow (PLAN_INTEGRATOR_SWAP.md
    // envelope: stochastic permanently out). Raises BEFORE first propagation.
    {
        const StateData* stochSrc[8] = {im->hubR, im->hubV, im->hubS, im->hubO,
                                        im->omegas, im->fuelSrc,
                                        im->passSrc[0], im->passSrc[1]};
        for (const StateData* s : stochSrc) {
            if (s == nullptr) continue;
            const size_t nSrc = s->getNumNoiseSources();
            if (nSrc > 0)
                this->bskLogger.bskError(
                    "OmniVecIntegrator: object %zu: state '%s' has %zu noise "
                    "source(s); stochastic diffusion is out of envelope "
                    "(Taylor != SDE, PLAN_INTEGRATOR_SWAP.md); run without "
                    "diffusion.",
                    obj, s->getName().c_str(), nSrc);
        }
    }
}

void OmniVecIntegrator::buildObjectFull(
    size_t obj, const std::vector<std::pair<std::string, size_t>>& census) {
    const auto t0 = std::chrono::steady_clock::now();
    ObjCfg& C = this->cfgFor(obj);
    this->impls_[obj] = std::make_unique<Impl>();
    Impl* im = this->impls_[obj].get();

    size_t nW = 0, nF = 0;
    this->claimAndGrab(obj, census, nW, nF);

    // Mirror into a fresh OmniSystem (order = stock order by construction).
    // Hub record takes the RAW hub offset (input I/mass are hub-dry; the
    // aggregate lives in per-tick pars + live B/C mapping, never in records).
    im->sys.set_gravity(C.mu);
    im->sys.set_use_gravity_gradient(C.useGG);
    im->sys.add_hub(C.mass,
                    {C.Ixx, C.Ixy, C.Ixz,
                     C.Ixy, C.Iyy, C.Iyz,
                     C.Ixz, C.Iyz, C.Izz},
                    {C.rBcB[0], C.rBcB[1], C.rBcB[2]});
    for (const auto& w : C.wheels)
        im->sys.add_reaction_wheel({w.gs[0], w.gs[1], w.gs[2]}, w.Js, 0.0);
    // Fuel LAST (documented layout).
    if (nF > 0) im->sys.add_fuel_tank();
    // Stage-2 thruster cluster (staircase only; timing models rejected
    // Python-side). Geometry arrives via addThruster records (joined
    // Python-side); thrusters add no states (census unchanged).
    if (!C.thrDirs.empty())
        im->sys.add_thrusters(C.thrDirs, C.thrPos, C.thrTmax);
    // WP5 env replay: bodies + J2 feed the graph (g_N channel); drag/SRP/mag
    // geometry lives module-side and enters via slots (A3). Any configured
    // effect registers the single 'e' cluster (same replay as the module).
    if (C.envConfigured) {
        for (size_t k = 0; k < C.envBodyMu.size(); ++k) {
            im->sys.addEnvironmentBody(C.envBodyMu[k],
                                       C.envBodyOff0[3 * k],
                                       C.envBodyOff0[3 * k + 1],
                                       C.envBodyOff0[3 * k + 2]);
        }
        if (C.j2On) im->sys.setJ2(C.envJ2);
        if (C.dragOn)
            im->sys.setDragParams(C.envCdA, C.envComOff[0],
                                  C.envComOff[1], C.envComOff[2]);
        for (const auto& f : C.srpFacets)
            im->sys.addSRPFacet(f.area, f.nHat[0], f.nHat[1], f.nHat[2],
                                f.rCop[0], f.rCop[1], f.rCop[2], f.cd, f.cs);
        if (C.cannonballOn) im->sys.setSRPCannonball(C.cbArea, C.cbCr);
        if (C.hasSun)
            im->sys.setSunState(C.sunState[0], C.sunState[1],
                                C.sunState[2]);
        if (C.magOn)
            im->sys.setMagneticMoment(C.magM[0], C.magM[1],
                                      C.magM[2]);
        im->sys.setMagneticDipole(C.magG10, C.magG11, C.magH11);
    }

    const size_t n = im->sys.n_states();
    if (n != 12 + nW + nF)
        this->bskLogger.bskError(
            "OmniVecIntegrator: object %zu: mirrored system has %zu states "
            "for %zu stock states; layout mismatch.", obj, n, 12 + nW + nF);
    im->nStates = n;
    im->fuelPos = 12 + nW;  // fuel always last (A1)
    if (nF > 0) {
        // Name-resolved like the rest (A1); must agree with the layout rule.
        size_t fp = 0;
        bool found = false;
        for (const auto& nm : im->sys.state_names()) {
            if (nm == "fuelMass") { found = true; break; }
            ++fp;
        }
        if (!found || fp != im->fuelPos)
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: fuel state layout broken.", obj);
    }
    // Name-based par slots for the per-tick fuel feedback (A2).
    if (nF > 0) {
        // WP25-C2 M14: a missing/short Isp list must RAISE at build, never
        // read as silent 0.0 (which would undercount mdot while thrust
        // fires). Leak-only rigs carry no thrusters, so they are exempt.
        if (!C.thrTmax.empty() && C.fuelIsps.size() != C.thrTmax.size())
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: %zu thrusters but %zu Isp "
                "entries; set per-thruster Isps (no silent 0.0).",
                obj, C.thrTmax.size(), C.fuelIsps.size());
        im->mdotOff = im->sys.par_index("mdot");
        im->massOff = im->sys.par_index("mass");
        im->IO[0] = im->sys.par_index("Ixx");
        im->IO[1] = im->sys.par_index("Iyy");
        im->IO[2] = im->sys.par_index("Izz");
        im->IO[3] = im->sys.par_index("Ixy");
        im->IO[4] = im->sys.par_index("Ixz");
        im->IO[5] = im->sys.par_index("Iyz");
    }
    // Thruster cB slots, per axis (masked-out axes have no slot; the
    // contributor reads the structural literal for those -- exact).
    if (!C.thrDirs.empty()) {
        static const char* kCN[3] = {"cBx", "cBy", "cBz"};
        for (int j = 0; j < 3; ++j)
            if (im->sys.has_par(kCN[j])) im->cOff[j] = im->sys.par_index(kCN[j]);
    }
    // Resolve canonical offsets by NAME (A1/A2); hub-first-12 is the
    // derived layout here, verified against state_names(), not assumed.
    // (Must run AFTER registration: state_names() refreshes the system.)
    {
        const std::vector<std::string> sn = im->sys.state_names();
        auto posOf = [&](const char* nm) -> size_t {
            for (size_t k = 0; k < sn.size(); ++k)
                if (sn[k] == nm) return k;
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: state '%s' missing.", obj, nm);
            return 0;
        };
        im->rOff = posOf("rx"); im->vOff = posOf("vx");
        im->sOff = posOf("s1"); im->oOff = posOf("omx");
        if (nW > 0) im->wOff = posOf("W1");
    }
    const auto sysrows = im->sys.build_symbolic();
    im->parsBase = im->sys.default_pars();
    im->wheelIn = im->sys.wheel_input_offsets();
    im->fricIn = im->sys.friction_input_offsets();
    {
        const auto es = im->sys.env_slot_offsets();
        im->hasEnv = es.has;
        im->envBody = es.body;
        im->envDrag = es.drag;
        im->envSrp = es.srp;
        im->envMag = es.mag;
        im->envNBodies = es.nBodies;
    }
    std::vector<double> y0(n, 0.0);
    // Suffix-drop geometry-degenerate trailing pars (throws on structural
    // gaps); ta owns exactly ta_npars: EVERY per-tick push must be bounded
    // by it (heap safety).
    im->ta_npars = omni::OmniSystem::checked_par_count(sysrows, im->parsBase.size());
    std::vector<double> pTrunc(im->parsBase.begin(),
                               im->parsBase.begin() + im->ta_npars);
    im->ta.emplace(sysrows, y0,
                   heyoka::kw::pars = pTrunc,
                   heyoka::kw::tol = this->tol_,
                   heyoka::kw::compact_mode = this->compactMode_,
                   heyoka::kw::high_accuracy = false);

    const auto t1 = std::chrono::steady_clock::now();
    C.buildSeconds = std::chrono::duration<double>(t1 - t0).count();
    C.built = true;
    C.dirty = false;
    this->bskLogger.bskLog(BSK_INFORMATION,
                           "OmniVecIntegrator: object %zu: %zu states, %zu pars, "
                           "%zu wheels; symbolic build %.2f ms, tol %g.",
                           obj, n, im->parsBase.size(), nW,
                           C.buildSeconds * 1e3, this->tol_);
}

// Shared-graph follower build: full census/claim/grab/diffusion checks (same
// loud rejects as the full build), but NO symbolic work -- the object steps
// through slot 0's graph with its own live pars/sources. Only valid because
// the fingerprints matched exactly (same states, same par layout); the
// copied offsets are re-verified against the object's own census below.
void OmniVecIntegrator::buildObjectShared(
    size_t obj, const std::vector<std::pair<std::string, size_t>>& census) {
    const auto t0 = std::chrono::steady_clock::now();
    ObjCfg& C = this->cfgFor(obj);
    Impl* G = this->impls_[0].get();
    if (G == nullptr || !G->ta.has_value())
        this->bskLogger.bskError(
            "OmniVecIntegrator: object %zu: shared-graph owner (slot 0) has "
            "no built graph; build order broken.", obj);
    this->impls_[obj] = std::make_unique<Impl>();
    Impl* im = this->impls_[obj].get();

    size_t nW = 0, nF = 0;
    this->claimAndGrab(obj, census, nW, nF);

    // Identical configs => identical graphs: copy the resolved layout and a
    // pristine pars template (default_pars, never the owner's live vector).
    // The per-tick writes below overwrite every live slot before first use,
    // exactly as in the full build.
    im->nStates = G->nStates;
    im->nWheels = nW;
    im->fuelPos = G->fuelPos;
    im->mdotOff = G->mdotOff;
    im->massOff = G->massOff;
    for (int k = 0; k < 6; ++k) im->IO[k] = G->IO[k];
    for (int j = 0; j < 3; ++j) im->cOff[j] = G->cOff[j];
    im->rOff = G->rOff; im->vOff = G->vOff;
    im->sOff = G->sOff; im->oOff = G->oOff; im->wOff = G->wOff;
    im->parsBase = G->sys.default_pars();
    im->wheelIn = G->wheelIn;
    im->fricIn = G->fricIn;
    im->ta_npars = G->ta_npars;
    im->hasEnv = G->hasEnv;
    im->envBody = G->envBody;
    im->envDrag = G->envDrag;
    im->envSrp = G->envSrp;
    im->envMag = G->envMag;
    im->envNBodies = G->envNBodies;
    // sys/ta stay empty on purpose: stepObject() routes graph calls to the
    // owner. Defense in depth: the copied layout must still describe this
    // object's own census (fingerprints matched, so it does -- prove it).
    if (im->nStates != 12 + nW + nF)
        this->bskLogger.bskError(
            "OmniVecIntegrator: object %zu: shared layout has %zu states "
            "for %zu stock states; fingerprint collision, rebuild split.",
            obj, im->nStates, 12 + nW + nF);
    if (im->parsBase.size() != G->parsBase.size())
        this->bskLogger.bskError(
            "OmniVecIntegrator: object %zu: shared pars template size "
            "mismatch; rebuild split.", obj);

    const auto t1 = std::chrono::steady_clock::now();
    C.buildSeconds = std::chrono::duration<double>(t1 - t0).count();
    C.built = true;
    C.dirty = false;
    this->bskLogger.bskLog(BSK_INFORMATION,
                           "OmniVecIntegrator: object %zu: shared graph "
                           "(owner slot 0), %zu states; attach %.2f ms.",
                           obj, im->nStates, C.buildSeconds * 1e3);
}

// WP5 per-tick (/per-arc) env refresh. y = gathered omni-order state
// (hub-first-12, A1); P = live par vector. Module-side doubles only (A3);
// TRUE laws from omni::env. P1 truth table enforced (density/sun/MTB link
// required when on; eclipse unlinked -> 1.0; wind unlinked -> inertial).
// Ported 1:1 from OmniSpacecraft::updateEnvInputs (same validated logic);
// the only difference is member routing (integrator owns no telemetry).
// WP25-A4: routed per object (own messages, own env config, own offsets).
void OmniVecIntegrator::updateEnvInputs(size_t obj, const double* y,
                                        std::vector<double>& P,
                                        const double cLive[3]) {
    ObjCfg& C = this->cfgFor(obj);
    Impl* O = this->impls_[obj].get();
    ObjMsgs M = this->msgsFor(obj);
    if (y == nullptr || !O->hasEnv) return;
    const size_t bO = O->envBody, dO = O->envDrag;
    const size_t sO = O->envSrp, mO = O->envMag;
    const size_t nB = O->envNBodies;
    const size_t nEnv = 3 * nB + 15;
    for (size_t k = 0; k < nEnv; ++k)
        if (bO + k < P.size()) P[bO + k] = 0.0;
    const size_t rO = O->rOff, vO = O->vOff, sgO = O->sOff;
    omni::env::Vec3 r{y[rO], y[rO + 1], y[rO + 2]};
    omni::env::Vec3 v{y[vO], y[vO + 1], y[vO + 2]};
    omni::env::Vec3 sig{y[sgO], y[sgO + 1], y[sgO + 2]};
    const omni::env::Mat3 R = omni::env::dcmBN_from_sigma(sig);

    // N-body offsets: refreshed PER TICK from msgs when linked (never frozen),
    // else the recorded static/initial offset (bench/fixture path).
    for (size_t i = 0; i < nB && i < C.envBodyMu.size(); ++i) {
        omni::env::Vec3 off{C.envBodyOff0[3 * i], C.envBodyOff0[3 * i + 1],
                            C.envBodyOff0[3 * i + 2]};
        if (i == 0 && M.sun->isLinked()) {
            const SpicePlanetStateMsgPayload sun = (*M.sun)();
            off = {sun.PositionVector[0], sun.PositionVector[1], sun.PositionVector[2]};
        }
        if (i == 1 && M.moon->isLinked()) {
            const SpicePlanetStateMsgPayload moon = (*M.moon)();
            off = {moon.PositionVector[0], moon.PositionVector[1], moon.PositionVector[2]};
        }
        if (bO + 3 * i + 2 < P.size()) {
            P[bO + 3 * i] = off[0]; P[bO + 3 * i + 1] = off[1]; P[bO + 3 * i + 2] = off[2];
        }
    }

    if (C.dragOn) {
        if (!M.atmo->isLinked()) {
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: drag on with atmoDensInMsg "
                "unlinked (stock bskError).", obj);
        } else {
            const AtmoPropsMsgPayload atmo = (*M.atmo)();
            omni::env::Vec3 vAir{0.0, 0.0, 0.0};
            if (M.wind->isLinked()) {
                const WindMsgPayload wind = (*M.wind)();
                vAir = {wind.v_air_N[0], wind.v_air_N[1], wind.v_air_N[2]};
            }
            const omni::env::Vec3 vRelB = omni::env::matvec_BN(
                R, omni::env::vec_sub(v, vAir));
            // Measured split (combo probe): drag arms do NOT shift.
            const omni::env::Vec3 dragArm{C.envComOff[0],
                                          C.envComOff[1],
                                          C.envComOff[2]};
            const omni::env::WrenchB w = omni::env::dragWrench(
                atmo.neutralDensity, C.envCdA, vRelB, dragArm, 0.0);
            if (dO + 5 < P.size()) {
                P[dO] = w.F[0]; P[dO + 1] = w.F[1]; P[dO + 2] = w.F[2];
                P[dO + 3] = w.T[0]; P[dO + 4] = w.T[1]; P[dO + 5] = w.T[2];
            }
        }
    }

    const bool srpOn = C.cannonballOn || !C.srpFacets.empty();
    if (srpOn) {
        const bool sunLinked = M.sun->isLinked();
        if (!sunLinked && !C.hasSun) {
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: SRP on with sun state "
                "unlinked (stock bskError).", obj);
        } else {
            omni::env::Vec3 sunPos = {C.sunState[0], C.sunState[1],
                                      C.sunState[2]};
            if (sunLinked) {
                const SpicePlanetStateMsgPayload sun = (*M.sun)();
                sunPos = {sun.PositionVector[0], sun.PositionVector[1], sun.PositionVector[2]};
            }
            const omni::env::Vec3 rSB = omni::env::vec_sub(sunPos, r);
            const double d = omni::env::vec_norm(rSB);
            if (d > 0.0 && sO + 5 < P.size()) {
                const double Pflx = omni::env_const::P_SRP_1AU *
                    (omni::env_const::AU_M / d) * (omni::env_const::AU_M / d);
                const omni::env::Vec3 sHatB = omni::env::matvec_BN(
                    R, omni::env::vec_scale(rSB, 1.0 / d));
                double shadow = 1.0;
                if (M.eclipse->isLinked())
                    shadow = (*M.eclipse)().shadowFactor;
                omni::env::Vec3 F{0.0, 0.0, 0.0}, T{0.0, 0.0, 0.0};
                if (C.cannonballOn) {
                    F = omni::env::vec_add(
                        F, omni::env::srpCannonball(C.cbArea, C.cbCr, Pflx,
                                                   shadow, sHatB));
                }
                for (const auto& f : C.srpFacets) {
                    // Measured split: facet arms shift by live aggregate c
                    // ((rCop - c) x F); drag arms do NOT shift (envComOff_).
                    const omni::env::WrenchB w = omni::env::srpFacet(
                        f.area, {f.nHat[0], f.nHat[1], f.nHat[2]},
                        {f.rCop[0] - cLive[0],
                         f.rCop[1] - cLive[1],
                         f.rCop[2] - cLive[2]}, f.cd, f.cs, Pflx,
                        shadow, sHatB);
                    F = omni::env::vec_add(F, w.F);
                    T = omni::env::vec_add(T, w.T);
                }
                P[sO] = F[0]; P[sO + 1] = F[1]; P[sO + 2] = F[2];
                P[sO + 3] = T[0]; P[sO + 4] = T[1]; P[sO + 5] = T[2];
            }
        }
    }

    if (C.magOn) {
        const bool mZero = (C.magM[0] == 0.0 && C.magM[1] == 0.0 &&
                            C.magM[2] == 0.0);
        if (!M.mag->isLinked()) {
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: mag on with magField msg "
                "unlinked (stock bskError).", obj);
        } else if (mZero) {
            this->bskLogger.bskError(
                "OmniVecIntegrator: object %zu: mag on with zero dipole (MTB "
                "cmd/params unlinked).", obj);
        } else {
            const MagneticFieldMsgPayload mag = (*M.mag)();
            const omni::env::Vec3 BN{mag.magField_N[0], mag.magField_N[1], mag.magField_N[2]};
            const omni::env::Vec3 BB = omni::env::matvec_BN(R, BN);
            omni::env::Vec3 m{C.magM[0], C.magM[1], C.magM[2]};
            const double mn = omni::env::vec_norm(m);
            if (mn > C.maxMtbDipole && mn > 0.0)
                m = omni::env::vec_scale(m, C.maxMtbDipole / mn);
            const omni::env::Vec3 tau = omni::env::magTorque(m, BB);
            if (mO + 2 < P.size()) { P[mO] = tau[0]; P[mO + 1] = tau[1]; P[mO + 2] = tau[2]; }
        }
    }
}

void OmniVecIntegrator::integrate(double /*currentTime*/, double timeStep) {
    // Sync guards run EVERY tick (cheap integer/flag compares): a second
    // dynamics object appearing mid-run, an unmirrored slot, or a
    // double-step-prone ownership change must fail loudly here, never run
    // against stale records.
    this->checkSyncGuards();
    if (this->needsBuild()) this->buildOnce();
    if (!(timeStep > 0.0) || !std::isfinite(timeStep)) return;

    // WP25-B2.4: stock-side freeze, per object (see freezeObject).
    const auto census = this->takeCensus();
    for (size_t i = 0; i < this->dynPtrs.size(); ++i)
        this->freezeObject(i, census[i]);

    // Phase-0 ordering (swap_p0_ordering.py:47,73): this single call steps
    // EVERY dynPtr exactly once -- the leader's integrateState covers synced
    // followers, and each follower's own integrateState returns early, so
    // stock never invokes us twice per tick. Never one integrate() call per
    // object: that would double-step. The per-object step counters below
    // back the no-double-step audit (steps[i] == ticks for every object).
    for (size_t i = 0; i < this->dynPtrs.size(); ++i)
        this->stepObject(i, timeStep);
    ++this->integrateCalls_;
}

// WP25-B2.4: stock-side freeze. Records freeze at build (first integrate,
// post-InitializeSimulation); stock-side edits after that (new/removed
// states, a second dynamics object) would run against stale records, so
// detect here and reject LOUDLY instead of rebuilding (detect-or-rebuild:
// detect). A normal stop/restart with an unchanged census passes silently.
// Compared as a SET per object (index-keyed census: same-named states on
// two objects never collide); the single-object verdict is unchanged.
void OmniVecIntegrator::freezeObject(
    size_t obj, const std::vector<std::pair<std::string, size_t>>& census) {
    Impl* im = this->impls_[obj].get();
    std::vector<std::pair<std::string, size_t>> now = census;
    std::vector<std::pair<std::string, size_t>> want;
    for (size_t k = 0; k < im->claimed.size() && k < im->claimedRows.size(); ++k)
        want.emplace_back(im->claimed[k], im->claimedRows[k]);
    std::sort(now.begin(), now.end());
    std::sort(want.begin(), want.end());
    if (now != want)
        this->bskLogger.bskError(
            "OmniVecIntegrator: stock state census changed after the build "
            "freeze (records freeze at first integrate); stock-side edits "
            "post-InitializeSimulation are rejected -- rebuild the "
            "integrator instead.");
}

void OmniVecIntegrator::stepObject(size_t obj, double timeStep) {
    ObjCfg& C = this->cfgFor(obj);
    Impl* im = this->impls_[obj].get();
    // Shared graph: slot 0 owns sys/ta (stepped sequentially per object);
    // live pars, sources, and offsets still come from THIS object (copies
    // verified against its own census at build).
    Impl* gr = (this->sharedGraph_ && obj > 0) ? this->impls_[0].get() : im;
    ObjMsgs M = this->msgsFor(obj);

    // Live aggregate c_B this tick (fuel ? formula : verbatim record, so the
    // no-fuel path is bit-identical to the static record). Used for the B/C
    // boundary, I_C feedback and thruster arms below -- no rebuilds, no lag.
    double cLive[3] = {C.rBcB[0], C.rBcB[1], C.rBcB[2]};
    double mFuelLive = 0.0;
    if (im->fuelSrc != nullptr) {
        mFuelLive = im->fuelSrc->getState()(0, 0);
        if (!(mFuelLive >= 0.0) || !std::isfinite(mFuelLive))
            mFuelLive = 0.0;
        const double mT = C.mass + mFuelLive;
        if (mT > 0.0) {
            for (int i = 0; i < 3; ++i)
                cLive[i] = (C.mass * C.rBcB[i]
                            + mFuelLive * C.fuelRTB[i]) / mT;
        }
    }

    // WP4 B/C boundary: the graph integrates C-point states; stock hub
    // states are B-point. Transform at the boundary with the LIVE aggregate
    // c_B (cLive: exact record without fuel, fuel-weighted with fuel).
    // MEASURED stock convention (offset probe, BSK 2.12.0:
    // r_BN - r_CN = -c exactly):
    //   r_BN = r_CN - R^T c,  v_BN = v_CN - R^T (w x c),  R = dcm_BN(sigma),
    // i.e. r_BcB_B points B->C. (DESIGN/test_wp4 pin the opposite sign from
    // self-consistent math only -- flagged for SysModel-path audit; the swap
    // follows the stock measurement.)
    // c = 0 reduces to exact no-ops (zero-offset regression bit-identical).
    const omni::env::Vec3 cB{cLive[0], cLive[1], cLive[2]};
    auto toInertial = [&](const omni::env::Mat3& R, const omni::env::Vec3& vB) {
        omni::env::Vec3 vN{};
        for (int i = 0; i < 3; ++i)
            vN[i] = R[0][i] * vB[0] + R[1][i] * vB[1] + R[2][i] * vB[2];
        return vN;
    };

    // Gather in omni canonical order: r, v, sigma, omega, wheels.
    const size_t nW = im->nWheels;
    std::vector<double> y;
    y.reserve(12 + nW);
    // Stock B-point hub states first, then convert to C-point.
    omni::env::Vec3 rB{}, vB{}, sB{}, wB{};
    {
        const Eigen::MatrixXd mr = im->hubR->getState();
        const Eigen::MatrixXd mv = im->hubV->getState();
        const Eigen::MatrixXd ms = im->hubS->getState();
        const Eigen::MatrixXd mo = im->hubO->getState();
        for (int i = 0; i < 3; ++i) {
            rB[i] = mr(i, 0); vB[i] = mv(i, 0);
            sB[i] = ms(i, 0); wB[i] = mo(i, 0);
        }
    }
    {
        const omni::env::Mat3 R = omni::env::dcmBN_from_sigma(sB);
        const omni::env::Vec3 Rc = toInertial(R, cB);
        const omni::env::Vec3 wxc = omni::env::vec_cross(wB, cB);
        const omni::env::Vec3 Rwxc = toInertial(R, wxc);
        for (int i = 0; i < 3; ++i) y.push_back(rB[i] + Rc[i]);
        for (int i = 0; i < 3; ++i) y.push_back(vB[i] + Rwxc[i]);
        for (int i = 0; i < 3; ++i) y.push_back(sB[i]);
        for (int i = 0; i < 3; ++i) y.push_back(wB[i]);
    }
    std::vector<double> passVals;
    for (int k = 0; k < 2; ++k) {
        if (im->passSrc[k] == nullptr) continue;
        const Eigen::MatrixXd m = im->passSrc[k]->getState();
        passVals.push_back(m(0, 0));
        passVals.push_back(m(1, 0));
        passVals.push_back(m(2, 0));
    }
    if (nW > 0) {
        const Eigen::MatrixXd m = im->omegas->getState();
        for (size_t i = 0; i < nW; ++i) y.push_back(m(static_cast<int>(i), 0));
    }
    // Fuel mass LAST (documented layout; stock name mapped here).
    if (im->fuelSrc != nullptr) {
        const Eigen::MatrixXd m = im->fuelSrc->getState();
        y.push_back(m(0, 0));
    }

    // RW motor torques: read back from the stock wheel-log messages, which
    // the stock RW UpdateState publishes from the SAME u_current the stock
    // integrator consumes (post ConfigureRWRequests saturation: u_max clamp,
    // u_min deadband, P_max foldback, Omega_max cut). Timing-identical by
    // construction under every task order; unlinked slots read 0 exactly
    // like a never-configured stock wheel. Graph purity (A3): module-side.
    std::vector<double>& P = im->parsBase;
    for (size_t i = 0; i < nW && i < im->wheelIn.size(); ++i) {
        double v = 0.0;
        if (i < M.rwLogs->size() && (*M.rwLogs)[i].isLinked())
            v = (*M.rwLogs)[i]().u_current;
        const size_t o = im->wheelIn[i];
        if (o < P.size()) P[o] = v;
    }
    // Phase-2 friction (Stribeck, stock-exact incl. stribeck flag + limit
    // cycle + omegaBefore state; module-side doubles, A3-clean: tanh lives
    // here, never in the graph). Off (default) writes identical zeros.
    {
        // Live wheel speeds in stock order (gathered above into y).
        for (size_t i = 0; i < nW && i < im->fricIn.size(); ++i) {
            const size_t fo = im->fricIn[i];
            if (fo >= P.size()) continue;
            double tauF = 0.0;
            if (C.frictionOn && i < C.fricRecs.size()
                && i < C.fricState.size()) {
                const FricRec& fr = C.fricRecs[i];
                FricState& st = C.fricState[i];
                const double W = y[im->wOff + i];
                if (std::fabs(W) < 0.10 * fr.omegaLimitCycle && fr.betaStatic > 0.0)
                    st.stribeck = 1;
                const double signO = ((W > 0.0) ? 1.0 : 0.0) - ((W < 0.0) ? 1.0 : 0.0);
                const double omegaDot = W - st.omegaBefore;
                const double signOD = ((omegaDot > 0.0) ? 1.0 : 0.0)
                                    - ((omegaDot < 0.0) ? 1.0 : 0.0);
                if (st.stribeck == 1 && std::fabs(signO - signOD) < 2.0
                    && fr.betaStatic > 0.0) {
                    st.stribeck = 1;
                } else {
                    st.stribeck = 0;
                }
                double frictionForce;
                double frictionForceAtLimitCycle;
                if (st.stribeck == 1) {
                    const double e = std::exp(-(W / fr.betaStatic) * (W / fr.betaStatic) / 2.0);
                    const double k = std::sqrt(2.0 * std::exp(1.0));
                    frictionForce = k * (fr.fStatic - fr.fCoulomb) * e * W
                                        / (fr.betaStatic * std::sqrt(2.0))
                                    + fr.fCoulomb * std::tanh(W * 10.0 / fr.betaStatic)
                                    + fr.cViscous * W;
                    const double oL = fr.omegaLimitCycle;
                    const double eL = std::exp(-(oL / fr.betaStatic) * (oL / fr.betaStatic) / 2.0);
                    frictionForceAtLimitCycle =
                        k * (fr.fStatic - fr.fCoulomb) * eL * oL / (fr.betaStatic * std::sqrt(2.0))
                        + fr.fCoulomb * std::tanh(oL * 10.0 / fr.betaStatic)
                        + fr.cViscous * oL;
                } else {
                    frictionForce = signO * fr.fCoulomb + fr.cViscous * W;
                    frictionForceAtLimitCycle =
                        fr.fCoulomb + fr.cViscous * fr.omegaLimitCycle;
                }
                if (std::fabs(W) < fr.omegaLimitCycle && fr.omegaLimitCycle > 0.0)
                    frictionForce = frictionForceAtLimitCycle / fr.omegaLimitCycle * W;
                tauF = -frictionForce;
                st.omegaBefore = W;
            }
            P[fo] = tauF;
        }
    }
    // WP25-C2 M15 (was stale "friction slots stay 0"): friction slots carry
    // the live stock-exact Stribeck tauF computed above (module-side
    // doubles); friction-off (default) writes identical zeros.

    // WP5 per-tick env refresh (bodies never frozen; P1 truth table inside).
    if (im->hasEnv) this->updateEnvInputs(obj, y.data(), P, cLive);

    // Thruster channel (stage 2, staircase only): edge-triggered mailbox.
    // Fire rule (measured stock cells: fresh/write-once/frozen/advance):
    // tick 0 fires the PRE-STAGED snapshot (Reset-equivalent staging:
    // pre-init command fires, Reset-zeros fill idles); later ticks fire
    // the current message iff first-sight or timestamp strictly advanced
    // (frozen ticks idle). WP3b timing models rejected Python-side; the
    // graph sees q-slots only (A3).
    {
        const size_t nt = C.thrDirs.size();
        std::vector<double> openT(nt, 0.0);
        if (nt > 0) {
            if (C.havePreStaged) {
                if (!C.preStaged.empty())
                    openT = gr->sys.thruster_on_times(timeStep, C.preStaged.data(),
                                                      C.preStaged.size());
                C.havePreStaged = false;
                C.preStaged.clear();
                // Seed the edge detector at the current stamp so the
                // pre-init write is not re-fired as "first sight" (write-once
                // fires tick 0 exactly once).
                if (M.thrCmd->isLinked())
                    C.lastThrStamp = static_cast<int64_t>(
                        M.thrCmd->timeWritten());
            } else if (M.thrCmd->isLinked()) {
                const int64_t stamp =
                    static_cast<int64_t>(M.thrCmd->timeWritten());
                if (stamp > C.lastThrStamp) {
                    const THRArrayOnTimeCmdMsgPayload msg = (*M.thrCmd)();
                    constexpr size_t NAVAIL =
                        sizeof(msg.OnTimeRequest) / sizeof(msg.OnTimeRequest[0]);
                    std::vector<double> staged(std::min(nt, NAVAIL), 0.0);
                    for (size_t i = 0; i < staged.size(); ++i)
                        staged[i] = msg.OnTimeRequest[i];
                    openT = gr->sys.thruster_on_times(timeStep, staged.data(),
                                                      staged.size());
                    C.lastThrStamp = stamp;
                }
            }
        }
        // Phase-2 fuel per-tick update (needs openT duties from staging above,
        // runs before the arc loop; pars pushed per-arc below).
        // mdot = sum over thrusters duty*Tmax/(Isp*g0) + leak, clamped so the
        // tank never crosses zero (module-side doubles, A3); hub mass/I pars
        // overwritten from live fuel (m = mDry + mFuel, I = I_dry + mFuel*K).
        // WP25-C2 M15 (was stale "c_B stays build-time static"): the live
        // aggregate c tracks depletion per tick (cOff slots below + B/C).
        if (im->fuelSrc != nullptr && timeStep > 0.0) {
            constexpr double G0 = 9.80665;
            const double mFuel = y[im->fuelPos];
            double mdot = C.fuelLeak;
            for (size_t j = 0; j < openT.size() && j < C.thrTmax.size(); ++j) {
                const double duty = (timeStep > 0.0) ? openT[j] / timeStep : 0.0;
                // WP25-C2 M14: the 0.0 fallback is unreachable-by-construction
                // when fuel+thrusters are mirrored (build raises on a short
                // Isp list); it survives only as defensive indexing.
                const double isp = (j < C.fuelIsps.size()) ? C.fuelIsps[j] : 0.0;
                if (isp > 0.0 && duty > 0.0)
                    mdot += duty * C.thrTmax[j] / (isp * G0);
            }
            if (mdot < 0.0) mdot = 0.0;
            const double mdotMax = mFuel / timeStep;
            if (mdot > mdotMax) mdot = (mdotMax > 0.0) ? mdotMax : 0.0;
            // WP25-C2 M14: <P.size() guards mirror the cOff guard discipline
            // (offsets come from par_index; ta may own a suffix-truncated
            // par vector, so every per-tick push is heap-bounded).
            if (im->mdotOff < P.size()) P[im->mdotOff] = mdot;
            if (im->massOff < P.size()) P[im->massOff] = C.mass + mFuel;
            // Inertia about TRUE C (not B!): I_C = I_hub_Bc
            //   + mHub*PA(rBcB - c) + mFuel*PA(rTB - c), c = live aggregate.
            // Using I_B (about B) instead is wrong by M*(|c|^2I - cc^T);
            // with c = 5 mm that is 0.05/900-scale in D and costs 0.2 m +
            // 2e-2 rad over a 600 s burn (measured). No-fuel and centered
            // cases reduce exactly (PA(0) = 0). Packed Ixx..Iyz order.
            {
                const double cx = cLive[0], cy = cLive[1], cz = cLive[2];
                const double hx = C.rBcB[0] - cx,
                             hy = C.rBcB[1] - cy,
                             hz = C.rBcB[2] - cz;
                const double fx = C.fuelRTB[0] - cx,
                             fy = C.fuelRTB[1] - cy,
                             fz = C.fuelRTB[2] - cz;
                const double hPA[6] = {
                    hy * hy + hz * hz, hx * hx + hz * hz, hx * hx + hy * hy,
                    -hx * hy, -hx * hz, -hy * hz};
                const double fPA[6] = {
                    fy * fy + fz * fz, fx * fx + fz * fz, fx * fx + fy * fy,
                    -fx * fy, -fx * fz, -fy * fz};
                const double Id[6] = {C.Ixx, C.Iyy, C.Izz,
                                      C.Ixy, C.Ixz, C.Iyz};
                // WP25-C2 M14: IO writes bounded like mdotOff/massOff above.
                for (int k = 0; k < 6; ++k)
                    if (im->IO[k] < P.size()) P[im->IO[k]] = Id[k] + C.mass * hPA[k] + mFuel * fPA[k];
            }
            // Thruster arm offsets track live c (same tick values as above;
            // masked-out axes have no slot -- contributor uses the record).
            for (int j = 0; j < 3; ++j)
                if (im->cOff[j] < P.size()) P[im->cOff[j]] = cLive[j];
        }
        heyoka::taylor_adaptive<double>& ta = *(gr->ta);
        double* pd = ta.get_pars_data();
        double* sd = ta.get_state_data();
        // Switch list: 0, interior quantized open-times (epsilon-merged), dt.
        std::vector<double> sw = {0.0};
        const double eps = 1e-9 * timeStep;
        for (double t : openT) {
            if (!(t > eps && t < timeStep - eps)) continue;
            bool dup = false;
            for (double s : sw)
                if (std::fabs(s - t) <= eps) { dup = true; break; }
            if (!dup) sw.push_back(t);
        }
        std::sort(sw.begin(), sw.end());
        sw.push_back(timeStep);
        std::vector<double> eff(openT.size(), 0.0);
        // NOTE (measured, kept simple deliberately): module-side env fields
        // freeze body-frame projections per arc while stock re-rotates every
        // RK stage -- an O(|w|*dt) force-direction class (tumbling LEO drag:
        // 1e-4 m / 3.5e-7 m/s per 600 s at |w| = 0.037; stabilized: 4e-6).
        // Rotation-capped subdivision was tried (4x gain at 4x cost) and
        // REVERTED: the residual is systematic lag bias, which subdivision
        // cannot remove (only higher-order terms). An arc-midpoint refresh
        // would, at the cost of extrapolated-attitude machinery; deferred
        // until a consumer needs better than the honest bound below. The
        // stabilized envelope (the swap target) is unaffected (4e-6 class).
        for (size_t k = 0; k + 1 < sw.size(); ++k) {
            const double seg = sw[k + 1] - sw[k];
            if (!(seg > 0.0)) continue;
            const double tArc = sw[k];
            // NO cutoff-lag filter here by design (measured bit-exact):
            // stock cutoff is INERT without ramps (10 vs 1000 identical
            // over fire AND shutoff transients), and ramps stay rejected.
            // An unconditional first-order filter DIVERGES from stock (it
            // attenuates fresh steps 0.63x at cutoff 10 -- reverted). If
            // ramps are ever modeled, cutoff must be modeled WITH them
            // (M2 composition, not standalone).
            for (size_t j = 0; j < openT.size(); ++j)
                eff[j] = (openT[j] > tArc) ? 1.0 : 0.0;
            // WP5-arc-hook: refresh fast-varying env wrenches (drag/SRP/mag)
            // from the CURRENT arc state y. Writes env slots ONLY.
            if (im->hasEnv) this->updateEnvInputs(obj, y.data(), P, cLive);
            if (!openT.empty())
                gr->sys.fill_thruster_inputs(timeStep, eff.data(), eff.size(), P);
            for (size_t i = 0; i < P.size() && i < im->ta_npars; ++i) pd[i] = P[i];
            for (size_t i = 0; i < y.size(); ++i) sd[i] = y[i];
            ta.set_time(0.0);
            try {
                ta.propagate_for(seg);
            } catch (const std::exception& e) {
                this->bskLogger.bskError("OmniVecIntegrator: propagation failed: %s.",
                                         e.what());
            }
            y = ta.get_state();
        }
        // Preserve the non-thruster single-arc path exactly (no openT => one
        // segment already handled above with eff all zero).
    }

    // WP25-B2.5: count MRP shadow-set crossings per tick (scatter hook). Stock
    // owns the switch (postIntegration flips the scattered B-point state);
    // the integrator only records that |sigma|^2 > 1 post-propagation -- the
    // same >1 threshold as the SysModel path. Leg-total gate vs stock is
    // equality +-1 with the tick trace RECORDED (tick-exact equality is NOT
    // required: saturation-edge timing jitter can legitimately flip
    // individual ticks -- flag it if observed, do not force it).
    {
        const double s0 = y[im->sOff], s1 = y[im->sOff + 1], s2 = y[im->sOff + 2];
        if (s0 * s0 + s1 * s1 + s2 * s2 > 1.0) {
            ++this->mrpSwitchCount_;
            ++C.mrpCount;
        }
    }

    // Scatter back into the stock states (C -> B). MRP shadow switching, DV
    // accumulators, energy/momentum and telemetry stay stock-owned
    // (postIntegration) -- the integrator never duplicates them.
    {
        const omni::env::Vec3 sN{y[im->sOff], y[im->sOff + 1], y[im->sOff + 2]};
        const omni::env::Vec3 wN{y[im->oOff], y[im->oOff + 1], y[im->oOff + 2]};
        const omni::env::Mat3 R = omni::env::dcmBN_from_sigma(sN);
        auto toI = [&](const omni::env::Vec3& vB) {
            omni::env::Vec3 vN{};
            for (int i = 0; i < 3; ++i)
                vN[i] = R[0][i] * vB[0] + R[1][i] * vB[1] + R[2][i] * vB[2];
            return vN;
        };
        const omni::env::Vec3 Rc = toI(cB);
        const omni::env::Vec3 Rwxc = toI(omni::env::vec_cross(wN, cB));
        auto pop3 = [&](StateData* s, size_t k, const omni::env::Vec3& shift) {
            Eigen::MatrixXd m(3, 1);
            m(0, 0) = y[k] - shift[0];
            m(1, 0) = y[k + 1] - shift[1];
            m(2, 0) = y[k + 2] - shift[2];
            s->setState(m);
        };
        pop3(im->hubR, im->rOff, Rc);
        pop3(im->hubV, im->vOff, Rwxc);
        pop3(im->hubS, im->sOff, omni::env::Vec3{0.0, 0.0, 0.0});
        pop3(im->hubO, im->oOff, omni::env::Vec3{0.0, 0.0, 0.0});
    }
    if (nW > 0) {
        Eigen::MatrixXd m(static_cast<int>(nW), 1);
        for (size_t i = 0; i < nW; ++i)
            m(static_cast<int>(i), 0) = y[im->wOff + i];
        im->omegas->setState(m);
    }
    if (im->fuelSrc != nullptr) {
        Eigen::MatrixXd m(1, 1);
        m(0, 0) = y[im->fuelPos];
        im->fuelSrc->setState(m);
    }
    // Passthrough states go back verbatim (zero dynamics; refreshed by
    // stock preIntegration every tick).
    {
        size_t k = 0;
        for (int j = 0; j < 2; ++j) {
            if (im->passSrc[j] == nullptr) continue;
            Eigen::MatrixXd m(3, 1);
            m(0, 0) = passVals[k]; m(1, 0) = passVals[k + 1]; m(2, 0) = passVals[k + 2];
            im->passSrc[j]->setState(m);
            k += 3;
        }
    }
    // WP25-A4 no-double-step audit: exactly one increment per object per
    // integrate() call (survives rebuilds; cumulative since construction).
    ++C.stepCount;
}
