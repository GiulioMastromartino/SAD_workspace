// ISC License - see native_rw/LICENSE.  See omni/vec_integrator.hpp.
#include "omni/vec_integrator.hpp"

#include <algorithm>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
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
      dyn_(static_cast<DynamicObject*>(static_cast<void*>(sc))),
      impl(std::make_unique<Impl>()) {
    // The void* round-trip is valid under stock's single-inheritance layout
    // (Spacecraft : DynamicObject : SysModel); the base ctor registers
    // dynPtrs[0] = this object, which setIntegrator() requires.
    if (dyn_ == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null Spacecraft pointer.");
}

OmniVecIntegrator::~OmniVecIntegrator() = default;

void OmniVecIntegrator::setMass(double m) {
    if (!(m > 0.0) || !std::isfinite(m))
        this->bskLogger.bskError("OmniVecIntegrator: mass must be finite positive.");
    this->mass_ = m;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setInertia(double a, double b, double c,
                                   double d, double e, double f) {
    this->Ixx_ = a; this->Iyy_ = b; this->Izz_ = c;
    this->Ixy_ = d; this->Ixz_ = e; this->Iyz_ = f;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setCoMOffset(double x, double y, double z) {
    this->rBcB_[0] = x; this->rBcB_[1] = y; this->rBcB_[2] = z;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setMu(double mu) {
    if (!(mu > 0.0) || !std::isfinite(mu))
        this->bskLogger.bskError("OmniVecIntegrator: mu must be finite positive.");
    this->mu_ = mu;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setUseGravityGradient(bool v) {
    this->useGG_ = v;
    this->configDirty_ = true;
}

void OmniVecIntegrator::addEnvironmentBody(double mu, double ox, double oy, double oz) {
    if (!(mu > 0.0) || !std::isfinite(mu))
        this->bskLogger.bskError("OmniVecIntegrator: env body mu must be finite positive.");
    this->envBodyMu_.push_back(mu);
    this->envBodyOff0_.push_back(ox);
    this->envBodyOff0_.push_back(oy);
    this->envBodyOff0_.push_back(oz);
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setJ2(double J2) {
    if (!std::isfinite(J2) || J2 < 0.0)
        this->bskLogger.bskError("OmniVecIntegrator: J2 must be finite non-negative.");
    this->envJ2_ = J2;
    this->j2On_ = true;
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setDragParams(double CdA, double cx, double cy, double cz) {
    if (!(CdA >= 0.0) || !std::isfinite(CdA))
        this->bskLogger.bskError("OmniVecIntegrator: CdA must be finite non-negative.");
    this->envCdA_ = CdA;
    this->envComOff_[0] = cx; this->envComOff_[1] = cy; this->envComOff_[2] = cz;
    this->dragOn_ = true;
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::addSRPFacet(double area, double nx, double ny, double nz,
                                    double copx, double copy, double copz,
                                    double c_diffuse, double c_specular) {
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
    this->srpFacets_.push_back(f);
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setSRPCannonball(double area, double cr) {
    if (!(area > 0.0))
        this->bskLogger.bskError("OmniVecIntegrator: cannonball area must be positive.");
    if (!(cr >= 0.0) || !(cr <= 1.0))
        this->bskLogger.bskError(
            "OmniVecIntegrator: cr must be the specular fraction in [0,1] "
            "(stock C_R total multiplier maps as cr = C_R - 1).");
    this->cbArea_ = area; this->cbCr_ = cr;
    this->cannonballOn_ = true;
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setSunState(double px, double py, double pz) {
    this->sunState_[0] = px; this->sunState_[1] = py; this->sunState_[2] = pz;
    this->hasSun_ = true;
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setMagneticMoment(double mx, double my, double mz) {
    this->magM_[0] = mx; this->magM_[1] = my; this->magM_[2] = mz;
    this->magOn_ = true;
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setMagneticDipole(double g10, double g11, double h11) {
    this->magG10_ = g10; this->magG11_ = g11; this->magH11_ = h11;
    this->envConfigured_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setMaxMtbDipole(double dmax) {
    if (!(dmax > 0.0) || !std::isfinite(dmax))
        this->bskLogger.bskError("OmniVecIntegrator: maxMtbDipole must be finite positive.");
    this->maxMtbDipole_ = dmax;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setFuelStateName(const std::string& name) {
    if (name.empty())
        this->bskLogger.bskError("OmniVecIntegrator: fuel state name must be nonempty.");
    this->fuelStateName_ = name;
    this->hasFuel_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setFuelTank(double rTBx, double rTBy, double rTBz, double leak) {
    if (!std::isfinite(rTBx + rTBy + rTBz) || !std::isfinite(leak) || leak < 0.0)
        this->bskLogger.bskError("OmniVecIntegrator: fuel tank record invalid.");
    this->fuelRTB_[0] = rTBx; this->fuelRTB_[1] = rTBy; this->fuelRTB_[2] = rTBz;
    this->fuelLeak_ = leak;
    this->hasFuel_ = true;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setThrusterIsps(const std::vector<double>& isp) {
    for (double v : isp) {
        if (!(v > 0.0) || !std::isfinite(v))
            this->bskLogger.bskError("OmniVecIntegrator: thruster Isp must be finite positive.");
    }
    this->fuelIsps_ = isp;
    this->configDirty_ = true;
}

void OmniVecIntegrator::addThruster(double dx, double dy, double dz,
                                   double px, double py, double pz, double tmax) {
    const double n = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniVecIntegrator: thruster direction must be nonzero.");
    if (!(tmax > 0.0) || !std::isfinite(tmax))
        this->bskLogger.bskError("OmniVecIntegrator: thruster Tmax must be finite positive.");
    this->thrDirs_.push_back({dx / n, dy / n, dz / n});
    this->thrPos_.push_back({px, py, pz});
    this->thrTmax_.push_back(tmax);
    this->configDirty_ = true;
}

void OmniVecIntegrator::setInitialThrusterCmd(const std::vector<double>& onTimes) {
    this->preStaged_ = onTimes;
    this->havePreStaged_ = !onTimes.empty();
    this->configDirty_ = true;
}

void OmniVecIntegrator::addWheel(double gsx, double gsy, double gsz, double Js) {
    const double n = std::sqrt(gsx * gsx + gsy * gsy + gsz * gsz);
    if (!(n > 0.0) || !std::isfinite(n))
        this->bskLogger.bskError("OmniVecIntegrator: wheel spin axis must be nonzero.");
    if (!(Js > 0.0) || !std::isfinite(Js))
        this->bskLogger.bskError("OmniVecIntegrator: Js must be finite positive.");
    WheelRec w{};
    w.gs[0] = gsx / n; w.gs[1] = gsy / n; w.gs[2] = gsz / n;
    w.Js = Js;
    this->wheelRecs_.push_back(w);
    this->fricRecs_.push_back(FricRec{});
    this->fricState_.push_back(FricState{});
    this->configDirty_ = true;
}

void OmniVecIntegrator::linkWheelLog(size_t i, Message<RWConfigLogMsgPayload>* src) {
    if (src == nullptr)
        this->bskLogger.bskError("OmniVecIntegrator: null wheel-log source.");
    if (this->rwLogInMsgs.size() <= i) this->rwLogInMsgs.resize(i + 1);
    this->rwLogInMsgs[i].subscribeTo(src);
}

void OmniVecIntegrator::setFrictionEnabled(bool on) {
    this->frictionOn_ = on;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setFrictionParams(size_t i, double fCoulomb, double fStatic,
                                          double betaStatic, double cViscous,
                                          double omegaLimitCycle) {
    if (i >= this->fricRecs_.size())
        this->bskLogger.bskError("OmniVecIntegrator: wheel %zu not mirrored.", i);
    if (!std::isfinite(fCoulomb + fStatic + betaStatic + cViscous + omegaLimitCycle))
        this->bskLogger.bskError("OmniVecIntegrator: friction params must be finite.");
    FricRec& r = this->fricRecs_[i];
    r.fCoulomb = fCoulomb; r.fStatic = fStatic; r.betaStatic = betaStatic;
    r.cViscous = cViscous; r.omegaLimitCycle = omegaLimitCycle;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setTol(double tol) {
    if (!(tol > 0.0) || !std::isfinite(tol))
        this->bskLogger.bskError("OmniVecIntegrator: tol must be finite positive.");
    this->tol_ = tol;
    this->configDirty_ = true;
}

void OmniVecIntegrator::setCompactMode(bool compact) {
    this->compactMode_ = compact;
    this->configDirty_ = true;
}

size_t OmniVecIntegrator::nStates() const { return this->impl->nStates; }
size_t OmniVecIntegrator::nPars() const { return this->impl->parsBase.size(); }
size_t OmniVecIntegrator::nWheels() const { return this->impl->nWheels; }
double OmniVecIntegrator::getBuildSeconds() const { return this->buildSeconds_; }
std::vector<std::string> OmniVecIntegrator::claimedNames() const {
    return this->impl->claimed;
}

// --- census + claim + mirror + symbolic build (first integrate) --------------
void OmniVecIntegrator::buildOnce() {
    const auto t0 = std::chrono::steady_clock::now();
    auto* im = this->impl.get();

    // One primary object. Synced/multi-spacecraft stepping is Phase 3.
    // (dynPtrs is our own inherited StateVecIntegrator member; the base
    // ctor registered dyn_[0], and setIntegrator() preserves the old set.)
    if (this->dynPtrs.size() != 1)
        this->bskLogger.bskError(
            "OmniVecIntegrator: %zu dynamics objects registered; stage 1 supports "
            "exactly one (syncDynamicsIntegration is Phase 3).",
            this->dynPtrs.size());

    // Census: every registered (name, rows) in stock order. This enumerates
    // ALL states, so the claim check below is airtight without object access.
    std::vector<std::pair<std::string, size_t>> census;
    {
        ExtendedStateVector ev = ExtendedStateVector::fromStates(this->dynPtrs);
        ev.apply([&](const size_t&, const std::string& nm, const Eigen::MatrixXd& m) {
            census.emplace_back(nm, static_cast<size_t>(m.rows()));
        });
    }
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
                "OmniVecIntegrator: required hub state '%s' missing or not 3x1 "
                "(point-mass translational mode is out of envelope).", nm);
    }
    const size_t nW = rows_of("reactionWheelOmegas");
    // Phase-2 fuel: the mirrored stock state name is claimed here (exact
    // match); anything else unclaimed is rejected as before.
    const size_t nF = (this->hasFuel_ && !this->fuelStateName_.empty()
                       && rows_of(this->fuelStateName_.c_str()) == 1)
                          ? 1
                          : 0;
    if (this->hasFuel_ && nF == 0)
        this->bskLogger.bskError(
            "OmniVecIntegrator: mirrored fuel state '%s' not registered.",
            this->fuelStateName_.c_str());
    // Gravity DV-reference states (stock pre/postIntegration bookkeeping,
    // refreshed every tick by matchGravitytoVelocityState, zero dynamics):
    // carried through UNCHANGED, never part of the symbolic system.
    const char* passNames[2] = {"hubGravVelocity", "BcGravVelocity"};
    for (const char* nm : passNames) {
        if (rows_of(nm) != 0 && rows_of(nm) != 3)
            this->bskLogger.bskError(
                "OmniVecIntegrator: passthrough state '%s' is not 3x1.", nm);
    }
    for (const auto& e : census) {
        const bool known = (e.first == "hubPosition" || e.first == "hubVelocity"
                            || e.first == "hubSigma" || e.first == "hubOmega"
                            || e.first == "reactionWheelOmegas"
                            || e.first == "hubGravVelocity"
                            || e.first == "BcGravVelocity"
                            || (nF == 1 && e.first == this->fuelStateName_));
        if (!known)
            this->bskLogger.bskError(
                "OmniVecIntegrator: registered state '%s' (%zu rows) has no "
                "contributor; envelope is hub + balanced wheels + staircase "
                "thrusters + fuel (when mirrored).",
                e.first.c_str(), e.second);
    }
    im->claimed.clear();
    for (const auto& e : census) im->claimed.push_back(e.first);

    // Cross-check: mirrored wheel records (Python side, stock order) must
    // match the registered omegas rows exactly.
    if (this->wheelRecs_.size() != nW)
        this->bskLogger.bskError(
            "OmniVecIntegrator: %zu mirrored wheels but reactionWheelOmegas has "
            "%zu rows; mirror_stock_config missed wheels (or none attached).",
            this->wheelRecs_.size(), nW);
    im->nWheels = nW;

    // Cache live StateData sources.
    auto grab = [&](const char* nm) -> StateData* {
        StateData* s = this->dyn_->dynManager.getStateObject(std::string(nm));
        if (s == nullptr)
            this->bskLogger.bskError("OmniVecIntegrator: state '%s' vanished.", nm);
        return s;
    };
    im->hubR = grab("hubPosition");
    im->hubV = grab("hubVelocity");
    im->hubS = grab("hubSigma");
    im->hubO = grab("hubOmega");
    im->omegas = (nW > 0) ? grab("reactionWheelOmegas") : nullptr;
    im->fuelSrc = (nF > 0) ? grab(this->fuelStateName_.c_str()) : nullptr;
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

    // Mirror into a fresh OmniSystem (order = stock order by construction).
    // Hub record takes the RAW hub offset (input I/mass are hub-dry; the
    // aggregate lives in per-tick pars + live B/C mapping, never in records).
    im->sys.set_gravity(this->mu_);
    im->sys.set_use_gravity_gradient(this->useGG_);
    im->sys.add_hub(this->mass_,
                    {this->Ixx_, this->Ixy_, this->Ixz_,
                     this->Ixy_, this->Iyy_, this->Iyz_,
                     this->Ixz_, this->Iyz_, this->Izz_},
                    {this->rBcB_[0], this->rBcB_[1], this->rBcB_[2]});
    for (const auto& w : this->wheelRecs_)
        im->sys.add_reaction_wheel({w.gs[0], w.gs[1], w.gs[2]}, w.Js, 0.0);
    // Fuel LAST (documented layout).
    if (nF > 0) im->sys.add_fuel_tank();
    // Stage-2 thruster cluster (staircase only; timing models rejected
    // Python-side). Geometry arrives via addThruster records (joined
    // Python-side); thrusters add no states (census unchanged).
    if (!this->thrDirs_.empty())
        im->sys.add_thrusters(this->thrDirs_, this->thrPos_, this->thrTmax_);
    // WP5 env replay: bodies + J2 feed the graph (g_N channel); drag/SRP/mag
    // geometry lives module-side and enters via slots (A3). Any configured
    // effect registers the single 'e' cluster (same replay as the module).
    if (this->envConfigured_) {
        for (size_t i = 0; i < this->envBodyMu_.size(); ++i) {
            im->sys.addEnvironmentBody(this->envBodyMu_[i],
                                       this->envBodyOff0_[3 * i],
                                       this->envBodyOff0_[3 * i + 1],
                                       this->envBodyOff0_[3 * i + 2]);
        }
        if (this->j2On_) im->sys.setJ2(this->envJ2_);
        if (this->dragOn_)
            im->sys.setDragParams(this->envCdA_, this->envComOff_[0],
                                  this->envComOff_[1], this->envComOff_[2]);
        for (const auto& f : this->srpFacets_)
            im->sys.addSRPFacet(f.area, f.nHat[0], f.nHat[1], f.nHat[2],
                                f.rCop[0], f.rCop[1], f.rCop[2], f.cd, f.cs);
        if (this->cannonballOn_) im->sys.setSRPCannonball(this->cbArea_, this->cbCr_);
        if (this->hasSun_)
            im->sys.setSunState(this->sunState_[0], this->sunState_[1],
                                this->sunState_[2]);
        if (this->magOn_)
            im->sys.setMagneticMoment(this->magM_[0], this->magM_[1],
                                      this->magM_[2]);
        im->sys.setMagneticDipole(this->magG10_, this->magG11_, this->magH11_);
    }

    const size_t n = im->sys.n_states();
    if (n != 12 + nW + nF)
        this->bskLogger.bskError(
            "OmniVecIntegrator: mirrored system has %zu states for %zu stock "
            "states; layout mismatch.", n, 12 + nW + nF);
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
            this->bskLogger.bskError("OmniVecIntegrator: fuel state layout broken.");
    }
    // Name-based par slots for the per-tick fuel feedback (A2).
    if (nF > 0) {
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
    if (!this->thrDirs_.empty()) {
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
            for (size_t i = 0; i < sn.size(); ++i)
                if (sn[i] == nm) return i;
            this->bskLogger.bskError("OmniVecIntegrator: state '%s' missing.", nm);
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
    this->buildSeconds_ = std::chrono::duration<double>(t1 - t0).count();
    this->bskLogger.bskLog(BSK_INFORMATION,
                           "OmniVecIntegrator: %zu states, %zu pars, %zu wheels; "
                           "symbolic build %.2f ms, tol %g.",
                           n, im->parsBase.size(), nW,
                           this->buildSeconds_ * 1e3, this->tol_);
}

// WP5 per-tick (/per-arc) env refresh. y = gathered omni-order state
// (hub-first-12, A1); P = live par vector. Module-side doubles only (A3);
// TRUE laws from omni::env. P1 truth table enforced (density/sun/MTB link
// required when on; eclipse unlinked -> 1.0; wind unlinked -> inertial).
// Ported 1:1 from OmniSpacecraft::updateEnvInputs (same validated logic);
// the only difference is member routing (integrator owns no telemetry).
void OmniVecIntegrator::updateEnvInputs(const double* y, std::vector<double>& P,
                                        const double cLive[3]) {
    auto* im = this->impl.get();
    if (y == nullptr || !im->hasEnv) return;
    const size_t bO = im->envBody, dO = im->envDrag;
    const size_t sO = im->envSrp, mO = im->envMag;
    const size_t nB = im->envNBodies;
    const size_t nEnv = 3 * nB + 15;
    for (size_t k = 0; k < nEnv; ++k)
        if (bO + k < P.size()) P[bO + k] = 0.0;
    const size_t rO = im->rOff, vO = im->vOff, sgO = im->sOff;
    omni::env::Vec3 r{y[rO], y[rO + 1], y[rO + 2]};
    omni::env::Vec3 v{y[vO], y[vO + 1], y[vO + 2]};
    omni::env::Vec3 sig{y[sgO], y[sgO + 1], y[sgO + 2]};
    const omni::env::Mat3 R = omni::env::dcmBN_from_sigma(sig);

    // N-body offsets: refreshed PER TICK from msgs when linked (never frozen),
    // else the recorded static/initial offset (bench/fixture path).
    for (size_t i = 0; i < nB && i < this->envBodyMu_.size(); ++i) {
        omni::env::Vec3 off{this->envBodyOff0_[3 * i], this->envBodyOff0_[3 * i + 1],
                            this->envBodyOff0_[3 * i + 2]};
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
                "OmniVecIntegrator: drag on with atmoDensInMsg unlinked (stock bskError).");
        } else {
            const AtmoPropsMsgPayload atmo = this->atmoDensInMsg();
            omni::env::Vec3 vAir{0.0, 0.0, 0.0};
            if (this->windVelInMsg.isLinked()) {
                const WindMsgPayload wind = this->windVelInMsg();
                vAir = {wind.v_air_N[0], wind.v_air_N[1], wind.v_air_N[2]};
            }
            const omni::env::Vec3 vRelB = omni::env::matvec_BN(
                R, omni::env::vec_sub(v, vAir));
            // Measured split (combo probe): drag arms do NOT shift.
            const omni::env::Vec3 dragArm{this->envComOff_[0],
                                          this->envComOff_[1],
                                          this->envComOff_[2]};
            const omni::env::WrenchB w = omni::env::dragWrench(
                atmo.neutralDensity, this->envCdA_, vRelB, dragArm, 0.0);
            if (dO + 5 < P.size()) {
                P[dO] = w.F[0]; P[dO + 1] = w.F[1]; P[dO + 2] = w.F[2];
                P[dO + 3] = w.T[0]; P[dO + 4] = w.T[1]; P[dO + 5] = w.T[2];
            }
        }
    }

    const bool srpOn = this->cannonballOn_ || !this->srpFacets_.empty();
    if (srpOn) {
        const bool sunLinked = this->sunStateInMsg.isLinked();
        if (!sunLinked && !this->hasSun_) {
            this->bskLogger.bskError(
                "OmniVecIntegrator: SRP on with sun state unlinked (stock bskError).");
        } else {
            omni::env::Vec3 sunPos = {this->sunState_[0], this->sunState_[1],
                                      this->sunState_[2]};
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
                double shadow = 1.0;
                if (this->eclipseInMsg.isLinked())
                    shadow = this->eclipseInMsg().shadowFactor;
                omni::env::Vec3 F{0.0, 0.0, 0.0}, T{0.0, 0.0, 0.0};
                if (this->cannonballOn_) {
                    F = omni::env::vec_add(
                        F, omni::env::srpCannonball(this->cbArea_, this->cbCr_, Pflx,
                                                   shadow, sHatB));
                }
                for (const auto& f : this->srpFacets_) {
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

    if (this->magOn_) {
        const bool mZero = (this->magM_[0] == 0.0 && this->magM_[1] == 0.0 &&
                            this->magM_[2] == 0.0);
        if (!this->magFieldInMsg.isLinked()) {
            this->bskLogger.bskError(
                "OmniVecIntegrator: mag on with magField msg unlinked (stock bskError).");
        } else if (mZero) {
            this->bskLogger.bskError(
                "OmniVecIntegrator: mag on with zero dipole (MTB cmd/params unlinked).");
        } else {
            const MagneticFieldMsgPayload mag = this->magFieldInMsg();
            const omni::env::Vec3 BN{mag.magField_N[0], mag.magField_N[1], mag.magField_N[2]};
            const omni::env::Vec3 BB = omni::env::matvec_BN(R, BN);
            omni::env::Vec3 m{this->magM_[0], this->magM_[1], this->magM_[2]};
            const double mn = omni::env::vec_norm(m);
            if (mn > this->maxMtbDipole_ && mn > 0.0)
                m = omni::env::vec_scale(m, this->maxMtbDipole_ / mn);
            const omni::env::Vec3 tau = omni::env::magTorque(m, BB);
            if (mO + 2 < P.size()) { P[mO] = tau[0]; P[mO + 1] = tau[1]; P[mO + 2] = tau[2]; }
        }
    }
}

void OmniVecIntegrator::integrate(double /*currentTime*/, double timeStep) {
    if (!this->built_ || this->configDirty_) {
        this->impl = std::make_unique<Impl>();
        this->buildOnce();
        this->built_ = true;
        this->configDirty_ = false;
    }
    auto* im = this->impl.get();
    if (!(timeStep > 0.0) || !std::isfinite(timeStep)) return;

    // Live aggregate c_B this tick (fuel ? formula : verbatim record, so the
    // no-fuel path is bit-identical to the static record). Used for the B/C
    // boundary, I_C feedback and thruster arms below -- no rebuilds, no lag.
    double cLive[3] = {this->rBcB_[0], this->rBcB_[1], this->rBcB_[2]};
    double mFuelLive = 0.0;
    if (im->fuelSrc != nullptr) {
        mFuelLive = im->fuelSrc->getState()(0, 0);
        if (!(mFuelLive >= 0.0) || !std::isfinite(mFuelLive))
            mFuelLive = 0.0;
        const double mT = this->mass_ + mFuelLive;
        if (mT > 0.0) {
            for (int i = 0; i < 3; ++i)
                cLive[i] = (this->mass_ * this->rBcB_[i]
                            + mFuelLive * this->fuelRTB_[i]) / mT;
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
        if (i < this->rwLogInMsgs.size() && this->rwLogInMsgs[i].isLinked())
            v = this->rwLogInMsgs[i]().u_current;
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
            if (this->frictionOn_ && i < this->fricRecs_.size()
                && i < this->fricState_.size()) {
                const FricRec& fr = this->fricRecs_[i];
                FricState& st = this->fricState_[i];
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
    // Friction slots stay 0 (stage 1: friction off on both arms by the
    // Python-side guard in mirror_stock_config).

    // WP5 per-tick env refresh (bodies never frozen; P1 truth table inside).
    if (im->hasEnv) this->updateEnvInputs(y.data(), P, cLive);

    // Thruster channel (stage 2, staircase only): edge-triggered mailbox.
    // Fire rule (measured stock cells: fresh/write-once/frozen/advance):
    // tick 0 fires the PRE-STAGED snapshot (Reset-equivalent staging:
    // pre-init command fires, Reset-zeros fill idles); later ticks fire
    // the current message iff first-sight or timestamp strictly advanced
    // (frozen ticks idle). WP3b timing models rejected Python-side; the
    // graph sees q-slots only (A3).
    {
        const size_t nt = this->thrDirs_.size();
        std::vector<double> openT(nt, 0.0);
        if (nt > 0) {
            if (this->havePreStaged_) {
                if (!this->preStaged_.empty())
                    openT = im->sys.thruster_on_times(timeStep, this->preStaged_.data(),
                                                      this->preStaged_.size());
                this->havePreStaged_ = false;
                this->preStaged_.clear();
                // Seed the edge detector at the current stamp so the
                // pre-init write is not re-fired as "first sight" (write-once
                // fires tick 0 exactly once).
                if (this->thrOnTimeInMsg.isLinked())
                    this->lastThrStamp_ = static_cast<int64_t>(
                        this->thrOnTimeInMsg.timeWritten());
            } else if (this->thrOnTimeInMsg.isLinked()) {
                const int64_t stamp =
                    static_cast<int64_t>(this->thrOnTimeInMsg.timeWritten());
                if (stamp > this->lastThrStamp_) {
                    const THRArrayOnTimeCmdMsgPayload msg = this->thrOnTimeInMsg();
                    constexpr size_t NAVAIL =
                        sizeof(msg.OnTimeRequest) / sizeof(msg.OnTimeRequest[0]);
                    std::vector<double> staged(std::min(nt, NAVAIL), 0.0);
                    for (size_t i = 0; i < staged.size(); ++i)
                        staged[i] = msg.OnTimeRequest[i];
                    openT = im->sys.thruster_on_times(timeStep, staged.data(),
                                                      staged.size());
                    this->lastThrStamp_ = stamp;
                }
            }
        }
        // Phase-2 fuel per-tick update (needs openT duties from staging above,
        // runs before the arc loop; pars pushed per-arc below).
        // mdot = sum over thrusters duty*Tmax/(Isp*g0) + leak, clamped so the
        // tank never crosses zero (module-side doubles, A3); hub mass/I pars
        // overwritten from live fuel (m = mDry + mFuel, I = I_dry + mFuel*K).
        // c_B stays build-time static (depletion drift labeled, gated).
        if (im->fuelSrc != nullptr && timeStep > 0.0) {
            constexpr double G0 = 9.80665;
            const double mFuel = y[im->fuelPos];
            double mdot = this->fuelLeak_;
            for (size_t j = 0; j < openT.size() && j < this->thrTmax_.size(); ++j) {
                const double duty = (timeStep > 0.0) ? openT[j] / timeStep : 0.0;
                const double isp = (j < this->fuelIsps_.size()) ? this->fuelIsps_[j] : 0.0;
                if (isp > 0.0 && duty > 0.0)
                    mdot += duty * this->thrTmax_[j] / (isp * G0);
            }
            if (mdot < 0.0) mdot = 0.0;
            const double mdotMax = mFuel / timeStep;
            if (mdot > mdotMax) mdot = (mdotMax > 0.0) ? mdotMax : 0.0;
            P[im->mdotOff] = mdot;
            P[im->massOff] = this->mass_ + mFuel;
            // Inertia about TRUE C (not B!): I_C = I_hub_Bc
            //   + mHub*PA(rBcB - c) + mFuel*PA(rTB - c), c = live aggregate.
            // Using I_B (about B) instead is wrong by M*(|c|^2I - cc^T);
            // with c = 5 mm that is 0.05/900-scale in D and costs 0.2 m +
            // 2e-2 rad over a 600 s burn (measured). No-fuel and centered
            // cases reduce exactly (PA(0) = 0). Packed Ixx..Iyz order.
            {
                const double cx = cLive[0], cy = cLive[1], cz = cLive[2];
                const double hx = this->rBcB_[0] - cx,
                             hy = this->rBcB_[1] - cy,
                             hz = this->rBcB_[2] - cz;
                const double fx = this->fuelRTB_[0] - cx,
                             fy = this->fuelRTB_[1] - cy,
                             fz = this->fuelRTB_[2] - cz;
                const double hPA[6] = {
                    hy * hy + hz * hz, hx * hx + hz * hz, hx * hx + hy * hy,
                    -hx * hy, -hx * hz, -hy * hz};
                const double fPA[6] = {
                    fy * fy + fz * fz, fx * fx + fz * fz, fx * fx + fy * fy,
                    -fx * fy, -fx * fz, -fy * fz};
                const double Id[6] = {this->Ixx_, this->Iyy_, this->Izz_,
                                      this->Ixy_, this->Ixz_, this->Iyz_};
                for (int k = 0; k < 6; ++k)
                    P[im->IO[k]] = Id[k] + this->mass_ * hPA[k] + mFuel * fPA[k];
            }
            // Thruster arm offsets track live c (same tick values as above;
            // masked-out axes have no slot -- contributor uses the record).
            for (int j = 0; j < 3; ++j)
                if (im->cOff[j] < P.size()) P[im->cOff[j]] = cLive[j];
        }
        heyoka::taylor_adaptive<double>& ta = *(im->ta);
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
            if (im->hasEnv) this->updateEnvInputs(y.data(), P, cLive);
            if (!openT.empty())
                im->sys.fill_thruster_inputs(timeStep, eff.data(), eff.size(), P);
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
}
