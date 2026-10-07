/* ISC License - see native_rw/LICENSE. See heyokaRWDynamics.h for the contract. */

#include "heyokaRWDynamics.h"

#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include <heyoka/heyoka.hpp>

namespace {
using heyoka::expression;
using heyoka::prime;

/*! Build the 15-state MRP+RW Taylor system (mu baked as a constant).
 *  pars: [u1,u2,u3, m, Ixx,Iyy,Izz,Ixy,Ixz,Iyz,
 *         invIxx,invIyy,invIzz,invIxy,invIxz,invIyz, Js1,Js2,Js3].
 *  P(3) (mass) is kept for layout compat but unused (no body thrust). */
std::vector<std::pair<expression, expression>> buildRwSystem(double mu) {
    auto vars = heyoka::make_vars("rx", "ry", "rz", "vx", "vy", "vz",
                                  "s1", "s2", "s3", "omx", "omy", "omz",
                                  "W1", "W2", "W3");
    expression rx = vars[0], ry = vars[1], rz = vars[2];
    expression vx = vars[3], vy = vars[4], vz = vars[5];
    expression s1 = vars[6], s2 = vars[7], s3 = vars[8];
    expression omx = vars[9], omy = vars[10], omz = vars[11];
    expression W1 = vars[12], W2 = vars[13], W3 = vars[14];

    auto P = [&](int i) { return expression(heyoka::par[i]); };
    expression u1 = P(0), u2 = P(1), u3 = P(2);
    expression Ixx = P(4), Iyy = P(5), Izz = P(6);
    expression Ixy = P(7), Ixz = P(8), Iyz = P(9);
    expression ixx = P(10), iyy = P(11), izz = P(12);
    expression ixy = P(13), ixz = P(14), iyz = P(15);
    expression J1 = P(16), J2 = P(17), J3 = P(18);

    // Kepler translation (no DCM: RW demo has no body thrust)
    expression Rnorm = heyoka::sqrt(rx * rx + ry * ry + rz * rz);
    expression invR3 = 1.0 / (Rnorm * Rnorm * Rnorm);
    expression ax = -mu * rx * invR3;
    expression ay = -mu * ry * invR3;
    expression az = -mu * rz * invR3;

    // MRP kinematics
    expression sn2 = s1 * s1 + s2 * s2 + s3 * s3;
    expression B11 = 1.0 - sn2 + 2.0 * s1 * s1, B12 = 2.0 * (s1 * s2 - s3),
               B13 = 2.0 * (s1 * s3 + s2);
    expression B21 = 2.0 * (s2 * s1 + s3), B22 = 1.0 - sn2 + 2.0 * s2 * s2,
               B23 = 2.0 * (s2 * s3 - s1);
    expression B31 = 2.0 * (s3 * s1 - s2), B32 = 2.0 * (s3 * s2 + s1),
               B33 = 1.0 - sn2 + 2.0 * s3 * s3;
    expression s1dot = 0.25 * (B11 * omx + B12 * omy + B13 * omz);
    expression s2dot = 0.25 * (B21 * omx + B22 * omy + B23 * omz);
    expression s3dot = 0.25 * (B31 * omx + B32 * omy + B33 * omz);

    // coupled hub + wheels (body axes = wheel axes)
    expression h1 = J1 * (W1 + omx);
    expression h2 = J2 * (W2 + omy);
    expression h3 = J3 * (W3 + omz);
    expression Hx = Ixx * omx + Ixy * omy + Ixz * omz + h1;
    expression Hy = Ixy * omx + Iyy * omy + Iyz * omz + h2;
    expression Hz = Ixz * omx + Iyz * omy + Izz * omz + h3;
    expression gx = -(omy * Hz - omz * Hy) - u1;
    expression gy = -(omz * Hx - omx * Hz) - u2;
    expression gz = -(omx * Hy - omy * Hx) - u3;
    expression wdotx = ixx * gx + ixy * gy + ixz * gz;
    expression wdoty = ixy * gx + iyy * gy + iyz * gz;
    expression wdotz = ixz * gx + iyz * gy + izz * gz;
    expression W1dot = u1 / J1;
    expression W2dot = u2 / J2;
    expression W3dot = u3 / J3;

    return {{prime(rx) = vx, prime(ry) = vy, prime(rz) = vz,
             prime(vx) = ax, prime(vy) = ay, prime(vz) = az,
             prime(s1) = s1dot, prime(s2) = s2dot, prime(s3) = s3dot,
             prime(omx) = wdotx, prime(omy) = wdoty, prime(omz) = wdotz,
             prime(W1) = W1dot, prime(W2) = W2dot, prime(W3) = W3dot}};
}

/*! Analytic inverse of a symmetric 3x3, packed [xx,yy,zz,xy,xz,yz].
 *  Returns false when singular/non-finite (callers must bskError). */
bool symInv33(const double I[6], double inv[6]) {
    const double a = I[0], b = I[1], c = I[2];
    const double d = I[3], e = I[4], f = I[5];
    const double det = a * (b * c - f * f) - d * (d * c - e * f)
        + e * (d * f - b * e);
    if (!std::isfinite(det) || std::fabs(det) < 1e-30) {
        return false;
    }
    const double s = 1.0 / det;
    inv[0] = (b * c - f * f) * s;
    inv[1] = (a * c - e * e) * s;
    inv[2] = (a * b - d * d) * s;
    inv[3] = (e * f - d * c) * s;
    inv[4] = (d * f - b * e) * s;
    inv[5] = (d * e - a * f) * s;
    return true;
}

constexpr double RPM2RPS = 2.0 * 3.141592653589793 / 60.0;
}  // namespace

/*! PIMPL holder: keeps heyoka types out of the header/SWIG wrapper TU. */
struct HeyokaRWDynamics::TaHolder {
    heyoka::taylor_adaptive<double> ta;
    TaHolder(std::vector<std::pair<heyoka::expression, heyoka::expression>> sys,
             std::vector<double> state, double tol)
        : ta(std::move(sys), std::move(state), heyoka::kw::tol = tol,
             heyoka::kw::compact_mode = false, heyoka::kw::high_accuracy = false) {}
    TaHolder(const TaHolder &) = default;
    TaHolder(TaHolder &&) = default;
    TaHolder &operator=(const TaHolder &) = default;
    TaHolder &operator=(TaHolder &&) = default;
};

HeyokaRWDynamics::HeyokaRWDynamics() = default;
HeyokaRWDynamics::~HeyokaRWDynamics() = default;

void HeyokaRWDynamics::buildIntegrator() {
    if (!(this->mass > 0.0) || !std::isfinite(this->mass)) {
        this->bskLogger.bskError("HeyokaRWDynamics: mass must be finite positive.");
    }
    if (!(this->J1 > 0.0) || !(this->J2 > 0.0) || !(this->J3 > 0.0)
        || !std::isfinite(this->J1 + this->J2 + this->J3)) {
        this->bskLogger.bskError("HeyokaRWDynamics: rotor inertias must be finite positive.");
    }
    if (!(this->uMax >= 0.0) || !std::isfinite(this->uMax)) {
        this->bskLogger.bskError("HeyokaRWDynamics: uMax must be finite non-negative.");
    }
    if (!(this->tol > 0.0) || !std::isfinite(this->tol)) {
        this->bskLogger.bskError("HeyokaRWDynamics: tol must be finite positive.");
    }
    if (!(this->mu > 0.0) || !std::isfinite(this->mu)) {
        this->bskLogger.bskError("HeyokaRWDynamics: mu must be finite positive.");
    }
    // packed inverse inertia from the configured full inertia (no I6 packing
    // on the Python side: this is where the flatten() bug used to live)
    const double I6[6] = {this->Ixx, this->Iyy, this->Izz,
                          this->Ixy, this->Ixz, this->Iyz};
    if (!symInv33(I6, this->invI6)) {
        this->bskLogger.bskError("HeyokaRWDynamics: inertia matrix is singular.");
    }
    // Process-wide template cache keyed by (tol, mu): building a Taylor
    // integrator hits Heyoka's LLVM cache, which serializes pool workers
    // against each other (zero parallel speedup). Copy per use instead:
    // state/pars are overwritten every tick anyway.
    try {
        struct Key {
            double tol, mu;
            bool operator==(const Key &o) const {
                return tol == o.tol && mu == o.mu;
            }
        };
        struct KeyHash {
            std::size_t operator()(const Key &k) const noexcept {
                std::size_t h = std::hash<double>{}(k.tol);
                h ^= std::hash<double>{}(k.mu) + 0x9e3779b9 + (h << 6) + (h >> 2);
                return h;
            }
        };
        static std::unordered_map<Key, TaHolder, KeyHash> tplCache;
        const Key key{this->tol, this->mu};
        auto it = tplCache.find(key);
        if (it == tplCache.end()) {
            it = tplCache
                     .emplace(key, TaHolder(buildRwSystem(this->mu),
                                            std::vector<double>(15, 0.0), this->tol))
                     .first;
        }
        this->ta = std::make_unique<TaHolder>(it->second);
    } catch (const std::exception &e) {
        this->bskLogger.bskError("HeyokaRWDynamics: integrator build failed: %s.", e.what());
    }
    this->configDirty = false;
}

void HeyokaRWDynamics::writeMsgs(uint64_t CurrentSimNanos) {
    SCStatesMsgPayload sc = this->scStateOutMsg.zeroMsgPayload;
    for (int i = 0; i < 3; ++i) {
        sc.r_CN_N[i] = this->st[i];
        sc.v_CN_N[i] = this->st[3 + i];
        sc.r_BN_N[i] = this->st[i];  // body origin == CoM in this model
        sc.v_BN_N[i] = this->st[3 + i];
        sc.sigma_BN[i] = this->st[6 + i];
        sc.omega_BN_B[i] = this->st[9 + i];
    }
    sc.MRPSwitchCount = this->mrpSwitches;
    this->scStateOutMsg.write(&sc, this->moduleID, CurrentSimNanos);
    RWSpeedMsgPayload rw = this->rwSpeedOutMsg.zeroMsgPayload;
    for (int i = 0; i < 3; ++i) {
        rw.wheelSpeeds[i] = this->st[12 + i];
    }
    this->rwSpeedOutMsg.write(&rw, this->moduleID, CurrentSimNanos);
}

void HeyokaRWDynamics::Reset(uint64_t CurrentSimNanos) {
    this->initDone = false;
    this->prevNanos = 0;
    this->mrpSwitches = 0;
    this->buildIntegrator();
    this->bskLogger.bskLog(BSK_INFORMATION,
                           "HeyokaRWDynamics reset with tol %g and mu %g.",
                           this->tol, this->mu);

    SCStatesMsgPayload scZero = this->scStateOutMsg.zeroMsgPayload;
    this->scStateOutMsg.write(&scZero, this->moduleID, CurrentSimNanos);
    RWSpeedMsgPayload rwZero = this->rwSpeedOutMsg.zeroMsgPayload;
    this->rwSpeedOutMsg.write(&rwZero, this->moduleID, CurrentSimNanos);
}

void HeyokaRWDynamics::UpdateState(uint64_t CurrentSimNanos) {
    if (!this->initDone) {
        // first tick: publish ICs (no motor message exists yet)
        for (int i = 0; i < 3; ++i) {
            this->st[i] = this->rNInit[i];
            this->st[3 + i] = this->vNInit[i];
            this->st[6 + i] = this->sigmaInit[i];
            this->st[9 + i] = this->omegaInit[i];
            this->st[12 + i] = this->wsInitRPM[i] * RPM2RPS;
        }
        this->writeMsgs(CurrentSimNanos);
        this->prevNanos = CurrentSimNanos;
        this->initDone = true;
        return;
    }

    if (this->configDirty) {
        // post-Reset config change: rebuild rather than silently going stale
        this->buildIntegrator();
    }
    // scheduler step from message timestamps (monotonic in normal runs)
    const double dt = static_cast<double>(CurrentSimNanos - this->prevNanos) * 1e-9;
    this->prevNanos = CurrentSimNanos;
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        this->bskLogger.bskLog(BSK_WARNING,
                               "HeyokaRWDynamics: non-positive dt, republishing state.");
        this->writeMsgs(CurrentSimNanos);
        return;
    }

    // motor torques (zero when unlinked, matching the Python wrapper)
    double u[3] = {};
    if (this->rwMotorCmdInMsg.isLinked()) {
        const ArrayMotorTorqueMsgPayload msg = this->rwMotorCmdInMsg();
        for (int i = 0; i < 3; ++i) {
            const double v = msg.motorTorque[i];
            u[i] = v < -this->uMax ? -this->uMax : (v > this->uMax ? this->uMax : v);
        }
    }

    heyoka::taylor_adaptive<double> &t = this->ta->ta;
    double *ST = t.get_state_data();
    double *PA = t.get_pars_data();
    for (int i = 0; i < 15; ++i) {
        ST[i] = this->st[i];
    }
    PA[0] = u[0];
    PA[1] = u[1];
    PA[2] = u[2];
    PA[3] = this->mass;
    PA[4] = this->Ixx;
    PA[5] = this->Iyy;
    PA[6] = this->Izz;
    PA[7] = this->Ixy;
    PA[8] = this->Ixz;
    PA[9] = this->Iyz;
    for (int i = 0; i < 6; ++i) {
        PA[10 + i] = this->invI6[i];
    }
    PA[16] = this->J1;
    PA[17] = this->J2;
    PA[18] = this->J3;

    t.set_time(0.0);
    try {
        t.propagate_for(dt);
    } catch (const std::exception &e) {
        this->bskLogger.bskError("HeyokaRWDynamics: propagation failed: %s.", e.what());
    }

    double n2 = ST[6] * ST[6] + ST[7] * ST[7] + ST[8] * ST[8];
    if (n2 > 1.0) {  // MRP shadow-set reset
        ST[6] /= -n2;
        ST[7] /= -n2;
        ST[8] /= -n2;
        ++this->mrpSwitches;
    }
    for (int i = 0; i < 15; ++i) {
        this->st[i] = ST[i];
    }
    this->writeMsgs(CurrentSimNanos);
}

void HeyokaRWDynamics::setR_CN_NInit(double x, double y, double z) {
    this->rNInit[0] = x;
    this->rNInit[1] = y;
    this->rNInit[2] = z;
}
void HeyokaRWDynamics::setV_CN_NInit(double x, double y, double z) {
    this->vNInit[0] = x;
    this->vNInit[1] = y;
    this->vNInit[2] = z;
}
void HeyokaRWDynamics::setSigmaBNInit(double s1, double s2, double s3) {
    this->sigmaInit[0] = s1;
    this->sigmaInit[1] = s2;
    this->sigmaInit[2] = s3;
}
void HeyokaRWDynamics::setOmegaBNInit(double x, double y, double z) {
    this->omegaInit[0] = x;
    this->omegaInit[1] = y;
    this->omegaInit[2] = z;
}
void HeyokaRWDynamics::setWheelSpeedsInitRPM(double w1, double w2, double w3) {
    this->wsInitRPM[0] = w1;
    this->wsInitRPM[1] = w2;
    this->wsInitRPM[2] = w3;
}
void HeyokaRWDynamics::setMass(double m) { this->mass = m; }
void HeyokaRWDynamics::setInertia(double Ixx, double Iyy, double Izz,
                                  double Ixy, double Ixz, double Iyz) {
    this->Ixx = Ixx;
    this->Iyy = Iyy;
    this->Izz = Izz;
    this->Ixy = Ixy;
    this->Ixz = Ixz;
    this->Iyz = Iyz;
    this->configDirty = true;
}
void HeyokaRWDynamics::setRotorInertia(double J1, double J2, double J3) {
    this->J1 = J1;
    this->J2 = J2;
    this->J3 = J3;
}
void HeyokaRWDynamics::setUMax(double uMax) { this->uMax = uMax; }

double HeyokaRWDynamics::getSigmaBN(int i) {
    if (i < 0 || i > 2) {
        this->bskLogger.bskError("HeyokaRWDynamics: getSigmaBN index out of range.");
        return NAN;
    }
    return this->st[6 + i];
}
double HeyokaRWDynamics::getOmegaBN(int i) {
    if (i < 0 || i > 2) {
        this->bskLogger.bskError("HeyokaRWDynamics: getOmegaBN index out of range.");
        return NAN;
    }
    return this->st[9 + i];
}
double HeyokaRWDynamics::getWheelSpeed(int i) {
    if (i < 0 || i > 2) {
        this->bskLogger.bskError("HeyokaRWDynamics: getWheelSpeed index out of range.");
        return NAN;
    }
    return this->st[12 + i];
}
void HeyokaRWDynamics::setTol(double tol) {
    this->tol = tol;
    this->configDirty = true;
}
void HeyokaRWDynamics::setMu(double mu) {
    this->mu = mu;
    this->configDirty = true;
}
