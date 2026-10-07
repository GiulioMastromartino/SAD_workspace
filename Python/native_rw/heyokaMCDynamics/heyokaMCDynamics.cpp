/* ISC License - see native_rw/LICENSE. See heyokaMCDynamics.h for the contract. */

#include "heyokaMCDynamics.h"

#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include <heyoka/heyoka.hpp>

namespace {
using heyoka::expression;
using heyoka::prime;

/*! Build the 12-state MRP system (mu/gg baked as constants).
 *  pars: [Fx,Fy,Fz,Tx,Ty,Tz, m, Ixx,Iyy,Izz,Ixy,Ixz,Iyz,
 *         invIxx,invIyy,invIzz,invIxy,invIxz,invIyz] (same as pack_pars). */
std::vector<std::pair<expression, expression>> buildMcSystem(double mu, bool gg) {
    auto vars = heyoka::make_vars("rx", "ry", "rz", "vx", "vy", "vz", "s1", "s2",
                                  "s3", "omx", "omy", "omz");
    expression rx = vars[0], ry = vars[1], rz = vars[2], vx = vars[3],
               vy = vars[4], vz = vars[5], s1 = vars[6], s2 = vars[7],
               s3 = vars[8], omx = vars[9], omy = vars[10], omz = vars[11];
    auto P = [&](int i) { return expression(heyoka::par[i]); };
    expression Fx = P(0), Fy = P(1), Fz = P(2), Tx = P(3), Ty = P(4), Tz = P(5);
    expression m = P(6);
    expression Ixx = P(7), Iyy = P(8), Izz = P(9), Ixy = P(10), Ixz = P(11),
               Iyz = P(12);
    expression ixx = P(13), iyy = P(14), izz = P(15), ixy = P(16), ixz = P(17),
               iyz = P(18);

    // MRP -> DCM (body-from-inertial)
    expression sn2 = s1 * s1 + s2 * s2 + s3 * s3;
    expression d = (1.0 + sn2) * (1.0 + sn2);
    expression c = 4.0 * (1.0 - sn2) / d, k8 = 8.0 / d;
    expression s11 = -s2 * s2 - s3 * s3, s12 = s1 * s2, s13 = s1 * s3;
    expression s21 = s1 * s2, s22 = -s1 * s1 - s3 * s3, s23 = s2 * s3;
    expression s31 = s1 * s3, s32 = s2 * s3, s33 = -s1 * s1 - s2 * s2;
    expression R00 = 1.0 + k8 * s11, R01 = k8 * s12 + c * s3, R02 = k8 * s13 - c * s2;
    expression R10 = k8 * s21 - c * s3, R11 = 1.0 + k8 * s22, R12 = k8 * s23 + c * s1;
    expression R20 = k8 * s31 + c * s2, R21 = k8 * s32 - c * s1, R22 = 1.0 + k8 * s33;

    // translation: Kepler + body thrust rotated to ECI (F_N = R^T F_B)
    expression Rnorm = heyoka::sqrt(rx * rx + ry * ry + rz * rz);
    expression invR3 = 1.0 / (Rnorm * Rnorm * Rnorm);
    expression ax = -mu * rx * invR3 + (R00 * Fx + R10 * Fy + R20 * Fz) / m;
    expression ay = -mu * ry * invR3 + (R01 * Fx + R11 * Fy + R21 * Fz) / m;
    expression az = -mu * rz * invR3 + (R02 * Fx + R12 * Fy + R22 * Fz) / m;

    // gravity-gradient torque with full inertia
    expression tggx{0.0}, tggy{0.0}, tggz{0.0};
    if (gg) {
        expression rbx = R00 * rx + R01 * ry + R02 * rz;
        expression rby = R10 * rx + R11 * ry + R12 * rz;
        expression rbz = R20 * rx + R21 * ry + R22 * rz;
        expression invR5 = invR3 / (Rnorm * Rnorm);
        expression k = 3.0 * mu * invR5;
        expression Irx = Ixx * rbx + Ixy * rby + Ixz * rbz;
        expression Iry = Ixy * rbx + Iyy * rby + Iyz * rbz;
        expression Irz = Ixz * rbx + Iyz * rby + Izz * rbz;
        tggx = k * (rby * Irz - rbz * Iry);
        tggy = k * (rbz * Irx - rbx * Irz);
        tggz = k * (rbx * Iry - rby * Irx);
    }

    // MRP kinematics
    expression B11 = 1.0 - sn2 + 2.0 * s1 * s1, B12 = 2.0 * (s1 * s2 - s3),
               B13 = 2.0 * (s1 * s3 + s2);
    expression B21 = 2.0 * (s2 * s1 + s3), B22 = 1.0 - sn2 + 2.0 * s2 * s2,
               B23 = 2.0 * (s2 * s3 - s1);
    expression B31 = 2.0 * (s3 * s1 - s2), B32 = 2.0 * (s3 * s2 + s1),
               B33 = 1.0 - sn2 + 2.0 * s3 * s3;
    expression s1dot = 0.25 * (B11 * omx + B12 * omy + B13 * omz);
    expression s2dot = 0.25 * (B21 * omx + B22 * omy + B23 * omz);
    expression s3dot = 0.25 * (B31 * omx + B32 * omy + B33 * omz);

    // hub Euler with full inertia
    expression Iwx = Ixx * omx + Ixy * omy + Ixz * omz;
    expression Iwy = Ixy * omx + Iyy * omy + Iyz * omz;
    expression Iwz = Ixz * omx + Iyz * omy + Izz * omz;
    expression gx = -(omy * Iwz - omz * Iwy) + Tx + tggx;
    expression gy = -(omz * Iwx - omx * Iwz) + Ty + tggy;
    expression gz = -(omx * Iwy - omy * Iwx) + Tz + tggz;
    expression wdotx = ixx * gx + ixy * gy + ixz * gz;
    expression wdoty = ixy * gx + iyy * gy + iyz * gz;
    expression wdotz = ixz * gx + iyz * gy + izz * gz;

    return {{prime(rx) = vx, prime(ry) = vy, prime(rz) = vz, prime(vx) = ax,
             prime(vy) = ay, prime(vz) = az, prime(s1) = s1dot,
             prime(s2) = s2dot, prime(s3) = s3dot, prime(omx) = wdotx,
             prime(omy) = wdoty, prime(omz) = wdotz}};
}

/*! Analytic inverse of a symmetric 3x3, packed [xx,yy,zz,xy,xz,yz]. */
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

/*! Measured thrusterDynamicEffector staircase (cf. quantize_duty_cycle). */
inline double quantizeStair(double u) {
    if (u <= 0.0) {
        return 0.0;
    }
    if (u < 0.5) {
        return 1.0 / 6.0;
    }
    if (u < 1.0) {
        return 5.0 / 6.0;
    }
    return 1.0;
}
}  // namespace

/*! PIMPL holder: keeps heyoka types out of the header/SWIG wrapper TU. */
struct HeyokaMCDynamics::TaHolder {
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

HeyokaMCDynamics::HeyokaMCDynamics() = default;
HeyokaMCDynamics::~HeyokaMCDynamics() = default;

void HeyokaMCDynamics::buildIntegrator() {
    if (!(this->mass > 0.0) || !std::isfinite(this->mass)) {
        this->bskLogger.bskError("HeyokaMCDynamics: mass must be finite positive.");
    }
    if (!(this->tol > 0.0) || !std::isfinite(this->tol)) {
        this->bskLogger.bskError("HeyokaMCDynamics: tol must be finite positive.");
    }
    if (!(this->mu > 0.0) || !std::isfinite(this->mu)) {
        this->bskLogger.bskError("HeyokaMCDynamics: mu must be finite positive.");
    }
    if (this->MF.size() != 3 * NTHR || this->MT.size() != 3 * NTHR) {
        this->bskLogger.bskError("HeyokaMCDynamics: wrench matrices need 36 entries.");
    }
    const double I6[6] = {this->Ixx, this->Iyy, this->Izz,
                          this->Ixy, this->Ixz, this->Iyz};
    if (!symInv33(I6, this->invI6)) {
        this->bskLogger.bskError("HeyokaMCDynamics: inertia matrix is singular.");
    }
    // process-wide template cache keyed by (tol, mu, gg): building hits
    // Heyoka's LLVM cache, which serializes pool workers; copy per use
    try {
        struct Key {
            double tol, mu;
            bool gg;
            bool operator==(const Key &o) const {
                return tol == o.tol && mu == o.mu && gg == o.gg;
            }
        };
        struct KeyHash {
            std::size_t operator()(const Key &k) const noexcept {
                std::size_t h = std::hash<double>{}(k.tol);
                h ^= std::hash<double>{}(k.mu) + 0x9e3779b9 + (h << 6) + (h >> 2);
                h ^= std::hash<int>{}((int)k.gg);
                return h;
            }
        };
        static std::unordered_map<Key, TaHolder, KeyHash> tplCache;
        const Key key{this->tol, this->mu, this->useGG};
        auto it = tplCache.find(key);
        if (it == tplCache.end()) {
            it = tplCache
                     .emplace(key, TaHolder(buildMcSystem(this->mu, this->useGG),
                                            std::vector<double>(12, 0.0), this->tol))
                     .first;
        }
        this->ta = std::make_unique<TaHolder>(it->second);
    } catch (const std::exception &e) {
        this->bskLogger.bskError("HeyokaMCDynamics: integrator build failed: %s.", e.what());
    }
    this->configDirty = false;
}

void HeyokaMCDynamics::writeMsgs(uint64_t CurrentSimNanos) {
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
}

void HeyokaMCDynamics::Reset(uint64_t CurrentSimNanos) {
    this->initDone = false;
    this->prevNanos = 0;
    this->mrpSwitches = 0;
    this->buildIntegrator();
    this->bskLogger.bskLog(BSK_INFORMATION,
                           "HeyokaMCDynamics reset with tol %g and mu %g.",
                           this->tol, this->mu);

    SCStatesMsgPayload scZero = this->scStateOutMsg.zeroMsgPayload;
    this->scStateOutMsg.write(&scZero, this->moduleID, CurrentSimNanos);
}

void HeyokaMCDynamics::UpdateState(uint64_t CurrentSimNanos) {
    // throttle from the on-time message (12 used slots, duty = onTime/dt)
    const double dtMsg = static_cast<double>(CurrentSimNanos - this->prevNanos) * 1e-9;
    double ot[12] = {};
    if (this->thrOnTimeInMsg.isLinked()) {
        const THRArrayOnTimeCmdMsgPayload msg = this->thrOnTimeInMsg();
        constexpr int NAVAIL =
            sizeof(msg.OnTimeRequest) / sizeof(msg.OnTimeRequest[0]);
        for (int i = 0; i < NTHR && i < NAVAIL; ++i) {
            ot[i] = msg.OnTimeRequest[i];
        }
    }

    if (!this->initDone) {
        // first tick: publish ICs, stage current wrench for the next tick
        // (no motor message exists yet on tick 0)
        for (int i = 0; i < 3; ++i) {
            this->st[i] = this->rNInit[i];
            this->st[3 + i] = this->vNInit[i];
            this->st[6 + i] = this->sigmaInit[i];
            this->st[9 + i] = this->omegaInit[i];
        }
        for (int i = 0; i < 3; ++i) {
            this->prevF[i] = 0.0;
            this->prevT[i] = 0.0;
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
    const double dt = dtMsg <= 0.0 ? 0.0 : dtMsg;
    this->prevNanos = CurrentSimNanos;
    if (!(dt > 0.0) || !std::isfinite(dt)) {
        this->bskLogger.bskLog(BSK_WARNING,
                               "HeyokaMCDynamics: non-positive dt, republishing state.");
        this->writeMsgs(CurrentSimNanos);
        return;
    }

    // current-tick wrench from throttle duty (quantized like the effector)
    double Fx = 0.0, Fy = 0.0, Fz = 0.0, Tx = 0.0, Ty = 0.0, Tz = 0.0;
    for (int i = 0; i < NTHR; ++i) {
        double duty = ot[i] / dt;
        duty = duty < 0.0 ? 0.0 : (duty > 1.0 ? 1.0 : duty);
        const double q = this->quantize ? quantizeStair(duty) : duty;
        Fx += this->MF[0 * NTHR + i] * q;
        Fy += this->MF[1 * NTHR + i] * q;
        Fz += this->MF[2 * NTHR + i] * q;
        Tx += this->MT[0 * NTHR + i] * q;
        Ty += this->MT[1 * NTHR + i] * q;
        Tz += this->MT[2 * NTHR + i] * q;
    }

    // propagate with the PREVIOUS tick's wrench (Basilisk mailbox/ZOH
    // semantics: the allocator output read this tick drove next tick)
    heyoka::taylor_adaptive<double> &t = this->ta->ta;
    double *ST = t.get_state_data();
    double *PA = t.get_pars_data();
    for (int i = 0; i < 12; ++i) {
        ST[i] = this->st[i];
    }
    PA[0] = this->prevF[0];
    PA[1] = this->prevF[1];
    PA[2] = this->prevF[2];
    PA[3] = this->prevT[0];
    PA[4] = this->prevT[1];
    PA[5] = this->prevT[2];
    PA[6] = this->mass;
    PA[7] = this->Ixx;
    PA[8] = this->Iyy;
    PA[9] = this->Izz;
    PA[10] = this->Ixy;
    PA[11] = this->Ixz;
    PA[12] = this->Iyz;
    for (int i = 0; i < 6; ++i) {
        PA[13 + i] = this->invI6[i];
    }

    t.set_time(0.0);
    try {
        t.propagate_for(dt);
    } catch (const std::exception &e) {
        this->bskLogger.bskError("HeyokaMCDynamics: propagation failed: %s.", e.what());
    }

    double n2 = ST[6] * ST[6] + ST[7] * ST[7] + ST[8] * ST[8];
    if (n2 > 1.0) {  // MRP shadow-set reset
        ST[6] /= -n2;
        ST[7] /= -n2;
        ST[8] /= -n2;
        ++this->mrpSwitches;
    }
    for (int i = 0; i < 12; ++i) {
        this->st[i] = ST[i];
    }
    this->prevF[0] = Fx;
    this->prevF[1] = Fy;
    this->prevF[2] = Fz;
    this->prevT[0] = Tx;
    this->prevT[1] = Ty;
    this->prevT[2] = Tz;
    this->writeMsgs(CurrentSimNanos);
}

void HeyokaMCDynamics::setR_CN_NInit(double x, double y, double z) {
    this->rNInit[0] = x;
    this->rNInit[1] = y;
    this->rNInit[2] = z;
}
void HeyokaMCDynamics::setV_CN_NInit(double x, double y, double z) {
    this->vNInit[0] = x;
    this->vNInit[1] = y;
    this->vNInit[2] = z;
}
void HeyokaMCDynamics::setSigmaBNInit(double s1, double s2, double s3) {
    this->sigmaInit[0] = s1;
    this->sigmaInit[1] = s2;
    this->sigmaInit[2] = s3;
}
void HeyokaMCDynamics::setOmegaBNInit(double x, double y, double z) {
    this->omegaInit[0] = x;
    this->omegaInit[1] = y;
    this->omegaInit[2] = z;
}
void HeyokaMCDynamics::setMass(double m) { this->mass = m; }
void HeyokaMCDynamics::setInertia(double Ixx, double Iyy, double Izz,
                                  double Ixy, double Ixz, double Iyz) {
    this->Ixx = Ixx;
    this->Iyy = Iyy;
    this->Izz = Izz;
    this->Ixy = Ixy;
    this->Ixz = Ixz;
    this->Iyz = Iyz;
    this->configDirty = true;
}
void HeyokaMCDynamics::setForceMatrix(const std::vector<double> &MF) {
    this->MF = MF;
}
void HeyokaMCDynamics::setTorqueMatrix(const std::vector<double> &MT) {
    this->MT = MT;
}
void HeyokaMCDynamics::setQuantize(bool quantize) { this->quantize = quantize; }
void HeyokaMCDynamics::setUseGravityGradient(bool useGG) {
    this->useGG = useGG;
    this->configDirty = true;
}
void HeyokaMCDynamics::setTol(double tol) {
    this->tol = tol;
    this->configDirty = true;
}
void HeyokaMCDynamics::setMu(double mu) {
    this->mu = mu;
    this->configDirty = true;
}

double HeyokaMCDynamics::getSigmaBN(int i) {
    if (i < 0 || i > 2) {
        this->bskLogger.bskError("HeyokaMCDynamics: getSigmaBN index out of range.");
        return NAN;
    }
    return this->st[6 + i];
}
double HeyokaMCDynamics::getOmegaBN(int i) {
    if (i < 0 || i > 2) {
        this->bskLogger.bskError("HeyokaMCDynamics: getOmegaBN index out of range.");
        return NAN;
    }
    return this->st[9 + i];
}
