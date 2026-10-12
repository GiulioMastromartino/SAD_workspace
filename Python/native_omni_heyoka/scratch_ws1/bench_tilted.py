"""Tilted-wheel demo: stock Basilisk vs omni runtime-composed dynamics.

WHY THIS BENCHMARK SHAPE
------------------------
The aligned hub+RW case is exactly what native_rw already does, so timing it
proves nothing about generality.  Tilted/canted spin axes are different:

  * stock Basilisk supports them -- rwFactory.create(rwType, gsHat_B) takes an
    arbitrary unit spin axis,
  * omni supports them by construction -- a contributor's gs is a free vector,
    so the SAME solve handles them with no new code,
  * native_rw CANNOT -- its documented envelope is "3 body-axis-aligned wheels".

COMPARISON, STATED HONESTLY
---------------------------
stock = full Basilisk pipeline: spacecraft + RW effector + FSW + nav + logging.
omni  = dynamics only: the Taylor integrator over the runtime-composed symbolic
        system, driven by the same PD law and the same generalised allocation.

These are not the same amount of work.  stock includes FSW, navigation and
message logging; omni includes only the dynamics.  The omni number is therefore
the cost of the SWAPPED SLICE -- which is exactly what the README's Amdahl
argument is about, and why a 10x dynamics speedup can shrink to ~1x when the
FSW dominates.  Both are reported so the slice share is visible.
"""
from __future__ import annotations

import sys
import time

import numpy as np

SRC = "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python"
sys.path.insert(0, SRC)
sys.path.insert(0, SRC + "/native_omni_heyoka")

import heyoka as hy

from cubesat_12u_sim.backends.heyoka import heyoka_rw as hrw
from cubesat_12u_sim.backends.heyoka.heyoka_attitude import mrp_diff
from omniheyoka._parser import parse_system

# ---------------------------------------------------------------------------
# demo configuration
# ---------------------------------------------------------------------------
DURATION, DT = 600.0, 0.1
K, P = 3.5, 30.0
U_MAX = 0.2
S3 = 1.0 / np.sqrt(3.0)
AXES = np.array([[S3, S3, S3], [-S3, S3, S3], [S3, -S3, S3]])
MU, MASS = 3.986004418e14, 750.0
INERTIA = np.diag([900.0, 800.0, 600.0])
JS = np.full(3, 0.07957747154594767)
RPM0 = np.array([100.0, 200.0, 300.0])


def ics():
    from Basilisk.utilities import macros, orbitalMotion
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    return orbitalMotion.elem2rv(MU, oe)


def aligned_u(Lr):
    """Body-axis-aligned allocation (native_rw / heyoka_rw convention)."""
    return np.clip(-np.asarray(Lr, float), -U_MAX, U_MAX)


def tilted_u(Lr, axes=AXES):
    """Generalised allocation for ARBITRARY spin axes.

    Wheel motor torque u_i produces bus reaction -sum_i u_i g_i; require it to
    equal the demanded Lr, so G u = -Lr with G = [g1 g2 g3].
    """
    G = np.asarray(axes, float).T  # (3,3), columns are the spin axes
    return np.clip(np.linalg.solve(G, -np.asarray(Lr, float)), -U_MAX, U_MAX)


# ---------------------------------------------------------------------------
# STOCK: full Basilisk pipeline with tilted wheels
# ---------------------------------------------------------------------------
def run_tilted_stock(duration=DURATION, dt=DT):
    from Basilisk.architecture import messaging
    from Basilisk.fswAlgorithms import (
        attTrackingError, inertial3D, mrpFeedback, rwMotorTorque)
    from Basilisk.simulation import reactionWheelStateEffector, simpleNav
    from Basilisk.utilities import (
        SimulationBaseClass, macros, simHelpers, simIncludeGravBody, simIncludeRW)

    r0, v0 = ics()
    I = INERTIA
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("simProcess")
    proc.addTask(sim.CreateNewTask("simTask", macros.sec2nano(dt)))

    from Basilisk.simulation import spacecraft
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "bsk-Sat"
    sc.hub.mHub = MASS
    sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(np.asarray(I, float).ravel())
    sim.AddModelToTask("simTask", sc, 1)

    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)

    rw_factory = simIncludeRW.rwFactory()
    for i in range(3):  # TILTED spin axes -- the only difference from stock
        rw_factory.create("Honeywell_HR16", [float(v) for v in AXES[i]],
                          maxMomentum=50.0, Omega=float(RPM0[i]),
                          rWB_B=[0.5, 0.5, 0.5])
    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = "RW_cluster"
    rw_factory.addToSpacecraft(rw_effector.ModelTag, rw_effector, sc)
    sim.AddModelToTask("simTask", rw_effector, 2)

    nav = simpleNav.SimpleNav()
    sim.AddModelToTask("simTask", nav)
    ref = inertial3D.inertial3D()
    sim.AddModelToTask("simTask", ref)
    ref.sigma_R0N = [0.0, 0.0, 0.0]
    err = attTrackingError.attTrackingError()
    sim.AddModelToTask("simTask", err)
    ctl = mrpFeedback.mrpFeedback()
    sim.AddModelToTask("simTask", ctl)
    ctl.K, ctl.Ki, ctl.P = float(K), -1, float(P)
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1
    alloc = rwMotorTorque.rwMotorTorque()
    sim.AddModelToTask("simTask", alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    vc_msg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=np.asarray(I, float).ravel()))
    fsw_msg = rw_factory.getConfigMessage()

    sc.hub.r_CN_NInit = r0
    sc.hub.v_CN_NInit = v0
    sc.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    sc.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    nav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
    err.attNavInMsg.subscribeTo(nav.attOutMsg)
    err.attRefInMsg.subscribeTo(ref.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
    ctl.vehConfigInMsg.subscribeTo(vc_msg)
    ctl.rwParamsInMsg.subscribeTo(fsw_msg)
    ctl.rwSpeedsInMsg.subscribeTo(rw_effector.rwSpeedOutMsg)
    alloc.rwParamsInMsg.subscribeTo(fsw_msg)
    alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
    rw_effector.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)

    state_log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("simTask", state_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    ns = macros.NANO2SEC
    return {"sigma_BN": np.asarray(state_log.sigma_BN, float),
            "omega_BN_B": np.asarray(state_log.omega_BN_B, float),
            "r_CN_N": np.asarray(state_log.r_CN_N, float)}


# ---------------------------------------------------------------------------
# OMNI: dynamics only, runtime-composed symbolic system, Taylor stepping
# ---------------------------------------------------------------------------
def load_omni(path="/tmp/omni_tilted_dump.txt"):
    rows = {}
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        lhs, rhs = line.split(" = ", 1)
        rows[lhs.strip()] = rhs.strip()
    names = list(rows)
    sysrows = parse_system([rows[n] for n in names], names, 19)
    return sysrows, names


def run_omni(duration=DURATION, dt=DT, sysrows=None):
    """Same closed loop as stock; dynamics swapped for the composed system."""
    if sysrows is None:
        sysrows, _ = load_omni()
    I6 = np.concatenate([np.diag(INERTIA), np.zeros((3, 3))], axis=1).ravel()
    P = np.concatenate([[0.0, 0.0, 0.0], [MASS],
                        np.diag(INERTIA), np.diag(np.linalg.inv(INERTIA)), JS])
    r0, v0 = ics()
    y = np.concatenate([r0, v0, [0.1, 0.2, -0.3], [0.001, -0.01, 0.03],
                        RPM0 * 2 * np.pi / 60.0])

    ta = hy.taylor_adaptive(sysrows, y.tolist(), pars=list(map(float, P)),
                            tol=1e-13, compact_mode=False)
    n = int(round(duration / dt))
    out = np.zeros((n + 1, 15))
    out[0] = y
    for k in range(1, n + 1):
        st = out[k - 1]
        e = mrp_diff(st[6:9], np.zeros(3))
        Lr = -K * e - P * st[9:12]
        u = tilted_u(Lr)
        ta.pars[:] = list(map(float, u))
        ta.state[:] = st
        ta.propagate_for(dt)
        sn = np.asarray(ta.state, float)
        s2 = sn[6:9] @ sn[6:9]
        if s2 > 1.0:
            sn[6:9] = -sn[6:9] / s2
        out[k] = sn
    return out


def timeit(fn, *a, repeat=3, warmup=1, **kw):
    for _ in range(warmup):
        fn(*a, **kw)
    ts = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        r = fn(*a, **kw)
        ts.append(time.perf_counter() - t0)
    return min(ts), r


if __name__ == "__main__":
    print(__doc__)
    sysrows, names = load_omni()

    t_stock, stock = timeit(run_tilted_stock)
    t_omni, omni = timeit(run_omni, sysrows=sysrows)

    print(f"config : 3 tilted wheels (pyre, 1/sqrt(3) each), diag(900,800,600),"
          f" 750 kg")
    print(f"         {DURATION} s at dt={DT} -> {int(DURATION/DT)} ticks")
    print()
    print(f"stock Basilisk (FULL pipeline, tilted) : {t_stock*1e3:9.2f} ms")
    print(f"omni (dynamics slice only, tilted)     : {t_omni*1e3:9.2f} ms")
    print(f"  -> dynamics slice is {t_stock/t_omni:5.2f}x cheaper than the "
          f"whole stock pipeline")
    print(f"  -> slice share of stock runtime: {100*t_omni/t_stock:5.1f}%")
    print()

    # parity
    n = min(len(stock["sigma_BN"]), len(omni))
    s_s, s_o = stock["sigma_BN"][:n], omni[:n, 6:9]
    att = np.degrees(4 * np.arctan(np.linalg.norm(mrp_diff(s_o[-1], s_s[-1]))))
    rate = np.degrees(np.linalg.norm(omni[-1, 9:12] - stock["omega_BN_B"][-1]))
    print(f"parity (final state, stock vs omni):")
    print(f"  attitude : {att:.3e} deg")
    print(f"  rate     : {rate:.3e} deg/s")
    print(f"  |sigma|  : omni {np.linalg.norm(omni[-1,6:9]):.6f}  "
          f"stock {np.linalg.norm(s_s[-1]):.6f}")
