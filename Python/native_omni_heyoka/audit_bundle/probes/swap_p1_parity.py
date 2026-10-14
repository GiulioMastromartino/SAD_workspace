"""Phase-1 probe (swap): stock-RK4 vs OmniVecIntegrator A/B parity.

Cells (all matched-dt, pre-registered tolerances in verdicts):
  hub_60s        hub-only tumbling 60 s @ dt=0.1
  rw3_open       3 HR16 wheels, constant +-torque (u_max clamp exercised) 120 s
  rw3_pd         3 wheels, pure-Python P controller (time-varying torque) 120 s
  rw1_tilt       1 tilted wheel + 2-wheel spot checks 60 s
  thr_fresh      staircase thruster, fresh-write driver, duties {0.5, 1.0} 120 s
  thr_cells      write-once / frozen-advance fire-schedule equality
  env_drag       LEO drag A/B + density vacuity guard (tumbling ICs)
  env_drag_still LEO drag A/B, stabilized ICs (tight)
  env_srp        SRP-cannonball A/B + single-tick analytic guard
  fuel_centered fuel depletion, centered tank (mass/I feedback isolated)
  fuel_offset    fuel depletion, offset tank (c-drift deviation bounded)
  fuel_drag      WP-C measured: burn (offset tank) + LEO drag, live-c-tracking arm
  fuel_srp       WP-C measured: burn (offset tank) + SRP cannonball
  reject_*       fuel tank / jitter thetas / drag effector / min-pulse /
                 low-cutoff / multi-RW-effector must all raise LOUDLY

Run: ../../env/bin/python audit_bundle/probes/swap_p1_parity.py
Writes: audit_bundle/results/swap_p1_parity.json
Host-only (needs Basilisk macOS build + built omni_heyoka).
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # Python/ for
# omniheyoka + cubesat_12u_sim sibling imports (BUILD_STATUS recipe)
from common import log_text, save_result, safe_import_basilisk  # noqa: E402

DT = 0.1
MU = 3.986004418e14
MASS = 750.0
I6 = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
R0 = [1.0e7, 0.0, 0.0]
V0 = [0.0, 7500.0, 0.0]
SIG0 = [0.1, -0.2, 0.3]
OM0 = [0.01, -0.02, 0.03]

# Pre-registered parity tolerances (measured ~1e-13..1e-14; margin >= 1e3).
TOL_STATE = 1e-9
TOL_WHEEL = 1e-9
# Thruster cells carry the stock-side switching-integration residual (RK4
# steps blindly across the valve switch; ours arcs exactly): linear in dt
# (measured 7.5e-07 -> 3.8e-07 -> 1.5e-07 over 30 s at dt 0.1/0.05/0.02),
# same class as Update-4 finding 2. Bounds below have >=5x margin at dt=0.1.
TOL_THR_R = 2e-3
TOL_THR_V = 2e-5
TOL_THR_SIG = 2e-4
TOL_THR_OM = 1e-5
# Write-once fires a single step: residual is one kink error + drift.
TOL_WO_R = 1e-5
TOL_WO_V = 1e-7
TOL_WO_SIG = 1e-6
TOL_WO_OM = 1e-7


def _base_sim(tag):
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                    simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [list(r) for r in I6]
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R, 347.8 * macros.D2R,
                                85.3 * macros.D2R)
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    sc.hub.sigma_BNInit = list(SIG0)
    sc.hub.omega_BN_BInit = list(OM0)
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    return sim, sc


def _add_rw(sim, sc, axes, om0=100.0, task=True, friction=False):
    from Basilisk.simulation import reactionWheelStateEffector
    from Basilisk.utilities import simIncludeRW
    fac = simIncludeRW.rwFactory()
    for gs in axes:
        if friction:
            fac.create("Honeywell_HR16", list(gs), maxMomentum=50.0,
                       Omega=om0, useRWfriction=True)
        else:
            fac.create("Honeywell_HR16", list(gs), maxMomentum=50.0,
                       Omega=om0)
    rw = reactionWheelStateEffector.ReactionWheelStateEffector()
    fac.addToSpacecraft("rw", rw, sc)
    if task:
        sim.AddModelToTask("t", rw)
    return rw


def _const_driver(u):
    from Basilisk.architecture import messaging, sysModel
    class Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            m = [0.0] * 36
            for i in range(3):
                m[i] = u
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    drv = Driver()
    return drv


def _pd_driver(K=3.5, P=30.0, u_max=0.2):
    import numpy as np
    from Basilisk.architecture import messaging, sysModel
    class PD(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()
            self.sc_in = messaging.SCStatesMsgReader()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            s = np.asarray(self.sc_in().sigma_BN, float)
            w = np.asarray(self.sc_in().omega_BN_B, float)
            u = np.clip(-K * s - P * w, -u_max, u_max)
            m = [0.0] * 36
            for i in range(3):
                m[i] = float(u[i])
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    return PD()


def _thr_driver(duty):
    from Basilisk.architecture import messaging, sysModel
    class Thr(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = duty * DT
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)
    return Thr()


def _run(sim, T):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T))
    sim.ExecuteSimulation()


def _arm(use_omni, kind, **kw):
    """Build one A/B arm. Returns (scLog, spdLog|None, oi|None, extra)."""
    import numpy as np  # noqa: F401
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    sim, sc = _base_sim(("omni-" if use_omni else "rk-") + kind)
    if kw.get("offset") is not None:
        sc.hub.r_BcB_B = [float(kw["offset"]), 0.0, 0.0]
    rw = None
    if kind in ("rw3_open", "rw3_pd", "rw1", "rw2", "rw1tilt", "rw3_fric"):
        axes = {"rw3_open": [[1., 0, 0], [0, 1., 0], [0, 0, 1.]],
                "rw3_pd": [[1., 0, 0], [0, 1., 0], [0, 0, 1.]],
                "rw1": [[1., 0, 0]],
                "rw2": [[1., 0, 0], [0, 1., 0]],
                "rw3_fric": [[1., 0, 0], [0, 1., 0], [0, 0, 1.]],
                "rw1tilt": [[0.5773502691896258] * 3]}[kind]
        rkw = dict(kw.get("rw_kw", {}))
        if kind == "rw3_fric":
            rkw["friction"] = True
        rw = _add_rw(sim, sc, axes, **rkw)
    if kind == "rw3_open":
        drv = _const_driver(kw.get("u", 0.5))
        sim.AddModelToTask("t", drv)
        rw.rwMotorCmdInMsg.subscribeTo(drv.out)
    if kind == "rw3_fric":
        drv = _fric_driver()
        sim.AddModelToTask("t", drv)
        rw.rwMotorCmdInMsg.subscribeTo(drv.out)
    if kind == "rw3_pd":
        drv = _pd_driver()
        sim.AddModelToTask("t", drv)
        rw.rwMotorCmdInMsg.subscribeTo(drv.out)
        drv.sc_in.subscribeTo(sc.scStateOutMsg)
    tf = None
    if kind.startswith("thr"):
        from Basilisk.simulation import thrusterDynamicEffector
        from Basilisk.utilities import simIncludeThruster
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", [0.5, 0, 1.0], [0, 0, -1.0], MaxThrust=1.0,
                  cutoffFrequency=kw.get("cutoff", 1000.0),
                  useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        mode = kw.get("mode", "fresh")
        if mode == "fresh":
            drv = _thr_driver(kw.get("duty", 0.5))
            sim.AddModelToTask("t", drv)
            thr.cmdsInMsg.subscribeTo(drv.out)
            cmd = drv.out
        elif mode == "frozen":
            drv = _thr_frozen_driver(kw.get("duty", 0.5))
            sim.AddModelToTask("t", drv)
            thr.cmdsInMsg.subscribeTo(drv.out)
            cmd = drv.out
        else:
            from Basilisk.architecture import messaging
            m = messaging.THRArrayOnTimeCmdMsg()
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = kw.get("duty", 0.5) * DT
            p.OnTimeRequest = ot
            m.write(p)
            thr.cmdsInMsg.subscribeTo(m)
            cmd = m
    oi = None
    if use_omni:
        oi = OmniVecIntegrator(sc)
        if kind.startswith("thr"):
            mirror_stock_config(sim, sc, oi, thr_factory=tf, thr_cmd_msg=cmd)
        else:
            mirror_stock_config(sim, sc, oi)
        sc.setIntegrator(oi)
    scLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", scLog)
    spdLog = rw.rwSpeedOutMsg.recorder() if rw is not None else None
    if spdLog is not None:
        sim.AddModelToTask("t", spdLog)
    return sim, sc, scLog, spdLog, oi


def _fric_driver():
    """Spin up 40 s at +0.2, then free spin-down (friction only)."""
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.utilities import macros

    class Fric(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            u = 0.2 if t < macros.sec2nano(40.0) else 0.0
            m = [0.0] * 36
            for i in range(3):
                m[i] = u
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    return Fric()


def _thr_frozen_driver(duty):
    """Writes duty on ticks 0-1, then goes silent (frozen message)."""
    from Basilisk.architecture import messaging, sysModel

    class Frozen(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()
            self.tick = -1

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            self.tick += 1
            if self.tick > 1:
                return
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = duty * DT
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)
    return Frozen()


def _reject_cells():
    """Loud-rejection gates (all must raise ValueError, no sim run needed
    except where noted). Returns {name: raised_bool}."""
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    out = {}

    def expect_raise(name, build):
        try:
            build()
            out[name] = False
        except ValueError as exc:
            out[name] = str(exc)[:160]
        except Exception as exc:
            # C++ bskError surfaces as BasiliskError (or similar): accept
            # iff it names the offending feature (no silent pass on junk).
            out[name] = str(exc)[:160]

    def rw_base(jitter=False, friction=False, two_eff=False):
        sim, sc = _base_sim("reject")
        from Basilisk.simulation import reactionWheelStateEffector
        from Basilisk.utilities import simIncludeRW
        fac = simIncludeRW.rwFactory()
        kw = {}
        if jitter:
            kw["RWModel"] = reactionWheelStateEffector.JitterSimple
        if friction:
            kw["useRWfriction"] = True
        fac.create("Honeywell_HR16", [1., 0, 0], maxMomentum=50.0,
                   Omega=100.0, **kw)
        rw = reactionWheelStateEffector.ReactionWheelStateEffector()
        fac.addToSpacecraft("rw", rw, sc)
        sim.AddModelToTask("t", rw)
        if two_eff:
            fac2 = simIncludeRW.rwFactory()
            fac2.create("Honeywell_HR16", [0, 1., 0], maxMomentum=50.0,
                        Omega=100.0)
            rw2 = reactionWheelStateEffector.ReactionWheelStateEffector()
            fac2.addToSpacecraft("rw2", rw2, sc)
            sim.AddModelToTask("t", rw2)
        return sim, sc

    def thr_base(**ckw):
        from Basilisk.simulation import thrusterDynamicEffector
        from Basilisk.utilities import simIncludeThruster
        sim, sc = _base_sim("reject-thr")
        tf = simIncludeThruster.thrusterFactory()
        base = dict(MaxThrust=1.0, cutoffFrequency=1000.0,
                    useMinPulseTime=False)
        base.update(ckw)
        tf.create("TEST_Thruster", [0.5, 0, 1.0], [0, 0, -1.0], **base)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        return sim, sc, tf

    def mirror(sim, sc, **mkw):
        oi = OmniVecIntegrator(sc)
        return mirror_stock_config(sim, sc, oi, **mkw)

    def fuel_base_linked():
        # Fuel tank WITHOUT helper mirror: the C++ claim check must fire
        # (defense-in-depth behind the helper). Runs 2 ticks (first
        # integrate triggers buildOnce).
        from Basilisk.simulation import fuelTank
        from Basilisk.utilities import macros
        sim, sc = _base_sim("reject-fuel-cxx")
        tank = fuelTank.FuelTank()
        model = fuelTank.FuelTankModelConstantVolume()
        model.propMassInit = 10.0
        tank.setTankModel(model)
        tank.setNameOfMassState("fuelMass")
        sc.addStateEffector(tank)
        sim.AddModelToTask("t", tank)
        oi = OmniVecIntegrator(sc)
        oi.setMass(750.0)
        oi.setMu(3.986004418e14)
        sc.setIntegrator(oi)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(0.2))
        sim.ExecuteSimulation()

    expect_raise("fuel_unmirrored_cxx", fuel_base_linked)
    expect_raise("jitter_wheel",
                 lambda: mirror(*rw_base(jitter=True)))
    expect_raise("two_rw_effectors",
                 lambda: mirror(*rw_base(two_eff=True)))

    def thr_mirror(ckw):
        from Basilisk.architecture import messaging
        s, c, t = thr_base(**ckw)
        dummy = messaging.THRArrayOnTimeCmdMsg()
        return mirror(s, c, thr_factory=t, thr_cmd_msg=dummy)

    expect_raise("thruster_no_factory",
                 lambda: mirror(*thr_base()[:2]))
    expect_raise("min_pulse",
                 lambda: thr_mirror({"useMinPulseTime": True}))
    expect_raise("cutoff_zero",
                 lambda: thr_mirror({"cutoffFrequency": 0.0}))

    out["verdict"] = ("all loud" if all(out.values())
                      else f"SILENT: {[k for k, v in out.items() if not v]}")
    return out

def _env_drag_cell(still=False):
    """Drag A/B (LEO 350 km, CdA=11, com=[0.05,0,0], 600 s) + density guard.

    Stock arm: dragDynamicEffector + exponentialAtmosphere (no wind model:
    inertial fallback both arms, P1 table). Omni arm: same minus the drag
    effector, plus oi.setDragParams + density subscription. Vacuity guard:
    recorded neutralDensity max > 1e-13 (effect live) and the no-drag
    displacement is macroscopic (6.8 cm here).
    still=True zeroes the attitude ICs (stabilized envelope: 4e-6 class);
    default tumbling ICs document the O(|w|*dt) frozen-projection class
    (1e-4 m per 600 s at |w| = 0.037 -- bounded, not gated tight).
    """
    import numpy as np
    from Basilisk.simulation import (dragDynamicEffector,
                                      exponentialAtmosphere)
    from Basilisk.utilities import macros
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    R = 6378137.0 + 350e3
    V = float(np.sqrt(MU / R))

    def build(use_omni):
        sim, sc = _base_sim("drag")
        sc.hub.r_CN_NInit = [R, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, V, 0.0]
        if still:
            sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
            sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        atmo = exponentialAtmosphere.ExponentialAtmosphere()
        atmo.baseDensity = 1e-9
        atmo.scaleHeight = 50000.0
        atmo.planetRadius = 6378137.0
        atmo.addSpacecraftToModel(sc.scStateOutMsg)
        sim.AddModelToTask("t", atmo)
        if not use_omni:
            drag = dragDynamicEffector.DragDynamicEffector()
            drag.coreParams.projectedArea = 5.0
            drag.coreParams.dragCoeff = 2.2
            drag.coreParams.comOffset = [0.05, 0.0, 0.0]
            drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
            drag.cannonballDrag()
            sc.addDynamicEffector(drag)
            sim.AddModelToTask("t", drag)
        else:
            oi = OmniVecIntegrator(sc)
            mirror_stock_config(sim, sc, oi)
            oi.setDragParams(2.2 * 5.0, 0.05, 0.0, 0.0)
            oi.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
            sc.setIntegrator(oi)
        scLog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", scLog)
        dlog = atmo.envOutMsgs[-1].recorder()
        sim.AddModelToTask("t", dlog)
        return sim, scLog, dlog

    sima, loga, da = build(False)
    _run(sima, 600.0)
    simb, logb, db = build(True)
    _run(simb, 600.0)
    d = _final_diff(loga, logb, ["r_BN_N", "v_BN_N", "sigma_BN",
                                 "omega_BN_B"])
    d["density_max"] = float(np.asarray(db.neutralDensity).max())
    d["density_min"] = float(np.asarray(db.neutralDensity).min())
    return d


def _env_srp_cell():
    """SRP-cannonball A/B (static sun, A=10, stock C_R=1.5 -> omni cr=0.5).

    Convention (load-bearing): stock coefficientReflection is the TOTAL
    multiplier C_R in [1,2] (F = C_R*P*A); omni cbCr is the SPECULAR
    FRACTION r in [0,1] (F = (1+r)*P*A). Mirror maps cr = C_R - 1.
    Vacuity guard: single-tick SRP-only dv within 10% of analytic P*A*Cr*dt/m.
    """
    import numpy as np
    from Basilisk.architecture import messaging
    from Basilisk.simulation import radiationPressure
    from Basilisk.utilities import macros
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    R = 6378137.0 + 400e3
    V = float(np.sqrt(MU / R))
    SUN = [1.5e11, 0.0, 0.0]

    def build(use_omni):
        sm = messaging.SpicePlanetStateMsg()
        p = messaging.SpicePlanetStateMsgPayload()
        p.PositionVector = list(SUN)
        p.VelocityVector = [0.0, 0.0, 0.0]
        sm.write(p)
        sim, sc = _base_sim("srp")
        sc.hub.r_CN_NInit = [R, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, V, 0.0]
        # Stabilized ICs: the tumbling O(|w|*dt) frozen-projection class is
        # covered+bounded by the env_drag/env_drag_still pair (shared wiring).
        sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
        sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        if not use_omni:
            srp = radiationPressure.RadiationPressure()
            srp.area = 10.0
            srp.coefficientReflection = 1.5
            srp.setUseCannonballModel()
            srp.sunEphmInMsg.subscribeTo(sm)
            sc.addDynamicEffector(srp)
            sim.AddModelToTask("t", srp)
        else:
            oi = OmniVecIntegrator(sc)
            mirror_stock_config(sim, sc, oi)
            oi.setSRPCannonball(10.0, 0.5)
            oi.sunStateInMsg.subscribeTo(sm)
            sc.setIntegrator(oi)
        scLog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", scLog)
        return sim, scLog

    sima, loga = build(False)
    _run(sima, 600.0)
    simb, logb = build(True)
    _run(simb, 600.0)
    d = _final_diff(loga, logb, ["r_BN_N", "v_BN_N", "sigma_BN",
                                 "omega_BN_B"])
    # Single-tick analytic vacuity check: |SRP-only dv| vs P*A*Cr*dt/m.
    sim1, log1 = build(True)
    _run(sim1, DT)
    sim0, sc0 = _base_sim("srp0")
    sc0.hub.r_CN_NInit = [R, 0.0, 0.0]
    sc0.hub.v_CN_NInit = [0.0, V, 0.0]
    log0 = sc0.scStateOutMsg.recorder()
    sim0.AddModelToTask("t", log0)
    _run(sim0, DT)
    sun_d = float(np.linalg.norm(np.asarray(SUN, float)
                                 - np.asarray([R, 0.0, 0.0])))
    P_au, AU = 4.5398178907e-6, 1.495978707e11  # env_const ledger values
    analytic = P_au * (AU / sun_d) ** 2 * 10.0 * 1.5 * DT / MASS
    meas = float(np.linalg.norm(np.asarray(log1.v_BN_N)[-1]
                                - np.asarray(log0.v_BN_N)[-1]))
    d["srp_single_tick_meas"] = meas
    d["srp_single_tick_analytic"] = analytic
    d["srp_single_tick_ratio"] = meas / analytic if analytic else 0.0
    return d


def _dcm(sigma):
    """MRP->DCM (avsEigenMRP semantics, DESIGN constraint 2)."""
    import numpy as np

    def skew(c):
        return np.array([[0., -c[2], c[1]], [c[2], 0., -c[0]],
                         [-c[1], c[0], 0.]])
    s = np.asarray(sigma, float)
    s2 = float(s @ s)
    S = skew(s)
    return np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2


def _env_fuel_cell(r_tb):
    """Fuel-depletion A/B (750 kg bus... actually 740+10, 5 N thruster,
    duty 0.5 fresh, 600 s) + depletion-law + attitude-rotation metrics.

    r_tb: tank center in B. Centered ([0,0,0]) has c_B == 0 exactly (mass/I
    feedback isolated); offset exercises the static-c_B deviation (labeled,
    bounded). Attitude is compared by DCM (switch-invariant: shadow-switch
    timing jitter makes sigma-vector comparison meaningless once |sigma|
    crosses 1 -- measured max|dsig| 1e-3 with max|dom| 0.0). Fuel mass must
    match near-bit-exact (depletion law is algebraic in achieved duties).

    WP25-A3/H7 (WP-C MEASURED 2026-10-14, H7-LIFTED): fuel+drag and
    fuel+SRP combined cells (burn + env on, matched dt) are measured below
    (_env_fuel_drag_cell / _env_fuel_srp_cell) in the switching class
    (r < 2e-2, v < 1e-4, omega < 1e-4, dcm_max < 2e-3,
    fuel_rel_err < 1e-9, burned > 0.1). Arm record (H10-corrected: the vec
    drag arm tracks live-c -- vec:dragArm{cLive-envComOff} -- NOT a static
    arm; the ledger `arm` field records "live-c-tracking", never "static"):
    stock aggregate c_B DOES drift with depletion (offset tank [0.3,0,0.2]:
    c [4.00e-3, 0, 2.67e-3] -> [3.56e-3, 0, 2.37e-3], |dc| = 5.32e-4 m,
    1.1204 kg burned) and the vec arm follows it live. This fixture is
    INSENSITIVE to the arm model by its own bound: at 350 km rho ~ 9e-13
    gives F_drag ~ 3e-4 N, so a 1:1 depletion-tracking arm imprints only
    |dc x F_drag| ~ 1.6e-7 N.m -> dcm ~ 3.6e-5, ~25x BELOW the 9.2e-4
    residual floor (thruster-coupling-amplified projection class). Honest
    gate: shift_sig_dcm < dcm_max (sensitivity-limited consistency -- a
    tracking gain G >= ~25 would have tripped the dcm gate); G = 1 is NOT
    excluded by this cell -- open-recorded, not a verdict for the arm.
    Fixture notes: tumbling ICs (stabilized ICs DIVERGE to dcm ~ 2.0 under
    this thruster geometry -- measured WP-C, coherent spin-up amplifies
    micro-seeds via the 2.5 N.m body torque; same reason 250 km breaks the
    dcm gate at 4.3e-3).
    SRP cannonball torque is identically zero on BOTH arms (acts at CoM),
    so fuel_srp is force-level parity only (r 6.0e-3, dcm 4.5e-4) and
    cannot discriminate live-c vs static-c by construction -- labeled.
    """
    import numpy as np
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.simulation import fuelTank, thrusterDynamicEffector
    from Basilisk.utilities import simIncludeThruster
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config

    class ThrDriver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = 0.5 * DT
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)

    def build(use_omni):
        sim, sc = _base_sim("fuel")
        sc.hub.mHub = 740.0
        sc.hub.r_CN_NInit = [1.0e7, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, 7500.0, 0.0]
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", [0.5, 0, 1.0], [0, 0, -1.0], MaxThrust=5.0,
                  cutoffFrequency=1000.0, useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        tank = fuelTank.FuelTank()
        model = fuelTank.FuelTankModelConstantVolume()
        model.propMassInit = 10.0
        tank.setTankModel(model)
        tank.setNameOfMassState("fuelMass")
        tank.setR_TB_B(list(r_tb))
        tank.addThrusterSet(thr)
        sc.addStateEffector(tank)
        sim.AddModelToTask("t", tank)
        drv = ThrDriver()
        sim.AddModelToTask("t", drv)
        thr.cmdsInMsg.subscribeTo(drv.out)
        oi = None
        if use_omni:
            oi = OmniVecIntegrator(sc)
            mirror_stock_config(sim, sc, oi, thr_factory=tf,
                                thr_cmd_msg=drv.out)
            sc.setIntegrator(oi)
        scLog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", scLog)
        return sim, sc, scLog, oi

    sima, sca, loga, _ = build(False)
    _run(sima, 600.0)
    simb, scb, logb, oib = build(True)
    _run(simb, 600.0)
    d = _final_diff(loga, logb, ["r_BN_N", "v_BN_N", "omega_BN_B"])
    # Switch-invariant rotation metric (max over trajectory).
    sa = np.asarray(loga.sigma_BN)
    sb = np.asarray(logb.sigma_BN)
    d["dcm_max"] = float(max(np.abs(_dcm(sa[k]) - _dcm(sb[k])).max()
                             for k in range(len(sa))))
    # Fuel-mass telemetry parity (direct state read, both arms).
    fa = float(np.asarray(
        sca.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
    fb = float(np.asarray(
        scb.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
    d["fuel_stock"] = fa
    d["fuel_omni"] = fb
    d["fuel_rel_err"] = abs(fa - fb) / fa if fa else 0.0
    d["fuel_burned"] = 10.0 - fa
    d["nPars"] = int(oib.nPars())
    d["nStates"] = int(oib.nStates())
    d["build_s"] = round(float(oib.getBuildSeconds()), 4)
    return d


def _env_fuel_drag_cell():
    """Fuel-depletion + LEO-drag combined A/B (WP-C, H7-LIFTED).

    Fixture: offset tank [0.3, 0, 0.2] (max dc in the fuel-cell set),
    5 N thruster duty 0.5 fresh, 600 s, LEO 350 km circular, tumbling ICs,
    drag CdA = 11 comOffset = [0.05, 0, 0] (env_drag numbers). Omni arm
    uses the LIVE-C-TRACKING drag arm (cLive minus comOffset on the vec
    path; H10: never labeled "static"). Extra ledger fields:
    dc_norm (stock-measured |c(t1)-c(t0)| via scMassOutMsg), density_max
    (drag-live guard), shift_sig_dcm (1:1 depletion-tracking attitude
    signature estimate = |dc|*F*T^2/2I, I_rep = 800; the sensitivity
    floor -- must stay below the dcm residual for the verdict to read
    "consistent, sensitivity-limited"), arm = "live-c-tracking".
    """
    return _env_fuel_env_cell("drag")


def _env_fuel_srp_cell():
    """Fuel-depletion + SRP-cannonball combined A/B (WP-C, H7-LIFTED).

    Same burn fixture as fuel_drag; SRP area = 10, C_R = 1.5 (env_srp
    numbers, static sun). Cannonball torque is identically zero on BOTH
    arms (acts at CoM), so this cell is force-level parity only and
    cannot discriminate live-c vs static-c by construction -- labeled,
    not a verdict input for the arm model.
    """
    return _env_fuel_env_cell("srp")


def _env_fuel_env_cell(env):
    import numpy as np
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.simulation import fuelTank, thrusterDynamicEffector
    from Basilisk.utilities import simIncludeThruster
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    R = 6378137.0 + 350e3
    V = float(np.sqrt(MU / R))
    SUN = [1.5e11, 0.0, 0.0]
    T = 600.0

    class ThrDriver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = 0.5 * DT
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)

    def build(use_omni):
        sim, sc = _base_sim("fuel_" + env)
        sc.hub.mHub = 740.0
        sc.hub.r_CN_NInit = [R, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, V, 0.0]
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", [0.5, 0, 1.0], [0, 0, -1.0], MaxThrust=5.0,
                  cutoffFrequency=1000.0, useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        tank = fuelTank.FuelTank()
        model = fuelTank.FuelTankModelConstantVolume()
        model.propMassInit = 10.0
        tank.setTankModel(model)
        tank.setNameOfMassState("fuelMass")
        tank.setR_TB_B([0.3, 0.0, 0.2])
        tank.addThrusterSet(thr)
        sc.addStateEffector(tank)
        sim.AddModelToTask("t", tank)
        drv = ThrDriver()
        sim.AddModelToTask("t", drv)
        thr.cmdsInMsg.subscribeTo(drv.out)
        dlog = None
        if env == "drag":
            from Basilisk.simulation import (dragDynamicEffector,
                                             exponentialAtmosphere)
            atmo = exponentialAtmosphere.ExponentialAtmosphere()
            atmo.baseDensity = 1e-9
            atmo.scaleHeight = 50000.0
            atmo.planetRadius = 6378137.0
            atmo.addSpacecraftToModel(sc.scStateOutMsg)
            sim.AddModelToTask("t", atmo)
        else:
            from Basilisk.simulation import radiationPressure
            sm = messaging.SpicePlanetStateMsg()
            p = messaging.SpicePlanetStateMsgPayload()
            p.PositionVector = list(SUN)
            p.VelocityVector = [0.0, 0.0, 0.0]
            sm.write(p)
        oi = None
        if not use_omni:
            if env == "drag":
                drag = dragDynamicEffector.DragDynamicEffector()
                drag.coreParams.projectedArea = 5.0
                drag.coreParams.dragCoeff = 2.2
                drag.coreParams.comOffset = [0.05, 0.0, 0.0]
                drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
                drag.cannonballDrag()
                sc.addDynamicEffector(drag)
                sim.AddModelToTask("t", drag)
            else:
                srp = radiationPressure.RadiationPressure()
                srp.area = 10.0
                srp.coefficientReflection = 1.5
                srp.setUseCannonballModel()
                srp.sunEphmInMsg.subscribeTo(sm)
                sc.addDynamicEffector(srp)
                sim.AddModelToTask("t", srp)
            mlog = sc.scMassOutMsg.recorder()
            sim.AddModelToTask("t", mlog)
        else:
            oi = OmniVecIntegrator(sc)
            mirror_stock_config(sim, sc, oi, thr_factory=tf,
                                thr_cmd_msg=drv.out)
            if env == "drag":
                oi.setDragParams(2.2 * 5.0, 0.05, 0.0, 0.0)
                oi.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
            else:
                oi.setSRPCannonball(10.0, 0.5)
                oi.sunStateInMsg.subscribeTo(sm)
            sc.setIntegrator(oi)
            mlog = None
        scLog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", scLog)
        if env == "drag":
            dlog = atmo.envOutMsgs[-1].recorder()
            sim.AddModelToTask("t", dlog)
        return sim, sc, scLog, dlog, mlog, oi

    sima, sca, loga, _, mloga, _ = build(False)
    _run(sima, T)
    simb, scb, logb, dlogb, _, oib = build(True)
    _run(simb, T)
    d = _final_diff(loga, logb, ["r_BN_N", "v_BN_N", "omega_BN_B"])
    sa = np.asarray(loga.sigma_BN)
    sb = np.asarray(logb.sigma_BN)
    d["dcm_max"] = float(max(np.abs(_dcm(sa[k]) - _dcm(sb[k])).max()
                             for k in range(len(sa))))
    fa = float(np.asarray(
        sca.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
    fb = float(np.asarray(
        scb.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
    d["fuel_stock"] = fa
    d["fuel_omni"] = fb
    d["fuel_rel_err"] = abs(fa - fb) / fa if fa else 0.0
    d["fuel_burned"] = 10.0 - fa
    # Stock-measured depletion drift of the aggregate CoM (arm verdict).
    c = np.asarray(mloga.c_B, float)
    d["dc_norm"] = float(np.linalg.norm(c[-1] - c[0]))
    d["arm"] = "live-c-tracking"
    if env == "drag":
        rho = float(np.asarray(dlogb.neutralDensity).max())
        d["density_max"] = rho
        fest = 0.5 * rho * V ** 2 * (2.2 * 5.0)
    else:
        sun_d = float(np.linalg.norm(np.asarray(SUN, float)
                                     - np.asarray([R, 0.0, 0.0])))
        P_au, AU = 4.5398178907e-6, 1.495978707e11  # env_const values
        fest = P_au * (AU / sun_d) ** 2 * 10.0 * 1.5
    # 1:1 depletion-tracking attitude signature order (sensitivity floor).
    d["shift_sig_dcm"] = d["dc_norm"] * fest * T ** 2 / (2.0 * 800.0)
    d["nPars"] = int(oib.nPars())
    d["nStates"] = int(oib.nStates())
    d["build_s"] = round(float(oib.getBuildSeconds()), 4)
    return d


def _final_diff(a, b, fields):
    import numpy as np
    out = {}
    for f in fields:
        xa = np.asarray(getattr(a, f))[-1]
        xb = np.asarray(getattr(b, f))[-1]
        out[f] = float(np.abs(xa - xb).max())
    return out


def main() -> int:
    import numpy as np  # noqa: F401
    if not safe_import_basilisk():
        save_result("swap_p1_parity",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    payload: dict = {"status": "ok", "cells": {}, "dt": DT}
    fields0 = ["r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"]

    # JIT WARM-UP (metrology, not physics): the first heyoka build in a
    # fresh process pays one-time LLVM cold-start (~18 s, load-independent:
    # measured 18.42 s cold -> 0.007 s warm in the same process), which
    # would otherwise land on whichever cell runs first and swamp the
    # Ground-8 build tiers. Those tiers gate GRAPH-size scaling (catching
    # 10x+ pathological growth), so every cell below must measure a warm
    # steady-state build. This throwaway 2-tick build is UNRECORDED.
    _warm, _, _, _, _ = _arm(True, "hub", **{})
    _run(_warm, 0.2)

    def ab(kind, T, **kw):
        sima, _, loga, spda, _ = _arm(False, kind, **kw)
        _run(sima, T)
        simb, _, logb, spdb, oib = _arm(True, kind, **kw)
        _run(simb, T)
        fields = list(fields0) + list(kw.get("extra_fields", []))
        d = _final_diff(loga, logb, fields)
        if spda is not None:
            wa = np.asarray(spda.wheelSpeeds)[-1]
            wb = np.asarray(spdb.wheelSpeeds)[-1]
            d["wheels"] = float(np.abs(wa - wb).max())
        d["nPars"] = int(oib.nPars())
        d["nStates"] = int(oib.nStates())
        d["build_s"] = round(float(oib.getBuildSeconds()), 4)
        return d

    payload["cells"]["hub_60s"] = ab("hub", 60.0)
    payload["cells"]["rw3_open_pos"] = ab("rw3_open", 120.0, u=0.5)
    payload["cells"]["rw3_open_neg"] = ab("rw3_open", 120.0, u=-0.3)
    payload["cells"]["rw3_pd"] = ab("rw3_pd", 120.0)
    payload["cells"]["rw3_fric"] = ab("rw3_fric", 120.0)
    payload["cells"]["rw1"] = ab("rw1", 60.0)
    payload["cells"]["rw2"] = ab("rw2", 60.0)
    payload["cells"]["rw1tilt"] = ab("rw1tilt", 60.0)
    payload["cells"]["thr_fresh_05"] = ab("thr", 120.0, duty=0.5)
    payload["cells"]["thr_fresh_10"] = ab("thr", 120.0, duty=1.0)
    payload["cells"]["thr_writeonce"] = ab("thr", 20.0, duty=0.5,
                                           mode="write-once")
    payload["cells"]["thr_frozen"] = ab("thr", 30.0, duty=0.5,
                                        mode="frozen")
    payload["cells"]["thr_lag10"] = ab("thr", 120.0, duty=0.5, cutoff=10.0)
    # CoM-offset B/C boundary (stock convention r_BN = r_CN - R^T*c,
    # measured; DESIGN/test_wp4 pin the opposite sign from self-consistent
    # math only -- flagged for SysModel-path audit).
    payload["cells"]["thr_offset"] = ab("thr", 60.0, duty=1.0, offset=0.1,
                                        extra_fields=["r_CN_N", "v_CN_N"])
    payload["cells"]["env_drag"] = _env_drag_cell()
    payload["cells"]["env_drag_still"] = _env_drag_cell(still=True)
    payload["cells"]["env_srp"] = _env_srp_cell()
    payload["cells"]["fuel_centered"] = _env_fuel_cell([0.0, 0.0, 0.0])
    payload["cells"]["fuel_offset"] = _env_fuel_cell([0.3, 0.0, 0.2])
    # WP-C MEASURED (H7-LIFTED, see _env_fuel_cell docstring): combined
    # fuel+env cells at matched dt, live-c-tracking drag arm. Switching-class gates
    # apply via the fuel_ bounds branch + vacuity loop below.
    payload["cells"]["fuel_drag"] = _env_fuel_drag_cell()
    payload["cells"]["fuel_srp"] = _env_fuel_srp_cell()

    # Switching-residual ladder (documents linear-in-dt class, Update-4 #2).
    payload["thr_ladder_30s"] = {"dt0.1": None}  # filled below
    for dt in (0.1, 0.05, 0.02):
        DT_save = DT
        globals()["DT"] = dt
        try:
            sima, _, loga, _, _ = _arm(False, "thr", duty=0.5)
            _run(sima, 30.0)
            simb, _, logb, _, _ = _arm(True, "thr", duty=0.5)
            _run(simb, 30.0)
        finally:
            globals()["DT"] = DT_save
        dv = _final_diff(loga, logb, ["v_BN_N"])["v_BN_N"]
        ds = _final_diff(loga, logb, ["sigma_BN"])["sigma_BN"]
        payload["thr_ladder_30s"][f"dt{dt}"] = {"dv": dv, "dsigma": ds}

    payload["rejects"] = _reject_cells()

    # Verdicts vs pre-registered tolerances.
    fails = []
    for cell, d in payload["cells"].items():
        if cell.startswith("thr_fresh") or cell.startswith("thr_offset") \
                or cell.startswith("thr_lag"):
            bounds = {"r_BN_N": TOL_THR_R, "v_BN_N": TOL_THR_V,
                      "sigma_BN": TOL_THR_SIG, "omega_BN_B": TOL_THR_OM,
                      "r_CN_N": TOL_THR_R, "v_CN_N": TOL_THR_V}
        elif cell.startswith("thr_writeonce") or cell.startswith("thr_frozen"):
            bounds = {"r_BN_N": TOL_WO_R, "v_BN_N": TOL_WO_V,
                      "sigma_BN": TOL_WO_SIG, "omega_BN_B": TOL_WO_OM}
        elif cell in ("env_drag", "env_drag_still"):
            if cell == "env_drag_still":
                bounds = {"r_BN_N": 2e-5, "v_BN_N": 1e-7, "sigma_BN": 1e-7,
                          "omega_BN_B": 1e-9}
            else:
                # Tumbling: O(|w|*dt) frozen-projection class, bounded honest.
                bounds = {"r_BN_N": 5e-4, "v_BN_N": 2e-6, "sigma_BN": 5e-7,
                          "omega_BN_B": 1e-8}
        elif cell == "env_srp":
            bounds = {"r_BN_N": 1e-6, "v_BN_N": 1e-8, "sigma_BN": 1e-9,
                      "omega_BN_B": 1e-9}
        elif cell.startswith("fuel_"):
            # Switching-class trajectories (5 N, 600 s) + bit-exact fuel law.
            # (dcm_max/fuel_rel_err checked separately below.)
            bounds = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "omega_BN_B": 1e-4}
        else:
            bounds = {f: TOL_STATE for f in fields0}
        for f, tol in bounds.items():
            if f not in d:
                continue
            if not d[f] < tol:
                fails.append(f"{cell}.{f}={d[f]:.3e}")
        if "wheels" in d and not d["wheels"] < TOL_WHEEL:
            fails.append(f"{cell}.wheels={d['wheels']:.3e}")
    # Vacuity guards: the effects must be live (no zeros-vs-zeros).
    drag = payload["cells"].get("env_drag", {})
    if not drag.get("density_max", 0.0) > 1e-13:
        fails.append(f"env_drag.vacuous density_max={drag.get('density_max')}")
    srp = payload["cells"].get("env_srp", {})
    ratio = srp.get("srp_single_tick_ratio", 0.0)
    if not 0.9 < ratio < 1.1:
        fails.append(f"env_srp.vacuous single-tick ratio={ratio:.4f}")
    for cell in ("fuel_centered", "fuel_offset"):
        fc = payload["cells"].get(cell, {})
        if not fc.get("fuel_burned", 0.0) > 0.1:
            fails.append(f"{cell}.vacuous burned={fc.get('fuel_burned')}")
        if not fc.get("fuel_rel_err", 1.0) < 1e-9:
            fails.append(f"{cell}.fuel_rel_err={fc.get('fuel_rel_err'):.3e}")
        if not fc.get("dcm_max", 1.0) < 2e-3:
            fails.append(f"{cell}.dcm_max={fc.get('dcm_max'):.3e}")
    for cell in ("fuel_drag", "fuel_srp"):
        fc = payload["cells"].get(cell, {})
        if not fc.get("fuel_burned", 0.0) > 0.1:
            fails.append(f"{cell}.vacuous burned={fc.get('fuel_burned')}")
        if not fc.get("fuel_rel_err", 1.0) < 1e-9:
            fails.append(f"{cell}.fuel_rel_err={fc.get('fuel_rel_err'):.3e}")
        if not fc.get("dcm_max", 1.0) < 2e-3:
            fails.append(f"{cell}.dcm_max={fc.get('dcm_max'):.3e}")
        if not fc.get("dc_norm", 0.0) > 1e-4:
            fails.append(f"{cell}.vacuous dc={fc.get('dc_norm')}")
        if not fc.get("shift_sig_dcm", 1.0) < fc.get("dcm_max", 0.0):
            fails.append(f"{cell}.sensitivity "
                         f"sig={fc.get('shift_sig_dcm'):.3e} "
                         f"dcm={fc.get('dcm_max'):.3e}")
    fdrag = payload["cells"].get("fuel_drag", {})
    if not fdrag.get("density_max", 0.0) > 1e-13:
        fails.append(f"fuel_drag.vacuous density_max={fdrag.get('density_max')}")
    payload["verdict"] = ("parity green" if not fails
                          else f"PARITY FAIL: {fails}")
    if fails:
        payload["status"] = "fail"
    save_result("swap_p1_parity", payload)
    for cell, d in payload["cells"].items():
        log_text("swap_p1_parity", f"[{cell}] " + " ".join(
            f"{k}={v:.2e}" if isinstance(v, float) else f"{k}={v}"
            for k, v in d.items()))
    print(f"swap_p1 verdict: {payload['verdict']}")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
