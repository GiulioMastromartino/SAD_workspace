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


def _add_rw(sim, sc, axes, om0=100.0, task=True):
    from Basilisk.simulation import reactionWheelStateEffector
    from Basilisk.utilities import simIncludeRW
    fac = simIncludeRW.rwFactory()
    for gs in axes:
        fac.create("Honeywell_HR16", list(gs), maxMomentum=50.0, Omega=om0)
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
    rw = None
    if kind in ("rw3_open", "rw3_pd", "rw1", "rw2", "rw1tilt"):
        axes = {"rw3_open": [[1., 0, 0], [0, 1., 0], [0, 0, 1.]],
                "rw3_pd": [[1., 0, 0], [0, 1., 0], [0, 0, 1.]],
                "rw1": [[1., 0, 0]],
                "rw2": [[1., 0, 0], [0, 1., 0]],
                "rw1tilt": [[0.5773502691896258] * 3]}[kind]
        rw = _add_rw(sim, sc, axes)
    if kind == "rw3_open":
        drv = _const_driver(kw.get("u", 0.5))
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
                  cutoffFrequency=1000.0, useMinPulseTime=False)
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
        except ValueError:
            out[name] = True

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

    def fuel_base():
        from Basilisk.simulation import fuelTank
        sim, sc = _base_sim("reject-fuel")
        model = fuelTank.FuelTankModelConstantVolume()
        model.propMassInit = 10.0
        tank = fuelTank.FuelTank()
        tank.setTankModel(model)
        tank.setNameOfMassState("fuelMass")
        sc.addStateEffector(tank)
        sim.AddModelToTask("t", tank)
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

    expect_raise("fuel_tank",
                 lambda: mirror(*fuel_base()))
    expect_raise("jitter_wheel",
                 lambda: mirror(*rw_base(jitter=True)))
    expect_raise("friction_wheel",
                 lambda: mirror(*rw_base(friction=True)))
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
    expect_raise("low_cutoff",
                 lambda: thr_mirror({"cutoffFrequency": 10.0}))

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
    fields = ["r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"]

    def ab(kind, T, **kw):
        sima, _, loga, spda, _ = _arm(False, kind, **kw)
        _run(sima, T)
        simb, _, logb, spdb, oib = _arm(True, kind, **kw)
        _run(simb, T)
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
    payload["cells"]["rw1"] = ab("rw1", 60.0)
    payload["cells"]["rw2"] = ab("rw2", 60.0)
    payload["cells"]["rw1tilt"] = ab("rw1tilt", 60.0)
    payload["cells"]["thr_fresh_05"] = ab("thr", 120.0, duty=0.5)
    payload["cells"]["thr_fresh_10"] = ab("thr", 120.0, duty=1.0)
    payload["cells"]["thr_writeonce"] = ab("thr", 20.0, duty=0.5,
                                           mode="write-once")
    payload["cells"]["thr_frozen"] = ab("thr", 30.0, duty=0.5,
                                        mode="frozen")
    payload["cells"]["env_drag"] = _env_drag_cell()
    payload["cells"]["env_drag_still"] = _env_drag_cell(still=True)
    payload["cells"]["env_srp"] = _env_srp_cell()

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
        if cell.startswith("thr_fresh"):
            bounds = {"r_BN_N": TOL_THR_R, "v_BN_N": TOL_THR_V,
                      "sigma_BN": TOL_THR_SIG, "omega_BN_B": TOL_THR_OM}
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
        else:
            bounds = {f: TOL_STATE for f in fields}
        for f, tol in bounds.items():
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
