"""Task cost split (stock Basilisk): full loop minus one family at a time.

Answers "where does a tick go" by ablation, with no profiler hooks: six
cumulative legs on one fixture (hub + 3 RW + 1 staircase thruster + single
tank + central gravity, 600 ticks @ dt=0.1), 3 fresh sims each, means +
medians + min/max. Attribution by subtraction on MEDIANS (outlier-proof at
n=3; means are recorded alongside). Units: per-tick ms + wall throughput.

Legs (cumulative):
  L0_hub            sc + gravity
  L1_rw             + 3 wheels (motor cmd unlinked: idle)
  L2_thr_tank_idle  + thruster + tank + zero-duty driver (linked, idle thrust)
  L3_fire           burn driver at duty 0.5 (firing)
  L4_python_driver  + PD SysModel (numpy read/compute/write per tick, RW loop)
  L5_recorders      + scState + rwSpeed recorders
Parts: rw=L1-L0, thr_tank_idle=L2-L1, fire=L3-L2, python_driver=L4-L3,
recorders=L5-L4, total=L5-L0 (cross-check).

Needs Basilisk only (pure stock -- no wheel). Nanos audit on every run (a
fast wall with short nanos is a short run, not a fast sim -- loud).
Ledger: audit_bundle/results/loop_split.json.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Repo root (Python/) on sys.path no matter the cwd (omniheyoka-free probe,
# but keep the convention so sibling imports resolve if added later).
_REPO_ROOT = str(Path(__file__).resolve().parents[3])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from common import log_text, save_result, write_exception  # noqa: E402

DT = 0.1
T_LEG = 60.0
TICKS = int(T_LEG / DT)
N_RUNS = 3
MU = 398600441800000.0
MASS_DRY, PROP0 = 20.0, 10.0
I6 = [[10.0, 0.0, 0.0], [0.0, 11.0, 0.0], [0.0, 0.0, 12.0]]
LEG_ORDER = ("L0_hub", "L1_rw", "L2_thr_tank_idle", "L3_fire",
             "L4_python_driver", "L5_recorders")


def _loadavg():
    try:
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return []


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
    sc.hub.mHub = MASS_DRY
    sc.hub.IHubPntBc_B = [list(r) for r in I6]
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R, 347.8 * macros.D2R,
                                85.3 * macros.D2R)
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    return sim, sc


def _burn_driver(duty=0.5):
    from Basilisk.architecture import messaging, sysModel

    class Burn(sysModel.SysModel):
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
    return Burn()


def _pd_driver(K=3.5, P=30.0, u_max=0.2):
    """PD reader/writer (closed_loop_fuel.py pattern, verbatim gains)."""
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


def _build(level, duty=0.5):
    """Cumulative fixture; level 0..5 selects how much is wired."""
    from Basilisk.simulation import (fuelTank, reactionWheelStateEffector,
                                      thrusterDynamicEffector)
    from Basilisk.utilities import simIncludeRW, simIncludeThruster

    sim, sc = _base_sim(f"rk-splitL{level}")
    rw = None
    if level >= 1:
        fac = simIncludeRW.rwFactory()
        for gs in ([1., 0, 0], [0, 1., 0], [0, 0, 1.]):
            fac.create("Honeywell_HR16", list(gs), maxMomentum=50.0,
                       Omega=100.0)
        rw = reactionWheelStateEffector.ReactionWheelStateEffector()
        fac.addToSpacecraft("rw", rw, sc)
        sim.AddModelToTask("t", rw)
    if level >= 2:
        tf = simIncludeThruster.thrusterFactory()
        tf.create("TEST_Thruster", [0.0, 0.0, 1.0], [0.0, 0.0, -1.0],
                  MaxThrust=5.0, cutoffFrequency=1000.0,
                  useMinPulseTime=False)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        burn = _burn_driver(duty if level >= 3 else 0.0)
        sim.AddModelToTask("t", burn)
        thr.cmdsInMsg.subscribeTo(burn.out)
        tank = fuelTank.FuelTank()
        model = fuelTank.FuelTankModelConstantVolume()
        model.propMassInit = PROP0
        tank.setTankModel(model)
        tank.setNameOfMassState("fuelMass")
        tank.setR_TB_B([0.0, 0.0, 0.0])
        tank.addThrusterSet(thr)
        sc.addStateEffector(tank)
        sim.AddModelToTask("t", tank)
    if level >= 4:
        pd = _pd_driver()
        sim.AddModelToTask("t", pd)
        rw.rwMotorCmdInMsg.subscribeTo(pd.out)
        pd.sc_in.subscribeTo(sc.scStateOutMsg)
    if level >= 5:
        scLog = sc.scStateOutMsg.recorder()
        sim.AddModelToTask("t", scLog)
        spdLog = rw.rwSpeedOutMsg.recorder()
        sim.AddModelToTask("t", spdLog)
    return sim


def _run(sim):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    t0 = time.perf_counter()
    sim.ConfigureStopTime(macros.sec2nano(T_LEG))
    sim.ExecuteSimulation()
    return time.perf_counter() - t0


def _end_nanos(sim):
    return int(sim.TotalSim.CurrentNanos)


def _expected_nanos():
    from Basilisk.utilities import macros
    return int(macros.sec2nano(T_LEG))


def _med(xs):
    return sorted(xs)[1]


def main() -> int:
    try:
        import Basilisk  # noqa: F401
    except Exception as exc:
        save_result("loop_split", {"status": "skipped-no-basilisk",
                                   "needs_wheel": False,
                                   "reason": f"{type(exc).__name__}: {exc}"})
        print("loop_split: skipped (no Basilisk on this host; wheel not needed)")
        return 0
    try:
        expected = _expected_nanos()
        legs: dict = {}
        # Warm-up throwaway: the first sim in a fresh process absorbs
        # one-time costs (dyld lazy binding, allocator arenas, task-graph
        # setup) that would otherwise inflate whichever leg runs first.
        # Without this, L0 medians read ~1 ms hot and fake a monotonicity
        # violation against L1 (measured 2026-10-14).
        _run(_build(0))
        for depth, name in enumerate(LEG_ORDER):
            walls = []
            for _ in range(N_RUNS):
                sim = _build(depth)
                walls.append(_run(sim))
                if _end_nanos(sim) != expected:
                    raise RuntimeError(
                        f"time audit failed ({name}): short run, not fast sim")
            walls = sorted(walls)
            legs[name] = {
                "wall_samples_s": [round(w, 4) for w in walls],
                "wall_mean_s": round(sum(walls) / len(walls), 4),
                "wall_median_s": round(_med(walls), 4),
                "per_tick_median_ms": round(_med(walls) / TICKS * 1000.0, 4),
            }
        med = {n: legs[n]["wall_median_s"] for n in LEG_ORDER}
        per_tick = {n: legs[n]["per_tick_median_ms"] for n in LEG_ORDER}

        def _d(a, b):
            return round((med[a] - med[b]) / TICKS * 1000.0, 4)

        attribution = {
            "rw_ms": _d("L1_rw", "L0_hub"),
            "thr_tank_idle_ms": _d("L2_thr_tank_idle", "L1_rw"),
            "fire_ms": _d("L3_fire", "L2_thr_tank_idle"),
            "python_driver_ms": _d("L4_python_driver", "L3_fire"),
            "recorders_ms": _d("L5_recorders", "L4_python_driver"),
            "total_ms": _d("L5_recorders", "L0_hub"),
            "basis": "median walls / 600 ticks; signed (noise can print small negatives)",
        }
        payload = {
            "status": "ok",
            "n_runs": N_RUNS,
            "fixture": ("stock hub+3RW+1thr+tank+central, "
                        f"{TICKS} ticks @ dt={DT}; L2 idle-linked, L3 duty 0.5, "
                        "L4 PD reader/writer, L5 recorders"),
            "ticks": TICKS,
            "nanos_ok": True,
            "nanos_expected": expected,
            "legs": legs,
            "per_tick_median_ms": per_tick,
            "attribution_ms": attribution,
            "throughput_ticks_per_s": round(TICKS / med["L5_recorders"], 1),
            "loadavg": _loadavg(),
            "note": ("ablation: each part = heavier leg minus lighter leg on "
                     "medians; full-loop throughput from the L5 median wall."),
        }
        out = save_result("loop_split", payload)
        log_text("loop_split",
                 " ".join(f"{n}={legs[n]['per_tick_median_ms']:.4f}ms"
                           for n in LEG_ORDER) +
                 f" throughput={payload['throughput_ticks_per_s']:.0f}t/s")
        print(out)
        return 0
    except Exception as exc:
        write_exception("loop_split", exc)
        print(f"loop_split FAILED: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
