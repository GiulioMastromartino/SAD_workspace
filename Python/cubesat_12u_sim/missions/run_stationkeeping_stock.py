"""Stock-`Spacecraft` stationkeeping sim (staircase RCS + single fuel tank).

Scenario (adapted from the analytic ``missions/run_stationkeeping.py``):
  injection error leaves the 12U ~0.5% slow (``dv_frac=-0.005``; elliptical,
  radius sags). A staircase-RCS driver at the controller cadence
  (``DT_CONTROL=10.0 s`` -> 0.10 Hz) fires prograde/retrograde jets to
  restore the 12000 km-altitude circular orbit (``R_ORBIT``). Attitude
  starts detumbled (slow drift) to stay clear of singularities.

Swap scope (PLAN_INTEGRATOR_SWAP.md Script-3 rows S1/S2):
  S1: the analytic scripts have no stock integrator to exchange -- this file
      is the stock-`Spacecraft` rewrite that provides the exchange surface.
  S2: staircase RCS + `FuelTank` + `setTankModel`; single-tank guard (exactly
      ONE `FuelTank` -- a 2nd tank is a bucket-(a) `bskError` under the swap);
      `THRArrayCmdForceMsg` ABSENT (force link disarmed; OnTime only).

Wiring pattern follows ``audit_bundle/probes/closed_loop_fuel.py`` (verified):
  factory `create`, `addToSpacecraft`, `cmdsInMsg.subscribeTo`, tank wiring
  (`setTankModel` / `setNameOfMassState` / `setR_TB_B` / `addThrusterSet`),
  `gravBodyFactory` Earth-central gravity.

Run: ``python missions/run_stationkeeping_stock.py`` (from the repo root or
the ``cubesat_12u_sim`` package directory).
Host-only (needs a Basilisk build). NO omniheyoka imports in this file.
"""

from __future__ import annotations

import numpy as np

from cubesat_12u_sim.core import config as C

# Scenario parameters carried over from missions/run_stationkeeping.py:
#   dv_frac=-0.005 (demo_state default), R_ORBIT (R_des print),
#   DT_CONTROL cadence (ctrl@1/DT_CONTROL Hz print), t_end=7200.0 (main default).
DV_FRAC_DEFAULT = -0.005
T_END_DEFAULT = 7200.0


def _stationkeeping_driver(dt_control: float, r_des: float, mu: float,
                            mass: float, fmax: float,
                            kp_a: float = 3.0e-7, db_a: float = 500.0,
                            a_max: float = 0.05):
    """Staircase fresh-write driver: semi-major-axis P-law -> OnTime slots.

    Slot 0 = prograde (+Y body), slot 1 = retrograde (-Y body). Both jets sit
    at the hub CoM (zero moment arm), so the burn line is torque-free.
    """
    from Basilisk.architecture import messaging, sysModel

    class StationkeepingDriver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()
            self.sc_in = messaging.SCStatesMsgReader()

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            r = np.asarray(self.sc_in().r_BN_N, float)
            v = np.asarray(self.sc_in().v_BN_N, float)
            rn, v2 = float(np.linalg.norm(r)), float(v @ v)
            a_osc = 1.0 / (2.0 / rn - v2 / mu)
            e_a = r_des - a_osc
            ot = [0.0] * 36
            if abs(e_a) > db_a:
                a_des = float(np.clip(kp_a * e_a, -a_max, a_max))
                duty = float(np.clip(abs(a_des) * mass / fmax, 0.0, 1.0))
                slot = 0 if a_des > 0.0 else 1
                ot[slot] = duty * dt_control
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)

    return StationkeepingDriver()


def build_sim(dv_frac: float = DV_FRAC_DEFAULT,
              prop_kg: float = 2.0, tag: str = "stock-sk"):
    """Build the stock stationkeeping sim. Returns (sim, sc, handles)."""
    from Basilisk.simulation import spacecraft, fuelTank, thrusterDynamicEffector
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                    simIncludeGravBody, simIncludeThruster)

    dt_control = float(C.DT_CONTROL)
    r_des = float(C.R_ORBIT)
    mu = float(C.MU_EARTH)

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(dt_control)))

    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = float(C.MASS)
    sc.hub.IHubPntBc_B = [[float(v) for v in row] for row in np.asarray(C.INERTIA)]
    # Injection-error ICs: circular state slowed by dv_frac (analytic demo_state).
    r0 = np.asarray(C.R0_ECI, float)
    v0 = np.asarray(C.V0_ECI, float) * (1.0 + float(dv_frac))
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    # Detumbled start (analytic: euler deg [5,2,0], omega deg/s [0.05,0.02,-0.03]).
    sc.hub.sigma_BNInit = [0.02, -0.01, 0.005]
    sc.hub.omega_BN_BInit = list(np.radians([0.05, 0.02, -0.03]))
    sim.AddModelToTask("t", sc)

    # Earth-central point-mass gravity.
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)

    # Staircase RCS: 2 x TEST_Thruster at the CoM (torque-free line),
    # prograde +Y / retrograde -Y, TMAX_JET each, Isp bookkeeping ISP_JET.
    tf = simIncludeThruster.thrusterFactory()
    tf.create("TEST_Thruster", [0.0, 0.0, 0.0], [0.0, 1.0, 0.0],
              MaxThrust=float(C.TMAX_JET), steadyIsp=float(C.ISP_JET),
              cutoffFrequency=1000.0, useMinPulseTime=False)
    tf.create("TEST_Thruster", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0],
              MaxThrust=float(C.TMAX_JET), steadyIsp=float(C.ISP_JET),
              cutoffFrequency=1000.0, useMinPulseTime=False)
    thr = thrusterDynamicEffector.ThrusterDynamicEffector()
    tf.addToSpacecraft("thr", thr, sc)
    # FIRST-HOST-RUN correction (2026-10-14, ledger audit_bundle/results/
    # stationkeeping_stock.json): as first written this was a default-priority
    # `sim.AddModelToTask("t", thr)` listed BEFORE the driver below, so the
    # effector's UpdateState ran before the driver's and always consumed the
    # PREVIOUS tick's THRArrayOnTimeCmdMsg. Basilisk's OnTime bookkeeping then
    # under-delivered the burn hard (0.043 kg burned over 7200 s against
    # ~3.3 kg of commanded on-time, and only ~28/721 ticks delivered any
    # thrust at all), so the orbit never recovered. Priority 2 puts the
    # thruster after the driver on the same task -- the exact pattern the
    # repo demo uses (`basilisk_sim.py`: AddModelToTask(..., thrusterSet, 2))
    # and the audit-bundle loop probes (`fsw_loop.py` / `swap_speed.py`:
    # `sim.AddModelToTask("t", thr, 2)`). Dynamics-neutral wiring only.
    sim.AddModelToTask("t", thr, 2)

    drv = _stationkeeping_driver(dt_control, r_des, mu,
                                 float(C.MASS), float(C.TMAX_JET))
    sim.AddModelToTask("t", drv)
    thr.cmdsInMsg.subscribeTo(drv.out)
    drv.sc_in.subscribeTo(sc.scStateOutMsg)
    # NOTE: THRArrayCmdForceMsg is deliberately ABSENT (S2 force-link absent:
    # OnTime only; the force path stays disarmed).

    # S2 single-tank guard: exactly ONE FuelTank in this sim. A second tank
    # (2nd addFuelTank / addStateEffector(tank2)) is a bucket-(a) bskError
    # under the swap -- never add one here.
    tank = fuelTank.FuelTank()
    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = float(prop_kg)
    tank.setTankModel(model)
    tank.setNameOfMassState("fuelMass")
    tank.setR_TB_B([0.0, 0.0, 0.0])  # centered tank: mass/I feedback isolated
    tank.addThrusterSet(thr)
    sc.addStateEffector(tank)
    sim.AddModelToTask("t", tank)

    # Phase-3 swap surface (stock default integrator active; two-line exchange):
    #   from omniheyoka import OmniVecIntegrator
    #   sc.setIntegrator(OmniVecIntegrator(sc, tol=1e-12))

    sc_log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", sc_log)
    return sim, sc, {"sc_log": sc_log, "drv": drv, "thr_factory": tf,
                     "r0": r0, "prop0": float(prop_kg)}


def main(t_end: float = T_END_DEFAULT,
         dv_frac: float = DV_FRAC_DEFAULT,
         prop_kg: float = 2.0, plot: bool = True):
    """Run the stock stationkeeping recovery and report radius/fuel stats."""
    from Basilisk.utilities import macros

    print(f"[info] injection error {dv_frac*100:.1f}% dV; "
          f"R_des={C.R_ORBIT/1e3:.1f} km, ctrl@{1.0/C.DT_CONTROL:.2f} Hz, "
          f"t_end={t_end/3600:.2f} h")
    sim, sc, h = build_sim(dv_frac=dv_frac, prop_kg=prop_kg)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(float(t_end)))
    sim.ExecuteSimulation()

    log = h["sc_log"]
    r = np.asarray(log.r_BN_N, float)
    rerr = np.linalg.norm(r, axis=1) - float(C.R_ORBIT)
    fuel = float(np.asarray(
        sc.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
    burned = h["prop0"] - fuel
    print(f"[stock-closed] max|r-err|={np.abs(rerr).max()/1e3:.2f} km  "
          f"rms={np.sqrt((rerr**2).mean())/1e3:.2f} km  "
          f"final={rerr[-1]/1e3:.2f} km")
    print(f"[stock-closed] prop burned={burned:.4f} kg, "
          f"remaining={fuel:.4f} kg")

    if plot:
        import matplotlib.pyplot as plt
        t = np.asarray(log.times(), float) * 1e-9 / 3600.0
        fig, ax = plt.subplots(1, 1, figsize=(9, 4))
        ax.plot(t, rerr / 1e3, label="stock staircase-RCS recovery")
        ax.axhline(0.0, color="k", linewidth=0.8)
        ax.set_ylabel("radius error [km]")
        ax.set_xlabel("time [h]")
        ax.legend()
        ax.grid(True)
        fig.suptitle("12U station-keeping @ 12000 km alt -- stock Spacecraft")
        fig.tight_layout()
        plt.show()
    return {"rerr": rerr, "fuel": fuel, "burned": burned}


if __name__ == "__main__":
    main()
