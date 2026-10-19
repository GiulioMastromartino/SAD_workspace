"""Stock-``Spacecraft`` LEO-drag sim for the 12U bus (Phase-3 Script-2 fixture).

Phase-3 entry script: the repo demo (``basilisk_sim.py``) flies a 12000 km
orbit where atmospheric density is vacuous, so drag there cannot gate
anything. This script is the LEO counterpart: Earth-central point-mass
gravity + ``dragDynamicEffector`` + ``exponentialAtmosphere`` on a ~400 km
circular orbit, with an explicit ``zeroWindModel`` wind link.

Envelope pins (``PLAN_INTEGRATOR_SWAP.md`` Phase-3 Script-2 table):
  L1 -- ``dragDynamicEffector`` + ``exponentialAtmosphere`` with the
      ``atmoDensInMsg`` REQUIRED link subscribed, plus ``zeroWindModel``
      subscribed to ``windVelInMsg``. Absent-link truth table (``DESIGN.md``
      A13): density-unlinked is fatal (stock ``bskError``); wind-unlinked is
      NOT fatal -- stock silently falls back to inertial velocity (~6% LEO
      error, labeled). This script links both, so the fixture is the
      corotation baseline; see the wind-link comment at the subscription.
      HOST-MEASURED on the first run (see same comment): ``WindBase``
      additionally needs its own ``planetPosInMsg`` (MODEL side), which is
      wired here from a stock ``SpicePlanetStateMsg``.
  L2 -- ~400 km circular LEO fixture (``ALTITUDE_LEO``), never the 12000 km
      demo orbit. Firing gate for any comparison run: recorded
      ``neutralDensity`` max must exceed 1e-13 kg/m^3 (effect live).
  L3 -- thrusters / RW / fuel / eclipse: NONE armed here. This is a
      drag-only fixture (no ``ThrusterDynamicEffector``, no reaction
      wheels, no ``FuelTank``), so ``cutoffFrequency`` / ``useMinPulseTime``
      / ramps / blowdown / swirl / friction / jitter set nothing and cannot
      drift. If RCS is ever added, use the stock-demo defaults
      ``cutoffFrequency=1000.0, useMinPulseTime=False`` (``basilisk_sim.py``
      thruster factory call) and re-pin against the D2/D3 + fuel rows.

Deliberate differences from ``basilisk_sim.py`` (all dynamics-neutral):
  * No Sun body: the demo's Sun carries the rotating ``SunEphemeris`` for
    Vizard/CSS only, not dynamics (D1 pin). No CSS/IMU/FSW/Vizard here --
    pure readers, no dynamics claim (D4 pin).
  * Stabilized attitude ICs (sigma/omega zero), i.e. the ``env_drag_still``
    shape: no tumbling ``O(|w|*dt)`` frozen-projection class in the fixture.
  * ``comOffset=[0,0,0]`` with ``r_BcB_B=[0,0,0]``: pure drag force, no
    drag torque arm. Stock arm rule (DESIGN.md A13) is
    ``tau = F x (rBcB - comOffset)``; a nonzero offset would add torque.

NO omniheyoka imports: this is a stock sim script. The future swap mirror
reads: ``mHub`` / ``IHubPntBc_B`` / ``r_BcB_B`` (hub), ``projectedArea`` /
``dragCoeff`` / ``comOffset`` (drag core params), the atmo link, the wind
link, and the Earth-central gravity registration below.
"""
from __future__ import annotations

import argparse

import numpy as np

from cubesat_12u_sim.core import config as C

# ---------------- LEO fixture (L2) ----------------
ALTITUDE_LEO: float = 400.0e3  # [m] fixture altitude above C.R_EARTH
R_LEO: float = float(C.R_EARTH + ALTITUDE_LEO)  # [m] ~= 6778.137 km
V_LEO: float = float(np.sqrt(C.MU_EARTH / R_LEO))  # [m/s] ~= 7669
T_LEO: float = float(2.0 * np.pi * np.sqrt(R_LEO**3 / C.MU_EARTH))  # [s] ~= 5554

# ---------------- Drag / atmosphere model params (L1) ----------------
# Probe-matched exponential atmosphere (swap_p1_parity `_env_drag_cell`):
# rho(h) = baseDensity * exp(-(r - planetRadius) / scaleHeight), so at
# 400 km: 1e-9 * exp(-8) ~= 3.4e-13 kg/m^3, above the 1e-13 firing gate.
BASE_DENSITY: float = 1.0e-9  # [kg/m^3] surface-anchored reference density
SCALE_HEIGHT: float = 50.0e3  # [m] exponential scale height
# Probe-matched cannonball product CdA = 2.2 * 5.0 = 11 (exaggerated paddle
# so the effect is macroscopic over one orbit at 400 km; a true 12U face is
# ~0.06 m^2). Drag decel order: 0.5*rho*CdA/m*v^2 ~= 5e-6 m/s^2.
PROJECTED_AREA: float = 5.0  # [m^2]
DRAG_COEFF: float = 2.2  # [-]
DENSITY_FIRING_GATE: float = 1.0e-13  # [kg/m^3] vacuity guard (L1)

# Earth sidereal rotation rate [rad/s] for the corotation baseline (same
# constant as native_omni_heyoka/audit_bundle/probes/wp5_env.py `wE`).
EARTH_ROT_RATE: float = 7.2921159e-5


def run_simulation(
    duration: float = T_LEO,
    altitude: float = ALTITUDE_LEO,
    projected_area: float = PROJECTED_AREA,
    drag_coeff: float = DRAG_COEFF,
    task_step: float = 1.0,
    spacecraft_name: str = "cubesat_12u_leo",
) -> dict[str, np.ndarray | float | None]:
    """Run the stock-Spacecraft LEO-drag fixture.

    Mirrors ``basilisk_sim.run_simulation`` conventions: one process / one
    task via ``SimBaseClass`` + ``macros.sec2nano``, hub configured from
    ``config`` (20 kg class bus, diagonal inertia, ``r_BcB_B = 0``),
    Earth-central gravity via ``gravBodyFactory``, ``InitializeSimulation``
    / ``ConfigureStopTime`` / ``ExecuteSimulation`` run block, and a result
    dict keyed like the demo (``time`` in seconds via ``macros.NANO2SEC``,
    ``truth_*`` state histories) plus the density log and fixture echoes
    the swap mirror needs.
    """
    from Basilisk.simulation import (
        dragDynamicEffector,
        exponentialAtmosphere,
        spacecraft,
        zeroWindModel,
    )
    from Basilisk.utilities import (
        SimulationBaseClass,
        macros,
        simHelpers,
        simIncludeGravBody,
    )
    from Basilisk.architecture import messaging

    orbit_radius = float(C.R_EARTH + altitude)
    orbit_velocity = float(np.sqrt(C.MU_EARTH / orbit_radius))

    simulation = SimulationBaseClass.SimBaseClass()
    simulationTaskName = "leoDragTask"
    simulationProcess = simulation.CreateNewProcess("leoDragProcess")
    simulationProcess.addTask(simulation.CreateNewTask(simulationTaskName, macros.sec2nano(task_step)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = spacecraft_name
    scObject.hub.mHub = C.MASS
    scObject.hub.r_BcB_B = [0.0, 0.0, 0.0]
    scObject.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(C.INERTIA.flatten())
    scObject.hub.r_CN_NInit = [orbit_radius, 0.0, 0.0]
    scObject.hub.v_CN_NInit = [0.0, orbit_velocity, 0.0]
    scObject.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    scObject.hub.omega_BN_BInit = [0.0, 0.0, 0.0]

    # Earth as the central body: point-mass central gravity, same factory
    # pattern as the demo (D1 pin). No Sun body: nothing here reads it.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)

    simulation.AddModelToTask(simulationTaskName, scObject)

    stateLogger = scObject.scStateOutMsg.recorder()
    simulation.AddModelToTask(simulationTaskName, stateLogger)

    # Exponential atmosphere density source (L1). The spacecraft state is
    # registered so the model writes one AtmoPropsMsg per step.
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "exponentialAtmosphere"
    atmo.baseDensity = BASE_DENSITY
    atmo.scaleHeight = SCALE_HEIGHT
    atmo.planetRadius = C.R_EARTH
    atmo.addSpacecraftToModel(scObject.scStateOutMsg)
    simulation.AddModelToTask(simulationTaskName, atmo)

    # Explicit corotation wind model (L1). Linked below: this is the
    # corotation baseline. If this link were omitted, stock would NOT
    # error -- it would silently use inertial velocity for the relative
    # wind (DESIGN.md A13 wind-unlinked row, ~6% LEO error, labeled).
    # Contrast the density link: leaving THAT unlinked is fatal
    # (stock bskError), so the subscribe below is load-bearing.
    #
    # FIRST-HOST-RUN correction (2026-10-14, ledger audit_bundle/results/
    # leo_drag.json): Basilisk 2.12.0 `WindBase::Reset` ALSO requires
    # `planetPosInMsg` -- as first written the script raised
    # `BasiliskError: WindBase: planetPosInMsg is not linked` at
    # `InitializeSimulation`. That is a MODEL-side link (`WindBase`), not the
    # effector-side `drag.windVelInMsg` the A13 row describes, so the A13
    # "wind-unlinked is not fatal" truth stays true and is NOT contradicted
    # here. Wiring: a stock `SpicePlanetStateMsg` carrying Earth at the
    # inertial origin (the same origin the Earth-central point-mass gravity
    # above already assumes). `epochInMsg` was probed and is NOT required
    # (left unlinked, recorded in the ledger).
    #
    # Corotation is made real, not nominal: with `useSpiceOmegaFlag=True`
    # (default) the planet rate comes from the Spice DCM, and a zeroed
    # payload yields omega == 0 -> v_air_N == 0, i.e. the inertial fallback
    # wearing the corotation label. So the sidereal rate is set explicitly.
    wind = zeroWindModel.ZeroWindModel()
    wind.ModelTag = "zeroWindModel"
    wind.addSpacecraftToModel(scObject.scStateOutMsg)
    earthStateMsg = messaging.SpicePlanetStateMsg()
    earthStateMsg.write(messaging.SpicePlanetStateMsgPayload())
    wind.planetPosInMsg.subscribeTo(earthStateMsg)
    wind.setUseSpiceOmegaFlag(False)
    wind.setPlanetOmega_N([0.0, 0.0, EARTH_ROT_RATE])
    simulation.AddModelToTask(simulationTaskName, wind)

    # Cannonball drag (L1). REQUIRED link: atmoDensInMsg. Optional link:
    # windVelInMsg (linked here; see comment above for the unlinked label).
    drag = dragDynamicEffector.DragDynamicEffector()
    drag.ModelTag = "cannonballDrag"
    drag.coreParams.projectedArea = projected_area
    drag.coreParams.dragCoeff = drag_coeff
    drag.coreParams.comOffset = [0.0, 0.0, 0.0]
    drag.cannonballDrag()
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
    drag.windVelInMsg.subscribeTo(wind.envOutMsgs[-1])
    scObject.addDynamicEffector(drag)
    simulation.AddModelToTask(simulationTaskName, drag)

    densityLogger = atmo.envOutMsgs[-1].recorder()
    simulation.AddModelToTask(simulationTaskName, densityLogger)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(duration))
    simulation.ExecuteSimulation()

    density = np.asarray(densityLogger.neutralDensity, dtype=float)
    return {
        "time": stateLogger.times() * macros.NANO2SEC,
        "truth_position": stateLogger.r_CN_N,
        "truth_velocity": stateLogger.v_CN_N,
        "truth_attitude": stateLogger.sigma_BN,
        "truth_gyro": stateLogger.omega_BN_B,
        "density": density,
        "density_max": float(density.max()) if density.size else float("nan"),
        "altitude": float(altitude),
        "projected_area": float(projected_area),
        "drag_coeff": float(drag_coeff),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Stock 12U LEO-drag fixture (Phase-3 Script-2)")
    parser.add_argument("--duration", type=float, default=T_LEO)
    parser.add_argument("--altitude-km", type=float, default=ALTITUDE_LEO / 1e3)
    parser.add_argument("--projected-area", type=float, default=PROJECTED_AREA)
    parser.add_argument("--drag-coeff", type=float, default=DRAG_COEFF)
    parser.add_argument("--task-step", type=float, default=1.0)
    parser.add_argument("--spacecraft-name", default="cubesat_12u_leo")
    args = parser.parse_args()

    result = run_simulation(
        duration=args.duration,
        altitude=args.altitude_km * 1e3,
        projected_area=args.projected_area,
        drag_coeff=args.drag_coeff,
        task_step=args.task_step,
        spacecraft_name=args.spacecraft_name,
    )
    radius = np.linalg.norm(np.asarray(result["truth_position"]), axis=1)
    print(
        f"[LEO-drag] samples={np.asarray(result['time']).shape[0]} "
        f"duration={np.asarray(result['time'])[-1]:.1f}s "
        f"altitude={result['altitude'] / 1e3:.1f}km "
        f"density_max={result['density_max']:.3e}kg/m^3 "
        f"(gate>{DENSITY_FIRING_GATE:.0e}) "
        f"radius0={radius[0]:.3f}m radiusF={radius[-1]:.3f}m "
        f"decay={radius[0] - radius[-1]:.3f}m"
    )


if __name__ == "__main__":
    main()
