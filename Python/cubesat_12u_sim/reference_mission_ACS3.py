"""Reference mission: NASA ACS3 (Advanced Composite Solar Sail System) — 12U CubeSat in LEO.

Use this file as the real-world anchor for ``cubesat_12u_sim``. The default
``config.py`` baseline (circular equatorial free-fall at 12000 km altitude,
20 kg generic 12U bus, point-mass gravity, no drag, J2 off) is a clean
dynamics test case — it is NOT the ACS3 orbit. This module records the ACS3
mission facts and gives ready-to-use orbit numbers + an inclined-SSO initial
state compatible with the 12-state EOM ``y = [r, v, euler, omega]``.

Mission summary
---------------
- Name: Advanced Composite Solar Sail System (ACS3), NASA STMD technology demo.
- Operator / lead: NASA Ames Research Center (mission) / Langley Research
  Center (composite booms + sail); bus built by NanoAvionics (now Kongsberg
  NanoAvionics); flight dynamics / navigation by NASA Ames FDS team.
- Form factor / mass: 12U CubeSat, ~16 kg (≈16.1 kg launch/dry mass).
- Sail: ~80 m² (≈81 m² quoted post-flight; ~9 m side), composite booms,
  PEN film + metals, 3-axis stabilised bus.
- Launch: 23 Apr 2024 UTC (24 Apr NZDT) on Rocket Lab Electron
  ("Beginning Of The Swarm") from LC-1 Mahia, New Zealand; rideshare with
  NEONSAT-1. COSPAR 2024-077B, NORAD 59588.
- Sail deployment: 29 Aug 2024. Mission ongoing into 2025 (boom-bend anomaly
  noted Oct 2024, operations continued).
- Primary objective: deploy composite booms + sail in orbit and demonstrate
  orbit maneuvering (semi-major-axis / altitude change) with solar radiation
  pressure; validate high-fidelity SRP + drag propagation and GPS-based orbit
  determination for future large-sail missions.

Nominal orbit (descriptive values for simulation)
------------------------------------------------
- Orbit type: near-circular Sun-synchronous LEO, ~1000 km class. Chosen high
  enough that solar radiation pressure on the deployed sail can compete with
  / exceed aerodynamic drag.
- Pre-flight design (NASA papers): 1000 km circular SSO.
- Independent optical/TLE tracking just after launch: ~994 x 1023 km,
  i ≈ 97.4°. Post-deployment tracked set (KeepTrack/NORAD 59588, Feb 2025
  snapshot): ~860 x 958 km, i ≈ 97.2°, T ≈ 103 min (orbit lowered/decayed
  under drag + sail operations).
- Values adopted here as the sim reference (rounded, near-circular):
    perigee ≈ 960–1000 km, apogee ≈ 1000 km,
    mean altitude ≈ 1000 km, e ≈ 0.001,
    inclination ≈ 97.4° observed / 99–99.5° classical SSO design band,
    period ≈ 105.1 min (at exactly 1000 km circular).

  NOTE on inclination: classical SSO theory at 1000 km gives i ≈ 99.5°;
  flown/observed TLEs cluster near 97–98°. Both numbers appear in the
  literature, so both are recorded below (INC_DESIGN_DEG vs INC_OBSERVED_DEG).
  Either is defensible for an attitude-dynamics sim; state clearly which one
  you use.

Primary sources: NASA STMD ACS3 mission page (nasa.gov/mission/acs3),
NASA NTRS 20230004066 ("Flight Dynamics for a Solar Sail Technology
Demonstration Mission") and NTRS 20240004424 ("Navigation for the ACS3 Solar
Sail Mission"), Rocket Lab "Beginning Of The Swarm" launch kit,
independent SatTrackCam Leiden post-deployment analysis + KeepTrack/NORAD
59588 TLE snapshots (as-flown cross-check only).

Implications for cubesat_12u_sim
--------------------------------
- Inclination matters: default R0/V0 in config.py is equatorial (i = 0°).
  Use ``initial_state_eci()`` here for an inclined SSO start.
- J2 matters: SSO node precession IS the J2 effect; keep ``USE_J2 = False``
  only for the Keplerian baseline — switch it on (or add an SSO model) for
  any ACS3-anchored run.
- Drag/SRP matter: at 1000 km both are small but non-zero (and SRP is the
  experiment once the 80 m² sail is out). The current dynamics.py has neither;
  treat runs with this file as "gravity + attitude baseline", not a full
  ACS3 propagator.
- Gravity-gradient scales as 1/R³: ~ (18378/7378)³ ≈ 15× stronger at
  1000 km than at the 12000 km default. Bus mass also differs (16 vs 20 kg).
"""
from __future__ import annotations

import math
import numpy as np

# ---------------- Mission facts ----------------
MISSION_NAME = "ACS3 (Advanced Composite Solar Sail System)"
OPERATOR = "NASA Ames Research Center / Langley Research Center"
BUS_BUILDER = "NanoAvionics (Kongsberg NanoAvionics)"
FORM_FACTOR = "12U CubeSat"
MASS_KG = 16.1  # [kg] launch/dry mass (config.py default is a generic 20 kg 12U)
SAIL_AREA_M2 = 80.0  # [m^2] design value (~81 m^2 quoted in NTRS 20240004424)
LAUNCH_DATE_UTC = "2024-04-23"  # 24 Apr NZDT; commonly cited as "April 24, 2024"
LAUNCH_VEHICLE = "Rocket Lab Electron (Beginning Of The Swarm)"
LAUNCH_SITE = "LC-1, Mahia, New Zealand"
COSPAR_ID = "2024-077B"
NORAD_ID = "59588"
OBJECTIVE = (
    "Deploy composite booms + ~80 m^2 sail from a 12U bus and demonstrate "
    "orbit maneuvering (semi-major-axis change) using solar radiation pressure."
)

# ---------------- Earth constants (same as config.py) ----------------
MU_EARTH: float = 3.986004418e14  # [m^3/s^2]
R_EARTH: float = 6378.137e3  # [m]

# ---------------- Reference orbit (nominal, near-circular LEO) ----------------
ALT_PERIGEE: float = 960e3  # [m] lower edge of quoted band
ALT_APOGEE: float = 1000e3  # [m]
ALT_MEAN: float = 1000e3  # [m] design circular altitude ("1000 km SSO")
ECCENTRICITY: float = 0.001  # near-circular
# Classical SSO design band at ~1000 km vs. as-flown observed value:
INC_DESIGN_DEG: float = 99.0  # representative of the quoted 99–99.5 deg band
INC_OBSERVED_DEG: float = 97.4  # SatTrackCam/KeepTrack post-launch TLE set
INC_DEG: float = INC_OBSERVED_DEG  # adopted default for initial_state_eci()

A_SEMI: float = R_EARTH + ALT_MEAN  # [m] ≈ 7378.137 km
R_PERIGEE: float = R_EARTH + ALT_PERIGEE
R_APOGEE: float = R_EARTH + ALT_APOGEE
V_CIRC: float = float(math.sqrt(MU_EARTH / A_SEMI))  # [m/s] ≈ 7350 m/s
T_ORBIT: float = float(2.0 * math.pi * math.sqrt(A_SEMI**3 / MU_EARTH))  # [s] ≈ 6307 s

ORBIT = {
    "type": "Sun-synchronous LEO (near-circular)",
    "alt_perigee_m": ALT_PERIGEE,
    "alt_apogee_m": ALT_APOGEE,
    "alt_mean_m": ALT_MEAN,
    "semi_major_axis_m": A_SEMI,
    "eccentricity": ECCENTRICITY,
    "inc_design_deg": INC_DESIGN_DEG,
    "inc_observed_deg": INC_OBSERVED_DEG,
    "inc_adopted_deg": INC_DEG,
    "velocity_circ_m_s": V_CIRC,
    "period_s": T_ORBIT,
    "period_min": T_ORBIT / 60.0,
}


def initial_state_eci(inc_deg: float = INC_DEG, anomaly_deg: float = 0.0):
    """Circular inclined-LEO initial state in ECI (SSO-compatible).

    Start on the +X axis (ascending node) by default and rotate the
    equatorial circular velocity into the inclined plane::

        r = [a, 0, 0]
        v = [0, V*cos(i), V*sin(i)]  (|v| = V, inclination = i)

    Args:
        inc_deg: orbit inclination [deg] (default: adopted ACS3 value).
        anomaly_deg: rotate position/velocity within the orbital plane from
            the node [deg] (default 0 = node crossing).

    Returns:
        (r_eci, v_eci) in SI units [m, m/s].
    """
    inc = math.radians(inc_deg)
    u = math.radians(anomaly_deg)
    # In-plane circular state, then tilt about the line of nodes (X axis).
    r_pf = A_SEMI * np.array([math.cos(u), math.sin(u), 0.0])
    v_pf = V_CIRC * np.array([-math.sin(u), math.cos(u), 0.0])
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(inc), -math.sin(inc)],
        [0.0, math.sin(inc), math.cos(inc)],
    ])
    return Rx @ r_pf, Rx @ v_pf


def summary() -> str:
    """One-block human-readable summary (mission + orbit + sim deltas)."""
    return (
        f"{MISSION_NAME} — {FORM_FACTOR}, {MASS_KG:.1f} kg — {OPERATOR}\n"
        f"Launch {LAUNCH_DATE_UTC} on {LAUNCH_VEHICLE} ({LAUNCH_SITE}); "
        f"{COSPAR_ID} / NORAD {NORAD_ID}\n"
        f"Sail ~{SAIL_AREA_M2:.0f} m^2. Objective: {OBJECTIVE}\n"
        f"Orbit: SSO LEO, perigee {ALT_PERIGEE/1e3:.0f} km / apogee "
        f"{ALT_APOGEE/1e3:.0f} km, e~{ECCENTRICITY}, "
        f"i design ~{INC_DESIGN_DEG:.1f} deg / observed ~{INC_OBSERVED_DEG:.1f} deg, "
        f"T~{T_ORBIT/60.0:.1f} min, V~{V_CIRC:.0f} m/s.\n"
        f"Sim note: config.py default is equatorial @12000 km / 20 kg / no-J2 / "
        f"no-drag; for ACS3 use this module's inclined state, 16.1 kg, J2 on, "
        f"and add drag+SRP for sail-deployed phases."
    )


if __name__ == "__main__":
    print(summary())
    r, v = initial_state_eci()
    print(f"r0 = {r/1e3} km  |r| = {np.linalg.norm(r)/1e3:.3f} km")
    print(f"v0 = {v} m/s  |v| = {np.linalg.norm(v):.2f} m/s")
