"""Sun ephemeris and sun-pointing attitude reference.

Split out of `basilisk_sim` so the simulation driver keeps its lazy Basilisk
import while these Basilisk-backed models stay directly testable.

The constrained boresight is `config.SUN_POINTING_AXIS_B` (body +X); the solar
wings lie in the body XY plane (normal +Z, see `config.VIZ_PANEL_NORMAL_B`).
Everything here exists to hold that boresight on the Sun line.
"""
from __future__ import annotations

import numpy as np
from Basilisk.architecture import messaging, sysModel

from cubesat_12u_sim.core import config as C


class SunEphemeris(sysModel.SysModel):
    """Rotating Sun state in the Earth-centred inertial frame.

    The frame is Earth-centred, so the Sun sweeps a full circle once per tropical
    year. Position, velocity and the derived unit direction are mutually
    consistent: the velocity is the exact derivative of the circular path, so
    the gravity effector, the CSS cluster and the attitude reference all see the
    same Sun. Adding the Sun to the gravity field therefore perturbs the orbit
    only through tidal terms, since the uniform component cancels against the
    central body.
    """

    def __init__(self, distance: float, phase0: float):
        super().__init__()
        self.distance = float(distance)
        self.angularRate = 2.0 * np.pi / C.SUN_YEAR
        self.phase0 = float(phase0)
        self.sunOutMsg = messaging.SpicePlanetStateMsg()
        self.payload = messaging.SpicePlanetStateMsgPayload()
        self.direction = np.array([0.0, 0.0, 0.0])

    def directionAt(self, simTime: float) -> np.ndarray:
        """Unit vector from Earth to the Sun at ``simTime`` [s]."""
        theta = self.phase0 + self.angularRate * simTime
        return np.array([np.cos(theta), np.sin(theta), 0.0])

    def _write(self, CurrentSimNanos: int) -> None:
        simTime = CurrentSimNanos * 1.0e-9
        theta = self.phase0 + self.angularRate * simTime
        cosT, sinT = np.cos(theta), np.sin(theta)
        rate = self.angularRate * self.distance
        self.payload.PlanetName = C.VIZ_SUN_BODY_NAME
        self.payload.PositionVector = [cosT * self.distance, sinT * self.distance, 0.0]
        self.payload.VelocityVector = [-sinT * rate, cosT * rate, 0.0]
        self.direction = np.array([cosT, sinT, 0.0])
        self.sunOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

    def Reset(self, CurrentSimNanos: int) -> None:
        self._write(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        self._write(CurrentSimNanos)


class SunPointingReference(sysModel.SysModel):
    """Attitude reference holding ``SUN_POINTING_AXIS_B`` on the Sun.

    Basilisk's ``spacecraftPointing`` module would do this, but it requires a
    ``NavTransMsg`` for both the chief and the deputy body, which would mean
    duplicating the Sun state this scenario already publishes as a
    ``SpicePlanetStateMsg``. Computing the reference from the same ephemeris
    keeps one source of truth for the Sun direction.
    """

    def __init__(self, ephemeris: SunEphemeris, rollReference_N: np.ndarray):
        super().__init__()
        self.ephemeris = ephemeris
        self.rollReference = np.asarray(rollReference_N, dtype=float)
        self.attRefOutMsg = messaging.AttRefMsg()
        self.payload = messaging.AttRefMsgPayload()
        self.sigma = np.zeros(3)
        # The Sun sweeps the inertial XY plane, so the reference attitude is a
        # rotation about inertial +Z. Its rate is therefore exactly the Sun's
        # angular rate, which is analytic -- no MRP difference needed.
        self.omega = np.array([0.0, 0.0, ephemeris.angularRate])

    def sigmaAt(self, simTime: float) -> np.ndarray:
        return sun_pointing_sigma(
            self.ephemeris.directionAt(simTime) * C.VIZ_SUN_DISTANCE, self.rollReference
        )

    def _write(self, CurrentSimNanos: int) -> None:
        simTime = CurrentSimNanos * 1.0e-9
        self.sigma = self.sigmaAt(simTime)
        self.payload.sigma_RN = self.sigma.tolist()
        self.payload.omega_RN_N = self.omega.tolist()
        self.payload.domega_RN_N = [0.0, 0.0, 0.0]
        self.attRefOutMsg.write(self.payload, CurrentSimNanos, self.moduleID)

    def Reset(self, CurrentSimNanos: int) -> None:
        self._write(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        self._write(CurrentSimNanos)


def sun_pointing_sigma(
    sunDir_N: np.ndarray,
    rollReference_N: np.ndarray = C.SUN_ROLL_REFERENCE_N,
) -> np.ndarray:
    """Body attitude (MRP) that aims `SUN_POINTING_AXIS_B` at the Sun.

    The remaining two axes are filled in from `rollReference_N`, projected
    perpendicular to the Sun line, using a cyclic assignment so the triad is
    always right-handed. The DCM is converted with Basilisk's own C2MRP so the
    convention matches `sigma_BN` exactly. C2MRP takes the body axes as DCM
    *columns*, so column k of MRP2C(sigma) is body axis k in the inertial frame.
    """
    from Basilisk.utilities.RigidBodyKinematics import C2MRP

    sunDir = np.asarray(sunDir_N, dtype=float)
    sunDir = sunDir / np.linalg.norm(sunDir)
    boresightIndex = _axis_index(C.SUN_POINTING_AXIS_B, "SUN_POINTING_AXIS_B")

    reference = np.asarray(rollReference_N, dtype=float)
    projected = reference - np.dot(reference, sunDir) * sunDir
    if np.linalg.norm(projected) < 1e-9:
        raise ValueError("roll reference is parallel to the Sun direction")
    projected = projected / np.linalg.norm(projected)

    # b[k] is the Sun direction, b[m] the projected roll reference, and the third
    # axis the cross product, so b[k] x b[m] = b[n] and det(DCM) = +1.
    k = boresightIndex
    m = (k + 1) % 3
    n = (k + 2) % 3
    basis = np.zeros((3, 3), dtype=float)
    basis[k] = sunDir
    basis[m] = projected
    basis[n] = np.cross(basis[k], basis[m])
    return np.asarray(C2MRP(basis.T), dtype=float)


def _axis_index(vector, name: str) -> int:
    """Index of the body axis a direction names; must be axis-aligned.

    Uses the magnitude because a signed unit vector such as (0, 0, -1) must
    resolve to the z axis, which a plain argmax would get wrong.
    """
    axis = np.asarray(vector, dtype=float)
    if axis.shape != (3,):
        raise ValueError(f"{name} must be a 3-vector, got shape {axis.shape}")
    if not np.isclose(np.linalg.norm(axis), 1.0, atol=1e-9):
        raise ValueError(f"{name} must be a unit vector, got {np.linalg.norm(axis)}")
    nonzero = np.flatnonzero(np.abs(axis) > 1e-9)
    if nonzero.size != 1:
        raise ValueError(
            f"{name} must be aligned with a single body axis, got {tuple(axis)}"
        )
    return int(nonzero[0])
