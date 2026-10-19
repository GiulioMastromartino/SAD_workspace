"""Generate a Vizard-compatible ``.bin`` playback of the SAD best optimizer run.

Pipeline
--------
1. Load the best parameter vector from ``results_opt/opt_result.json`` (falling
   back to ``optimization/params.py:DEFAULTS`` if the file is absent).
2. Fly the detumble -> slew -> track mission with the ``sad_py`` C++ modules,
   recording the full ECI state history (position, velocity, quaternion, rate).
3. Resample onto a uniform grid and write a ``DataFileToViz`` CSV:
   ``time, r_CN_N(3), v_CN_N(3), sigma_BN(3), omega_BN_B(3)`` in meters / MRP,
   followed by one thrust-force column per thruster.
4. Build a minimal Basilisk simulation, wire ``dataFileToViz -> vizInterface``
   (states + thruster plumes) and write the ``.bin`` that Vizard plays back.
   The custom 6U OBJ carries a nozzle box at each thruster location.

The dummy Basilisk spacecraft exists only so Vizard has a named body, a planet
and a model to attach; its own dynamics are ignored because the viz reads the
recorded states straight from the ``dataFileToViz`` messages. This is the
``scenarioDataToViz`` pattern from Basilisk.

Run from the ``Python/`` directory (sad_py and Basilisk must be importable)::

    PYTHONPATH=. env/bin/python SAD_Project/backends/basilisk/make_vizard_bin.py

Then open the produced ``.bin`` in Vizard.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]  # .../SAD_Project
_PACKAGE_ROOT = _ROOT.parent                 # .../Python
for _p in (str(_ROOT / "python"), str(_ROOT), str(_PACKAGE_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# --- display constants (kept local so config.py stays dynamics-only) ---------
SPACECRAFT_NAME = "6uSpacecraft"
SUN_BODY_NAME = "sun"
SUN_DISTANCE = 1.496e11
SUN_YEAR = 365.25 * 86400.0
SUN_PHASE0 = 0.5 * np.pi
# The SAD sun-pointing reference aligns body +X with the Sun line (see
# core/sun_model.py: the reference x-axis is the sun direction).
SUN_POINTING_AXIS_B = (1.0, 0.0, 0.0)
SUN_CONE_ANGLE_DEG = 15.0
SUN_CONE_HEIGHT = 2.0
SUN_CONE_COLOR = "lime"
SUN_LINE_COLOR = "yellow"
VIZ_DISPLAY_SCALE = 100.0
VIZ_ORBIT_LINE_SEGMENTS = 200
# Solar wings: cells face the +X sun boresight, wings deploy out the ±Y sides.
PANEL_NORMAL_B = (1.0, 0.0, 0.0)
PANEL_DEPLOY_AXIS_B = (0.0, 1.0, 0.0)
PANEL_DEPLOY = 0.4
PANEL_THICKNESS = 0.01

PARAM_TO_CONFIG = {
    "detumble_K": "ACS_K_DETUMBLE",
    "detumble_P": "ACS_P_DETUMBLE",
    "detumble_Ki": "ACS_KI_DETUMBLE",
    "slew_K": "ACS_K_SLEW",
    "slew_P": "ACS_P_SLEW",
    "slew_Ki": "ACS_KI_SLEW",
    "track_K": "ACS_K_TRACK",
    "track_P": "ACS_P_TRACK",
    "track_Ki": "ACS_KI_TRACK",
}


# --------------------------------------------------------------------------- #
# 1. best parameters
# --------------------------------------------------------------------------- #
def load_best_params(result_path: Path) -> tuple[dict, dict | None]:
    """Best control vector from opt_result.json, else the package defaults."""
    if result_path.exists():
        payload = json.loads(result_path.read_text())
        params = {k: float(v) for k, v in payload["params"].items()}
        return params, payload
    from SAD_Project.optimization.params import DEFAULTS, PARAM_NAMES

    return {n: float(v) for n, v in zip(PARAM_NAMES, DEFAULTS)}, None


def apply_params(config, params: dict) -> None:
    """Push the best gains and TS onto the config module (in place)."""
    for key, attr in PARAM_TO_CONFIG.items():
        setattr(config, attr, params[key])
    config.TS = float(params["TS"])
    config.T_END = config.TS + 2.0 * config.T_ORBIT


# --------------------------------------------------------------------------- #
# 2. fly the mission and record the state history
# --------------------------------------------------------------------------- #
def run_mission_history(params: dict, duration: float | None) -> dict[str, np.ndarray]:
    """Run the SAD mission tick loop and record t, r, v, q, w at every step."""
    import sad_py

    from SAD_Project.core import config as C
    from SAD_Project.core import dynamics as D

    apply_params(C, params)

    cpp_params = sad_py.DynamicsParams()
    cpp_params.mass = C.MASS
    cpp_params.mu = C.MU_EARTH
    cpp_params.gravity_gradient = True
    cpp_params.inertia = C.INERTIA
    cpp_params.inv_inertia = C.INV_INERTIA
    cpp_params.r0 = C.R0_ECI.copy()
    cpp_params.v0 = C.V0_ECI.copy()
    cpp_params.q0 = np.array([0.0, 0.0, 0.0, 1.0])
    cpp_params.w0 = C.OMEGA0_BODY.copy()
    state = sad_py.DynamicsState()
    sad_py.dynamics_init(state, cpp_params)

    thruster_params = sad_py.ThrusterParams()
    thruster_params.n = len(C.THRUSTER_POS)
    thruster_params.pos = [np.array(p, dtype=float) for p in C.THRUSTER_POS]
    thruster_params.dir = [np.array(d, dtype=float) for d in C.THRUSTER_DIR]
    thruster_params.tmax = list(C.THRUSTER_TMAX)
    thruster_params.isp = C.ISP_JET
    thruster_params.g0 = C.G0
    thruster_state = sad_py.ThrusterState()
    sad_py.thruster_init(thruster_state, thruster_params)
    allocator = sad_py.ThrusterAllocator()
    sad_py.thruster_allocator_init(allocator, thruster_params)

    sun_params = sad_py.SunParams()
    sun_params.distance = SUN_DISTANCE
    sun_params.year = SUN_YEAR
    sun_params.phase0 = SUN_PHASE0
    sun_params.roll_ref = np.array([1.0, 0.0, 0.0])
    sun_state = sad_py.SunState()
    sad_py.sun_init(sun_state, sun_params)

    pd_params = sad_py.QuatPDParams()
    pd_params.detumble_K = C.ACS_K_DETUMBLE
    pd_params.detumble_P = C.ACS_P_DETUMBLE
    pd_params.detumble_Ki = C.ACS_KI_DETUMBLE
    pd_params.slew_K = C.ACS_K_SLEW
    pd_params.slew_P = C.ACS_P_SLEW
    pd_params.slew_Ki = C.ACS_KI_SLEW
    pd_params.track_K = C.ACS_K_TRACK
    pd_params.track_P = C.ACS_P_TRACK
    pd_params.track_Ki = C.ACS_KI_TRACK
    pd_params.inertia = np.diag(C.INERTIA)
    pd_params.implicit_dt = bool(getattr(C, "IMPLICIT_PD", True))
    pd_params.implicit_track_only = True
    pd_state = sad_py.QuatPDState()
    sad_py.quatpd_init(pd_state, pd_params)

    mission_params = sad_py.MissionParams()
    mission_state = sad_py.MissionState()
    sad_py.mission_init(mission_state, mission_params)

    star_params = sad_py.StarTrackerParams()
    star_params.sigma = C.STAR_SIGMA
    star_params.rate_cap = C.STAR_RATE_CAP
    star_params.dropout_prob = C.STAR_DROPOUT
    star_params.seed = 7
    star_state = sad_py.StarTrackerState()
    sad_py.startracker_init(star_state, star_params)

    end_time = C.T_END if duration is None else min(float(duration), C.T_END)
    detumble_phase = sad_py.MissionPhase.Detumble
    slew_phase = sad_py.MissionPhase.Slew

    history: dict[str, list] = {k: [] for k in ("t", "r", "v", "q", "w", "thr")}
    t = 0.0
    while t < end_time:
        dt = C.DT if mission_state.phase in (detumble_phase, slew_phase) else C.TRACK_DT

        sun_state.params.roll_ref = D.quat_to_dcm(state.q).T @ np.array([0.0, 1.0, 0.0])
        sun_in = sad_py.SunInput()
        sun_in.t = t
        sun_out = sad_py.SunOutput()
        sad_py.sun_step(sun_in, sun_state, sun_out, dt)

        star_in = sad_py.StarTrackerInput()
        star_in.q_true = state.q.copy()
        star_in.omega_body = state.w.copy()
        star_in.sun_dir_b = np.zeros(3)
        star_out = sad_py.StarTrackerOutput()
        sad_py.startracker_step(star_in, star_state, star_out, dt)

        mission_in = sad_py.MissionInput()
        mission_in.q = state.q.copy()
        mission_in.omega = state.w.copy()
        mission_in.q_sun = sun_out.q_sun_pointing
        mission_in.t = t
        mission_out = sad_py.MissionOutput()
        sad_py.mission_step(mission_in, mission_state, mission_out, dt)

        pd_in = sad_py.QuatPDInput()
        pd_in.q_current = state.q.copy()
        pd_in.omega = state.w.copy()
        pd_in.q_desired = mission_out.q_desired
        pd_in.mode = mission_out.ctrl_mode
        pd_out = sad_py.QuatPDOutput()
        sad_py.quatpd_step(pd_in, pd_state, pd_out, dt)

        thruster_forces = np.asarray(
            sad_py.thruster_allocate_cached(
                allocator, thruster_params, np.zeros(3), pd_out.torque_b
            ),
            dtype=float,
        )
        throttle = [
            thruster_forces[i] / C.THRUSTER_TMAX[i] if C.THRUSTER_TMAX[i] > 0 else 0.0
            for i in range(len(thruster_forces))
        ]
        thruster_in = sad_py.ThrusterInput()
        thruster_in.u = throttle
        thruster_in.is_on_time = False
        thruster_in.dt = dt
        thruster_out = sad_py.ThrusterOutput()
        sad_py.thruster_step(thruster_in, thruster_state, thruster_out, dt)

        dyn_in = sad_py.DynamicsInput()
        dyn_in.force_b = thruster_out.force_b
        dyn_in.torque_b = thruster_out.torque_b
        dyn_out = sad_py.DynamicsOutput()
        sad_py.dynamics_step(dyn_in, state, dyn_out, dt)

        history["t"].append(t)
        history["r"].append(np.array(state.r, dtype=float))
        history["v"].append(np.array(state.v, dtype=float))
        history["q"].append(np.array(state.q, dtype=float))
        history["w"].append(np.array(state.w, dtype=float))
        history["thr"].append(thruster_forces.copy())
        t += dt

    return {
        "t": np.asarray(history["t"]),
        "r": np.asarray(history["r"]),
        "v": np.asarray(history["v"]),
        "q": np.asarray(history["q"]),
        "w": np.asarray(history["w"]),
        "thr": np.asarray(history["thr"]),
    }


# --------------------------------------------------------------------------- #
# 3. resample + CSV for dataFileToViz
# --------------------------------------------------------------------------- #
def quat_to_mrp(quat: np.ndarray) -> np.ndarray:
    """Short-set MRP from a scalar-last quaternion (x, y, z, w)."""
    q = np.asarray(quat, dtype=float)
    q = q / np.linalg.norm(q)
    vector, scalar = q[:3], q[3]
    if scalar < 0.0:
        vector, scalar = -vector, -scalar
    return vector / (1.0 + scalar)


def _slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        out = q0 + u * (q1 - q0)
        return out / np.linalg.norm(out)
    theta0 = np.arccos(np.clip(dot, -1.0, 1.0))
    return (
        np.sin((1.0 - u) * theta0) * q0 + np.sin(u * theta0) * q1
    ) / np.sin(theta0)


def resample_uniform(history: dict[str, np.ndarray], dt: float) -> dict[str, np.ndarray]:
    """Interpolate the non-uniform history onto a uniform grid with spacing dt."""
    t_hist = history["t"]
    duration = float(t_hist[-1])
    n = max(int(round(duration / dt)), 1)
    t_grid = np.arange(n) * dt

    upper = np.clip(np.searchsorted(t_hist, t_grid, side="right"), 1, len(t_hist) - 1)
    lower = upper - 1
    span = np.where(t_hist[upper] > t_hist[lower], t_hist[upper] - t_hist[lower], 1.0)
    frac = np.clip((t_grid - t_hist[lower]) / span, 0.0, 1.0)

    out: dict[str, np.ndarray] = {"t": t_grid}
    for key in ("r", "v", "w"):
        out[key] = history[key][lower] + (history[key][upper] - history[key][lower]) * frac[:, None]
    # Thruster forces are piecewise constant commands; hold them, don't blend.
    out["thr"] = history["thr"][lower]
    quats = np.empty_like(history["q"][lower])
    for i in range(n):
        quats[i] = _slerp(history["q"][lower[i]], history["q"][upper[i]], float(frac[i]))
    out["mrp"] = np.array([quat_to_mrp(q) for q in quats])
    return out


def write_data_file(
    csv_path: Path, sampled: dict[str, np.ndarray], include_thrusters: bool = True
) -> None:
    """Write the comma-delimited DataFileToViz table.

    Columns: ``time, r(3), v(3), sigma_MRP(3), omega(3)`` followed, when
    ``include_thrusters`` is set, by one thrust-force column per thruster in
    ``config.THRUSTER_POS`` order.
    """
    columns = [sampled["t"], sampled["r"], sampled["v"], sampled["mrp"], sampled["w"]]
    names = ["time", "rx", "ry", "rz", "vx", "vy", "vz", "m1", "m2", "m3", "wx", "wy", "wz"]
    if include_thrusters and "thr" in sampled:
        columns.append(sampled["thr"])
        names += [f"thr{i + 1}" for i in range(sampled["thr"].shape[1])]
    table = np.column_stack(columns)
    np.savetxt(csv_path, table, delimiter=",", header=",".join(names), comments="", fmt="%.10e")


# --------------------------------------------------------------------------- #
# 4. Basilisk: dataFileToViz -> vizInterface -> .bin
# --------------------------------------------------------------------------- #
def _make_sun_ephemeris(distance: float, phase0: float):
    """Basilisk SysModel publishing a rotating Earth-centred Sun (Spice msg)."""
    from Basilisk.architecture import messaging, sysModel

    class SunEphemeris(sysModel.SysModel):
        def __init__(self) -> None:
            super().__init__()
            self.distance = float(distance)
            self.rate = 2.0 * np.pi / SUN_YEAR
            self.phase0 = float(phase0)
            self.sunOutMsg = messaging.SpicePlanetStateMsg()
            self.payload = messaging.SpicePlanetStateMsgPayload()

        def _write(self, now_nanos: int) -> None:
            theta = self.phase0 + self.rate * now_nanos * 1e-9
            cos_t, sin_t = np.cos(theta), np.sin(theta)
            omega_dist = self.rate * self.distance
            self.payload.PlanetName = SUN_BODY_NAME
            self.payload.PositionVector = [cos_t * self.distance, sin_t * self.distance, 0.0]
            self.payload.VelocityVector = [-sin_t * omega_dist, cos_t * omega_dist, 0.0]
            self.sunOutMsg.write(self.payload, now_nanos, self.moduleID)

        def Reset(self, now_nanos: int) -> None:
            self._write(now_nanos)

        def UpdateState(self, now_nanos: int) -> None:
            self._write(now_nanos)

    return SunEphemeris()


def build_bin(
    sampled: dict[str, np.ndarray],
    csv_path: Path,
    bin_path: Path,
    dt: float,
    name: str,
    model_path: Path | None,
    sun_cone: bool,
    thrusters: bool = True,
) -> Path:
    """Run the playback sim and write the Vizard ``.bin``."""
    from Basilisk.architecture import messaging
    from Basilisk.simulation import dataFileToViz, spacecraft, vizInterface
    from Basilisk.utilities import (
        SimulationBaseClass,
        macros,
        simHelpers,
        simIncludeGravBody,
        vizSupport,
    )

    from SAD_Project.core import config as C

    if not vizSupport.vizFound:
        raise RuntimeError("Basilisk was built without Vizard vizInterface support")

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("vizProcess")
    task_name = "vizTask"
    process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    # Sun ephemeris first so its message is populated before the gravity reset.
    sun_ephemeris = _make_sun_ephemeris(SUN_DISTANCE, SUN_PHASE0)
    sun_ephemeris.ModelTag = "sunEphemeris"
    sim.AddModelToTask(task_name, sun_ephemeris)

    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True
    sun_body = grav_factory.createSun()
    sun_body.planetBodyInMsg.subscribeTo(sun_ephemeris.sunOutMsg)

    # Dummy body: gives Vizard a name, a planet and a model. Its dynamics are
    # ignored; the viz state messages are re-pointed at dataFileToViz below.
    sc = spacecraft.Spacecraft()
    sc.ModelTag = name
    sc.hub.mHub = C.MASS
    sc.hub.r_BcB_B = [0.0, 0.0, 0.0]
    sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(C.INERTIA.flatten())
    sc.hub.r_CN_NInit = sampled["r"][0].tolist()
    sc.hub.v_CN_NInit = sampled["v"][0].tolist()
    sc.hub.sigma_BNInit = sampled["mrp"][0].tolist()
    sc.hub.omega_BN_BInit = sampled["w"][0].tolist()
    grav_factory.addBodiesTo(sc)
    sim.AddModelToTask(task_name, sc)

    data_module = dataFileToViz.DataFileToViz()
    data_module.ModelTag = "stateFile"
    data_module.setNumOfSatellites(1)
    data_module.dataFileName = str(csv_path)
    data_module.delimiter = ","
    data_module.attitudeType = 0  # MRP
    data_module.convertPosToMeters = 1.0  # states already in meters / m/s
    data_module.headerLine = True

    thruster_positions = np.asarray(C.THRUSTER_POS, dtype=float)
    thruster_dirs = np.asarray(C.THRUSTER_DIR, dtype=float)
    thruster_tmax = np.asarray(C.THRUSTER_TMAX, dtype=float)
    n_thrusters = int(thruster_positions.shape[0])
    if thrusters:
        thr_map = dataFileToViz.ThrClusterMap()
        thr_map.thrTag = "thrusters"
        thr_map.color = vizSupport.toRGBA255("orange")
        # Must mirror the cluster/thruster order of the CSV force columns.
        data_module.appendThrClusterMap(
            dataFileToViz.VizThrConfig([thr_map]), dataFileToViz.IntVector([n_thrusters])
        )
        for position, direction, force_max in zip(
            thruster_positions, thruster_dirs, thruster_tmax
        ):
            data_module.appendThrPos(position.tolist())
            data_module.appendThrDir(direction.tolist())
            data_module.appendThrForceMax(float(force_max))

    sim.AddModelToTask(task_name, data_module)

    viz = vizSupport.enableUnityVisualization(
        sim, task_name, [sc], saveFile=str(bin_path)
    )
    # Use the file states, not the dummy spacecraft's, for rendering.
    viz.scData[0].spacecraftName = name
    viz.scData[0].scStateInMsg.subscribeTo(data_module.scStateOutMsgs[0])

    if thrusters:
        # Route the dataFileToViz thruster messages into Vizard so the plumes
        # fire in sync with the recorded actuation. One cluster, n_thrusters
        # messages, so every message maps back to that single cluster tag.
        thr_list = [msg.addSubscriber() for msg in data_module.thrScOutMsgs[0]]
        clusters = list(data_module.thrMsgDataSC[0])
        thr_info = [clusters[0]] * len(thr_list)
        viz.scData[0].thrInMsgs = messaging.THROutputMsgInMsgsVector(thr_list)
        viz.scData[0].thrInfo = vizInterface.ThrClusterVector(thr_info)
        vizSupport.setActuatorGuiSetting(
            viz,
            viewThrusterPanel=1,
            viewThrusterHUD=1,
            showThrusterLabels=1,
        )

    if model_path is not None:
        vizSupport.createCustomModel(
            viz,
            modelPath=str(model_path),
            simBodiesToModify=[name],
            scale=[1.0, 1.0, 1.0],
        )

    viz.settings.spacecraftSizeMultiplier = VIZ_DISPLAY_SCALE
    viz.settings.showSpacecraftAsSprites = 1
    viz.settings.showCelestialBodiesAsSprites = 1
    viz.settings.showSpacecraftLabels = 1
    viz.settings.orbitLinesOn = 1
    viz.settings.orbitLineSegments = VIZ_ORBIT_LINE_SEGMENTS
    viz.settings.trueTrajectoryLinesOn = 0
    viz.settings.mainCameraTarget = name
    viz.settings.forceStartAtSpacecraftLocalView = 1

    if sun_cone:
        vizSupport.createConeInOut(
            viz,
            toBodyName=SUN_BODY_NAME,
            coneColor=SUN_CONE_COLOR,
            isKeepIn=True,
            normalVector_B=list(SUN_POINTING_AXIS_B),
            incidenceAngle=np.deg2rad(SUN_CONE_ANGLE_DEG),
            coneHeight=SUN_CONE_HEIGHT,
            fromBodyName=name,
            coneName="sunKeepInCone",
        )
        vizSupport.createPointLine(
            viz,
            toBodyName=SUN_BODY_NAME,
            fromBodyName=name,
            lineColor=SUN_LINE_COLOR,
        )

    duration = float(sampled["t"][-1] + dt)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(duration))
    sim.ExecuteSimulation()
    return bin_path


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--result",
        default=str(_ROOT / "results_opt" / "opt_result.json"),
        help="optimizer result JSON holding the best params",
    )
    parser.add_argument(
        "--out",
        default=str(_ROOT / "results_opt" / "viz_best_config.bin"),
        help="Vizard .bin to write",
    )
    parser.add_argument("--csv", default=None, help="intermediate DataFileToViz CSV")
    parser.add_argument(
        "--viz-dt", type=float, default=0.5, help="uniform playback step [s]"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="cap the mission length [s]; default is the full T_END",
    )
    parser.add_argument("--spacecraft-name", default=SPACECRAFT_NAME)
    parser.add_argument(
        "--no-model", action="store_true", help="skip the custom 6U OBJ model"
    )
    parser.add_argument(
        "--no-sun-cone", action="store_true", help="omit the sun keep-in cone/line"
    )
    parser.add_argument(
        "--no-thrusters",
        action="store_true",
        help="omit thruster plumes/nozzles and the thruster CSV columns",
    )
    args = parser.parse_args()

    result_path = Path(args.result)
    bin_path = Path(args.out)
    csv_path = Path(args.csv) if args.csv else bin_path.with_suffix(".csv")
    with_thrusters = not args.no_thrusters

    params, payload = load_best_params(result_path)
    if payload is None:
        print(f"[vizard] {result_path} not found; using optimization/params.py DEFAULTS")
    else:
        print(f"[vizard] best config: cost={payload.get('cost')} params={params}")

    print("[vizard] flying mission (sad_py)...")
    history = run_mission_history(params, args.duration)
    print(f"[vizard] recorded {history['t'].size} steps, T_end={history['t'][-1]:.1f}s")

    sampled = resample_uniform(history, args.viz_dt)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    write_data_file(csv_path, sampled, include_thrusters=with_thrusters)
    print(f"[vizard] wrote data file: {csv_path} ({sampled['t'].size} rows)")

    from SAD_Project.core import config as C

    model_path = None
    if not args.no_model:
        from SAD_Project.backends.basilisk import vizard_model

        model_path = vizard_model.write_panelled_bus(
            bin_path.with_suffix(".obj"),
            lx=C.LX,
            ly=C.LY,
            lz=C.LZ,
            deploy=PANEL_DEPLOY,
            thickness=PANEL_THICKNESS,
            normal_B=PANEL_NORMAL_B,
            deployAxis_B=PANEL_DEPLOY_AXIS_B,
            thruster_positions=C.THRUSTER_POS if with_thrusters else None,
            thruster_dirs=C.THRUSTER_DIR if with_thrusters else None,
        )
        print(f"[vizard] wrote model: {model_path}")

    build_bin(
        sampled,
        csv_path,
        bin_path,
        args.viz_dt,
        args.spacecraft_name,
        model_path,
        sun_cone=not args.no_sun_cone,
        thrusters=with_thrusters,
    )
    print(f"[vizard] wrote playback file: {bin_path}")
    print("[vizard] open it in Vizard (File > Open Results/Playback).")


if __name__ == "__main__":
    main()
