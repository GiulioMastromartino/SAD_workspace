# 12U CubeSat — Attitude + Orbit Monte Carlo — Free-fall @ 12000 km

Base 6-DOF project inside `Python/cubesat_12u_sim/`. Two attitude
representations coexist: Euler 3-2-1 for the legacy orbit demos, MRP for the
Monte Carlo + Heyoka paths (no singularity at working angles).

## Parameters (see `config.py`)
- **Bus 12U:** 200×200×300 mm, `MASS=20 kg`, prism inertia
  `Ix=m/12(y²+z²)` etc. computed automatically.
- **Attitude:** Euler angles 3-2-1 (ZYX yaw-pitch-roll), `euler=[phi,theta,psi]`,
  singularity at `theta=±90°` (regularised + warned); MRP `sigma_BN` in the
  Basilisk / Monte Carlo / Heyoka path (shadow-set kept short).
- **Orbit / free-fall:** circular equatorial, `ALTITUDE=12000 km` above surface,
  `R_ORBIT=R_EARTH+ALTITUDE`, `V_CIRC=sqrt(mu/R)`. Point-mass gravity (+optional
  gravity-gradient torque, J2 off) — no drag at this altitude.
- **Variable-thrust jets:** 12 throttlable thrusters `u∈[0,1]`, `Tmax=1 N` each,
  `F,tau = jets_to_wrench(u)`; `allocation_pinv(F_des,tau_des)` for control
  (`allocate_cached` reuses the per-run pseudoinverse). Measured
  `thrusterDynamicEffector` behavior is a 3-level staircase, see
  `actuators.quantize_duty_cycle` and the Heyoka section below.
- **Gyroscopes:** `sensors.RateGyro` — bias + ARW white noise + bias random walk.
  Optional `actuators.ReactionWheels` (3-axis, saturating) for attitude control.
- **Basilisk backend:** `basilisk_sim.py` — optional Basilisk spacecraft model with
  an IMU and three coarse Sun sensors, using the same 12U bus parameters.

## Layout
- `config.py` — all physical / sensor / sim parameters
- `dynamics.py` — DCM, Euler kinematics, gravity, 12-state EOM
- `actuators.py` — jets wrench + allocation (+cached pinv, effector quantizer) + reaction wheels
- `sensors.py` — rate gyro model
- `sun_model.py` — Sun ephemeris + sun-pointing MRP reference (shared by all backends)
- `basilisk_sim.py` — optional Basilisk IMU and coarse Sun-sensor simulation
- `monte_carlo.py` — Basilisk 3-maneuver Monte Carlo (`run_monte_carlo`, `--jobs`)
- `run_monte_carlo.py` — CLI runner for the Basilisk campaign
- `heyoka_attitude.py` — MRP Taylor system + closed-loop propagator mirroring the MC
- `heyoka_mc.py` — batch/ensemble Heyoka screening runner (SIMD + processes + C++)
- `heyoka_sidecar.py` — Basilisk SysModel flying a Heyoka shadow alongside truth
- `fastloop/` — pybind11 C++ per-tick control kernel (numpy fallback automatic)
- `propagate.py` — `solve_ivp` harness (free-fall default `u=0`)
- `controller.py` — simple RTN PID (`OrbitPID`) → jet throttles
- `heyoka_propagate.py` — heyoka Taylor 12-state propagation + closed-loop stepping
- `run_stationkeeping.py` — station-keeping demo (injection error + PID recovery)
- `run_3year.py` — 3-year orbit-only campaign
- `reference_mission_ACS3.py`, `vizard_model.py`, `models/` — reference mission, Vizard assets
- `main.py` — demo + plots
- `tests/` — smoke, sun-pointing, Heyoka/parity/quantizer/native tests (41 tests)
- `cpp_rw.py` — Python-SysModel C++-Heyoka RW dynamics (`run_cpp_rw_pipeline`,
  `run_cpp_rw_mission`), `run_cpp_rw_compare.py`, `run_rw_timing.py`,
  `run_rw_debug.py` — RW backend ladder (stock / py-mirror / cpp-pipeline /
  native / cpp-mission) with parity + wall-time reporting
- `cpp_native.py` — `NativeSpacecraft` (stock `Spacecraft` scripting API over
  the native module) + `run_native_rw_pipeline`
- `../native_rw/` — native extension sources (see its README): true compiled
  C++ `SysModel` (`HeyokaRWDynamics`) via `bsk-sdk`, no full-source build

## Run (from `Python/`)
```bash
./env/bin/python -m cubesat_12u_sim.main
./env/bin/python -m cubesat_12u_sim.main --burn   # with small jet burn
./env/bin/python -m pytest cubesat_12u_sim/tests -q
./env/bin/python -m cubesat_12u_sim.run_monte_carlo --runs 30 --jobs 4
./env/bin/python -m cubesat_12u_sim.heyoka_mc --runs 500 --jobs 4
./env/bin/python -m cubesat_12u_sim.run_rw_timing --repeats 5
```

## Basilisk sensor simulation
Install the optional Basilisk wheel with `./env/bin/python -m pip install bsk`, then run:
```bash
./env/bin/python -m cubesat_12u_sim.basilisk_sim
./env/bin/python -m cubesat_12u_sim.basilisk_sim --maneuver detumble --duration 600
./env/bin/python -m cubesat_12u_sim.basilisk_sim --maneuver slew --duration 600
```

`detumble` damps the initial body rate while holding the initial attitude; `slew`
drives the spacecraft to the 180-degree inertial target defined by
`config.ACS_SLEW_SIGMA` (`|sigma|=1`, i.e. `4*atan(1)`). Both use the Basilisk MRP feedback controller and the
12-thruster RCS through `thrusterDynamicEffector`; no reaction wheels or
external-torque actuator are used. The controller rate feedback is the
noisy simulated IMU output, while the attitude component is the available
spacecraft attitude state. `--heyoka-shadow` flies the Heyoka MRP shadow
alongside truth and reports divergence (see Heyoka section).

The observer comparison is available with:
```bash
./env/bin/python -m cubesat_12u_sim.basilisk_sim --compare --duration 600
```
It runs truth-rate, raw-IMU, and bias-observer feedback for both detumble and
slew, then writes `controller_comparison.png`.

Vizard visualization is optional and requires the separate Vizard application:
```bash
./env/bin/python -m cubesat_12u_sim.basilisk_sim --maneuver slew --duration 600 --visualize --live-stream
./env/bin/python -m cubesat_12u_sim.basilisk_sim --maneuver detumble --visualize --save-viz detumble_viz.bin
./env/bin/python -m cubesat_12u_sim.basilisk_sim --maneuver slew --visualize --live-stream --render-model 6USat
# both maneuvers back to back against a single Vizard instance
./env/bin/python -m cubesat_12u_sim.basilisk_sim --maneuver both --duration 600 --visualize --live-stream
./env/bin/python -m cubesat_12u_sim.basilisk_sim --maneuver sunpoint --duration 600 --visualize --live-stream
# orbit viewing: one full revolution is 24795 s, so record it and play it back
./env/bin/python -m cubesat_12u_sim.basilisk_sim --duration 24795 --visualize --save-viz orbit_1rev.bin
open /Applications/Vizard.app --args -loadFile "$PWD/orbit_1rev.bin"
```
One revolution at 12000 km is `config.T_ORBIT` = 24795 s (6.89 h). A 600 s run
therefore covers only 8.7° of arc, which deviates from a straight line by at
most 53 km out of an 18378 km radius — it looks straight even though the
trajectory is a correct circle. Use `--save-viz` over a long duration and play
it back rather than live-streaming: `liveStream` is lockstep with Vizard's
rendering, so it runs at wall-clock speed. The osculating orbit line
(`orbitLinesOn`) is enabled so the orbit shape is visible even on short runs.

`--maneuver both` runs detumble then slew in sequence through one Vizard
instance (`run_maneuver_pair`). Vizard is launched once on the first maneuver;
each maneuver is an independent `SimulationBaseClass`, so the vizInterface REQ
socket is torn down and re-bound on the same port between them and Vizard
reconnects automatically. The two scenes are labelled
`12uSpacecraft_detumble` and `12uSpacecraft_slew`. With
`--save-viz pair.bin` the recordings are split to `pair_detumble.bin` and
`pair_slew.bin`. Results are bit-identical to running each maneuver on its own.
Both converge inside the default 600 s (detumble 1.2e-2, slew 2.7e-2 final MRP
error).

Leave the flown-trajectory line off on live runs. Vizard appends a point per
step, so every VizMessage then carries an ever-growing polyline: with it
enabled a 600 s live-stream run took over 20 minutes versus 63 s without it.
`--trajectory-line` opts in, which is fine for short playback runs.

The saved `.bin` file can be opened later in Vizard; the visualization includes
the spacecraft, CSS sensors, IMU marker, and RCS thruster cluster. Use
`--render-model PANELS` (the default) for the 12U bus with two deployed solar
wings, `CUBE`, `CYLINDER`, or `SPHERE` for bare primitives, or `bskSat`,
`3USat`, and `6USat` for Vizard's built-in models; the key is matched
case-insensitively and any other value is rejected. Use `--spacecraft-name` to
change the displayed spacecraft name.

`PANELS` is generated at run time by `vizard_model.write_panelled_bus()` as a
Wavefront OBJ plus MTL in `cubesat_12u_sim/models/` (git-ignored, regenerated
each run and deliberately not deleted so saved `.bin` files can still replay
it). Vizard interprets spacecraft models in meters, so the geometry is authored
in the body frame at true scale and `createCustomModel` is called with
`scale=[1, 1, 1]`. Bus material is light grey, wings dark blue with a specular
cell sheen; wing length outboard of each ±X face is `config.VIZ_PANEL_DEPLOY`
(0.5 m, giving a 1.2 m total span) and thickness is
`config.VIZ_PANEL_THICKNESS`.

This is a visualization asset only. `config.INERTIA` stays the solid-prism value
for the bare bus and is never derived from the wings;
`tests/test_vizard_model.py` locks that invariant.

The primitive/bus geometry is drawn at true 12U dimensions so the thruster, CSS
and IMU overlays line up with the metre-valued body-frame offsets.
`config.VIZ_DISPLAY_SCALE`
only enlarges the Planet and Solar System views, where a 0.2 m bus would
otherwise be sub-pixel; the sprite fallback (`showSpacecraftAsSprites`) covers
the range where even that is too small, and Vizard opens in the spacecraft
local view so the bus is always framed. The live-stream endpoint
(`config.VIZ_COM_*`) and the Vizard launch path (`config.VIZ_APP_PATH`) are
single-sourced in `config.py`, so the client and the launched app cannot drift
apart.

Earth is registered as the central body via `simIncludeGravBody.gravBodyFactory`,
which serves two purposes: it closes the orbit (`R0_ECI`/`V0_ECI` describe a
12000 km circular orbit, so without a gravity body the spacecraft drifts in a
straight line), and it gives Vizard a parent body. `vizSupport` discovers grav
bodies through `scObject.gravField`, so without this there is no planet view and
`spacecraftSizeMultiplier` has no view to act on. Point-mass central gravity
applies no torque about the CoM and no gravity-gradient module is used, so the
attitude results are bit-identical with and without it.

The Sun is attached to the same factory so Vizard has a named body for the
constraint below to reference. It **must** carry a written ephemeris: left at the
default, `planetBodyInMsg` puts it at the origin next to Earth and the
trajectory diverges to NaN. With a consistent ephemeris the uniform pull cancels
against the central body and only tidal acceleration remains, measured at
23.281 km versus 23.048 km of one-period closure against the Earth-only
baseline. The Sun distance (`config.VIZ_SUN_DISTANCE`) is shared by the ephemeris
and the CSS sun vector, and the orbital rate is derived from the body `mu` so the
two cannot disagree. Note this is a fixed Sun direction, so it does not rotate
once per orbit the way a real Earth-Sun line would.

## Sun-pointing orientation constraint

The constrained boresight is body **+X** (`config.SUN_POINTING_AXIS_B`),
measured by `css1`; the solar wings lie in the body XY plane (normal +Z,
viewed by `css3`). The requirement is "keep +X inside a cone about the
Sun line":

- `vizSupport.createConeInOut(..., isKeepIn=True,
  normalVector_B=list(config.SUN_POINTING_AXIS_B))` draws
  the keep-in cone, `createPointLine` draws the boresight to the Sun. Both
  reference `config.VIZ_SUN_BODY_NAME`, which is the Sun's Basilisk `displayName`
  (not its `planetName`, `sun_planet_data`).
- Three coarse sun sensors form a +X/+Y/+Z triad (`css1/2/3`); `css1` is the
  sensor that measures the sun-pointing error.
- Half-angle is `config.VIZ_SUN_CONE_ANGLE_DEG` (15°), cone length
  `VIZ_SUN_CONE_HEIGHT` (2 m). Disable with `--no-sun-constraint`.

`--maneuver sunpoint` demonstrates it: it rotates from the nominal tilted
attitude until +X is on the Sun and holding (verified
inside the 15° cone). The target comes from `sun_pointing_sigma()`, which builds
a right-handed DCM with the boresight on the Sun, pins the remaining roll to
`config.SUN_ROLL_REFERENCE_N`, and converts with Basilisk's `C2MRP` so the
convention matches `sigma_BN` exactly. `tests/test_sun_pointing.py` locks this.

`detumble` and `slew` target inertial-fixed attitudes, so they deliberately sit
**outside** the keep-in cone. The cone is a requirement, not something those maneuvers satisfy; use `sunpoint` to see it met.

## Station-keeping (heyoka + PID)
Scenario: -0.5% injection-velocity error at 12000 km altitude. The RTN PID
(radial PD + transverse PI + normal PD, `config.PID_*`, 0.1 Hz ZOH) allocates
body-frame thrust to the 12 variable jets each tick; heyoka integrates the full
12-state EOM with thrust as parameters (`ta.pars` updated in place).
```bash
./env/bin/python -m cubesat_12u_sim.run_stationkeeping
```
2 h result: free-fall drifts ~230 km; PID holds ≤5.3 km (final 1.8 km) for
~56 m/s. Tune via `PID_KP_* / PID_KD_* / PID_KI_*`, `PID_A_MAX`, `DT_CONTROL`.
Heyoka vs scipy cross-check: <1e-6 m over 1 h (identical dynamics).

## 3-year station-keeping (heyoka, orbit-only)
`run_3year.py`: same injection error, 3 yr at 600 s control steps (157788
ticks, ~2 min runtime), `controller.SemiMajorPID` — feedback on the
*non-oscillating* semi-major axis `a = 1/(2/r − v²/μ)` (transverse energy
control) + weak radial damping. Raw RTN PIDs chatter on the natural epicycle
and waste >100 m/s/yr; element feedback converges then coasts.
```bash
./env/bin/python -m cubesat_12u_sim.run_3year  # -> results_3yr/*.png + results.npz
```
Result: drift without control 363 km; PID recovers in ~1 day (46.5 m/s,
peak 0.05 m/s², within the ~0.1 m/s² jet capability) then coasts 3 yr at
±65 m residual: total 46.5 m/s, 931 N s, ~427 g @ Isp 220 s.
Plots: `radius_3yr.png` (drift vs hold + residual inset), `energy_3yr.png`
(burn profile, cumulative dv/impulse, propellant).

## Monte Carlo (Basilisk truth)
`monte_carlo.py` runs the full 3-maneuver mission — detumble (80 s, null
tip-off rates) → slew (80 s, 180° to `sigma_slew`) → sun-pointing (140 s,
track the Sun inside the 15° keep-in cone) — on 12 RCS jets only, at 50 Hz
control with the MRP feedback controller, bias-observer rate feedback, and a
pinv thruster allocator. Dispersions per run (seeded, reproducible):
mass ±5%, inertia ±5% + products, CoM ±5 mm, thruster Tmax ±5% / canting
±1° / Isp ±5%, tip-off rates ±1.5°/s, attitude MRP ±0.08, orbit injection
±5 km / ±5 m/s, gyro bias/ARW, CSS scale/bias/noise.
```bash
./env/bin/python -m cubesat_12u_sim.run_monte_carlo --runs 500 --seed 100 --jobs 4 --save-dir results_bsk500
```
Flags: `--t-detumble/--t-slew/--t-sunpoint`, `--save-dir` (JSON summary +
compressed NPZ + 3 plots), `--no-plot`, `--jobs` (process pool; results are
bit-identical to serial). Each run also records per-tick torque demand and
on-time under `signals` for model-parity diagnostics.
KPIs per run: detumble settling time + residual rate, slew settling time +
residual error (exact `4*atan(|σ|)` conversion), final/steady-state sun angle
+ cone compliance, per-phase and total propellant/impulse/ΔV, thruster duty
cycles. Mission success = detumble residual < 0.2°/s **and** steady-state sun
inside the cone (the intermediate slew target is reported, not gating).

500-run Basilisk truth (`results_bsk500/`, seeds 100–599, 279 s on 4 jobs):

| KPI | Mean | Std | p95 | p99 |
|---|---|---|---|---|
| Success / cone compliance | 26.4% / 100% | — | — | — |
| Total propellant | 3.18 g | 0.20 g | 3.50 g | 3.70 g |
| Final sun angle | 2.21° | 0.85° | 3.57° | 4.60° |
| Final slew error | 16.48° | 2.01° | 19.83° | 21.14° |
| Final detumble rate | 0.25°/s | 0.07°/s | 0.36°/s | 0.39°/s |

Reading: the sun mission is robust (100% cone, size tanks for ~3.7 g p99);
detumble never reaches 0.1°/s in 80 s and the 180° slew leaves ~16.5°
residual — both phases are short for these gains, which is also why the
strict success gate understates mission health.

## Heyoka Tier-1 MC screening + Basilisk sidecar
- `heyoka_attitude.py` (Phases 1-2): 12-state MRP Taylor system
  `[r,v,sigma,omega]` with mass/full-inertia/wrench as pars (no Euler
  singularity), plus `propagate_closedloop_mrp()` mirroring
  `monte_carlo.run_single_mission` (3-phase guidance, MRP PD, nominal
  allocation + dispersed truth, gyro/observer, same KPI dict).
- `heyoka_mc.py` (Phase 3): `taylor_adaptive_batch` SIMD screening
  (default batch 8; `--batch` overrides), `ensemble_propagate_for` open-loop
  validation helper, same stats/plots/NPZ schema as Basilisk MC:
  ```bash
  ./env/bin/python -m cubesat_12u_sim.heyoka_mc --runs 30 --save-dir results_heyoka_mc
  ./env/bin/python -m cubesat_12u_sim.heyoka_mc --runs 500 --jobs 4  # ~12 s on 8 cores
  ```
  Flags: `--batch`, `--jobs`, `--compact-mode`, `--ideal-thrusters`
  (proportional plant), `--ctrl-every` (decimation; validated neutral only
  with `--ideal-thrusters` — with the quantized effector it lengthens the
  minimum impulse bit and inflates the limit cycle ~5x), `--no-fastloop`.
  Throughput (measured, Apple Silicon, 8 cores): serial ~0.09 s/run,
  `--jobs 4` ~0.03 s/run (500 runs in ~11 s incl. plots vs Basilisk 279 s: **25x**; `--jobs 8` is slower
  — spawn and memory-bandwidth bound, stick to 4). Serial vs parallel is
  bit-identical (locked by test). Batch internals: one JIT template per
  campaign, sun-ref timeline precomputed once and shared, batched MRP/DCM
  math, per-run cached allocation pseudoinverse, C++ control tick
  (`fastloop/`, pybind11, built inplace; numpy fallback automatic, results
  agree to ~3e-14) with `--no-fastloop` to force fallback. `--compact-mode`
  is wired but neutral for this 12-state system; single precision and
  tolerance relaxation were evaluated and rejected (precision risk).
  Parity note: Heyoka matches Basilisk Monte Carlo fuel to ~0.3% with the
  effector quantizer on (`quantize_thrusters=True`, the default:
  seed-100 zero-noise fuel 3.096 g vs 3.100 g, slew 13.7° vs 13.7°;
  full 500-run campaigns: 3.1795 g vs 3.1803 g mean, per-run worst 0.019 g).
  Gyro noise mirrors measured Basilisk `ImuSensor` behavior
  (`gyro_noise_std` = ARW per sample, rate-independent — measured flat
  0.01°/s at dt 0.01–1.0 s — not textbook ARW/√dt, which overstated chatter
  ~7x at 50 Hz). `quantize_thrusters=False` gives the ideal
  proportional-thruster model (~1.7 g, clean settle) for optimistic screening.
- `heyoka_sidecar.py` (Phase 4): `HeyokaShadowPropagator` SysModel flying
  alongside truth (`--heyoka-shadow` on maneuver runs, needs ACS task).
  Translation parity is mm-level; open-loop attitude shadow tracks to
  ~12°/60s (residual phasing/lag, no feedback correction). `summary()`
  reports divergence plus `mean_applied_torque_Nm` / `final_omega_err_deg_s`
  for forensics.
- `heyoka_rw.py` + `bsk_rw_stock.py` + `run_rw_compare.py`: reaction-wheel
  generalization, validated against stock `scenarioAttitudeFeedbackRW`
  (baseline: 750 kg, I=diag(900,800,600), 3× HR16 at 100/200/300 RPM,
  K=3.5/P=30, 600 s @ 0.1 s). 15-state MRP+wheel system, negated direct
  wheel allocation (sign verified against stock first-tick outputs),
  continuous RWs (no thruster quantizer). Stock vs mirror over the full
   mission: attitude 7e-5°, rate 1.4e-6°/s, wheels 4e-3 RPM; free-flight
   120 s: 0.4 mm / 0.002°. `run_rw_compare` writes `rw_stock_vs_heyoka.png`.
   (The RW loop is numpy-only; `fastloop/` covers the thruster path.)

## Native Heyoka RW dynamics (compiled SysModel, bsk-sdk)

`../native_rw/` builds `nativerw.HeyokaRWDynamics`, a true compiled C++
`SysModel` (persistent Heyoka Taylor integrator, ±0.2 Nm saturation,
central Kepler, MRP shadow reset) that replaces `spacecraft` +
`reactionWheelStateEffector` with stock FSW otherwise untouched
(`cpp_native.run_native_rw_pipeline`, `NativeSpacecraft` scripting API).
Build: `pip install "bsk-sdk==2.12.0" scikit-build-core "cmake<4.0" ninja
"swig==4.4.1"` then `pip install --no-build-isolation ./native_rw`
(see `../native_rw/README.md`). A second module in the same wheel,
`HeyokaMCDynamics`, covers the thruster pipeline (`native_mc.py`,
detumble 600 s @ 0.01 s: stock 9.55 s → native 9.09 s, 1.05x —
FSW-dominated at 100 Hz, see Amdahl note in `../native_rw/README.md`).
Warm 600 s @ 0.1 s reference
(Apple Silicon, 15 repeats): stock 67.5 ms → py-mirror 146.7 ms (0.46x)
→ cpp-pipeline 140.1 ms (0.48x) → native 19.6 ms (**3.44x**) →
cpp-mission 5.9 ms (11.4x), all at 7.19e-05° / 1.4e-06°/s /
0.25 RPM wheels max (`run_rw_timing --repeats 15`). Scope: rigid hub at
CoM, 3 aligned balanced wheels, central Kepler only.

Validation summary (`rw_monte_carlo`, dispersed mass/inertia/tip-off/
wheels/orbit, nominal FSW gains): 500–1000-run campaigns agree at
88–90% success on every backend with identical tail members; worst-seed
pairwise divergence (3.8°) shrinks 38–134x at 1200 s (`run_rw_forensic`),
i.e. window-limited slow transients, not model error. Pool guidance:
serial default for native/mission (per-run fixed ~3 s/worker: spawn +
cold imports + first-use LLVM compile); `--jobs N --start-method fork`
past ~1000 runs with pre-pool parent warmup (determinism-verified:
fork stats == serial stats); pool never helps below the
N > W·T_fixed/(T_run·(1−1/W)) crossover. Campaign reference: 1000 runs,
jobs 4, fork — native 4.8 s vs stock 43.4 s (9.0x).
- `actuators.quantize_duty_cycle` (measured): `thrusterDynamicEffector`
  maps commanded duty `f` to achieved duty as a 3-level staircase
  `{0, 1/6, 5/6, 1}` (edges at half/full period, period-relative,
  cutoff-independent). Small commands over-actuate ~8x, driving the
  limit-cycle chatter (0.3°/s residual) and fuel waste in the Basilisk runs.
  Fuel books commanded on-time (allocator log); dynamics use quantized.

## MLX/GPU evaluation ( revisited: not adopted, 2026-10-06 )
Reference: `Assa` repo, `stupid-parallel` branch, `Project02`
(hand-written Metal Taylor-7 kernel + GPU decode/LQR/costs in
`python/scripts/Task6_mlx.py`; JAX+Diffrax Metal path in `Task6_gpu.py`).
There, batch-64 order-7 ran ~10 ms vs 0.6 ms Heyoka-single (~4x per
trajectory), and the win only materialized after moving
decode+LQR+integrate+cost *all* onto the GPU (CMA-loop JSON: kernel
0.02–0.10 s vs CPU decode 0.23 s — Amdahl).
Measured here (MLX 0.32, Apple Silicon): KPI core of 300k MRP→DCM takes
numpy-batched 0.037 s vs MLX GPU 0.030 s (max diff 7e-7) — a wash that
also drops to float32, unacceptable for the 1.8e7 m orbit states.
Verdict: not wired in. Our loop is latency-bound (15 000 sequential
ticks × Heyoka handshake; GPU widens lanes but can't span ticks) and
dynamics is only ~4% of runtime, so a Metal Taylor kernel (like theirs)
would buy nothing; the KPI math is already vectorized past the point
where launch overhead pays. Revisit only for a fully GPU-resident
closed loop (control+quantizer+Taylor in Metal, 100+ lanes) — a rewrite,
not an optimization. `mlx`/`mlx-metal` are installed in `env/` for
experiments.

State vectors: legacy demos use `y=[r(3), v(3), euler(3), omega_body(3)]`
(12-D Euler); Monte Carlo / Heyoka / Basilisk-attitude paths use
`y=[r(3), v(3), sigma_BN(3), omega_body(3)]` (12-D MRP, no singularity).
Change altitude via `config.ALTITUDE`; bus via `MASS, LX, LY, LZ`.
Campaign outputs: `results_mc/` (smoke), `results_mc500cxx/` (Heyoka 500,
current), `results_bsk500/` (Basilisk truth 500; older Heyoka 500s under
`results_mc500[j]/`) — each with `mc_summary.json`,
`mc_results.npz`, and the three publication figures.
