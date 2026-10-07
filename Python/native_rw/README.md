# `native_rw` — native Heyoka RW dynamics Basilisk extension

True compiled C++ `SysModel` (`HeyokaRWDynamics`) replacing
`spacecraft` + `reactionWheelStateEffector` for the
`scenarioAttitudeFeedbackRW` baseline: 15-state hub+RW dynamics
`[r, v, sigma, omega, W]` propagated by an in-module Heyoka Taylor
integrator. Zero Python per tick. Built with [`bsk-sdk`](https://github.com/AVSLab/bsk_sdk)
(no full Basilisk source build needed).

Layout (one dir per module, upstream convention):

- `heyokaRWDynamics/` — `heyokaRWDynamics.{h,cpp,i}` (RW demo: 15-state
  hub+RW, motor-torque in, `SCStatesMsg`+`RWSpeedMsg` out)
- `heyokaMCDynamics/` — `heyokaMCDynamics.{h,cpp,i}` (thruster pipeline:
  12-state hub+wrench pars, on-time in, `SCStatesMsg` out; previous-tick
  wrench ZOH, effector staircase, optional gravity gradient)
- `shim/cSysModel.py` — top-level alias so SWIG's bare `import cSysModel`
  resolves to `Basilisk.architecture.cSysModel` (see below)
- `nativerw/__init__.py` — package init (compat guard, re-export)
- `tests/` — in-package tests (no sims; run against the installed wheel)
- `docs/heyokaRWDynamics.rst` — module doc page
- `.clang-format` — Basilisk style (Mozilla, 120 col)
- `CMakeLists.txt`, `pyproject.toml`, `LICENSE`

Envelope (by design — demo-scoped modules, not a library):

- RW: rigid hub at CoM, 3 body-axis-aligned balanced wheels (no
  jitter/friction/voltage IO), central-body point-mass Kepler only,
- MC: 12 fixed thrusters, row-major (3,12) effectiveness matrices,
  measured 3-level staircase, optional gravity gradient; no thrust-force
  input beyond on-time, no effector attachment points (not `DynamicObject`s).
- To generalize (canted wheels, GG/aero torques, N-body), extend
  `buildRwSystem` / `buildMcSystem` + add setters.

## Build (from `Python/`, macOS)

```bash
./env/bin/python -m pip install "bsk-sdk==2.12.0" scikit-build-core "cmake<4.0" ninja "swig==4.4.1"
./env/bin/python -m pip install --no-build-isolation ./native_rw
./env/bin/python -c "import nativerw; print(nativerw.HeyokaRWDynamics())"
```

Notes:

- `--no-build-isolation`: Heyoka headers live in the conda env / Homebrew,
  not on PyPI. Discovery order: `$HEYOKA_ROOT`, `/opt/homebrew`, env prefix.
- Link hygiene: the extension links `libheyoka` directly with an install
  rpath (no `DYLD_*`, no import-order dependence). If no library is found
  at configure time it falls back to `dynamic_lookup` mode (`import heyoka`
  first — kept in `__init__.py` either way). `test_heyoka_linked`
  asserts the link via `otool`.
- SWIG emits a bare `import cSysModel` for `%import "sys_model.i"`;
  the wheel ships a top-level `cSysModel.py` shim re-exporting
  `Basilisk.architecture.cSysModel`, so the wrapper inherits the
  scheduler-visible `SysModel` (no `sys.modules` patching).
- Pinned `bsk-sdk==2.12.0` / `bsk==2.12.0`: the SDK errors on version
  mismatch by design (ABI guard, also repeated at import).
  `tests/test_pins_lockstep` fails the suite if pins drift from the
  installed versions — bump all three in lockstep on Basilisk upgrades.

## Use

```python
from cubesat_12u_sim.cpp_native import NativeSpacecraft, run_native_rw_pipeline
res = run_native_rw_pipeline(duration=600.0, dt=0.1)  # same schema as stock
from cubesat_12u_sim.native_mc import NativeMCSpacecraft, run_native_mc_pipeline
res = run_native_mc_pipeline(duration=600.0, maneuver="detumble")  # == heyoka schema
```

`NativeSpacecraft` mimics the `spacecraft.Spacecraft` scripting API
(`hub.*`, `ModelTag`, `grav.addBodiesTo(sc)`, `AddModelToTask(task, sc, 1)`).
Scope: rigid hub at CoM, 3 aligned balanced wheels, central Kepler —
see `cpp_native.NativeSpacecraft` docstring.

## Verify

```bash
./env/bin/python -m pytest native_rw/tests -q
./env/bin/python -m pytest cubesat_12u_sim/tests/test_native_rw.py -q
./env/bin/python -m cubesat_12u_sim.run_rw_timing --repeats 5
```

Reference (Apple Silicon, warm): stock 67 ms → native 20 ms (3.4x),
parity 7.19e-05° / 1.4e-06°/s / 0.25 RPM wheels max.
Campaigns (`rw_monte_carlo`): serial default; pool past ~1000 runs with
`--jobs N --start-method fork` (pre-pool parent warmup; determinism
verified fork == serial). 1000 runs, jobs 4, fork: native 4.8 s vs
stock 43.4 s (9.0x), identical 89.5% success.
Thruster pipeline, detumble 600 s @ 0.01 s (`run_heyoka_pipeline_compare`):
stock 9.55 s → heyoka-python 14.16 s (0.67x) → native 9.09 s (**1.05x**),
parity identical to the validated Python path (9.61e-03 MRP /
5.19e-03 rad/s). Amdahl: at 100 Hz the FSW dominates both columns, so
the swapped dynamics slice buys little; the RW case won 3.4x on a fatter
slice. Campaign-scale MC stays with fused `cxxdrive` (25x, no scheduler).
