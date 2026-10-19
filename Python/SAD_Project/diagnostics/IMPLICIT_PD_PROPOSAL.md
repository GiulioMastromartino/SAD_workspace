# Proposal: implicit/semi-implicit PD update to remove the track limit cycle

Status: **implemented and validated (2026-10-19)**

> Implementation note: the shipped controller applies the scaling to the
> **track phase only** (`QuatPDParams::implicit_track_only = true`).  Detumble
> and slew run at `dt = 0.1` and their legacy gains are already stable; scaling
> them made the slew ~1.5 s slower and raised the (pre-existing) slew-timeout
> rate from 24% to 41%.  Set `implicit_track_only = false` to scale every phase
> (used by `run_detumble_slew.py`, which integrates all phases at one `dt`).

## 1. Problem

The track phase runs at `dt = 1.0 s`.  The controller computes the torque once
per step and the dynamics integrates it as a zero-order hold:

```
tau_n = -K e_n - P w_n - Ki * integral_n
w_{n+1} = w_n + dt * I^{-1} tau_n + ...
```

For the damping term this is an explicit-Euler map

```
w_{n+1} = (1 - P dt / I) w_n
```

which is stable only for `P dt / I < 2`.  With `diag(I) = [0.0867, 0.0667,
0.0333]` the track-axis limits at `dt = 1.0` are

| axis | I (kg m^2) | stable P |
|---|---:|---:|
| x | 0.0867 | 0.173 |
| y | 0.0667 | 0.133 |
| z | 0.0333 | **0.067** |

The defaults use `track_P = 2.92` and the 250-generation optimizer chose
`track_P = 6.70`; both are far outside the stable region.  The result is a
saturated limit cycle (final rate ~1.56 rad/s) and a cost floor.  The spring
term has the same problem (`K < 8 I / dt^2`, about 0.27 on z at `dt = 1`).

Running `dt = 0.1` fixes the per-step stability region (`P < 0.667` on z) and
cuts the final rate to ~0.47 rad/s, but costs exactly 10x more steps.  It also
does **not** make `track_P = 6.7` stable; it only reduces the limit-cycle
amplitude.  The 25% slew failures are unrelated (actuator authority).

## 2. Proposed method: effective gains from a backward-Euler update

Keep `dt = 1.0`, keep the existing controller -> allocator -> thruster ->
dynamics chain, and replace the continuous gains `K, P` by **step-aware
effective gains** `K_eff, P_eff` inside `quatpd_step`.

For the linearized per-axis loop (quaternion vector part `e ~ theta/2`):

```
e_{n+1} = e_n + 0.5 dt w_{n+1}
w_{n+1} = w_n - dt (K/I) e_{n+1} - dt (P/I) w_{n+1}
```

(the second line is backward Euler on the damping and spring).  Solving for
`w_{n+1}` gives

```
D      = 1 + dt P/I + 0.5 dt^2 K/I
w_{n+1} = (w_n - dt K/I e_n) / D
```

Now ask the *explicit* controller to reproduce this map by using scaled gains:

```
tau_n = -K_eff e_n - P_eff w_n
w_{n+1} = w_n - dt (K_eff/I) e_n - dt (P_eff/I) w_n
```

Matching coefficients gives the closed form

```
K_eff = K / D
P_eff = (P + 0.5 dt K) / D
D     = 1 + dt P/I + 0.5 dt^2 K/I
```

This is exact for the linearized per-axis loop and needs **no change to the
dynamics, allocator or thruster modules**.  For `dt -> 0`, `K_eff -> K` and
`P_eff -> P`.  For large `dt`, `P_eff -> I/dt` and `K_eff -> 2 I/dt^2`: the
controller asks for exactly the torque that nulls the rate/attitude in one
step, which is the correct discrete-time behaviour and is naturally limited by
the actuator saturation.

### Stability

The implicit map has the amplification matrix

```
M = [[1 - a, b],
     [-c,     d]]

a = 0.5 dt^2 K/(I D)
b = 0.5 dt / D
c = dt K/(I D)
d = 1 / D
```

with

```
det(M) = 1/D < 1
trace(M) = 1 + 1/D - a
```

`D > 1` for all positive `K, P, I, dt`, so both eigenvalues lie strictly inside
the unit circle (Jury criterion).  **The scaled explicit controller is
unconditionally stable for the linearized attitude/rate loop**, for any
`K, P > 0` and any `dt > 0`.  This removes the numerical limit cycle without
reducing the step size.

### Integral term

The integral mode is slow.  Its explicit stability limit is `Ki dt / I < 2`;
with `I_z = 0.0333`, `dt = 1`, that is `Ki < 0.067`, while the current values
are ~1e-3.  It is already safe.  For consistency we can scale it as
`Ki_eff = Ki / D`; this keeps the closed-loop pole locations invariant and is
recommended, but it is not required to fix the limit cycle.

### Three-axis form

The 6U inertia is diagonal in the body frame, so apply the scalar formula per
axis using the **nominal** inertia (the controller must not know the dispersed
plant inertia):

```
for i in x,y,z:
    D_i      = 1 + dt P/I_i + 0.5 dt^2 K/I_i
    K_eff_i  = K / D_i
    P_eff_i  = (P + 0.5 dt K) / D_i
    tau_i    = -K_eff_i e_i - P_eff_i w_i - (Ki/D_i) integral_i
```

If a full non-diagonal inertia is ever used, replace the scalar division by the
3x3 solve `(I + dt P I^{-1} + 0.5 dt^2 K I^{-2}) w_{n+1} = ...`; the current
6U model does not need it.

## 3. Why this is better than the alternatives

| option | stability | cost | changes | verdict |
|---|---|---|---:|---|
| track `dt = 0.1` | stable only for `P < 0.667` on z; current gains still unstable | **10x** | config | too slow, does not fix gains |
| gain bounds only | stable | 0 | optimizer bounds | safe but over-constrains |
| controller-side effective gains | **unconditional** (linearized) | ~0 | controller + params | **recommended** |
| implicit 3x3 solve in dynamics | unconditional | 3x3 solve/step | dynamics + controller | more invasive, equivalent |
| exact ZOH discrete gains | unconditional | ~0 | controller | equivalent to the scaling for this loop |

The scaling is the cheapest change that is mathematically equivalent to an
implicit update, preserves the modular interfaces, and keeps the optimizer at
~1 s/generation (the 4x250 run stays ~19 min instead of ~3 h).

## 4. Implementation plan

### 4.1 C++ controller (`cpp/control/sad_controller_quat_pd.{h,cpp}`)

Add to `QuatPDParams`:

```cpp
Vec3 inertia = Vec3(0.0867, 0.0667, 0.0333); // nominal 6U diag(I)
bool implicit_dt = true;                      // false = legacy A/B path
```

Add a helper (header, inline):

```cpp
inline void quatpd_effective_gains(double K, double P, double I, double dt,
                                   double& K_eff, double& P_eff) {
    const double D = 1.0 + dt*P/I + 0.5*dt*dt*K/I;
    K_eff = K / D;
    P_eff = (P + 0.5*dt*K) / D;
}
```

In `quatpd_step`, after selecting `K, P, Ki` by mode and computing `e`:

```cpp
y.torque_b.setZero();
for (int i = 0; i < 3; ++i) {
    double I = p.inertia[i] > 1e-12 ? p.inertia[i] : 1.0;
    double Ke = K, Pe = P, Kie = Ki;
    if (p.implicit_dt && dt > 0.0) {
        double D = 1.0 + dt*P/I + 0.5*dt*dt*K/I;
        Ke = K / D;
        Pe = (P + 0.5*dt*K) / D;
        Kie = Ki / D;
    }
    y.torque_b[i] = -Ke*e[i] - Pe*u.omega[i] - Kie*s.integral[i];
}
```

The integral accumulation/clamp stays as it is.  `implicit_dt = false` must
reproduce the current torque bit-for-bit (regression test).

### 4.2 Batch kernel (`cpp/mission_batch/sad_mission_batch.cpp`)

Before `quatpd_init`:

```cpp
qp.inertia = Vec3(IX, IY, IZ);   // nominal, not the dispersed inertia
qp.implicit_dt = true;
```

The nominal `IX, IY, IZ` already exist in the kernel (lines ~58-60).

### 4.3 Python reference (`control/controller.py`)

Import `INERTIA` and apply the same per-axis scaling in `quat_pd` so the
reference and the C++ module stay in parity.  Add a parity test comparing the
C++ `quatpd_step` torque with `quat_pd` for the same `q, omega, dt, gains`.

### 4.4 Single-run missions

`missions/run_monte_carlo.py` and `missions/run_detumble_slew.py` build
`QuatPDParams` directly; set `qp.inertia = np.diag(C.INERTIA)` there too.
Add `IMPLICIT_PD = True` to `core/config.py` if a single switch is preferred.

### 4.5 Diagnostics / toggles

* Keep the legacy path behind `implicit_dt = false` for A/B.
* Expose `implicit_dt` and `inertia` in the diagnostics so
  `profile_batch`/`bench_one_gen` can run both paths.
* Record the effective gains in the profile JSON if useful.

## 5. Validation plan

1. **Unit / stability test**
   * Linearized map with `K=0.9, P=2.9, I=0.0333, dt=1.0`:
     `D = 101.6`, `K_eff = 0.00886`, `P_eff = 0.0330`; spectral radius `1/D`
     (plus the small `e` coupling), < 1.
   * Sweep `P` in [0.05, 50] and `K` in [0.01, 20] at `dt=1`; assert
     `max|eig(M)| < 1` for all.
   * Show the legacy map has `|1 - P dt/I| > 1` for `P > 0.067`.

2. **Parity**
   * C++ vs Python reference torque, `implicit_dt` on and off, `atol=1e-12`.
   * Existing 4/4 parity tests must still pass.

3. **Kernel A/B** (`profile_batch`, N=50, timing on)
   * DEFAULTS and the 250-gen best, `dt=1.0`, `implicit_dt` on vs off.
   * Check: final rate (expect 1.56 -> <0.1 rad/s), success-only cost,
     per-step cost (expect <1% change), per-phase attribution.
   * Verify the 25% slew failures are unchanged (they are a geometry issue).

4. **Optimizer**
   * Re-run 4 restarts x 250 generations at `dt=1.0` with the implicit path
     (~19 min, not ~3 h).
   * Validate the global best on N=300 against defaults and the previous
     dt=1.0 result.
   * Expect the mean cost to drop from ~15.1 toward ~13-14 (the 25% x 50
     failure floor remains); the success-only median/cost should drop from
     ~3.9 to ~1-2.

5. **Regression**
   * `implicit_dt=false` reproduces the current torques and cost.
   * 4/4 parity tests.

## 6. Expected outcome

* Track end-rate: ~1.56 rad/s -> below the physical/actuator floor (target
  <0.1 rad/s; the gravity-gradient/actuator authority will set the final
  value).
* Success-only cost: ~3.9 -> ~1-2 (rate term ~0.05, orbit term ~1).
* Mean cost: ~15.1 -> ~13-14 until the thruster geometry is fixed.  The
  remaining cost is the 25% slew-failure floor, not the track loop.
* Runtime: unchanged at ~1 s/generation (the added arithmetic is a few
  multiplies/divides per axis, ~10-20 ns/step vs 1.5 us/step).
* The optimizer's gains become step-size invariant: high `P` no longer maps to
  a numerical explosion.  The high-gain cliff in the cost surface disappears,
  which should make CMA-ES much better behaved.

## 7. Risks / mitigations

* **Effective gains are much smaller at `dt=1`.**  For the defaults
  (`K=0.9, P=2.9, I_z=0.0333`) the scale factor is ~100 on z, so the controller
  is deliberately gentle (settling ~10-30 s).  Over two orbits this is fine,
  but the slew phase (dt=0.1, scale ~3) and detumble may need a gain retune.
  The optimizer will retune them; the first A/B should check the slew time.
* **Actuator saturation.**  The implicit law assumes the requested torque is
  applied.  The scaled torque is smaller, so saturation is less likely; the
  remaining physical limit is benign.  Validate the clamp/scale path.
* **Integral scaling.**  `Ki/D` slows the trim at large `dt`.  If that hurts
  the orbit-box hold, leave `Ki` unscaled (it is already well inside its
  stability limit) or use a slower, separate scale.
* **Dispersed inertia.**  The controller uses the nominal inertia, not the
  +/-5% dispersed plant.  This is the realistic case; the margin is small.
* **Geometry.**  This fixes the track limit cycle only.  The 25% slew failures
  (zero authority on +Tx, Fy, Ty) remain and need a separate layout fix.

## 8. Effort

| item | estimate |
|---|---:|
| C++ controller + params + batch kernel | 1 h |
| Python reference + parity test | 30 min |
| Stability/unit test + kernel A/B | 1 h |
| Rebuild + 4x250 optimizer + N=300 validation | 1 h (20 min run + analysis) |
| **total** | **~3.5 h** |

## 9. Recommendation

Implement the controller-side effective-gain scaling (Section 4.1-4.3) behind
`implicit_dt`, run the kernel A/B at `dt=1.0`, and if the track rate collapses
as predicted, do the 4x250 optimizer at `dt=1.0`.  Keep `implicit_dt=false`
as the regression/A-B path.  Do **not** combine this with a change to the
thruster geometry in the same step, so the effect can be attributed.

## 10. Implementation result

### A/B, N=300, seed 123, `dt=1.0`, track-only scaling

| params | PD | mean | median ok | fail | final rate (med/mean) |
|---|---|---:|---:|---:|---:|
| defaults | legacy | 15.25 | 2.92 | 74 | 1.542 / 1.572 |
| defaults | implicit | **14.13** | **1.64** | 73 | **0.277 / 0.289** |
| 250-gen legacy best | legacy | 15.47 | 3.00 | 75 | 1.545 / 1.591 |
| 250-gen legacy best | implicit | **13.81** | **1.41** | 72 | **0.046 / 0.048** |

The failure count is unchanged (the 25% slew-timeout geometry issue), the
track end-rate collapses by 30-35x, and the success-only cost drops by ~50%.

`dt=1.0` implicit and `dt=0.1` implicit give the same result (opt250:
rate 0.048 vs 0.046, mean 13.81 vs 13.82), confirming that the scaling makes
the gains step-size invariant.  The 10x step count of `dt=0.1` is no longer
needed.

### 4x250 optimizer at `dt=1.0`

- 1,000 generations, N=50, pop 15, 8 workers: **1,420 s (~24 min)**, versus the
  projected 3-4 h at `dt=0.1`.
- Best training cost **6.367**; validation (N=300) mean **14.11 / 12.33** at
  seeds 123 / 321, success-only median **1.39 / 1.77**, final rate
  **0.014 / 0.016** rad/s.
- Chosen gains: `track_K = 0.0207`, `track_P = 27.18`, `TS = 64.91 s`.
  The implicit scaling maps these to `K_eff ~ 2.5e-5`, `P_eff ~ I/dt` on z:
  the optimizer converged to a near-pure rate damper.  This is cost-optimal
  (the cost has no pointing term) but it is **not** an attitude-pointing
  solution.

### Remaining issues (not fixed by this change)

1. **Slew timeouts (25%).**  The 8-thruster layout has no `+Tx`, `Fy`, `Ty`
   authority; the failed runs sit in slew for the whole TS budget.  The
   optimizer can convert some failures into slow successes by raising TS, but
   the nominal geometry is the root cause.
2. **Track pointing is not being achieved.**  The single-run MC reports
   `mean_track_err ~ 2.9 rad` (167 deg) and cone compliance 0-23%.  The
   sun-pointing target pins the roll with a fixed `roll_ref`, which is ~180 deg
   from the post-slew attitude; the controller tries to flip that roll, but the
   layout cannot produce the required `+Tx/Ty`, so the pointing never settles.
   The cost function currently has no pointing/cone term, so the optimizer has
   no incentive to fix it (and picks a near-zero spring).
3. **Orbit box.**  `box_max` is ~32 km versus the 5 km target; the cost uses the
   mean radial error, not the max, so the box constraint is not enforced.

Recommended next step: fix the thruster geometry (canted 8- or 12-thruster)
and add the cone/pointing error (and optionally the box max) to the cost
before trusting the track gains.

## 11. Follow-up: standard 12-thruster geometry and frame fix (2026-10-19)

The 12-thruster standard layout (4 on ±X, 4 on ±Z, 4 on ±Y, face edges) is now
in `core/config.py` and the batch kernel.  `A` has rank 6 with ± authority on
every axis, so the zero `+Tx`, `Fy`, `Ty` columns are gone.  Two additional
bugs surfaced while validating it:

* `quat_to_dcm` returned the transpose of the DCM implied by the quaternion
  kinematics (`q_dot = 0.5 q ⊗ [omega, 0]`).  Fixed in C++ and Python.
* The sun target used a fixed inertial roll reference, which put the target
  ~180° from the post-slew attitude.  The target roll is now pinned to the
  current body +Y, i.e. the nearest-roll sun attitude.

After both fixes, the pre-optimizer gains pointed correctly:

* N=30 single-run MC: 90% reached track, mean track error 0.0348 rad (2.0°),
  100% cone compliance, final rate 5.1e-5 rad/s.
* N=300 batch with the 0.1 Nm command limit: mean cost 8.12 / 8.99,
  success-only median 1.49 / 1.62, failures 34/300 / 40/300 at seeds
  123 / 321.
* All remaining failures were slew timeouts.  The 12-thruster layout plus a
  0.1 Nm command limit reduced them from ~25% to 11-13%; at 0.2 Nm the same
  runs failed 57/300 and 63/300.

## 12. 4x250 optimizer with the 12-thruster layout (2026-10-19)

The optimizer was then re-run with the 12-thruster geometry, `tau_max=0.1 Nm`,
implicit track-only scaling, and the existing cost (no cone/box-max term yet).
It used 250 generations, pop 15, N=50, 4 multistart restarts, 8 workers:
**1,000 generations / 750,000 runs / 1,468.6 s (~24.5 min)**, best training
cost **1.7636** (previous 6.37).

Chosen parameters:

```
detumble: K=5.802556  P=1.203649  Ki=0.000719319
slew:     K=5.345669  P=1.616861  Ki=0.001999508
track:    K=0.025519  P=42.347744 Ki=0.000100664
TS=400.5679
```

Validation with the applied config:

* N=300 batch, seeds 42 / 123 / 321: mean cost **2.689 / 2.778 / 2.647**,
  success-only median **1.481 / 1.535 / 1.654**, failures **0/300**,
  reached track **300/300**, final rate ~0, fuel **0.00063 kg**.
* N=50 single-run validation: **0/50 failures, 50/50 reached track**, mean track
  error **0.00454 rad (0.26°)**, **100% cone compliance**, final rate
  **9.27e-7 rad/s**, slew settle **4.20 s** (max 4.70 s), fuel **0.000633 kg**.
* The 5 km orbit box is still not met: mean radial error 6.81 km, per-run max
  12.65 km mean / 28.73 km worst.  The orbit is ballistic and the cost uses the
  mean radial error, not the max.

`TS` is an inactive timeout budget: actual slew is 3.9-4.8 s, and N=600
validation is identical for `TS` from 12 s to 400 s.  The cost map from that
run had 750k points.  A future optimization should add
cone/pointing and box-max terms now that the geometry and frame issues are
fixed.



## 13. Box/cone cost and re-optimization (2026-10-19)

The optimizer cost now has a piecewise box term on the 5 km orbit box and a
piecewise cone term on the 15 deg sun cone:

```
pen(x, limit, w, k) = w * (x/limit)^2              if x <= limit
                    = w * exp(k * (x/limit - 1))  if x > limit
```

The box input is `box_max` (km), the cone input is `track_ang_max` (deg),
`w = 1`, `k = 1`.  `reduce_costs` adds these two run-level penalties to the
final rate (and the 10-point not-reached penalty); failures stay at 50.

The optimizer was relaunched after the cost moved to the per-run maximum form
and the injection dispersion was tightened to ±1 km / ±0.5 m/s.  It used one
CMA-ES run (restarts removed): 250 generations, pop 15, N=50, 8 workers,
seed 42, `sigma0=0.6`.  Runtime **383.9 s**, 187,500 mission runs, best training
cost **0.1411**.  Applied gains:

```
detumble: K=5.339150  P=0.882814  Ki=0.027321709
slew:     K=1.286718  P=0.445776  Ki=0.003147821
track:    K=4.409346  P=49.859388 Ki=0.000102648
TS=278.2200
```

Validation:

* N=300, seeds 42 / 123 / 321: mean cost **0.207 / 0.215 / 0.205**,
  success-only median **0.151 / 0.142 / 0.142**, **0/300 failures**,
  300/300 reached track, fuel **0.00054 / 0.00055 / 0.00055 kg**, `box_max`
  **4.67 / 4.97 / 4.66 km** (5 km box met).
* N=30 MC: 100% reached track, mean track error **0.000174 rad (0.0100°)**,
  max track error 0.0162 rad mean (0.93°), **100% cone compliance**, final rate
  **4.82e-6 rad/s**, slew settle **4.80 s**, fuel **0.000548 kg**, mean radial
  error **1.056 km**, `box_max` **1.858 km mean / 3.657 km worst**.
* N=50 single-run validation: 0/50 failures, 50/50 reached track, mean track
  error 0.000151 rad, max track error 0.0166 rad, final rate 4.19e-6 rad/s,
  slew 4.80 s, fuel 0.000543 kg, orbit error 1.12 km, `box_max` 1.96 km.
* Cost breakdown per run: box **~0.203-0.211**, cone **~0.0039**, final rate
  **~3.5e-6**, total **0.205-0.215**; both penalties are inside their quadratic
  regions, so no exponential tail is active.

Relative to the previous gains, the new run lowers cost by ~5%, reduces worst
cone error from 2.13° to 1.29°, and shortens slew settle from 5.5 s to 4.8 s.
Fuel rises from 0.00048 to 0.00055 kg and the final rate from ~5e-7 to
~3.5e-6 rad/s because fuel and time are not in the scalar cost.  Average track
error increases slightly (0.000134 -> 0.000174 rad), while the per-run maximum
improves (0.0331 -> 0.0162 rad).
