# Controller Parameter Optimisation — PID and NMPC

Tunes the two station-keeping arms of `cubesat_12u_sim` for **minimum Δv subject
to staying inside a station-keeping box**. Modelled on the cascade-controller
tuning in `Assa@cmaes` (`Project02/scripts/Task6_cma_v2.py`): log-space
parameters, multi-restart CMA-ES, a per-worker evaluation cache, a structural
sweep over the parameter that changes the problem size, and re-validation of the
winner on the real simulator.

---

## 1. Problem

| | |
|---|---|
| **Decision variables** | `θ = log` of the arm's gains/weights (see §3) |
| **Horizon** | 0.5 d closed loop from a −0.5 % velocity injection |
| **Initial condition** | `v0 = 0.995 · v_circ` ⇒ `e_a = −181.5 km` |
| **Cost** | `J = Δv + W_viol·excess² + W_final·(e_a(T)/50 km)²` |
| **Feasibility** | `max_t |‖r‖ − R_orbit| ≤ spec_box` (default 25 km) |
| **Ranking** | feasible-first, then lowest `J` |

The box term is a **specification constraint, not a preference**: a candidate
that leaves the box is never selected, only ranked by how badly
(`rank_key` puts feasibility ahead of cost).

Both arms get a coast threshold as an optimised parameter (`box_r` for the PID,
`db_a` for NMPC) *and* are judged against the same external `spec_box`, so
neither arm can satisfy the spec by simply suppressing thrust.

### Why energy, not radius

Both arms regulate the semi-major axis `a` (via `e_a = a − R_orbit`) rather than
radius. On an unperturbed arc `a` is constant while `r` swings by hundreds of km
(measured: ±372 km over 10 d from this IC). A radius-referenced cost therefore
burns forever chasing a natural epicycle; the epicycle is physical, not an error.

---

## 2. Three stages

```
stage 1  pid     OrbitPID gains (kp/ki/kd × R/T/N) + box_r          [9 params]
stage 2  nmpc    q_pos, q_vel, r_fuel, db_a, trim_a_max,
                 trim_dv_budget        × horizon ∈ {2,4,6,8,12}      [6 params]
stage 3  joint   both arms in one vector, seeded from stages 1 and 2  [15 params]
```

Stage 3 exists so the reported head-to-head is **tuned-vs-tuned**. Comparing a
tuned NMPC against an untuned PID would flatter the NMPC by however much the
PID happened to be mistuned.

The horizon sweep is treated as structural rather than as another parameter
because it changes the decision-vector size (3·N), not just a gain — each
horizon is an independent problem and the winners are ranked against each
other afterwards.

---

## 3. Parameter spaces

All parameters are optimised as `log θ`, which turns multiplicative bounds into
additive ones and gives CMA-ES an approximately isotropic search space.

### PID (stage 1)

| | bounds | default | unit |
|---|---|---|---|
| `kp_r`, `kp_n` | 1e−7 … 1e−4 | 2e−6 | 1/s² |
| `ki_r`, `ki_n` | 1e−12 … 1e−6 | 1e−9 | 1/s³ |
| `kd_r`, `kd_n` | 1e−4 … 1e−1 | 2e−3 | 1/s |
| `kp_t` | 1e−5 … 1e−1 | 1e−3 | 1/s |
| `ki_t` | 1e−9 … 1e−3 | 1e−6 | 1/s² |
| `box_r` | 1e−2 … 1e5 | 25e3 | m |

Bounds are ≈2 decades either side of the hand-tuned value: further out is either
unstable (high) or indistinguishable from zero (low).

### NMPC (stage 2)

| | bounds | default | unit |
|---|---|---|---|
| `q_pos` | 1e−2 … 1e3 | 1.0 | – |
| `q_vel` | 1e−3 … 1e2 | 0.1 | – |
| `r_fuel` | 1e−3 … 1e3 | 0.5 | – |
| `db_a` | 50 … 5e4 | 500 | m |
| `trim_a_max` | 1e−4 … 5e−2 | 5e−3 | m/s² |
| `trim_dv_budget` | 1e−3 … 50 | 0.5 | m/s |

---

## 4. Two fidelity points

These are the parts most likely to make a sweep produce confident nonsense.

### 4.1 Each arm is searched at the resolution it is flown at

The PID is a 10 s zero-order-hold loop (`C.DT_CONTROL`); the long-horizon NMPC
replans at 600 s with 60 s trim ticks. Searching both at one coarse step
misrepresents the PID — its integrator (`int += e·dt`) and deadband are
period-dependent. At a matched 0.5 d horizon, the config-default PID costs:

| control period | Δv | max box excursion |
|---|---|---|
| 1 s | 151.2 m/s | 40.63 km |
| **10 s (deployed)** | **154.0 m/s** | **40.79 km** |
| 60 s | 167.8 m/s | 41.05 km |
| 300 s | 289.8 m/s | 42.29 km |

A 300 s step nearly doubles the apparent fuel cost, so a sweep run at the NMPC's
resolution would tune against a PID that does not exist.

### 4.2 The search propagator applies `alloc_eff = 0.5`

The 12-jet allocator solves least squares over **one-way** jets (`u ≥ 0`). For a
pure force command the unconstrained solution has negative components; clipping
them to zero leaves exactly **half** the requested thrust:

```
unconstrained u for +x at 0.05 m/s²:  min=-0.250  max=0.250  Σmax(u,0)=0.500
measured efficiency: 0.500 (identical for every direction and magnitude tested)
```

`simulate()` therefore scales the commanded acceleration by `cfg.alloc_eff`
before applying *and* before counting Δv. Without this the optimiser sizes every
gain for a vehicle with twice the real authority, and the tuned result fails the
full-simulation check. `measure_alloc_eff()` re-derives the ratio from the
allocator and it is written into the results JSON, so the surrogate's fidelity
is reported rather than assumed.

With the correction, surrogate vs. full heyoka + thruster path at identical
`dt` and horizon:

| | surrogate Δv | full-sim Δv | surrogate box | full-sim box |
|---|---|---|---|---|
| NMPC (H=8) | 27.290 | 27.288 | 5.24 km | 5.25 km |
| PID (defaults) | 69.586 | 69.555 | 40.63 km | 40.63 km |

---

## 5. Numerical setup

- **Search propagator**: `_rk4_step_orbit` (pure numpy, 6-state `[r,v]`). A 2-day
  arc costs ~36 ms — negligible next to the NMPC solve. No adaptive-solver state
  leaks between candidates, so results are reproducible.
- **NMPC solve budget**: `maxiter=50`. Below ~30 the SLSQP budget itself changes
  the answer (Δv 47.5 → 27.1 m/s from maxiter 20 → 50), so the budget is pinned
  for the whole sweep rather than left to float.
- **Cache**: keyed by MD5 of `(arm, cfg fingerprint, horizon, rounded θ)`. CMA-ES
  re-proposes near-identical points, especially late in a run.
- **Restarts**: 4 by default; restart 0 seeds from the config defaults (stage 1/2)
  or the stage 1+2 result (stage 3), the rest from uniform samples.
- **Parallelism**: across *restarts*, not generations — CMA-ES generations are
  sequential and a synchronisation point each generation costs more than it
  saves at these population sizes.
- **Gain isolation**: `TunedOrbitPID` keeps gains instance-local. `OrbitPID`
  reads `C.PID_K*` module-globally, which is not safe to vary across concurrent
  workers; the control law is otherwise byte-identical (asserted by
  `test_tuned_pid_matches_stock_orbit_pid_control_law`).

---

## 6. Validation

`optimization/validate.py` re-runs each winner through the real path — heyoka
12-state dynamics, `throttle_command` into the 12-jet allocation — and reports
Δv, propellant by the rocket equation, max box excursion and final `e_a`. The
sweep optimises against a cheap RK4 rollout; anything reported has to come back
through the path that would actually ship.

---

## 7. Running

```bash
cd Python/
./env/bin/python -m cubesat_12u_sim.run_optimize --stage all
```

Options:

| flag | default | meaning |
|---|---|---|
| `--stage` | `all` | `pid` / `nmpc` / `joint` / `all` |
| `--restarts` | 3 | CMA-ES restarts per problem |
| `--maxiter` | 25 | generations per restart |
| `--popsize` | auto | `4 + 3·ln(n)` |
| `--workers` | `cpu−2` | process pool size |
| `--days` | 0.5 | closed-loop horizon for the search |
| `--spec-box` | 25e3 | box half-width [m] (feasibility) |
| `--alloc-eff` | 0.5 | commanded/achieved ratio of the allocator |
| `--pid-backend` | `cpp` | `cpp` = fused C++ batch (§10), `numpy` = reference |
| `--nmpc-backend` | `cpp` | `cpp` = Heyoka C++ batch cost+grad (§10), `cxx` = full C++ sim with embedded NLopt-SLSQP + early abort (§10), `numpy` = RK4 reference |
| `--validate-t` | 7200 | validation horizon [s] on the full sim |
| `--resume` | off | resume from per-restart checkpoint files |
| `--no-checkpoint` | off | disable per-restart checkpoint files |
| `--seed` | 20261007 | RNG seed |
| `--no-timing` | off | disable the hierarchical timing profiler (§12) |
| `--bench-reps` | 0 | repeat one fast sim N times per arm; report mean/std/min/max per section (§12) |
| `--bench-theta` | `seed` | theta used by `--bench-reps`: `seed` or `tuned` |

Output: `Python/results_optimization/res_optimization.json` — per-stage
parameters, every restart (with per-generation convergence history), and the
validation table. Per-stage JSON (`res_stage_pid.json`, `res_stage_nmpc.json`)
lands after each stage; per-restart checkpoints (`ckpt_*.jsonl`) land after
each restart — `--resume` picks up where a kill left off.

Figures (no re-running needed:
`./env/bin/python -m cubesat_12u_sim.optimization.plots res_optimization.json`):

| file | content |
|---|---|
| `fig_opt_convergence.png` | optimizer: best-cost-so-far vs evals, every restart |
| `fig_opt_horizons.png` | controller: NMPC horizon sweep, best dv/box per H |
| `fig_opt_validation.png` | controller: validation arms (dv, propellant, box, e_a) |
| `fig_opt_trajectories.png` | controller: winner re-simulations (r err, e_a, dv) |

---

## 10. C++ acceleration (Heyoka path + fused PID)

Two batch kernels in `cxxdrive/`, built with
`./env/bin/python cubesat_12u_sim/cxxdrive/setup.py build_ext --inplace`:

| kernel | what | measured |
|---|---|---|
| `_pidbatch.pid_batch_eval` | whole CMA-ES population (RTN law + RK4 + score) in one call | 10 evals in 2.5 ms vs 1330 ms numpy (~530x); parity 1.9e-9 |
| `_nmpcbatch.cost_batch` / `grad_batch` | shooting cost + forward-diff gradient on `heyoka::taylor_adaptive_batch` (6-state orbit system, same family as `cxxdrive/campaign.cpp`) | warm solve 5 ms vs 90 ms numpy (~18x with stacked constraints); cost parity 3e-4, solve cost within 0.2% |
| `_nmpcsolve.closed_loop_batch` | **whole NMPC sim in one call**: NLopt-SLSQP bulk plans (warm-started) + trim port + alloc_eff + safe early-abort vs incumbent | warm sim ~0.16 s vs ~1.3 s numpy (~8x); closed-loop dv within 5%, same verdict; deterministic |

### Which backend actually ran

`resolve_backends()` picks the *effective* backends once, before any stage, and
every stage plus validation uses that value — so the string in the results JSON
is the one that produced the numbers. `res_optimization.json` records both:

```json
"nmpc_backend": "cpp", "nmpc_backend_requested": "cpp",
"backend_fallbacks": []
```

If an extension is missing the run downgrades, logs a warning, and fills
`backend_fallbacks`. That field exists because the alternative is worse than no
record: a run where `_nmpcbatch` fails to load produces numpy-RK4 numbers while
the config block still reads `"nmpc_backend": "cpp"`. `validation.backend` is
recorded separately for the same reason — `cxx` cannot drive validation (it
collapses the whole closed loop into one crossing, and validation steps the real
12-state heyoka dynamics from Python), so it maps to `cpp`, the nearest
prediction family.

The default is `cpp` for both arms, matching how the committed sweep in §11 was
run. `numpy` is the reference path, kept for the parity tests.

Notes:

- The NMPC cost intentionally keeps `norm_penalty=0`: the norm cap stays a
  hard scipy constraint, identical to the numpy backend. Taylor prediction is
  more accurate than the 600 s RK4 step, so exact cost equality is not
  expected — the gates are cost ≤1e-3, same feasibility verdict, closed-loop
  dv within 5%.
- The numpy backend has no analytic cost gradient, so `scipy` estimates it by
  forward differences: **one full N-step RK4 rollout per decision variable per
  iteration**, i.e. `3N+1` cost calls. Measured `objective_evals_per_solve ÷
  nit_per_solve` matches `3N+1` closely (8.0/7 at H=2, 39/37 at H=12), and the
  solve time is exactly that rollout time — at H=12, 1947 rollouts x 414 us =
  806 ms against a measured 806 ms. `grad_batch` runs the same `3N+1` lanes as
  one expanded batch inside a single C++ crossing, which is where the ~18x
  comes from. This is why the default matters: profiling the numpy path
  characterises the fallback, not the pipeline.
- `_nmpcsolve` needs `import heyoka` first (libheyoka, same as `_cxxdrive`)
  **and** NLopt at `~/.local/nlopt` (built once from source with cmake;
  the extension rpaths it, so the conda env is untouched).
- Batch templates JIT once per worker (~7 s cold, ~0.17 s warm from disk
  cache): with a fresh pool per restart this is repaid per process, so the
  first solve in each worker is slow and the rest are fast.
- The embedded SLSQP is NLopt `LD_SLSQP` (Kraft, same family as scipy's),
  `ftol_rel=1e-6`, `maxeval = maxiter·(3N+3)`; solver failure falls back to
  the same energy-based fallback as `HighLevelNMPC`. A `maxeval_cap`
  override exists for budget probes (default 0 = auto mapping).
- The PID kernel is plain-C++ RK4, not Taylor: at these step sizes Taylor
  buys nothing over RK4 per step, while the exact RK4 port preserves the
  tuned numbers bit-near.
- Variational-equation analytic gradients and CMA-ES-as-online-solver were
  evaluated and deferred: the batched finite-difference `jac` already
  removed the dominant cost (19 Python round-trips per SLSQP iteration → 1
  C++ call).

## 8. Design choices

| Choice | Why |
|---|---|
| Log-space parameters | gains span 2–3 decades; log gives symmetric bounds and better conditioning |
| Feasible-first ranking | a cheap-but-infeasible candidate must never win |
| Energy metric not radius | radius oscillates naturally; `a` does not (§1) |
| Per-arm search resolution | each loop's gains are period-dependent (§4.1) |
| `alloc_eff` correction | allocator delivers 50 % of command (§4.2) |
| Horizon as structural sweep | changes decision-vector size, not a gain |
| Joint stage last | makes the comparison tuned-vs-tuned |
| RK4 search + full-sim validation | ~1000× cheaper per eval, agreement <1 % |
| Instance-local gains | safe across worker processes |
| Pinned solver budget | maxiter materially changes the answer (§5) |

---

## 11. Sweep results, 2026-10-07 (`--restarts 4 --maxiter 40 --workers 4`, cpp)

20.1 min wall. Search 0.5 d, validation 24 h heyoka + 12-jet allocation.

| arm | search dv / box | validation dv / box | transfers? |
|---|---|---|---|
| PID stage 1 | 23.96 / 0.89 km | 23.97 / 0.88 km | yes (<0.1 %) |
| PID joint peer | 24.17 / 1.18 km | 24.05 / 1.08 km | yes |
| NMPC H=12 (best) | 23.25 / 22.91 km | 36.16 / 33.02 km | **no** |
| NMPC joint | 22.72 / 24.92 km | 30.84 / 42.09 km | **no** |
| NMPC defaults H=6 | 36.44 / 29.69 km | 29.56 / 5.27 km | yes (conservative) |

Tuned NMPC params tell the story: H=4 took `q_vel=38` (default 0.1),
`db_a=3110 m` (default 500), trim budget 19.2 (default 0.5); H=12/joint took
`r_fuel≈13-15` (default 0.5) with `db_a≈3-6 km`. All exploit perfect
actuation: crank velocity damping (free in the surrogate, destabilizing
under misdirected thrust) or coast at the box edge on 2 km of margin that
attitude coupling eats. Post-hoc full-sim of H=4 (trim on *and* off):
dv≈2000+, box≈2200 km — trim is exonerated, the weights are the cause.

Consequences: the deployable result of this sweep is the **tuned PID**
(84 % under hand-tuned gains, validated). NMPC needs a robustness-aware
re-tune: cap `q_vel`/`db_a`/trim near defaults, or put actuation direction
noise + multi-IC into the surrogate cost.

---

## 9. Known limitations

- **Surrogate gap for high-gain NMPC (measured, see §11).** The search
  surrogate is translation-only with perfect thrust direction. PID tuning
  transfers to full-sim to <0.1 %, but NMPC weights with large `q_vel` /
  wide `db_a` / large trim budgets overfit the perfect-actuation assumption
  and diverge under attitude/allocation coupling. Do not quote surrogate
  NMPC numbers as flight numbers — only validation-table numbers.
- **Single initial condition.** Every evaluation uses the same −0.5 % injection.
  A gain set tuned on one point of a 9-dimensional initial-state space is not
  robust; a Monte-Carlo-weighted cost (or `cfg.v_frac` swept) would be the
  honest version and is the natural next step.
- **Single disturbance model.** No thruster misalignment, no mass variation, no
  SRP. The robustness machinery in `monte_carlo.py` is not wired into the
  objective.
- **Trim budget rarely binds at short horizons.** `TRIM_DV_BUDGET` only starts
  constraining once the outer window exceeds `budget / TRIM_A_MAX` (100 s at the
  default config), so at a 10 s window it is inert — the measured `trim_frac`
  is then set by authority saturation alone.
- **`box_r` lower bound is `box_r = 0`.** The config default is "always
  correct"; the log-space floor is 1e−2 m rather than 0, which is the same
  behaviour to within any practical tolerance.

---

## 12. Timing instrumentation

`profiling.py` records a nested section tree through the whole sweep and writes
it to `res_optimization.json` under `timing`, plus `res_timing.{json,txt}`. It
exists because until now the only timing that reached disk was `wall_seconds`
and `stage_wall_seconds`, while every detailed figure in §10 was measured by
hand and written into this file.

The tree is: `stage_*` > `horizons` > `horizonH{n}` > `restart{i}` >
`generation` > `ask` / `dispatch` / `tell` > `eval` > `sim` > `accel` / `rk4`,
and inside `accel` the full control path down to
`nmpc.solve.minimize`. Parent times are inclusive; `exclusive_s` subtracts
children, so a level that looks as expensive as its children is not.

Three properties worth knowing:

- **Averages, not per-call numbers.** No timed region is reported per
  individual step. Timed regions sit far enough above timer resolution, and what
  is read is `total / calls` accumulated over an entire sweep. `--bench-reps N`
  repeats a whole fast sim N times and reports mean/std/min/max, which is the
  only way to put an uncertainty on "one fast sim costs X".
- **Worker CPU is shipped back.** Candidates run in a `ProcessPoolExecutor`, so
  each worker serialises its subtree and the parent folds it in. Under
  parallelism `eval` totals therefore exceed wall time; both are reported,
  because the gap is the parallelism. A forked worker starts from an empty tree
  so it cannot report a copy of the parent's sections back.
- **Overhead is negligible.** One `perf_counter` pair per timed region. A test
  asserts a timed fast sim costs less than 3x the untimed one.

Per-horizon cost is reported alongside each horizon's `dv` in
`per_horizon[i]["timing"]`, and flattened into `nmpc.per_horizon_timing`:

| column | meaning |
|---|---|
| `wall_s_per_eval` | wall seconds per evaluated candidate |
| `plans_per_eval` | NMPC replans per candidate (one solve each) |
| `nit_per_solve` | SLSQP iterations per solve |
| `ms_per_nit` | per-iteration cost — isolates decision-vector size from iteration count |
| `objective_evals_per_solve` | `cost()` calls per solve: ~3N+1 on numpy (finite differences), ~1 on cpp |
| `grad_calls_per_solve` | explicit gradient calls per solve: 0 on numpy, ~1 per iteration on cpp |
| `lanes_per_grad` | rollouts inside one gradient call: `3N+1` on cpp, batched in C++ |

All rates are normalised by evaluation count. The horizons in a sweep are not
guaranteed to have run equal work — a failed restart or a resumed one changes
the denominators — so a raw millisecond total would compare effort rather than
cost per unit of effort.

Counter placement matters: `cost_evals`/`grad_evals` are incremented inside
the *closures handed to SLSQP*, including the C++ ones. Attaching them to the
Python `cost` closure alone made the cpp path report zero, which reads as "no
work" when it is really `3N+1` Taylor lanes per crossing.

`pred_traj` is off by default (`NMPCConfig.record_pred_traj`): nothing consumes
it, and it costs N Python RK4 steps per solve immediately after the solver used
Taylor — measured ~9 % of solve time. `fig_opt_trajectories` re-simulates the
winners, and passes the effective backend through; it used to default to numpy
regardless, so the plotted curve was a different controller from the one that
was tuned (and ~16x slower, which is how it surfaced).

The evaluation cache is shared across pool workers (a `multiprocessing.Manager`
dict, degrading to per-worker if unavailable). Per-worker caches made the hit
rate a function of `--workers`, which moved the denominator of every rate above.
The final re-evaluation deliberately bypasses the cache so it stays a real
re-evaluation rather than becoming a lookup.

Caveat: rates aggregate over every node with the matching name in the subtree.
Each restart contributes its own `solve` node, so a first-match lookup would
pair one restart's call count with iteration counts summed across all of them
and roughly double every rate. There is a test for exactly that.

---

## 13. Adjoint gradient

The shooting cost has no analytic gradient, so SLSQP finite-differences it: one
full N-step rollout per decision variable per iteration, `3N+1` lanes. Measured
on the `cpp` backend at H=12, that is **88.9 % of all Taylor lanes**
(3368 of 3790 us per solve; `grad_batch` batches the lanes into one crossing but
does not reduce their number).

`adjoint.py` replaces those `3N+1` perturbed rollouts with one forward rollout
plus one backward sweep. It is the **exact Jacobian-transpose of the same RK4
step the forward pass uses** — not a continuous adjoint, which paired with a
discretised forward pass is an inconsistent gradient and degrades SLSQP. Gravity
needs no variational system: the orbit is Keplerian, so
`dg/dr = -mu/|r|^3 (I - 3 rhat rhat^T)` is closed form.

Verified three ways in `tests/test_adjoint.py`: `step_value` against
`_rk4_step_orbit`, the step Jacobians `S`/`P` against finite differences, and
the assembled gradient against a central difference of the cost (rel. err ~1e-7
at N=2..12, with the deadband disabled). Each cost term is checked separately,
since a wrong gradient on one term hides behind a right gradient on another.

### Measured effect, numpy backend, seed theta, `maxiter=50`

| H | finite differences | adjoint | speedup | nit fd → adjoint | feasible |
|---|---|---|---|---|---|
| 2 | 14.8 ms | 7.2 ms | 2.1x | 67 → 51 | both |
| 4 | 69.9 ms | 22.8 ms | 3.1x | 108 → 73 | both |
| 6 | 190.1 ms | 46.8 ms | 4.1x | 145 → 107 | **fd fails**, adjoint converges |
| 8 | 333.2 ms | 75.8 ms | 4.4x | 150 → 136 | **fd fails**, adjoint converges |
| 12 | 737.3 ms | 125.5 ms | 5.9x | 150 → 140 | both fail |

The adjoint takes **fewer iterations everywhere**, because the gradient is exact.
At H=6 and H=8 that is the difference between converging and exhausting
`maxiter` and returning the zero (fallback) plan.

`--nmpc-backend numpy` is now a usable path rather than a 90-second-per-solve
curiosity. `ScipyShootingBackend(grad="fd")` keeps the reference implementation
the adjoint is tested against.

### Not yet done: the `cpp` backend

The adjoint is implemented in numpy, and the `cpp` cost/gradient are
Heyoka Taylor batches. The two cannot be mixed — a Taylor cost with an RK4
adjoint is exactly the inconsistency above — so `use_cpp=True` still uses
`grad_batch`. Doing it properly means adding variational equations to
`taylor_adaptive_batch` in `cxxdrive/nmpc_batch.cpp`, which is **not attempted
here**: there is no way to compile or test it in the development container, and
a ~5x on the cpp path is not worth shipping unverified. The numpy adjoint above
is the reference implementation and the parity gate it would have to pass.

### Caveat: the deadband

`db(e) = 0` for `|e| < db_a`, `e` otherwise, so `db'` is 0 or 1 almost everywhere
and undefined on the boundary. The adjoint uses that a.e. derivative. A centred
finite difference whose step straddles the boundary returns something else
entirely, so the two disagree *at the boundary only* — the parity tests
deliberately avoid placing the state there. This is a property of the cost
function, not of the method, but it does mean the two gradient strategies are not
interchangeable for a plan sitting exactly on the deadband edge.
