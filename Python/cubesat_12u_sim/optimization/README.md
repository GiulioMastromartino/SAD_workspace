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
| `--nmpc-backend` | `numpy` | `cpp` = Heyoka C++ batch cost+grad (§10), `numpy` = RK4 reference |
| `--validate-t` | 7200 | validation horizon [s] on the full sim |
| `--resume` | off | resume from per-restart checkpoint files |
| `--no-checkpoint` | off | disable per-restart checkpoint files |
| `--seed` | 20261007 | RNG seed |

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

| kernel | what | measured |
|---|---|---|
| `_pidbatch.pid_batch_eval` | whole CMA-ES population (RTN law + RK4 + score) in one call | 10 evals in 2.5 ms vs 1330 ms numpy (~530x); parity 1.9e-9 |
| `_nmpcbatch.cost_batch` / `grad_batch` | shooting cost + forward-diff gradient on `heyoka::taylor_adaptive_batch` (6-state orbit system, same family as `cxxdrive/campaign.cpp`) | warm solve 8 ms vs 90 ms numpy (~11x); cost parity 3e-4, solve cost within 0.2% |

Notes:

- The NMPC cost intentionally keeps `norm_penalty=0`: the norm cap stays a
  hard scipy constraint, identical to the numpy backend. Taylor prediction is
  more accurate than the 600 s RK4 step, so exact cost equality is not
  expected — the gates are cost ≤1e-3, same feasibility verdict, closed-loop
  dv within 5%.
- `_nmpcbatch` needs `import heyoka` first (it links libheyoka with
  dynamic_lookup — same pre-existing requirement as `_cxxdrive`).
- Batch templates JIT once per worker (~7 s cold, ~0.17 s warm from disk
  cache): with a fresh pool per restart this is repaid per process, so the
  first solve in each worker is slow and the rest are fast.
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

## 9. Known limitations

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
