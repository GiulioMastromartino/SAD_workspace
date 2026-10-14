# WP-D jitter reproducibility spike — report

**Verdict: NO-GO.** Bit-exact stock jitter replication is impossible, and the
reason is stronger than the DESIGN-cited one: stock `JitterSimple` performs
**zero random draws**. There is no RNG engine, no seed sequence, no draw order,
and no `normal_distribution` anywhere in the stock jitter path, so there is
nothing to pin. The distributional gate + DESIGN ban must stand (the ban's
*cited mechanism* should be corrected, not its verdict — see §5).

Time-boxed read-only spike. No existing file edited, no installs, no commits.
Host: macOS, Basilisk 2.12.0 (pip `.so`/dylib build), env python
`Python/env/bin/python`.

## 1. Stock side: engine, distribution, draw order

**Engine: none.** The installed version's C++ source was pulled from the
upstream repo at the matching tag
(`AVSLab/basilisk@v2.12.0`,
`src/simulation/dynamics/reactionWheels/reactionWheelStateEffector.cpp`, full
file read) and contains **no** `<random>`, `rand`, `mt19937`, `RNGSeed`, or
`normal_distribution` reference. `RNGSeed` (default 464374481, verified live on
this host) is a `SysModel`-base-class property that this effector never reads.

**What stock `JitterSimple` actually is** (v2.12.0 source, `updateContributions`
+ `computeDerivatives` + `registerStates`): a **deterministic
theta-driven imbalance model**, not a noise process:

- One `reactionWheelThetas` state per jitter wheel, `thetasDot = Omega`,
  `theta(0) = 0`. `numRWJitter` counts jitter wheels; `BalancedWheels`
  registers no theta state (recorded `wheelThetas` identically zero).
- Per evaluation, `w2Hat_B`/`w3Hat_B` rotate with `theta` via `eigenM1(theta)`.
- Imbalance load: static force `F = U_s * Omega^2 * w2Hat_B` added to
  `vecTrans`; torque `rWB_B x F + U_d * Omega^2 * w2Hat_B` added to `vecRot`.
- Knobs are `U_s` (static imbalance, kg·m) and `U_d` (dynamic imbalance,
  kg·m²) per `RWConfigPayload` — HR16 factory defaults `U_s = 2.8e-6`,
  `U_d = 7.7e-7`. There is **no torque-noise sigma knob**; nothing maps to a
  `jitterSigma_` in N·m.
- `RWModel` enum (live): `BalancedWheels = 0`, `JitterSimple = 1`,
  `JitterFullyCoupled = 2`.

**Draw order per tick per wheel: zero draws per tick per wheel.** The jitter
"signal" is a wheel-rate sinusoid of amplitude ∝ `Omega^2`, evaluated inside
the dynamics at integrator substeps (coupled: it feeds `vecTrans`/`vecRot`
back-substitution while `theta` co-integrates), not a per-tick sample.

**Opacity note:** the build links only `@loader_path` Basilisk dylibs +
`/usr/lib/libc++.1.dylib` + `libSystem.B.dylib` (`otool -L` on
`_reactionWheelStateEffector.so`); `nm`/`strings` show no RNG symbols (stripped
dylib). Opacity is real but secondary — the source (matching tag) already
proves absence of any RNG.

## 2. Our port (reference, read-only)

- `src/omni_spacecraft.cpp:42` — `std::vector<std::mt19937_64> jitterRng_`,
  one stream per wheel.
- `src/omni_spacecraft.cpp:1063-1096,1143-1144` — per-wheel `mt19937_64`
  seeded `jitterSeed_ ^ splitmix64(index+1)` (splitmix64 constants
  `0x9E3779B97F4A7C15/0xBF58476D1CE4E5B9/0x94D049BB133111EB`), one
  `std::normal_distribution<double>(0, jitterSigma_)` draw per wheel per tick
  added to the friction-torque slot. Defaults (`include/omni/spacecraft.hpp`):
  `jitterOn_ = false`, `jitterSeed_ = 0`, `jitterSigma_ = 0.0`.
- True horizon already landed: `audit_bundle/probes/wp2_friction_jitter_voltage.py:84-86`
  (`N_JITTER_TICKS = 400`, no cap).

**Model mismatch (not just distribution mismatch):** the port injects additive
white Gaussian torque noise — Omega-independent, per-tick, memoryless. Stock
produces a deterministic wheel-rate sinusoid ∝ `Omega^2`, theta-state-driven,
evaluated at integrator substeps. Different family, different scaling,
different state dependence, and the port has no theta states (J3 labels
`wheelThetas` zeros). No parameter mapping exists (`U_s`/`U_d` imbalance
coefficients ⇄ noise sigma is dimensionally and physically meaningless).

## 3. Empirical checks (this host, small read-only sims)

Harness: stock `Spacecraft` (750 kg hub) + `rwFactory` HR16 + 1 wheel,
`u = 0` rewrite-every-tick driver, 60 ticks @ 0.02 s, `wheelSpeeds` +
`wheelThetas` + `scState` recorders. (`==` avoided in sim code per shell rule;
comparisons via `max|a-b|`.)

| # | Comparison | Result |
|---|---|---|
| 1 | JitterSimple, same seed 20260511, 2 runs — speeds, thetas, hub sigma/omega | `max|d| = 0.0` everywhere — deterministic replay confirmed |
| 2 | JitterSimple, seed 20260511 vs 777 — speeds, thetas, hub sigma/omega | `max|d| = 0.0` everywhere — **RNGSeed unobservable; no RNG consumed** |
| 3 | JitterSimple vs BalancedWheels @100 RPM — wheelSpeeds | `max|d| = 0.0` (exact mechanism consequence: `rWB_B = 0` factory arm kills the static-torque arm; dynamic torque `U_d·Ω²·w2Hat` lies in the y–z plane ⊥ spin axis, so `Ω` sees nothing — jitter is invisible on this channel) |
| 4 | JitterSimple vs BalancedWheels @100 RPM — hub sigma / omega | `max|d| = 3.02e-09` / `2.02e-08` — jitter effect real, tiny (`U_d·Ω² ≈ 8.4e-5 N·m` on an 800 kg·m² hub), deterministic |
| 5 | JitterSimple vs BalancedWheels @0 RPM — wheelSpeeds | `max|d| = 0.0` — confirms `Omega^2` scaling (zero speed ⇒ zero jitter) |
| 6 | Theta rate check | `(theta[60]-theta[0])/1.2 s = 10.47197551196597` vs `Omega0 = 10.471975511965978` — `thetasDot = Omega` confirmed to 1e-14 |
| 7 | BalancedWheels thetas | identically zero — no theta state, as sourced |

Corollary for the existing ledger: probe J1's determinism claim is measured on
`wheelSpeeds`, a channel on which jitter-vs-balanced is *exactly* 0.0 even
across models (check #3) — the J1 metric cannot see jitter at all in this
config and would pass vacuously. The hub-channel checks above (#1/#2/#4) are
the non-vacuous determinism evidence. (Flagged as a probe-gap observation
only; no probe file touched per spike rules.)

Stream-matching attempt: **not applicable — there is no stock stream.**
Same-seed port-vs-stock draw comparison is categorically impossible (stock:
0 draws/tick/wheel; port: 1 normal draw/tick/wheel from an engine stock never
instantiates).

## 4. Verdict: NO-GO (three independent load-bearing bars)

1. **No stock randomness to replicate.** Bit-exact "jitter draws" are
   impossible because the draws do not exist (v2.12.0 source + diff-seed
   bit-identity, §1/§3). No engine + seed sequence + draw order + distribution
   recipe can be written — three of the four terms are empty.
2. **Wrong model family on the port side.** White Gaussian torque noise ≠
   deterministic `Omega^2` theta-driven imbalance. Even a distributional match
   would be matching the wrong physics; no `U_s`/`U_d` ⇄ `jitterSigma_` map
   exists.
3. **Even the deterministic path is bit-unreplicable.** Exact replication of
   stock's (deterministic) jitter would need theta states + Eigen `eigenM1`
   evaluation order + Basilisk's substepped RK integrator reproduced
   bit-identically under heyoka Taylor-adaptive arithmetic — different
   operation order ⇒ divergence. The port deliberately registers no theta
   states.

## 5. Standing orders (NO-GO only)

- **The distributional gate + DESIGN ban stand.** `JitterSimple` stays
  distributional-parity-only; bit-exact remains prohibited.
- **Rationale correction recommended (follow-up edit, out of scope for this
  read-only spike):** `DESIGN.md:596-601` currently bans bit-exact on the
  grounds of "opaque macOS dylib RNG" + "`normal_distribution` differs across
  libc++/libstdc++". Both cited mechanisms are inaccurate for this effector —
  the effector contains no RNG and no `normal_distribution` (verified against
  the matching `v2.12.0` source tag + live diff-seed bit-identity). Suggested
  replacement rationale: *stock JitterSimple is a deterministic theta-state
  imbalance model (no RNG; `RNGSeed` unobserved); the port models jitter as
  additive white Gaussian torque noise — a different model family with no
  parameter map — and cross-integrator (heyoka-Taylor vs Basilisk-RK) +
  missing-theta-state bars make bit-identity unachievable.* Verdict unchanged,
  grounds replaced.
- **Probe-gap note for Phase 2.6 owners:** J1 determinism should additionally
  assert on a jitter-visible channel (hub `omega_BN_B`/`sigma_BN`, or a
  config with nonzero `rWB_B` arm), since `wheelSpeeds` is provably
  jitter-blind in the current config (check #3).
