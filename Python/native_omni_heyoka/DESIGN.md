# `native_omni_heyoka` — design contract

Runtime-composed symbolic dynamics for Basilisk, using the **Option A** method:
carry truncated-Taylor / symbolic *scalars* through the back-substitution
contribution arithmetic so the equations of motion fall out as an expression
graph, then hand that graph to heyoka's `taylor_adaptive` (which needs zero
runtime evaluations of the vector field).

This file is the **interface contract**. All three workstreams must conform to
it. Do not change a signature here without updating the others.

## Provenance of the method

Validated by the Option A prototype: carrying jets through the Basilisk
`BackSubMatrices` solve reproduces the exact EOM coefficients to **1.7e-15**
against an independent symbolic reference, and adding / tilting wheels needs
**zero changes to the solve**. Two hard-won constraints from that prototype,
which are *load-bearing*:

1. **`Js * gs * gs^T` must NOT go into `matrixD`.** For a balanced rotor the
   term cancels exactly against the spin equation. Folding it in costs 8.1e-5
   relative error in `omegaDot` and breaks momentum conservation. The rotor
   contributes `-w x h - u*gs` to `vecRot` with the **RELATIVE** momentum
   `h = Js*W*gs` (W = relative spin rate; the absolute form
   `h = Js*(W + omega.gs)*gs` double-counts against a subtracted `D` and
   breaks conservation for non-orthonormal wheel sets -- swap fix 2026-10-12;
   the extra term cancels exactly for 3 orthonormal wheels, hiding the bug
   in every 3-wheel gate); `matrixD` gets `diag(Js)` only
   via the spin-state coupling. (With a hub-only `D` and decoupled spin,
   the absolute form is the consistent partner instead -- see A11 for the
   consistent pairs.)
2. **The MRP->DCM must be hand-written for non-`double` scalars.** BSK's
   `Eigen::MRPd` is a concrete type, so `toRotationMatrix()` is instantiated for
   `double` only. The prototype's first formula was wrong (half-angle); validate
   against `avsEigenMRP` semantics:
   `R = I + (8*S^2 - 4*(1-|s|^2)*S) / (1+|s|^2)^2`.

## Package layout and file ownership

```
Python/native_omni_heyoka/
  CMakeLists.txt            # WS1
  pyproject.toml            # WS1
  include/omni/             # WS1
    scalar_traits.hpp       #   NumTraits, is_jet, make_scalar
    mrp_t.hpp               #   templated MRP
    backsub_t.hpp           #   BackSubMatricesT<Scalar> + Schur solve
    contributor.hpp         #   ContributorT<Scalar> interface
    hub.hpp                 #   HubContributorT<Scalar>
    reaction_wheel.hpp      #   ReactionWheelContributorT<Scalar>
    gravity.hpp             #   CentralGravityContributorT<Scalar>
    system.hpp              #   OmniSystem
  src/
    omni_core.cpp           # WS1  (contributors + OmniSystem + symbolic build)
    omni_heyoka.i           # WS1  (SWIG %module omni_heyoka)
  omniheyoka/               # WS1 emits the .so here; WS2 writes the .py here
    __init__.py             # WS2
    shim.py                 # WS2  OmniTaylorAdaptive
    presets.py              # WS2
  tests/                    # WS3
    test_*.py
```

Build exactly like `native_rw` (same CMake conventions, same heyoka discovery
order `$HEYOKA_ROOT` -> `/opt/homebrew` -> env prefix, `-std=c++23`, `-O3`,
direct `libheyoka` link with install rpath).

## Core C++ contract (WS1)

### `scalar_traits.hpp`

```cpp
namespace omni {
template <typename S> struct is_heyoka_expression : std::false_type {};
template <> struct is_heyoka_expression<heyoka::expression> : std::true_type {};

// Promote a literal to the scalar type (double -> expression must work).
template <typename S> inline S make_scalar(double v);          // general
template <> inline heyoka::expression make_scalar<heyoka::expression>(double v);
}
```

Requirements: works with `Eigen::Matrix<S,3,3>`; specialize
`Eigen::NumTraits<S>` when `S = heyoka::expression` so Eigen accepts it.

### `mrp_t.hpp`

```cpp
template <typename S> struct MRPt {
    Eigen::Matrix<S,3,1> sigma;
    Eigen::Matrix<S,3,3> toRotationMatrix() const;  // body-from-inertial
};
```
Must be correct for `S = heyoka::expression`.

### `backsub_t.hpp`

```cpp
template <typename S> struct BackSubMatricesT {
    Eigen::Matrix<S,3,3> A, B, C, D;
    Eigen::Matrix<S,3,1> vecTrans, vecRot;
};

// Schur-complement solve, S = A - B D^-1 C
template <typename S>
void solve_backsub(const BackSubMatricesT<S>& m,
                   Eigen::Matrix<S,3,1>& rDDot_BN_N,
                   Eigen::Matrix<S,3,1>& omegaDot_BN_B);
```
Explicit 3x3 adjugate/determinant inverse (no `Eigen::Inverse`), identical
arithmetic for both scalar types. Verified benign conditioning
(`cond(S) = 1.0007`).

### `contributor.hpp`

```cpp
template <typename S> class ContributorT {
public:
    virtual ~ContributorT() = default;
    virtual void updateEffectorMassProps(double integTime) = 0;
    virtual void updateContributions(double integTime,
                                     BackSubMatricesT<S>& backSubContr,
                                     const MRPt<S>& sigma_BN,
                                     const Eigen::Matrix<S,3,1>& omega_BN_B,
                                     const Eigen::Matrix<S,3,1>& g_N) = 0;
    virtual size_t n_states() const = 0;      // contributed spin states
    virtual std::vector<std::string> state_names() const = 0;
};
```

**Contributors must be written exactly once**, templated on `S`, and
instantiated twice: `double` (numeric cross-check) and `heyoka::expression`
(symbolic build). They are kept in lockstep inside `OmniSystem`.

### Required contributors

| class | notes |
|---|---|
| `HubContributorT<S>` | `mHub`, `r_BcB_B`, `IHubPntBc_B`; hub translational + rotational blocks. `r_BcB_B = 0` is the modelled case (nonzero must throw). |
| `ReactionWheelContributorT<S>` | N wheels, each `gs` (spin axis), `Js`, `Jt`. Constraint (1) above is mandatory. Position `rWB_B` is *not* modelled (documented envelope, matching `native_rw`). |
| `CentralGravityContributorT<S>` | point-mass `mu`, fills `g_N`. |

### `system.hpp`

```cpp
class OmniSystem {
public:
    void add_hub(double mass, const std::array<double,9>& I, std::array<double,3> r_BcB_B = {0,0,0});
    void add_reaction_wheel(std::array<double,3> gs, double Js, double Jt);
    void set_gravity(double mu);
    void set_u_max(double uMax);          // motor torque saturation

    // --- symbolic build (the whole point) ---
    // state_names: ordered, e.g. ["rx","ry","rz","vx","vy","vz",
    //   "s1","s2","s3","omx","omy","omz","W1","W2","W3"]
    // Returns the heyoka system [(var_i, var_i_dot), ...].
    std::vector<std::pair<heyoka::expression, heyoka::expression>>
        build_symbolic(const std::vector<std::string>& state_names);

    // --- numeric cross-check: accelerations from the SAME contributor code ---
    // state layout as state_names; returns [rDDot(3), omegaDot(3), ...wheel]
    std::vector<double> accelerations(const std::vector<double>& state,
                                      const std::vector<double>& pars) const;

    size_t n_states() const;
    size_t n_pars() const;                 // 19 for hub+3RW (see below)
    std::vector<std::string> state_names() const;
    std::vector<double>   default_pars() const;
    std::vector<double>   default_state() const;
};
```

**Parameter layout** — must match `native_rw` / `heyoka_rw.py` exactly so all
existing consumers keep working:

```
par[0..2]   u1,u2,u3        saturated motor torques
par[3]      mass
par[4..9]   Ixx,Iyy,Izz,Ixy,Ixz,Iyz
par[10..15] invIxx..invIyz
par[16..18] Js1,Js2,Js3
```
Total 19 for 3 wheels. Config values that must become `heyoka::par[i]` for the
symbolic build: `mass`, `I6`, `invI6`, `Js`. The wheel `gs` axes and `Jt` are
structural constants.

**Saturation** is applied when reading `u` (clamp to `±uMax`), matching stock
effector behaviour.

**MRP shadow-set reset** (`sigma -> -sigma/|sigma|^2` when `|sigma|^2 > 1`) is
*not* inside the ODE — it is done by the caller, exactly as `native_rw` /
`heyoka_rw.py` do. Document this.

### Symbolic build requirements

- Variables via `heyoka::make_vars(...)` in `state_names` order.
- All config that varies per Monte-Carlo run must be `heyoka::par[...]`.
- Must produce an expression graph heyoka accepts for
  `taylor_adaptive(sys, state, pars=..., tol=...)`.
- Prune with `[:]=` if needed to keep the graph small.

## Python contract (WS2)

```python
# omniheyoka/__init__.py
from .shim import OmniTaylorAdaptive
from .presets import rw_demo_system, cubesat_12u_system
__all__ = ["OmniTaylorAdaptive", "rw_demo_system", "cubesat_12u_system"]
```

### `OmniTaylorAdaptive` — must be a **fully heyoka-compatible shim**

Subclass the *real* class so `isinstance(x, taylor_adaptive)` is True and every
property/method is inherited:

```python
import heyoka as hy
from heyoka._core import taylor_adaptive_dbl

class OmniTaylorAdaptive(taylor_adaptive_dbl):
    def __init__(self, system, state, *, pars=(), tol=1e-12, **kw):
        sys_ = system.build_symbolic()
        super().__init__(sys_, list(map(float, state)), pars=..., tol=float(tol), **kw)
        self.system = system          # custom attr (proven to work: __dict__ exists)
```

Requirements:
- Accepts the same kwargs as `hy.taylor_adaptive` (`high_accuracy`,
  `compact_mode`, etc.) and forwards them.
- `.state` and `.pars` remain writable numpy views (inherited).
- Exposes `.system` for introspection and for the numeric cross-check.
- Must survive `propagate_for`, `propagate_until`, `step`, `.tc`, `.order`.

**Verified heyoka surface to preserve:** properties `compact_mode, d_output,
decomposition, dim, dtime, high_accuracy, is_variational, last_h, llvm_state,
n_orig_sv, order, pars, state, sys, tc, time, tol, with_events`; methods
`eval_taylor_map, get_mindex, get_vslice, propagate_for, propagate_grid,
propagate_until, reset_cooldowns, step, step_backward, update_d_output`.
Note `tol` is read-only; `dim` is the state count (there is **no** `n_states`
or `n_pars` attribute on the C++ class).

### `presets.py`

`rw_demo_system()` — 750 kg hub, `diag(900,800,600)`, 3 HR16 wheels
(`Js = 0.07957747154594767`, `uMax = 0.2`), `mu = 3.986004418e14`.
Must match `native_rw` defaults and `RW_JS` / `RW_U_MAX` in
`backends/heyoka/heyoka_rw.py`.

## Validation contract (WS3)

`tests/` must prove, with numbers:

1. **Symbolic == numeric.** `system.accelerations(state, pars)` must equal the
   `c0` of the built symbolic system differentiated at the same point, to
   ~1e-12, over random states and random parameters.
2. **Symbolic == independent reference.** The built system must equal
   `cubesat_12u_sim/backends/heyoka/heyoka_rw.py::build_rw_system()` after
   substituting the same parameters — compare by evaluating both at many random
   states (expect ~1e-12 or better; the Option A prototype got 1.7e-15).
3. **End-to-end.** `OmniTaylorAdaptive` propagating from the same ICs must
   match `hy.taylor_adaptive(build_rw_system(), ...)` to ~1e-12 over 600 s.
4. **Composition.** 1 wheel, 2 wheels, 3 wheels, and a *tilted* spin axis
   must all work with no code change to the solve; the 3-wheel tilted case
   must match a hand-derived tilted-wheel Newton-Euler reference.
5. **Constraint (1).** Assert the balanced-rotor `matrixD` treatment: assert
   total angular momentum conservation and that `omegaDot` is unaffected by
   spinning the wheels.
6. **MRP shadow switch.** Driver must reproduce `mrpSwitches` counting and
   `|sigma| <= 1` after every step.
7. **Perf.** Compare `OmniTaylorAdaptive` against monolithic
   `hy.taylor_adaptive` on build time, per-step time, and accuracy.

---

# ADDENDUM 1 — decisions added after WS1/WS2 launched

These resolve ambiguities in the contract above. **This addendum wins** over any
conflicting statement above.

## A1. State layout is derived, and `state_names()` is the single source of truth

The number of states is **not** fixed at 15. It is:

```
hub states, always first, always in this order:
    r(3), v(3), sigma(3), omega(3)                       -> 12
then, for each contributor in REGISTRATION ORDER, that contributor's
contributed states:
    ReactionWheel -> W_i(3) per wheel, ordered by wheel index
```

So 3 wheels gives 15 states named
`rx,ry,rz,vx,vy,vz,s1,s2,s3,omx,omy,omz,W1,W2,W3` — identical to
`heyoka_rw.STATE_NAMES` and to `native_rw`'s `make_vars(...)`. One wheel gives
14 states (`W1` only). Four wheels gives 18.

`n_states()` is computed, not stored. `state_names()` returns the ordered list
and everything downstream must use it as the authority.

## A2. Parameter layout is ASSIGNED DYNAMICALLY — the "19 params" above is the 3-wheel case, not a fixed layout

This was the contradiction in the original contract (hard-coded layout +
generic interface). Resolved as follows.

`OmniSystem` allocates parameter slots **at registration time**, in this order:

```
[0 .. K-1]   input-port block: one contiguous block per contributor that
             declares an input port, in registration order.
             A ReactionWheel contributor declares a 3-vector input (motor torque).
par[...]     then each contributor's own parameters, in registration order.
```

Concretely for the RW preset (hub + 3 wheels) this must reproduce
`native_rw`'s layout **exactly**, because that is what makes it a drop-in:

```
par[0..2]    u1,u2,u3     saturated motor torque (the input block)
par[3]       mass         (hub)
par[4..9]    I6           (hub)
par[10..15]  invI6        (hub)
par[16..18]  Js1,Js2,Js3  (one per wheel, in registration order)
= 19 total
```

For a hub-only system (no contributor declares an input port) the layout is
simply `mass, I6, invI6` = 13 parameters, with no leading input block.

**Required accessor** (add to `OmniSystem`):
```cpp
std::vector<std::string> par_names() const;   // e.g. ["u1","u2","u3","mass",...]
```
Python code must be able to introspect the layout rather than hard-code indices.

`Jt` (transverse rotor inertia) is **not** a parameter — it is a structural
constant, because the balanced-wheel model never uses it in the dynamics once
constraint (1) is respected.

## A3. NO `min`/`max`/clamp/`abs` inside the symbolic graph

Motor-torque saturation is applied **externally by the caller when writing
`pars[0..2]`**, exactly as `native_rw` does (`u[i] = clamp(v, ±uMax)` in C++
before `PA[0] = u[0]`). The symbolic system must use `par[0..2]` **verbatim**.

Do not introduce `min`/`max`/`if`/sign functions into the expression graph: they
degrade the Taylor integrator and are not needed. `uMax` is a system-level
double (not a parameter) that the caller reads to apply the clamp itself.

Consequence for WS2: the shim must expose `system.u_max()` so the caller can
apply the clamp, and must NOT try to embed it.

## A4. No `EffectorMassProps` aggregation layer

Deliberately simpler than Basilisk: `HubContributorT` owns `mass`, `r_BcB_B` and
`IHubPntBc_B` **directly**. `OmniSystem` does **not** aggregate mass properties
across contributors, and there is no `EffectorMassProps` struct. Do not build
one — it is not needed for a hub-at-CoM model and it is exactly the
aggregate-mass-property path that bypasses effector contributions in the real
`spacecraft.cpp`.

## A5. Gravity ordering: `g_N` is computed FIRST, then passed down

`CentralGravityContributorT` is not just another contributor. It produces `g_N`,
which is an **input** to the hub and wheel contributions. So:

```
OmniSystem::evaluate(t):
    g_N = gravity_contributor.field(state)          // Vector3<Scalar>, a function of r
    for each other contributor in registration order:
        c->updateContributions(t, blocks, sigma_BN, omega_BN_B, g_N)
    solve_backsub(blocks, rDDot, omegaDot)
```

Both instantiations must follow the same order. `g_N` must be
`Eigen::Matrix<Scalar,3,1>` so that in the symbolic build it is naturally a
function of the `rx,ry,rz` variables.

If there is no gravity contributor, `g_N` is all zeros (this is the correct
degenerate behaviour, and must be tested).

## A6. The symbolic build is AUTONOMOUS — no time variable

Do not introduce a `heyoka` time variable into the symbolic system. No
contributor in this build depends explicitly on `t`. `integTime` is still
carried in the `updateContributions` signature (so a future time-dependent
contributor is possible), but the symbolic builder must not emit a time symbol.
If any contributor needs explicit time, that is out of scope — fail loudly.

## A7. MRP shadow-set reset is owned by the CALLER's stepping loop, and the package must ship the helper

Confirmed from the original contract, plus: `omniheyoka` must export
```python
omniheyoka.reset_mrp(state, inplace=True) -> (state, switched_count_delta)
```
implementing `sigma -> -sigma/|sigma|^2` when `|sigma|^2 > 1`, plus
```python
omniheyoka.sigma_indices() -> slice(6, 9)
```
so callers (and `run_rw_mirror`-style loops) do not each re-implement it. This
matches `native_rw`'s internal behaviour, moved to Python because heyoka's
propagator has no per-step callback.

## A8. Contributor interface is sufficient for a wrench/thruster contributor (no change needed now)

Worth recording because "generic" was chosen as the scope: a hypothetical
`ThrusterWrenchContributorT<S>` adds `F_B` and `T_B` to `vecTrans`/`vecRot`
respectively, declares a 6-vector input port (or reads a quantified duty-cycle
parameter block), and needs **no change** to `updateContributions`' signature or
to `solve_backsub`. It is therefore explicitly out of scope for this build but
provably additive. Do not implement it; just do not close the door.

## A9. Build output location — cross-workstream import path

WS1 must emit the built extension to a location importable as:

```
PYTHONPATH=/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/native_omni_heyoka \
  python -c "import omni_heyoka"
```

i.e. the `.so` (or `.so` + module `.py`) must land in
`native_omni_heyoka/omniheyoka/`, not only in a `_skbuild` staging dir. Copy it
there as a post-build step. Both workstreams rely on this exact path.

## A10. `pars=` must be passed to the C++ constructor as a sequence, and `tol` is read-only

Reconfirmed for WS2: build `super().__init__(sys_, state_list, pars=list(pars),
tol=float(tol), **kw)`. Do not attempt to assign `.tol` afterwards.

## A11. Input-inertia semantics follow stock: input I is TOTAL, D separates spin inertia, spin is coupled

`IHubPntBc_B` (and `OmniSystem::add_hub`) carry the TOTAL inertia including
the wheels (measured vs stock to 8 digits per axis). The hub block inverts
`D = I - sum_wheels Js*gs*gs^T` (full outer product — tilted wheels need no
special case); gravity-gradient and translation keep the total `I`. Wheel
spin is coupled and sequential, exactly stock's ordering: the hub solve
runs first, then `Wdot_i = (u_i - Js_i*(gs_i.omegadot))/Js_i` (the
`omegaDot` argument of `state_derivatives` exists for this). `Js` stays
fully parametric (read from the wheels' own par slots; only spin axes are
structural), so Monte-Carlo dispersion keeps working. This refines
constraint (1), it does not contradict it: the forbidden combination is
*adding* `Js*gs*gs^T` to an already-hub-only `D` with decoupled spin
(double counts). The consistent pairs are (hub-only D, decoupled spin) and
(total-minus-Js D, coupled spin); stock and omni both use the latter.

## A12. Thruster input path: staged mailbox + sub-step arcs

The on-time message is staged on first sight / timestamp advance and
consumed one-shot by the next propagation (tick 0 stages without
propagating): fresh commands idle the fill step then fire every step,
write-once commands fire once, same-timestamp rewrites never re-fire
(all verified cell-by-cell against the effector). The contributor owns the
duty -> quantized on-time map (`{0,1/6,5/6,1}`, edges exact); the module
propagates arc-by-arc over the quantized open-times at full thrust
(average == staircase duty; no clamps in the graph, A3). Motor torques stay
continuous (latest message, per-wheel `uMax` saturation); the module
resolves RW input slots via `wheel_input_offsets()`, never by assuming
offset 0. Pulsed (not averaged) emulation makes trajectories depend on PWM
frequency: validate stock-vs-omni at matched dt.

## Explicit non-goals for this build

- Not a drop-in for *existing* Basilisk `spacecraft`/`StateEffector` C++
  classes (they are hard-typed to `double` and their `.cpp` is not shipped).
  This package supplies its own templated contributors.
- No `DynamicObject` / `SysModel` subclassing in this first build — Python-level
  only. That is a later phase.
- No adaptive step control beyond what heyoka provides.
- Translation is point-mass Kepler only; no N-body, no drag, no SRP.

---

# ADDENDUM 2 — envelope completion (WP0…WP7, 2026-10-12)

Contract additions from the batch that closed the input-acceptance gaps.
All of it is measured (audit_bundle ledger) or explicitly labeled out.

## A13. Environment + telemetry contract (WP5/WP6)

**Single env cluster.** All env effects register through ONE
`EnvContributorT<S>` cluster (`include/omni/environment.hpp`, order code
`'e'` in `OmniSystem`), never per-effect order codes: effects have no
states and no own pars, and a single registration keeps the arc-hook
ordering simple. Input slots (A2, name-derived, visible in `par_names()`),
relative to the cluster's `input_offset`:

```
[body  + 3*i + {0,1,2}]  envG{i}x, envG{i}y, envG{i}z   N-body offsets (per-tick refreshed)
[drag  + 0..5]           dragFx..Fz, dragTx..Tz
[srp   + 0..5]           srpFx..Fz, srpTx..Tz
[mag   + 0..2]           magTx, magTy, magTz            torque-only
```
`n_inputs() = 3*Nb + 15`. Gravity (N-body + J2) flows through the **g_N
channel** (`gravityExtra()`, summed by `OmniSystem` before the block
pass), NOT through ForceTorque. The cluster is evaluated LAST in the
ForceTorque loop (two-pass) so the thruster contributor's overwrite
semantics are preserved; env accumulates.

**Absent-link truth table (per effect; enforced module-side).** There is
NO uniform rule — copied per effect:

| link | stock | omni |
|---|---|---|
| `atmoDensInMsg` | `bskError` when drag on | same (dragOn_ + unlinked = error) |
| `windVelInMsg` | unlinked = inertial velocity (no corotation) | same |
| `sunStateInMsg` | `bskError` when SRP on | same |
| `eclipseInMsg` | unlinked = **1.0 (full sun, ON)** | same |
| `magFieldInMsg` | `bskError` when mag on | same |
| MTB cmd/params | `bskError` | n/a (no MTB); zero dipole + mag on = error |

Per-tick refresh: N-body offsets are read from messages every tick when
linked (statically recorded offsets otherwise). ZOH staleness ~1e-10
m/s² at dt=0.1; a frozen-for-the-run body is forbidden (Sun tide 1.4e-6
m/s² is fatal over the mission). Fast-varying wrenches (drag/SRP/mag) are
re-refreshed per arc at the `// WP5-arc-hook` marker inside the module's
arc loop (WP3-compatible: it writes env slots only).

**Arm shift under hub offset (measured split — NOT uniform):** drag
arms do NOT shift (tau = comOffset x F); SRP facet arms DO (tau =
(rCop - c_B) x F); magnetic and gravity have no arms. Thruster dynamics
arms shift ((r - c_B) x F) while thruster *telemetry* `thrustTorquePntB_B`
stays r x F about B. Firing/scale gates are mandatory on every env-vs-env
comparison — a mis-wired effector once produced a vacuous zeros-vs-zeros
"shifted" verdict, which is why absolute-torque gates exist in the probe.

**B/C output sign (measured, WP25-A1):** `r_BN_N = r_CN_N − Rᵀ·c_B`,
`v_BN_N = v_CN_N − Rᵀ·(ω×c_B)` (R = C_BN) — stock-pinned by
`audit_bundle/results/swap_p1_parity.json:thr_offset` (c=0.1, omni-vs-stock
r 2.24e-08; wrong sign would be ~0.2 m). Any `+`-sign reading of the B/C
outputs (self-consistent math only, never stock-pinned) is
superseded-by-measurement; norm-only (`|d|==0.1`) gates are banned for sign
(per-axis `r_BN−r_CN+Rᵀc == 0` to ≤1e-9 instead).

**Telemetry.** `thrusterOutMsgs` (per thruster, `THROutputMsgPayload` 9/9,
steady-tick = end-of-step, tick0 = zeros + geometry echo) and `rwOutMsgs`
(per wheel, `RWConfigLogMsgPayload` 23 fields; `u_current` is read back
from the applied par slot, never re-derived). `SCStates` gains
`omegaDot_BN_B` (from end-state numeric accelerations) and the three DV
slots + `nonConservativeAccelpntB_B`, accumulated per tick in inertial
and body frames — **labeled non-parity** (stock integrates per substep).
Fuel: `FuelTankMsgPayload`-shaped bookkeeping from staircase-ACHIEVED
thrust (shrinks the commanded-vs-achieved gap); NO mass feedback, NO
blowdown scaling, NO depletion dynamics — labeled out, not parity.

## A14. Loud-rejection + layout hygiene (WP0/WP7)

`checkUnsupported()` (Reset + UpdateState heads) raises for what is
genuinely unmodeled: the `THRArrayCmdForceMsg` link (guard-only by design).
Everything else in the Risks trap table is modeled behind default-off
setters (friction, jitter-Simple, min-pulse, ramps, cutoff lag, blowdown,
swirl, J2, offset, env) or informational (voltage reuse: stock
`MotorVoltageInterface` stays in the task, `motorTorqueOutMsg` ->
`rwMotorCmdInMsg`; zero omni-side change). `JitterFullyCoupled` always
raises (theta states + aOmega/bOmega/cOmega are not registrable).

**Jitter policy.** Bit-exact is prohibited: stock's RNG is an opaque macOS
dylib and `normal_distribution` differs across libc++/libstdc++ hosts.
`JitterSimple` gets distributional parity with a measured determinism
ledger (identical seed replays exactly: max|delta wheelSpeeds| = 0.0 over
2x400 ticks). Omni's own streams are `mt19937_64` split per wheel by
seed ^ splitmix64(index+1) — a documented replica, never a stock clone.

**Layout discipline.** No numeric state indices in `src/`: the module
resolves `rOff_/vOff_/sigmaOff_/omegaOff_/wheelOff_` by NAME from
`OmniSystem::state_names()` in `buildIntegrator()`. No hard-coded par
counts anywhere (the 19-par reference layout is dead; the C++17 initial
slots give 16 for hub+3RW): `par_names()`/`par_index()`/`par_slice()`
(`omniheyoka/layout.py`) are the only sanctioned accessors, and
`scripts/ci_envelope_checks.sh` enforces all three rules by grep (A3
graph purity, no hard-coded counts, no numeric state indices).

**Jitter/telemetry/vec note.** `rwOutMsgs` is a value-vector; the
`rwLogOutMsg(i)` pointer accessor is the live-view path (recorders must
attach to the live element — pre-Reset attachment works because
`addWheel`/`addRatedWheel` size the vector at registration).
