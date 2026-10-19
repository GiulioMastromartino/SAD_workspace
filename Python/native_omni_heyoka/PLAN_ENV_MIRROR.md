# PLAN_ENV_MIRROR — stock env-effector mirroring spec (list.md §A1)

Status: SPEC ONLY. No mirror code exists yet (`omniheyoka/swap.py:39-40`
`_MIRRORED` is exactly `{Spacecraft, ReactionWheelStateEffector,
ThrusterDynamicEffector}`); none may half-land. The implementer wires one
reader per §2–§6 into `swap.py` following §8; each landing flips
`tests/test_replacement_matrix.py::test_env_mirror_gap_open` red by design
(registry update required at that point).

All stock names below were VERIFIED by grep against the wrapper
`Basilisk/simulation/*.py` files (cited `file:line`). All omni setters were
verified against `include/omni/vec_integrator.hpp` (cited `hpp:line`). C++
consumption rules were verified against `src/omni_vec_integrator.cpp`,
`src/omni_spacecraft.cpp`, `include/omni/environment.hpp`,
`include/omni/spacecraft.hpp` (cited). Nothing here was executed: this
container has no Python/compiler/Basilisk runtime; the host (Mac) owns
first execution via `tests/test_env_mirror_contract.py`.

## §1. Mirror discipline (binding)

1.1 **Field modules STAY, effectors are REPLACED.** Only `DynamicEffector` /
`StateEffector` subclasses are removed from the task lists and replaced by
`oi.set*` + subscribes. Pure field/model modules (atmosphere, wind,
eclipse, magnetic-field, ephemeris, spice — all plain `SysModel`) are not
dynamics and PASS THROUGH untouched in the task, exactly per the current
discipline (`swap.py:140-143`). The reader only ADDS subscribe links from
their out-messages; it never removes or reconfigures them.

1.2 **Reject wording.** Every loud reject uses the established envelope
message (`swap.py:146-150`, `swap.py:192-196`): `"mirror_stock_config:
task '<t>' holds <Cls>; envelope is hub + wheels + staircase thrusters +
central gravity + explicit omni env (stock env effectors: use omni env
setters instead)."` Env-specific rejects append the concrete reason
(e.g. `"; <Cls>: non-identity Gt unmirrorable"`).

1.3 **Gate principle (structural-invisibility rule).** MIRROR iff every
physics-affecting stock parameter is either (a) consumed per-tick from a
message omni reads, or (b) recorded verbatim into an omni setter that the
C++ graph fingerprints (`configFingerprintForObject`, `hpp:278-283`).
Anything structural that omni neither consumes nor fingerprints
(per-facet drag law, projection occlusion state, WMM-epoch-style hidden
law knobs where flagged, lookup tables, Gt maps, articulated joints)
is a LOUD REJECT. Never a silent divergence (`swap.py:25-26`).

1.4 **Per-object surface.** Every setter below has a `...ForObject` twin
(`hpp:172-187`) and every link a `subscribe...ForObject` twin
(`hpp:209-218`). Readers take `object_index` and use the twins, mirroring
the fuel/RW/thruster per-object pattern (`swap.py:256-299`). Message links
never mark a slot mirrored (`hpp:202-204`).

1.5 **Required vs optional links (DESIGN.md A13 truth table, enforced
C++-side — `src/omni_vec_integrator.cpp` re-verified):**

| link | stock | omni (verified) |
|---|---|---|
| `atmoDensInMsg` | `bskError` when drag on | same: drag on + unlinked = error (`cpp:1163-1167`) |
| `windVelInMsg` | unlinked = inertial velocity | same: `vAir=0` fallback (`cpp:1170-1174`) |
| `sunStateInMsg` | `bskError` when SRP on | same: SRP on + no sun = error (`cpp:1199-1205`) |
| `eclipseInMsg` | unlinked = 1.0 (full sun) | same (`cpp:1220-1222`) |
| `magFieldInMsg` | `bskError` when mag on | same (`cpp:1255-1258`) |
| MTB cmd/params | `bskError` | n/a (no MTB cmd path); zero dipole + mag on = error (`cpp:1259-1262`) |

## §1.6 Omni setter surface (all exist, `hpp` lines)

| setter (slot-0 / ForObject) | hpp |
|---|---|
| `setMass/setInertia/setCoMOffset/setMu/setUseGravityGradient` (existing) | 94-99 / 165-171 |
| `addEnvironmentBody(mu, ox, oy, oz)` | 106 / 172-173 |
| `setJ2(J2)` | 107 / 174 |
| `setDragParams(CdA, cx, cy, cz)` | 108 / 175-176 |
| `addSRPFacet(area, nx,ny,nz, copx,copy,copz, c_diffuse, c_specular)` | 109-111 / 177-180 |
| `setSRPCannonball(area, cr)` — `cr` is SPECULAR FRACTION r∈[0,1]; stock `coefficientReflection` is TOTAL C_R∈[1,2], so `cr = C_R − 1` | 112-116 / 181 |
| `setSunState(px, py, pz)` | 117 / 182 |
| `setMagneticMoment(mx, my, mz)` (MTB dipole, body) | 118 / 183-184 |
| `setMagneticDipole(g10, g11, h11)` (centered-dipole record) | 119 / 185-186 |
| `setMaxMtbDipole(dmax)` (identity-Gt clamp preset) | 120 / 187 |
| `subscribeAtmo/Wind/Eclipse/Sun/Moon/MagForObject` + slot-0 `atmoDensInMsg/windVelInMsg/eclipseInMsg/sunStateInMsg/moonStateInMsg/magFieldInMsg` | 209-218 / 239-244 |

C++ index conventions the reader must obey (verified):
- N-body slot 0 ↔ sun message, slot 1 ↔ moon message; static recorded
  offsets otherwise (`omni_vec_integrator.cpp:1147-1161`; same in
  `omni_spacecraft.cpp:1497-1505`). So the FIRST non-central body wires
 5527aec36f5f457aSun, the SECOND wires Moon; a third is unrepresentable.
- SRP `sunPos` = linked sun message else staged `setSunState`
  (`cpp:1207-1212`); flux `(AU/d)²`, `P_SRP_1AU`, `AU_M` (`cpp:1216-1217`).
- Drag arm = `cLive − comOffset`, SRP facet arm = `rCop − cLive`
  (DESIGN.md A13 corrected 2026-10-14; `cpp:1187-1191`, `cpp:1237-1245`).
  Readers pass `comOffset`/`rCop` VERBATIM — arms shift C++-side with
  live aggregate-c. Never pre-shift.
- `densityCorrection` is PINNED `0.0` in the drag call
  (`cpp:1190-1191` `dragWrench(..., 0.0)`); `hpp` has NO density-correction
  setter. Nonzero stock correction has no surface → reject (§2.1).
- Mag clamp `|m| ≤ maxMtbDipole`, default `1.0` (`hpp:358`;
  `cpp:1268-1270`); non-identity Gt is fatal on BOTH paths
  (`spacecraft.hpp:199`, `:335-336` `mtbGt_` identity default).

## §2. Drag + atmosphere + wind → `setDragParams` + atmo/wind subscribes

### §2.1 `DragDynamicEffector` — MIRROR (cannonball only)
- Class `dragDynamicEffector.py:609`. Params: `coreParams` `:643`
  (typed `DragBaseData`, `dragDynamicEffector.py:596-608`: `projectedArea`
  `:599`, `dragCoeff` `:600`, `comOffset` `:601`); links:
  `atmoDensInMsg` `:644` (REQUIRED, §1.5), `windVelInMsg` `:645`
  (OPTIONAL, §1.5); mode: `modelType` `:646`,
  `cannonballDrag()` `:635-636`, `updateDragDir` `:638`,
  `getDensity` `:641`; correction: `densityCorrectionStateName` `:649`,
  `densityCorrection` `:650`.
- Reader: `CdA = dragCoeff * projectedArea`, `com = comOffset` →
  `setDragParams(CdA, com…)` (precedent: hand-wire
  `oi.setDragParams(2.2*5.0, 0.05, 0.0, 0.0)` in
  `audit_bundle/probes/swap_p1_parity.py:456-467`, measured vs stock);
  `atmoDensInMsg.subscribeTo(<atmo envOutMsgs[i]>)` (same file `:459`);
  forward `windVelInMsg` if the stock sim linked one, else leave
  unlinked (inertial fallback, §1.5).
- Gates (loud reject): non-cannonball `modelType` (host must pin the enum
  spelling of the `cannonballDrag()` selection -- unverifiable statically);
  any configured nonzero `densityCorrection` (no omni surface: the C++ call
  pins the 0.0 literal, `src/omni_vec_integrator.cpp:1190-1191` -- REVISED
  2026-10-14: fresh-object VALUE reads are process-state-dependent (None in
  a sim-free process, StateData proxy after sims ran, same interpreter) and
  must NEVER be branched on; gate on LINKAGE -- a linked/configured
  correction state rejects, unset linkage passes); `|v|≤1e-12→0` and
  `(1+densityCorrection)≥0` stay stock-side law notes
  (`audit_bundle/probes/wp5_env.py:309`).
- Payload (for ledger, not mirrored — density flows via message):
  `AtmoPropsMsgPayload.neutralDensity` `:660`, `.localTemp` `:661`;
  consume `WindMsgPayload.v_air_N` `:672`, never `v_wind_N` `:673`
  (`wp5_env.py:202-208` notes).

### §2.2 `FacetDragDynamicEffector` — LOUD REJECT
- Class `facetDragDynamicEffector.py:610`; geometry
  `SpacecraftGeometryData` `:596-608` (`facetAreas` `:599`,
  `facetCoeffs` `:600`, `facetNormals_B` `:601`, `facetLocations_B`
  `:602`); `addFacet(area, dragCoeff, B_normal_hat, B_location)`
  `:639-640`, `numFacets` `:641`; links `:642-643` (same names as §2.1).
- Reason: `hpp` has NO per-facet drag setter (§1.3 structural
  invisibility). Reject message cites `setDragParams`-only surface.

### §2.3 Atmosphere models — STAY IN TASK, wire out-message (no removal)
- `ExponentialAtmosphere` (`exponentialAtmosphere.py:379`): `baseDensity`
  `:386`, `scaleHeight` `:387`, `localTemp` `:388`; base
  (`AtmosphereBase`): `addSpacecraftToModel` `:363`, `scStateInMsgs`
  `:368`, `envOutMsgs` `:369`, `planetPosInMsg` `:370`, `epochInMsg`
  `:371`, `envMinReach` `:372`, `envMaxReach` `:373`, `planetRadius`
  `:374`. Payload `neutralDensity` `:436`, `localTemp` `:437`.
- `MsisAtmosphere` (`msisAtmosphere.py:379`, same base `:363-374`;
  `localTemp` payload `:436` — `neutralDensity` on the MSIS payload is
  HOST-TO-CONFIRM, not asserted) and `TabularAtmosphere`
  (`tabularAtmosphere.py:379`: `altList` `:386`, `rhoList` `:387`,
  `tempList` `:388`; payload `:436-437`): same treatment.
- Reader: single-spacecraft → `envOutMsgs[0]` (plural-vector pattern,
  measured 2.12.0 — `tests/test_wp45_env_offset.py:317-345`
  `_atmo_outbox`); multi-sc → match by `addSpacecraftToModel` order,
  reject if ambiguous. Wire → `subscribeAtmo`. Record
  baseDensity/scaleHeight/planetRadius/envMaxReach for the ledger only.
- Non-`AtmosphereBase` atmo model → reject (unknown payload).

### §2.4 Wind — STAY IN TASK, wire out-message (optional link)
- `ZeroWindModel` (`zeroWindModel.py:389`); `WindBase`:
  `addSpacecraftToModel` `:367`, `scStateInMsgs` `:369`, `envOutMsgs`
  `:370`, `planetPosInMsg` `:371`, `epochInMsg` `:372`,
  `getPlanetOmega_N` `:375` / `setPlanetOmega_N` `:378`,
  `getUseSpiceOmegaFlag` `:381` / `setUseSpiceOmegaFlag` `:384`.
  Payload `v_air_N` `:442`, `v_wind_N` `:443`.
- Reader: `ZeroWindModel` → wire `envOutMsgs[0]` → `subscribeWind`
  (corotation baseline, `wp5_env.py:311-329`); unlinked stays legal
  (§1.5, ~6 % LEO error documented, not a reject). Non-`ZeroWindModel`
  `WindBase` subclass → reject (unknown law, §1.3).

### §2.5 `CannonballDrag` (MuJoCo) — OUT OF SCOPE, passes through
- Class `cannonballDrag.py:352` is plain `SysModel` (NOT a
  `DynamicEffector`): `dragGeometryInMsg` `:364`, `atmoDensInMsg` `:365`,
  `windVelInMsg` `:366`; `DragGeometryMsgPayload` (`projectedArea`
  `:479`, `dragCoeff` `:480`, `r_CP_S` `:481`). Not in the
  task-effector census path — untouched, no reader, no reject.

## §3. SRP cannonball + faceted → `setSRPCannonball` / `addSRPFacet` + sun/eclipse subscribes

### §3.1 `RadiationPressure` — MIRROR cannonball only
- Class `radiationPressure.py:598`; mode: `setUseCannonballModel`
  `:621`, `setUseFacetedCPUModel` `:624`; cannonball params: `area`
  `:635`, `coefficientReflection` `:636`; links: `sunEphmInMsg` `:637`
  (REQUIRED, §1.5), `sunEclipseInMsg` `:638` (OPTIONAL → 1.0, §1.5);
  lookup surface: `addForceLookupBEntry` `:627`,
  `addTorqueLookupBEntry` `:630`, `addSHatLookupBEntry` `:633`,
  `lookupForce_B` `:639`, `lookupTorque_B` `:640`, `lookupSHat_B`
  `:641`. Sun payload `PositionVector` `:651`.
- Reader: `setSRPCannonball(area, coefficientReflection − 1)` (convention
  `hpp:113-116`; precedent `swap_p1_parity.py:517-530`, C_R=1.5→cr=0.5
  with analytic vacuity gate); `sunEphmInMsg` target → `subscribeSun`;
  `sunEclipseInMsg` target → `subscribeEclipse` (else unlinked = full
  sun). Needs FULL `SpicePlanetStateMsg` (position + 1/r² flux scaling),
  never a bare direction (`wp5_env.py:202-208`).
- Gates: faceted-CPU mode or any lookup entry → reject (§1.3).

### §3.2 `FacetSRPDynamicEffector` — MIRROR static facets
- Class `facetSRPDynamicEffector.py:613`; `setNumFacets` `:633`,
  `setNumArticulatedFacets` `:636`, `getNumFacets` `:639`,
  `getNumArticulatedFacets` `:642`,
  `addFacet(area, dcm_F0B, nHat_F, rotHat_F, r_CopB_B, diffuseCoeff, specularCoeff)`
  `:645`, `addArticulatedFacet` `:648`; links: `sunInMsg` `:653`
  (REQUIRED), `sunEclipseInMsg` `:654` (OPTIONAL). Stored geometry
  `FacetedSRPSpacecraftGeometryData` `:596-605` (`facetAreaList` `:599`
  … `facetSpecularCoeffList` `:605`).
- Reader (per facet i): `nHat_B = dcm_F0Bᵀ · nHat_F`,
  `rotHat_B = dcm_F0Bᵀ · rotHat_F` →
  `addSRPFacet(area, nHat_B…, r_CopB_B…, diffuseCoeff, specularCoeff)`
  (precedent `test_wp45_env_offset.py:535-547` stock /
  `:556-585` omni `addSRPFacet(1.0, 1,0,0, 0,0.5,0, 0.2,0.3)` +
  `setSunState`, 0.5 % parity). `sunInMsg` → `subscribeSun`;
  `sunEclipseInMsg` → `subscribeEclipse`. Reader must cross-check
  `getNumFacets()` against the six list lengths.
- Gates (all reject): `getNumArticulatedFacets() > 0` or any
  `addArticulatedFacet` use; **nonzero `rotHat` stays REJECT**
  (binding constraint + list.md §E guard pins). Known tension, recorded
  honestly: the wp45 parity leg used `rotHat_F=[0,0,1]` with identity
  dcm and still matched omni at 0.5 % (suggesting rotHat is inert for
  static CPU facets), but the reject stands until a dedicated WP proves
  inertness with a ledger pin and relaxes it — a relax-into-model turns
  the guard red by design.

### §3.3 `FacetedSRPEffector` + `FacetedSpacecraftProjectedArea` — LOUD REJECT, stay in task
- `FacetedSRPEffector` (`facetedSRPEffector.py:596`):
  `facetElementBodyInMsgs` `:618`, `facetProjectedAreaInMsgs` `:619`;
  `FacetElementBody` payload (`area` `:628`, `r_CopB_B` `:629`,
  `nHat_B` `:630`, `rotHat_B` `:631`, `c_diffuse` `:632`, `c_specular`
  `:633`); `ProjectedArea.area` `:644`; eclipse `shadowFactor` `:673`.
- `FacetedSpacecraftProjectedArea` (`facetedSpacecraftProjectedArea.py:352`):
  `facetElementBodyInMsgs` `:380` (payload `:402-407`, area `:418`).
- Reason: message-driven facet feed + occlusion (`Aproj == max(0,n·s)A`
  must be proven occlusion-free first, `wp5_env.py:358-364`) is
  structural state omni cannot see (§1.3).

## §4. Magnetic + MTB → `setMagneticMoment` / `setMagneticDipole` / `setMaxMtbDipole` + mag subscribe

### §4.1 `MtbEffector` — MIRROR iff identity-Gt + latched dipole, else REJECT
- Class `MtbEffector.py:596`; links: `mtbCmdInMsg` `:620`,
  `magInMsg` `:621` (REQUIRED, §1.5), `mtbParamsInMsg` `:622`;
  `mtbOutMsg` `:618`, `hubSigma` `:619`. Command
  `MTBCmdMsgPayload.mtbDipoleCmds` `:630`; params
  `MTBArrayConfigMsgPayload` (`numMTB` `:652`, `GtMatrix_B` `:653`,
  `maxMtbDipoles` `:654`); torque out `MTBMsgPayload.mtbNetTorque_B`
  `:665`. Chain `magField_N → dcm_BN(σ) → B_B → τ = m × B_B`
  (`wp5_env.py:388-435`); 1-bar identity-Gt direct-equality gate ibid.
- Reader: require single-bar identity `GtMatrix_B` (**non-identity Gt
  stays REJECT** — binding; C++-fatal both paths, `spacecraft.hpp:199`);
  `maxMtbDipoles` → `setMaxMtbDipole`; dipole → `setMagneticMoment`
  from a LATCHED pre-written command snapshot (read at mirror time).
  `magInMsg` target (field module out-message) → `subscribeMag`.
- Latched-dipole rule (load-bearing limit, stated openly): omni has NO
  per-tick dipole input (mag cluster inputs are torque-only, DESIGN.md
  A13; `cpp:1252-1274` computes from staged `magM`). A post-mirror
  command rewrite is therefore CONTRADICTORY INPUT (documented) — same
  class as the thruster reset-vs-static rule (`swap.py:325-326`) but
  without the thrusters' continuous re-subscribe safety net. Any
  time-varying dipole driver → LOUD REJECT. Zero dipole + mag on →
  stock/omni both error (§1.5) — reader rejects upfront.

### §4.2 Field models — STAY IN TASK, wire out-message
- `MagneticFieldCenteredDipole` (`magneticFieldCenteredDipole.py:379`):
  `g10` `:386`, `g11` `:387`, `h11` `:388` → `setMagneticDipole`
  (verbatim record); base (`:363-374`, same shape as §2.3):
  `addSpacecraftToModel` `:363`, `envOutMsgs` `:369` → `subscribeMag`
  (REQUIRED). Payload `magField_N` `:436`.
- `MagneticFieldWMM` (`magneticFieldWMM.py:379`): `configureWMMFile`
  `:387`, `epochDateFractionalYear` `:389`, `wmmDataFullPath` `:391`
  — MIRROR-wire the message identically (field is a per-tick message
  input, §1.3(a)); record data path + epoch in the ledger (no omni
  surface needed on the vec path; coefficients-only `setMagneticDipole`
  stays dipole-exclusive). Payload `magField_N` `:438`.
- `Magnetometer` (`magnetometer.py:356`): `magInMsg` `:403`,
  `dcm_SB` `:405`, `tamTrue_S` `:408` — sensor, not dynamics/field:
  untouched, no reader (independent identity-`dcm_SB` check stays the
  validation path, `wp5_env.py:423-434`).

## §5. Eclipse — STAYS IN TASK, wire out-message (optional link)
- `Eclipse` (`eclipse.py:352`): `addSpacecraftToModel` `:369`,
  `sunInMsg` `:374`, `planetInMsgs` `:375`, `positionInMsgs` `:376`,
  `eclipseOutMsgs` `:377`, `rEqCustom` `:379`. Payload
  `EclipseMsgPayload.shadowFactor` `:386` — ONE field;
  `illuminationFactor` is a deprecated C alias, Python must use
  `shadowFactor` (`wp5_env.py:202-208`).
- Reader: single registered spacecraft → `eclipseOutMsgs[0]` →
  `subscribeEclipse`; multi → order-match, reject if ambiguous.
  Unlinked stays legal (factor 1.0, §1.5). Eclipse-crossing validation
  runs use DT ≤ 1 s (10 s ladder smears the penumbra edge;
  `wp5_env.py:367-372` recorded constraint).

## §6. Ephemeris / N-body / J2 → `addEnvironmentBody` + sun/moon subscribes, `setJ2`

### §6.1 `GravityEffector` / `GravBodyData` — central stays, non-central MIRROR (≤2 total extra → else reject)
- `GravityEffector` (`gravityEffector.py:816`): `setGravBodies` `:835`,
  `addGravBody` `:838`, `gravBodies` `:840`, read-only `centralBody`
  `:841`. `GravBodyData` (`:701`): `isCentralBody` `:724`,
  `gravityModel` `:725`, `mu` `:726`, `radEquator` `:727`,
  `planetBodyInMsg` `:732`. `Spacecraft.gravField` (`spacecraft.py:904`,
  class `:881`). Existing central-mu rule (`swap.py:63-72`) is unchanged.
- Reader (per NON-central body, stock `gravBodies` order): `mu` →
  `addEnvironmentBody(mu, ox, oy, oz)`; `planetBodyInMsg` link target →
  first extra body `subscribeSun`, second extra body `subscribeMoon`
  (§1.6 index rule); static `(ox,oy,oz)` = pre-written message snapshot
  when available, else recorded zeros (bench/fixture path,
  `cpp:1145-1146`).
- Gates: **total bodies > 2 stays REJECT** (binding; `omni_spacecraft.cpp:919`
  `nBodies>2[bucket-(a)]`); non-central body with UNLINKED
  `planetBodyInMsg` → reject (frozen-for-run offsets fatal: Sun tide
  ~1.4e-6 m/s², DESIGN.md A13; per-tick refresh required, ZOH ~1e-10
  at dt=0.1, `wp5_env.py:232-259`). `localPlanet`/`planetName`-style
  strings never read (§7.1).

### §6.2 J2 — MIRROR iff maxDeg == 2 on the central body, else reject
- `SphericalHarmonicsGravityModel`
  (`sphericalHarmonicsGravityModel.py:109`): `radEquator` `:124`,
  `muBody` `:125`, `maxDeg` `:126`, `cBar` `:127`, `sBar` `:128`,
  `loadFromFile(file, maxDeg)` `:130`. Default law is
  `PointMassGravityModel` (`pointMassGravityModel.py:52`, `muBody`
  `:64`) → no action.
- Reader: central body's `gravityModel` set → require `maxDeg == 2`
  (`maxDeg > 2` = higher-degree leakage → reject);
  `J2 = −√5 · Cbar20` (`wp5_env.py:262-292` rule; cBar-vector indexing
  is HOST-TO-PIN) → `setJ2`. ECI-equatorial analytic, no rotating frame
  (ibid.). Non-central J2 → reject.

### §6.3 Ephemeris providers — STAY IN TASK (no removal, no params mirrored)
- `SpiceInterface` (`spiceInterface.py:399`): `addPlanetNames` `:452`,
  `planetStateOutMsgs` `:478`, `epochInMsg` `:477`. `PlanetEphemeris`
  (`planetEphemeris.py:445`): `setPlanetNames` `:459`, `planetOutMsgs`
  `:461`. `EphemerisConverter` (`ephemerisConverter.py:352`):
  `addSpiceInputMsg` `:375`, `ephemOutMsgs` `:377`, `spiceInMsgs`
  `:378`. Sun/moon payload `PositionVector` `:523`, `VelocityVector`
  `:524` (`spiceInterface.py`; same `:421-422` eclipse, `:651`
  radiationPressure, `:861` gravityEffector).
- The reader only forwards the already-linked `planetBodyInMsg` object
  (§6.1) — no planet-name resolution needed. `PlanetEphemeris` (analytic
  elements) and `SpiceInterface` (kernels) are interchangeable sources.

## §7. Load-bearing SDK traps (implementer must read)

7.1 **Unreadable string attrs (thruster `.label` precedent, `swap.py:359-361`).**
SWIG string reads can abort — readers (and contract tests) must NEVER read
string-typed surfaces: `ModelTag`, `PlanetName`
(`spiceInterface.py:528`), `GravBodyData.planetName/displayName/modelDictionaryKey`
(`gravityEffector.py:729-731`), prop-name strings (`:842-846`). Use
`type(model).__name__` + numeric/message surfaces only. `hasattr` is safe
(no read); guarded `getattr` only for numerics/messages with skip-on-fail.

7.2 **Nondynamic instances.** Every wrapper calls `protectAllClasses`
(e.g. `dragDynamicEffector.py:683`) — stock objects reject monkeypatching.
Readers stage into plain dicts/records (fuel-mirror pattern,
`swap.py:441-476`) and call omni setters; never attach state to stock
objects.

7.3 **Census source.** `sc.states` / `sc.dynEffectors` are opaque SWIG
vectors (`swap.py:4-8`); the ONLY complete typed census is the task-model
scan (`task_effectors`, `swap.py:43-49`). Attachment without task entry
never contributes (combo-probe measured idle ~1e-21,
`test_wp45_env_offset.py:300-308` — `sc.addDynamicEffector` + task entry
both required). Mirror removes the env EFFECTORS from the task lists
(and documents the `sc` detachment); field modules stay.

7.4 **Out-message vectors, not singulars.** 2.12 exposes NO singular
density outbox — `envOutMsgs[0]` (atmo), same plural pattern for wind /
mag / eclipse (`test_wp45_env_offset.py:317-345`). Never assume
`atmoDensOutMsg`-style singulars.

7.5 **Sign/arm conventions (measured, not derived).** Drag `τ = F ×
(rBcB − comOffset)`; SRP facet `τ = (rCop − c_B) × F`; B/C outputs carry
`−Rᵀ·c_B` (DESIGN.md A13 `:541-561`; norm-only gates banned for sign).
Firing/scale gates mandatory on every env comparison (zeros-vs-zeros
vacuity precedent, ibid.).

7.6 **Fuel+env combos are OUT by label (H7, DESIGN.md A13 `:571-584`).**
Readers stay independent; no combined fuel+drag/SRP cell. SysModel path
has no depletion to compare — compare each path to stock, never
cross-path.

## §8. Implementer wiring checklist (not implementation)

1. Add `_mirror_drag`, `_mirror_srp`, `_mirror_mag`, `_mirror_eclipse_link`,
   `_mirror_env_bodies` (+ `_mirror_j2` inside env-bodies) to
   `omniheyoka/swap.py`, called from both `mirror_stock_config` (legacy)
   and `_mirror_single_object` (per-object, `...ForObject` twins, §1.4).
   Same ValueError discipline as §1.2.
2. Extend `_MIRRORED`-adjacent registry + `_reject_unmirrorable_tasks` so
   the newly-mirrorable classes no longer raise; `test_env_mirror_gap_open`
   goes red at that moment — update it in the SAME diff (list.md §A1).
3. Suggested signatures (mirroring `_mirror_fuel(models, oi, …)`):
   `_mirror_drag(drag_models, atmo_models, wind_models, oi, object_index)` →
   record; `_mirror_srp(srp_models, eclipse_models, oi, …)`;
   `_mirror_mag(mtb_models, magfield_models, oi, …)`;
   `_mirror_env_bodies(sc, oi, …)` (gravity + J2 + sun/moon subscribes).
   Explicit `[]`-when-absent lists on the per-object path (WP25-A4 rule,
   `swap.py:256-299`).
4. Host validation, in order: (a) `tests/test_env_mirror_contract.py` on
   the Mac (all stock-shape asserts green); (b) pin the three HOST-TO-PIN
   items (`modelType` cannonball spelling, `cBar` Cbar20 indexing,
   MSIS `neutralDensity`); (c) land readers one family at a time with
   firing parity legs at matched dt (drag-LEO still needs its acceptance
   sim first — list.md §B2).
