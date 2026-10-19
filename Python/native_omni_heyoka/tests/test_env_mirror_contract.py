"""Stock-side contract for PLAN_ENV_MIRROR (list.md §A1).

Host-gated static-shape asserts: every stock class / attribute / message
name the env-mirror spec depends on, pinned against the wrapper
`Basilisk/simulation/*.py` files. Pure `hasattr`/guarded-`getattr`
(shape only, no sims, no wheel needed): each test skips (never fails)
when its host module is missing. String-typed surfaces are
`hasattr`-only (never read — SWIG string-read trap, PLAN §7.1).
Value reads of live effector fields are BANNED by policy (2026-10-14:
fresh-object `densityCorrection` reads returned None in a sim-free
process and a StateData proxy after sims ran -- same interpreter, same
file -- so no value assertion on such fields is trustworthy; gates use
shape + C++ source contract instead).

Run on the Mac host:
    pytest Python/native_omni_heyoka/tests/test_env_mirror_contract.py -v
per-test SPEC cites point at PLAN_ENV_MIRROR.md sections.
"""

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


def _mod(name):
    """Import-or-skip one Basilisk simulation wrapper module."""
    return pytest.importorskip("Basilisk.simulation." + name,
                               reason="host-only: needs Basilisk " + name)


def _cls(mod, name):
    mod = _mod(mod) if isinstance(mod, str) else mod
    cls = getattr(mod, name, None)
    if cls is None:
        pytest.skip("host: class %s missing in %s" % (name, mod.__name__))
    return cls


def _has(obj, name):
    assert hasattr(obj, name), "host: %s missing attr %r" % (type(obj).__name__, name)


def _safe_get(obj, name):
    """Guarded read: numerics/messages only, skip (never fail) if unreadable."""
    try:
        return getattr(obj, name)
    except Exception as e:
        pytest.skip("host: unreadable attr %r: %r" % (name, e))


def _new(cls):
    try:
        return cls()
    except Exception as e:
        pytest.skip("host: cannot construct %s: %r" % (cls.__name__, e))


# --- §1 discipline: swap.py baseline still intact ---------------------------

def test_swap_baseline_intact():
    """SPEC §1: the three mirrored classes are still mirrored (subset, not
    equality — forward-compatible with the implementer extending it)."""
    swap = pytest.importorskip("omniheyoka.swap",
                               reason="host-only: needs repo omniheyoka")
    assert {"Spacecraft", "ReactionWheelStateEffector",
            "ThrusterDynamicEffector"} <= set(swap._MIRRORED)
    assert callable(swap.task_effectors)


# --- §2.1 cannonball drag ----------------------------------------------------

def test_drag_effector_shape():
    """SPEC §2.1: DragDynamicEffector params + links + mode surface."""
    m = _mod("dragDynamicEffector")
    cls = _cls(m, "DragDynamicEffector")
    for a in ("coreParams", "atmoDensInMsg", "windVelInMsg", "modelType",
              "densityCorrectionStateName", "densityCorrection"):
        _has(cls, a)
    for fn in ("cannonballDrag", "updateDragDir", "getDensity"):
        assert callable(getattr(cls, fn, None)), "host: missing " + fn
    base = _cls(m, "DragBaseData")
    for a in ("projectedArea", "dragCoeff", "comOffset"):
        _has(base, a)
    pay = _cls(m, "AtmoPropsMsgPayload")
    _has(pay, "neutralDensity")  # density flows via message (§2.1)
    _has(pay, "localTemp")
    wind = _cls(m, "WindMsgPayload")
    _has(wind, "v_air_N")  # consume v_air_N, never v_wind_N
    _has(wind, "v_wind_N")


def test_drag_correction_default_zero():
    """SPEC §2.1 gate (REVISED 2026-10-14): fresh-object VALUE reads of
    `densityCorrection` are process-state-dependent (None in a sim-free
    process, StateData proxy after sims ran -- measured same-interpreter,
    same file) and therefore unusable as a gate signal; the reason is
    unconfirmed (leading hypothesis: the getter resolves against
    process-global sim registries polluted by earlier runs -- do NOT
    depend on it). The gate is structural instead:
    (a) the field + its state-name exist (shape -- a missing surface
    fails here, loudly); (b) the omni C++ drag call hardcodes the 0.0
    correction literal with no setter (source contract -- a nonzero stock
    correction has no surface and must reject); (c) unset linkage (no
    correction state/message linked) is the pass condition. The future
    reader MUST gate on linkage, never on a live read of this field."""
    m = _mod("dragDynamicEffector")
    cls = _cls(m, "DragDynamicEffector")
    _has(cls, "densityCorrection")
    _has(cls, "densityCorrectionStateName")
    src = (REPO / "src" / "omni_vec_integrator.cpp").read_text(
        encoding="utf-8")
    assert "dragWrench(\n                atmo.neutralDensity, C.envCdA, vRelB, dragArm, 0.0)" in src, \
        "omni C++ drag call must pin the 0.0 correction literal"


def test_facet_drag_reject_shape():
    """SPEC §2.2: FacetDragDynamicEffector exists with the facet surface the
    spec pins as REJECT (no omni per-facet drag setter)."""
    m = _mod("facetDragDynamicEffector")
    cls = _cls(m, "FacetDragDynamicEffector")
    assert callable(getattr(cls, "addFacet", None))
    for a in ("atmoDensInMsg", "windVelInMsg", "numFacets"):
        _has(cls, a)
    geo = _cls(m, "SpacecraftGeometryData")
    for a in ("facetAreas", "facetCoeffs", "facetNormals_B",
              "facetLocations_B"):
        _has(geo, a)


# --- §2.3/§2.4 atmosphere + wind ---------------------------------------------

def test_atmosphere_shape():
    """SPEC §2.3: ExponentialAtmosphere params + AtmosphereBase outbox."""
    m = _mod("exponentialAtmosphere")
    cls = _cls(m, "ExponentialAtmosphere")
    for a in ("baseDensity", "scaleHeight", "localTemp", "planetRadius",
              "envMaxReach", "envMinReach", "scStateInMsgs", "envOutMsgs",
              "planetPosInMsg", "epochInMsg"):
        _has(cls, a)
    assert callable(getattr(cls, "addSpacecraftToModel", None))
    pay = _cls(m, "AtmoPropsMsgPayload")
    _has(pay, "neutralDensity")
    _has(pay, "localTemp")
    assert _cls(_mod("msisAtmosphere"), "MsisAtmosphere") is not None
    tab = _cls(_mod("tabularAtmosphere"), "TabularAtmosphere")
    for a in ("altList", "rhoList", "tempList"):
        _has(tab, a)


def test_wind_shape():
    """SPEC §2.4: ZeroWindModel + WindBase outbox + omega/flag surface."""
    m = _mod("zeroWindModel")
    _cls(m, "ZeroWindModel")
    base = _cls(m, "WindBase")
    for a in ("scStateInMsgs", "envOutMsgs", "planetPosInMsg", "epochInMsg"):
        _has(base, a)
    for fn in ("getPlanetOmega_N", "setPlanetOmega_N",
               "getUseSpiceOmegaFlag", "setUseSpiceOmegaFlag"):
        assert callable(getattr(base, fn, None)), "host: missing " + fn
    pay = _cls(m, "WindMsgPayload")
    _has(pay, "v_air_N")
    _has(pay, "v_wind_N")


def test_mujoco_cannonball_passthrough_shape():
    """SPEC §2.5: MuJoCo CannonballDrag is plain SysModel (not dynamics) —
    out of the census path, untouched."""
    m = _mod("cannonballDrag")
    cls = _cls(m, "CannonballDrag")
    assert "DynamicEffector" not in {c.__name__ for c in cls.__mro__}
    for a in ("dragGeometryInMsg", "atmoDensInMsg", "windVelInMsg"):
        _has(cls, a)
    geo = _cls(m, "DragGeometryMsgPayload")
    for a in ("projectedArea", "dragCoeff", "r_CP_S"):
        _has(geo, a)


# --- §3 SRP ------------------------------------------------------------------

def test_srp_cannonball_shape():
    """SPEC §3.1: RadiationPressure cannonball params + links + mode fns."""
    m = _mod("radiationPressure")
    cls = _cls(m, "RadiationPressure")
    for a in ("area", "coefficientReflection", "sunEphmInMsg",
              "sunEclipseInMsg"):
        _has(cls, a)
    for fn in ("setUseCannonballModel", "setUseFacetedCPUModel"):
        assert callable(getattr(cls, fn, None)), "host: missing " + fn
    pay = _cls(m, "SpicePlanetStateMsgPayload")
    _has(pay, "PositionVector")  # full msg, never bare direction
    _has(pay, "VelocityVector")
    _has(_cls(m, "EclipseMsgPayload"), "shadowFactor")


def test_srp_facet_shape():
    """SPEC §3.2: FacetSRPDynamicEffector facet feed + sun links."""
    m = _mod("facetSRPDynamicEffector")
    cls = _cls(m, "FacetSRPDynamicEffector")
    for fn in ("setNumFacets", "setNumArticulatedFacets", "getNumFacets",
               "getNumArticulatedFacets", "addFacet", "addArticulatedFacet"):
        assert callable(getattr(cls, fn, None)), "host: missing " + fn
    for a in ("sunInMsg", "sunEclipseInMsg"):
        _has(cls, a)
    geo = _cls(m, "FacetedSRPSpacecraftGeometryData")
    for a in ("facetAreaList", "facetDcm_F0BList", "facetNHat_FList",
              "facetRotHat_FList", "facetR_CopB_BList",
              "facetDiffuseCoeffList", "facetSpecularCoeffList"):
        _has(geo, a)


def test_srp_msgdriven_reject_shape():
    """SPEC §3.3: message-driven FacetedSRPEffector + projected-area
    companion exist (pinned REJECT: occlusion state invisible to omni)."""
    m = _mod("facetedSRPEffector")
    cls = _cls(m, "FacetedSRPEffector")
    for a in ("facetElementBodyInMsgs", "facetProjectedAreaInMsgs"):
        _has(cls, a)
    body = _cls(m, "FacetElementBodyMsgPayload")
    for a in ("area", "r_CopB_B", "nHat_B", "rotHat_B", "c_diffuse",
              "c_specular"):
        _has(body, a)  # rotHat nonzero stays REJECT (§3.2 gate)
    assert _cls(_mod("facetedSpacecraftProjectedArea"),
                "FacetedSpacecraftProjectedArea") is not None


# --- §4 magnetic + MTB -------------------------------------------------------

def test_mtb_shape():
    """SPEC §4.1: MtbEffector cmd/mag/params links + Gt/clamp records."""
    m = _mod("MtbEffector")
    cls = _cls(m, "MtbEffector")
    for a in ("mtbCmdInMsg", "magInMsg", "mtbParamsInMsg", "mtbOutMsg"):
        _has(cls, a)
    _has(_cls(m, "MTBCmdMsgPayload"), "mtbDipoleCmds")
    cfg = _cls(m, "MTBArrayConfigMsgPayload")
    for a in ("numMTB", "GtMatrix_B", "maxMtbDipoles"):
        _has(cfg, a)  # non-identity Gt stays REJECT (§4.1)
    _has(_cls(m, "MTBMsgPayload"), "mtbNetTorque_B")
    _has(_cls(m, "MagneticFieldMsgPayload"), "magField_N")


def test_mag_field_shape():
    """SPEC §4.2: centered-dipole coeffs + base outbox; WMM + sensor pins."""
    m = _mod("magneticFieldCenteredDipole")
    cls = _cls(m, "MagneticFieldCenteredDipole")
    for a in ("g10", "g11", "h11", "scStateInMsgs", "envOutMsgs",
              "addSpacecraftToModel"):
        _has(cls, a)
    _has(_cls(m, "MagneticFieldMsgPayload"), "magField_N")
    wmm = _cls(_mod("magneticFieldWMM"), "MagneticFieldWMM")
    for a in ("wmmDataFullPath", "epochDateFractionalYear"):
        _has(wmm, a)  # ledger-only; vec path consumes the message
    mag = _cls(_mod("magnetometer"), "Magnetometer")
    _has(mag, "magInMsg")  # sensor: untouched, no reader (§4.2)


# --- §5 eclipse ---------------------------------------------------------------

def test_eclipse_shape():
    """SPEC §5: Eclipse links + single-field shadowFactor payload."""
    m = _mod("eclipse")
    cls = _cls(m, "Eclipse")
    for a in ("sunInMsg", "planetInMsgs", "eclipseOutMsgs"):
        _has(cls, a)
    assert callable(getattr(cls, "addSpacecraftToModel", None))
    pay = _cls(m, "EclipseMsgPayload")
    _has(pay, "shadowFactor")  # ONE field; illuminationFactor is deprecated


# --- §6 gravity / J2 / ephemeris ----------------------------------------------

def test_gravity_shape():
    """SPEC §6.1: GravityEffector census + GravBodyData per-body record."""
    m = _mod("gravityEffector")
    cls = _cls(m, "GravityEffector")
    _has(cls, "gravBodies")
    for fn in ("setGravBodies", "addGravBody"):
        assert callable(getattr(cls, fn, None)), "host: missing " + fn
    body = _cls(m, "GravBodyData")
    for a in ("isCentralBody", "mu", "gravityModel", "planetBodyInMsg"):
        _has(body, a)
    sc = _cls(_mod("spacecraft"), "Spacecraft")
    _has(sc, "gravField")


def test_j2_shape():
    """SPEC §6.2: spherical-harmonics J2 surface (maxDeg==2 gate)."""
    m = _mod("sphericalHarmonicsGravityModel")
    cls = _cls(m, "SphericalHarmonicsGravityModel")
    for a in ("radEquator", "muBody", "maxDeg", "cBar", "sBar"):
        _has(cls, a)
    assert callable(getattr(cls, "loadFromFile", None))
    _has(_cls(_mod("pointMassGravityModel"), "PointMassGravityModel"),
         "muBody")  # default law: no action


def test_ephemeris_shape():
    """SPEC §6.3: ephemeris providers stay in task; reader only forwards
    the already-linked planet message."""
    sp = _cls(_mod("spiceInterface"), "SpiceInterface")
    assert callable(getattr(sp, "addPlanetNames", None))
    _has(sp, "planetStateOutMsgs")
    pay = _cls(_mod("spiceInterface"), "SpicePlanetStateMsgPayload")
    _has(pay, "PositionVector")
    _has(pay, "VelocityVector")
    pe = _cls(_mod("planetEphemeris"), "PlanetEphemeris")
    assert callable(getattr(pe, "setPlanetNames", None))
    _has(pe, "planetOutMsgs")
    ec = _cls(_mod("ephemerisConverter"), "EphemerisConverter")
    assert callable(getattr(ec, "addSpiceInputMsg", None))
    _has(ec, "ephemOutMsgs")


# --- §1.6 omni setter surface (wheel-gated; skips without build) --------------

_OMNI_SETTERS = ("addEnvironmentBody", "setJ2", "setDragParams",
                 "addSRPFacet", "setSRPCannonball", "setSunState",
                 "setMagneticMoment", "setMagneticDipole", "setMaxMtbDipole")
_OMNI_LINKS = ("atmoDensInMsg", "windVelInMsg", "eclipseInMsg",
               "sunStateInMsg", "moonStateInMsg", "magFieldInMsg")


def test_omni_env_setter_surface():
    """SPEC §1.6: every target setter + message member exists on the built
    OmniVecIntegrator (skips where the wheel is unbuilt)."""
    oh = pytest.importorskip("omniheyoka.omni_heyoka",
                             reason="host-only: needs built omni_heyoka")
    oi = getattr(oh, "OmniVecIntegrator", None)
    if oi is None:
        pytest.skip("host: OmniVecIntegrator missing")
    for name in _OMNI_SETTERS + tuple(s + "ForObject" for s in _OMNI_SETTERS):
        assert callable(getattr(oi, name, None)), "host: missing " + name
    for name in _OMNI_LINKS:
        assert hasattr(oi, name), "host: missing link member " + name
