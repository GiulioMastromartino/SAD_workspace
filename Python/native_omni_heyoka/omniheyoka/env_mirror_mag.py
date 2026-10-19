"""STAGED magnetic/MTB stock-mirror reader (PLAN_ENV_MIRROR.md §4).

STATUS: STAGED -- landed but NOT wired. Nothing imports this module yet,
``omniheyoka/swap.py`` is untouched (no ``_mirror_mag`` call sites, no
registry change), and ``tests/test_replacement_matrix.py::
test_env_mirror_gap_open`` stays GREEN by design. The PLAN §8 wiring diff
adds the call sites inside ``mirror_stock_config`` / ``_mirror_single_object``
and flips that gap test red in the SAME diff.

Host-free by construction. Every stock surface below was verified by grep
against the Basilisk wrappers (cited in-line), every omni setter against
``include/omni/vec_integrator.hpp`` (cited in-line), and the C++ consumption
rules against ``src/omni_vec_integrator.cpp``. Module-level imports are
stdlib-only; Basilisk is never imported here -- the reader consumes plain
objects (stock effectors/models on the host, fakes in the host-free tests).

Mirrored (PLAN §4):
  * ``MtbEffector`` -- iff single-bar identity-Gt + a pre-written (LATCHED)
    dipole command. ``mtbParamsInMsg`` (``numMTB``/``GtMatrix_B``/
    ``maxMtbDipoles``) gates the identity map and feeds ``setMaxMtbDipole``;
    ``mtbCmdInMsg`` (``mtbDipoleCmds``) feeds ``setMagneticMoment`` from the
    mirror-time snapshot; ``magInMsg`` must be linked (stock bskError
    otherwise) and its per-tick field reaches omni through the FIELD MODEL's
    ``envOutMsgs[0]`` (``oi.magFieldInMsg`` / ``subscribeMagForObject``).
  * ``MagneticFieldCenteredDipole`` -- ``g10``/``g11``/``h11`` verbatim to
    ``setMagneticDipole``; ``envOutMsgs[0]`` to the omni mag link (REQUIRED,
    PLAN §1.5/§4.2).

Loud rejects (ValueError, ``mirror_stock_config:`` discipline, PLAN §1.2):
non-identity Gt (multi-bar, mixed, or non-axis single bar), unlinked or
not-pre-written dipole command (time-varying driver), zero dipole with mag
on, WMM coefficient recording (message-only, no model), magnetometer-as-
actuator confusion, unknown classes, ambiguous multiplicities, and an MTB
with no supported field model to feed the mag link.

LATCHED-DIPOLE LIMITATION (load-bearing; documented, NOT solved). Omni has
no per-tick dipole input: the mag cluster is torque-only
(``src/omni_vec_integrator.cpp:1252-1274`` computes tau = m x B_B from the
STAGED ``magM``), so the dipole is a MIRROR-TIME SNAPSHOT. A post-mirror
command rewrite is CONTRADICTORY INPUT -- the same class of trap as the
thruster reset-vs-static rule (``swap.py:422-437``) but WITHOUT the
thrusters' continuous re-subscribe safety net. The reader rejects everything
it can see (command message not pre-written at mirror time => time-varying
driver); the undetectable case (a driver that writes once pre-init, then
rewrites per tick) stays a CALLER OBLIGATION, flagged by ``latched_dipole``
in the returned record.

Future swap wiring (PLAN §8.3 sketch ``_mirror_mag(mtb_models,
magfield_models, oi, ...)``) adapts to this surface as
``mirror_mag_models(list(mtb_models) + list(magfield_models), oi,
object_index)``; ``object_index=None`` is the legacy slot-0 shorthand
(same convention as the swap per-object callers, e.g. ``_mirror_wheels``
``swap.py:199-253``), any int selects the ``...ForObject`` twins.
"""

from __future__ import annotations

import math
from typing import Any

__all__ = ["SUPPORTED_CLASSES", "mirror_mag_models"]

# The only stock mag/MTB classes this reader mirrors (PLAN §4.1 + §4.2).
SUPPORTED_CLASSES = {"MtbEffector", "MagneticFieldCenteredDipole"}

# Single positive unit basis vector = the "identity-Gt" one-bar map: the stock
# body dipole is exactly m = cmd[0] * e_k, so the omni |m| <= maxMtbDipole
# clamp (src/omni_vec_integrator.cpp:1268-1270) is bit-identical to the stock
# per-bar |cmd| <= maxMtbDipoles[0] clamp. Anything else is a map omni has no
# surface for (mtbGt_ is identity on BOTH paths: spacecraft.hpp:335 and the
# setter at spacecraft.hpp:199 rejects non-identity).
_IDENTITY_BAR_AXES = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

# Fixed payload shapes (wrapper dtypes): GtMatrix_B is the 3x36 row-major
# alignment buffer, shape (108,) -- one bar occupies the first 3 entries and
# the remaining 105 must be zero; the per-bar command / clamp vectors are
# 36 long. One bar's axis is flat[0:3] under either row/col reading of the
# leading minor, so the identity gate is layout-robust.
_GT_LEN = 108
_CMD_LEN = 36
_DMAX_LEN = 36


def mirror_mag_models(models: Any, oi: Any,
                      object_index: Any = None) -> dict:
    """Mirror MTB + centered-dipole stock config into ``oi``.

    ``models`` is the mag/MTB slice of the task census (class-name keyed, per
    PLAN §7.1 -- never MRO/string reads). ``object_index=None`` stages the
    legacy slot-0 surface (``oi.setMagneticMoment`` / ``setMaxMtbDipole`` /
    ``setMagneticDipole`` + ``oi.magFieldInMsg.subscribeTo``); any int stages
    the ``...ForObject`` twins for that slot alone (only it is marked
    mirrored+dirty; links never mark a slot, vec_integrator.hpp:202-204).

    Returns the ledger record: counts, the latched dipole snapshot
    (``magnetic_moment``, ``max_mtb_dipole``, ``latched_dipole``), the
    recorded dipole coefficients (``dipole_gh``), and the mag-link wiring
    (``mag_msg_wired`` / ``mag_link`` / ``mag_link_src``).
    """
    if models is None:
        models = []
    mtb_models = []
    dipole_models = []
    for m in list(models):
        cls = type(m).__name__
        if cls == "MtbEffector":
            mtb_models.append(m)
        elif cls == "MagneticFieldCenteredDipole":
            dipole_models.append(m)
        else:
            _reject_unknown_mag_model(cls)
    if len(mtb_models) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(mtb_models)} MtbEffector task models; "
            f"stock registration order is ambiguous -- use one effector "
            f"(one dipole command surface, PLAN §4.1).")
    if len(dipole_models) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(dipole_models)} "
            f"MagneticFieldCenteredDipole task models; envOutMsgs[0] "
            f"attribution is ambiguous -- use one field model (PLAN §7.4 "
            f"out-message vectors, §4.2).")

    rec = {
        "object_index": 0 if object_index is None else int(object_index),
        "n_mtb": 0,
        "n_magfield": 0,
        "magnetic_moment": None,
        "max_mtb_dipole": None,
        "dipole_gh": None,
        "mag_msg_wired": False,
        "mag_link": None,
        "mag_link_src": None,
        "latched_dipole": False,
    }

    for eff in mtb_models:
        mtb_rec = _mirror_mtb(eff, oi, object_index)
        rec["n_mtb"] = 1
        rec["magnetic_moment"] = mtb_rec["magnetic_moment"]
        rec["max_mtb_dipole"] = mtb_rec["max_mtb_dipole"]
        rec["latched_dipole"] = True

    mag_src = None
    for mdl in dipole_models:
        gh = _read_dipole_coeffs(mdl)
        if object_index is None:
            oi.setMagneticDipole(gh[0], gh[1], gh[2])
        else:
            oi.setMagneticDipoleForObject(object_index, gh[0], gh[1], gh[2])
        mag_src = _mag_source(mdl)
        rec["n_magfield"] = 1
        rec["dipole_gh"] = gh

    if mag_src is None:
        if rec["n_mtb"]:
            raise ValueError(
                "mirror_stock_config: MtbEffector mirrored but no supported "
                "magnetic-field model in `models` to feed oi.magFieldInMsg "
                "(mag link REQUIRED, PLAN §1.5: mag on + unlinked = "
                "bskError, src/omni_vec_integrator.cpp:1255-1258); pass the "
                "MagneticFieldCenteredDipole task model (its envOutMsgs[0] "
                "is the link source).")
    else:
        rec["mag_link"] = _wire_mag_link(oi, mag_src, object_index)
        rec["mag_msg_wired"] = True
        rec["mag_link_src"] = mag_src
    return rec


def _reject_unknown_mag_model(cls: str) -> None:
    """Loud reject for any non-supported class in the mag/MTB census slice."""
    if cls == "MagneticFieldWMM":
        raise ValueError(
            f"mirror_stock_config: {cls}: WMM-coeff recording rejected -- "
            f"MagneticFieldWMM is message-only (no model surface here; "
            f"setMagneticDipole is centered-dipole-exclusive, PLAN §4.2). "
            f"Its envOutMsgs[0] per-tick wire belongs to the field-module "
            f"reader; SUPPORTED_CLASSES = "
            f"{sorted(SUPPORTED_CLASSES)}.")
    if cls == "Magnetometer":
        raise ValueError(
            f"mirror_stock_config: {cls}: magnetometer-as-actuator confusion "
            f"-- Magnetometer is a SENSOR (magInMsg/dcm_SB/tamTrue_S), never "
            f"a dipole source; no magnetic torque may be mirrored from it "
            f"(PLAN §4.2: untouched, no reader).")
    raise ValueError(
        f"mirror_stock_config: task model {cls} is outside the mag/MTB "
        f"mirror surface; SUPPORTED_CLASSES = {sorted(SUPPORTED_CLASSES)} "
        f"(PLAN §4).")


def _mirror_mtb(eff: Any, oi: Any, object_index: Any) -> dict:
    """Mirror one MtbEffector: identity-Gt gate + latched dipole + clamp.

    Reads (all names verified, Basilisk/simulation/MtbEffector.py):
    ``mtbCmdInMsg`` :620, ``magInMsg`` :621, ``mtbParamsInMsg`` :622;
    ``MTBCmdMsgPayload.mtbDipoleCmds`` :630 (per-bar scalars, shape (36,));
    ``MTBArrayConfigMsgPayload.numMTB`` :652 / ``GtMatrix_B`` :653 /
    ``maxMtbDipoles`` :654 (Gt row-major 3x36, shape (108,)).
    """
    cmd = _functor(eff, "mtbCmdInMsg")
    mag = _functor(eff, "magInMsg")
    params = _functor(eff, "mtbParamsInMsg")
    _require_linked(cmd, "mtbCmdInMsg")
    _require_linked(mag, "magInMsg")
    _require_linked(params, "mtbParamsInMsg")
    _require_written(params, "mtbParamsInMsg",
                     "cannot verify single-bar identity Gt")
    _require_written(cmd, "mtbCmdInMsg")

    p = _payload(params, "mtbParamsInMsg")
    c = _payload(cmd, "mtbCmdInMsg")
    try:
        num_mtb = int(p.numMTB)
    except Exception as e:
        raise ValueError(
            f"mirror_stock_config: cannot read mtbParamsInMsg numMTB "
            f"({e!r}).")
    gt = _flat(p.GtMatrix_B, "GtMatrix_B", _GT_LEN)
    dmax = _flat(p.maxMtbDipoles, "maxMtbDipoles", _DMAX_LEN)
    cmds = _flat(c.mtbDipoleCmds, "mtbDipoleCmds", _CMD_LEN)

    axis = (gt[0], gt[1], gt[2])
    if (num_mtb != 1 or axis not in _IDENTITY_BAR_AXES
            or any(x != 0.0 for x in gt[3:])):
        raise ValueError(
            f"mirror_stock_config: MtbEffector numMTB={num_mtb}, "
            f"GtMatrix_B[0:3]={list(axis)}: non-identity Gt unmirrorable -- "
            f"single-bar axis-aligned identity-Gt only (PLAN §4.1; omni has "
            f"no Gt surface, mtbGt_ is identity on both paths, "
            f"spacecraft.hpp:199/:335).")
    dmax0 = dmax[0]
    if not math.isfinite(dmax0) or not dmax0 > 0.0:
        raise ValueError(
            f"mirror_stock_config: MtbEffector maxMtbDipoles[0]={dmax0!r}; "
            f"setMaxMtbDipole would bskError (must be finite positive, "
            f"src/omni_vec_integrator.cpp:326-330).")
    c0 = cmds[0]
    m = [axis[i] * c0 for i in range(3)]
    if m[0] == 0.0 and m[1] == 0.0 and m[2] == 0.0:
        raise ValueError(
            f"mirror_stock_config: MtbEffector latched dipole {m} is zero "
            f"with mag on; stock and omni both bskError (PLAN §1.5, "
            f"src/omni_vec_integrator.cpp:1259-1262).")
    if object_index is None:
        oi.setMaxMtbDipole(dmax0)
        oi.setMagneticMoment(m[0], m[1], m[2])
    else:
        oi.setMaxMtbDipoleForObject(object_index, dmax0)
        oi.setMagneticMomentForObject(object_index, m[0], m[1], m[2])
    return {"magnetic_moment": m, "max_mtb_dipole": dmax0}


def _read_dipole_coeffs(mdl: Any) -> list:
    """Verbatim g10/g11/h11 (magneticFieldCenteredDipole.py:386-388)."""
    out = []
    for name in ("g10", "g11", "h11"):
        try:
            out.append(float(getattr(mdl, name)))
        except Exception as e:
            raise ValueError(
                f"mirror_stock_config: cannot read MagneticFieldCentered"
                f"Dipole.{name} ({e!r}).")
    return out


def _mag_source(mdl: Any):
    """The field model's out-message vector head (envOutMsgs[0], §7.4)."""
    try:
        outs = list(getattr(mdl, "envOutMsgs", []) or [])
    except Exception as e:
        raise ValueError(
            f"mirror_stock_config: cannot read MagneticFieldCenteredDipole "
            f"envOutMsgs ({e!r}).")
    if not outs:
        raise ValueError(
            f"mirror_stock_config: MagneticFieldCenteredDipole has no "
            f"envOutMsgs; cannot wire the omni mag link (REQUIRED, PLAN "
            f"§4.2: envOutMsgs[0] -> subscribeMag).")
    return outs[0]


def _wire_mag_link(oi: Any, src: Any, object_index: Any) -> str:
    """Feed the field message into oi (link only; never marks a slot).

    Slot 0 has no ``subscribeMag`` method -- verified absent from
    ``include/omni/vec_integrator.hpp``; the legacy surface is the
    ``magFieldInMsg`` ReadFunctor (hpp:244) with ``subscribeTo``, the same
    pattern as the thruster command link (swap.py:419). The per-object path
    uses ``subscribeMagForObject`` (hpp:217-218). No deferral needed.
    """
    if object_index is None:
        functor = getattr(oi, "magFieldInMsg", None)
        if functor is None:
            raise ValueError(
                "mirror_stock_config: oi has no magFieldInMsg member; cannot "
                "wire the mag link (expected vec_integrator.hpp:244).")
        functor.subscribeTo(src)
        return "magFieldInMsg.subscribeTo"
    subscribe = getattr(oi, "subscribeMagForObject", None)
    if not callable(subscribe):
        raise ValueError(
            "mirror_stock_config: oi has no subscribeMagForObject; cannot "
            "wire the mag link (expected vec_integrator.hpp:217).")
    subscribe(object_index, src)
    return "subscribeMagForObject"


def _functor(eff: Any, name: str):
    """Guarded getattr for a stock message functor (shape read, §7.1)."""
    try:
        return getattr(eff, name)
    except Exception as e:
        raise ValueError(
            f"mirror_stock_config: MtbEffector {name} unreadable ({e!r}).")


def _require_linked(functor: Any, name: str) -> None:
    """Linkage gate (blessed pattern, PLAN §2.1 revised note): stock
    bskErrors on unlinked MTB cmd/mag/params, so an unlinked functor is a
    loud reject, never a default."""
    try:
        linked = bool(functor.isLinked())
    except Exception as e:
        raise ValueError(
            f"mirror_stock_config: cannot read MtbEffector {name}.isLinked() "
            f"({e!r}); linkage gates are mandatory.")
    if not linked:
        raise ValueError(
            f"mirror_stock_config: MtbEffector {name} unlinked; stock "
            f"bskErrors on the MTB cmd/mag/params chain (PLAN §1.5, "
            f"PLAN_ENVELOPE.md:35).")


def _require_written(functor: Any, name: str, why: str = "") -> None:
    """Mirror-time pre-write gate. A command/params message that has never
    been written at mirror time can only be fed by a per-tick driver -- for
    the dipole that is a time-varying driver (contradictory input: omni has
    no per-tick dipole surface, PLAN §4.1)."""
    try:
        written = bool(functor.isWritten())
    except Exception as e:
        raise ValueError(
            f"mirror_stock_config: cannot read MtbEffector {name}.isWritten() "
            f"({e!r}); the pre-write gate is mandatory.")
    if not written:
        raise ValueError(
            f"mirror_stock_config: MtbEffector {name} not pre-written at "
            f"mirror time{' (' + why + ')' if why else ''}; a time-varying "
            f"dipole driver is contradictory input (latched snapshot only, "
            f"PLAN §4.1 -- no per-tick dipole surface in omni).")


def _payload(functor: Any, name: str):
    """Mirror-time payload snapshot through the stock reader functor."""
    try:
        return functor()
    except Exception as e:
        raise ValueError(
            f"mirror_stock_config: cannot read MtbEffector {name} payload at "
            f"mirror time ({e!r}).")


def _flat(v: Any, name: str, n: int) -> list:
    """Read a fixed-length numeric payload vector (numpy-free)."""
    try:
        vals = [float(x) for x in list(v)]
    except Exception as e:
        raise ValueError(
            f"mirror_stock_config: cannot read {name} as a flat float list "
            f"({e!r}).")
    if len(vals) != n:
        raise ValueError(
            f"mirror_stock_config: {name} has {len(vals)} entries, expected "
            f"the fixed payload length {n} (wrapper dtype).")
    return vals
