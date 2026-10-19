"""Host-free behavioral pins for the STAGED mag/MTB mirror reader
(``omniheyoka/env_mirror_mag.py``, PLAN_ENV_MIRROR.md §4).

Bare-pytest runnable: stdlib only, NO Basilisk, NO wheel, NO numpy (the
reader is numpy-free by construction). Stock objects are fakes
(SimpleNamespace payloads + linkage-gated functors + MRO-named classes),
because the reader keys off ``type(model).__name__`` and message-functor
surfaces only (PLAN §7.1). The module is loaded from its file path, so the
test never imports the ``omniheyoka`` package.

STAGED status pin: this suite exercises the reader in isolation; the reader
is NOT wired into ``omniheyoka/swap.py``, so
``tests/test_replacement_matrix.py::test_env_mirror_gap_open`` stays green
by design (the wiring diff flips it red in the SAME diff).

Each reject test names the envelope cell it pins. If a rejection is ever
relaxed into a model, the corresponding test goes red -- move the cell to
the parity suite, do not delete.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]


def _load_mag():
    path = REPO / "omniheyoka" / "env_mirror_mag.py"
    spec = importlib.util.spec_from_file_location("env_mirror_mag_under_test",
                                                  path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


mag = _load_mag()


def _fake(cls_name, **attrs):
    """Instance whose class name is ``cls_name`` (the reader's only key)."""
    inst = type(cls_name, (), {})()
    for k, v in attrs.items():
        setattr(inst, k, v)
    return inst


class _Functor:
    """Stock ReadFunctor fake: linkage/pre-write gates + payload snapshot."""

    def __init__(self, payload=None, linked=True, written=True):
        self.payload = payload
        self.linked = linked
        self.written = written

    def isLinked(self):
        return self.linked

    def isWritten(self):
        return self.written

    def __call__(self):
        if self.payload is None:
            raise RuntimeError("functor read without payload")
        return self.payload


def _pad(vals, n):
    out = [float(v) for v in vals]
    return out + [0.0] * (n - len(out))


def _params(num_mtb=1, gt=(1.0, 0.0, 0.0), dmax=(0.8,),
            linked=True, written=True):
    return _Functor(SimpleNamespace(numMTB=num_mtb,
                                    GtMatrix_B=_pad(gt, 108),
                                    maxMtbDipoles=_pad(dmax, 36)),
                    linked=linked, written=written)


def _cmd(cmds=(0.5,), linked=True, written=True):
    return _Functor(SimpleNamespace(mtbDipoleCmds=_pad(cmds, 36)),
                    linked=linked, written=written)


def _mag_link(linked=True):
    return _Functor(SimpleNamespace(magField_N=[0.0, 0.0, 0.0]),
                    linked=linked)


def _mtb(cmd=None, params=None, maglink=None):
    return _fake("MtbEffector",
                 mtbCmdInMsg=cmd if cmd is not None else _cmd(),
                 magInMsg=maglink if maglink is not None else _mag_link(),
                 mtbParamsInMsg=params if params is not None else _params())


def _dipole(g10=-29404.8, g11=-1450.9, h11=4652.9, n_out=1):
    return _fake("MagneticFieldCenteredDipole", g10=g10, g11=g11, h11=h11,
                 envOutMsgs=[object() for _ in range(n_out)])


class _MagLinkStub:
    def __init__(self):
        self.subscribed = []

    def subscribeTo(self, src):
        self.subscribed.append(src)


class _OiStub:
    """Records oi.* calls. ``block`` hides named surfaces so the
    missing-surface rejects can be exercised (lookup then raises)."""

    def __init__(self, block=()):
        self._block = set(block)
        self.calls = []
        self.magFieldInMsg = _MagLinkStub()

    def __getattribute__(self, name):
        if name != "_block" and name in object.__getattribute__(
                self, "_block"):
            raise AttributeError(f"{name} blocked in this stub")
        return object.__getattribute__(self, name)

    def setMaxMtbDipole(self, dmax):
        self.calls.append(("setMaxMtbDipole", dmax))

    def setMagneticMoment(self, mx, my, mz):
        self.calls.append(("setMagneticMoment", mx, my, mz))

    def setMagneticDipole(self, g10, g11, h11):
        self.calls.append(("setMagneticDipole", g10, g11, h11))

    def setMaxMtbDipoleForObject(self, obj, dmax):
        self.calls.append(("setMaxMtbDipoleForObject", obj, dmax))

    def setMagneticMomentForObject(self, obj, mx, my, mz):
        self.calls.append(("setMagneticMomentForObject", obj, mx, my, mz))

    def setMagneticDipoleForObject(self, obj, g10, g11, h11):
        self.calls.append(("setMagneticDipoleForObject", obj, g10, g11, h11))

    def subscribeMagForObject(self, obj, src):
        self.calls.append(("subscribeMagForObject", obj, src))


# --------------------------------------------------------------------------
# Positive read paths (exact setter args, host-free)
# --------------------------------------------------------------------------

def test_supported_classes_exact():
    assert mag.SUPPORTED_CLASSES == {"MtbEffector",
                                     "MagneticFieldCenteredDipole"}


def test_mtb_identity_gt_mirror_exact_setter_args():
    eff = _mtb(cmd=_cmd(cmds=(0.5,)),
               params=_params(gt=(1.0, 0.0, 0.0), dmax=(0.8,)))
    mdl = _dipole()
    oi = _OiStub()
    rec = mag.mirror_mag_models([eff, mdl], oi)
    assert oi.calls == [
        ("setMaxMtbDipole", 0.8),
        ("setMagneticMoment", 0.5, 0.0, 0.0),
        ("setMagneticDipole", -29404.8, -1450.9, 4652.9),
    ]
    assert oi.magFieldInMsg.subscribed == [mdl.envOutMsgs[0]]
    assert rec["n_mtb"] == 1 and rec["n_magfield"] == 1
    assert rec["magnetic_moment"] == [0.5, 0.0, 0.0]
    assert rec["max_mtb_dipole"] == 0.8
    assert rec["dipole_gh"] == [-29404.8, -1450.9, 4652.9]
    assert rec["mag_msg_wired"] is True
    assert rec["mag_link"] == "magFieldInMsg.subscribeTo"
    assert rec["mag_link_src"] is mdl.envOutMsgs[0]
    assert rec["latched_dipole"] is True
    assert rec["object_index"] == 0


@pytest.mark.parametrize("axis,cmd0,m", [
    ((0.0, 1.0, 0.0), 0.3, [0.0, 0.3, 0.0]),
    ((0.0, 0.0, 1.0), -0.25, [0.0, 0.0, -0.25]),
])
def test_mtb_identity_gt_axis_variants(axis, cmd0, m):
    eff = _mtb(cmd=_cmd(cmds=(cmd0,)), params=_params(gt=axis))
    oi = _OiStub()
    rec = mag.mirror_mag_models([eff, _dipole()], oi)
    assert ("setMagneticMoment", m[0], m[1], m[2]) in oi.calls
    assert rec["magnetic_moment"] == m


def test_dipole_only_no_mtb():
    mdl = _dipole()
    oi = _OiStub()
    rec = mag.mirror_mag_models([mdl], oi)
    assert oi.calls == [("setMagneticDipole", -29404.8, -1450.9, 4652.9)]
    assert oi.magFieldInMsg.subscribed == [mdl.envOutMsgs[0]]
    assert rec["n_mtb"] == 0 and rec["n_magfield"] == 1
    assert rec["magnetic_moment"] is None
    assert rec["max_mtb_dipole"] is None
    assert rec["latched_dipole"] is False


def test_empty_models_zero_record():
    oi = _OiStub()
    rec = mag.mirror_mag_models([], oi)
    assert oi.calls == [] and oi.magFieldInMsg.subscribed == []
    assert rec["n_mtb"] == 0 and rec["n_magfield"] == 0
    assert rec["mag_msg_wired"] is False and rec["mag_link"] is None
    assert rec["dipole_gh"] is None


# --------------------------------------------------------------------------
# Per-object routing (ForObject twins)
# --------------------------------------------------------------------------

def test_forobject_routing():
    eff = _mtb()
    mdl = _dipole()
    oi = _OiStub()
    rec = mag.mirror_mag_models([eff, mdl], oi, object_index=3)
    assert oi.calls == [
        ("setMaxMtbDipoleForObject", 3, 0.8),
        ("setMagneticMomentForObject", 3, 0.5, 0.0, 0.0),
        ("setMagneticDipoleForObject", 3, -29404.8, -1450.9, 4652.9),
        ("subscribeMagForObject", 3, mdl.envOutMsgs[0]),
    ]
    assert oi.magFieldInMsg.subscribed == []
    assert rec["object_index"] == 3
    assert rec["mag_link"] == "subscribeMagForObject"


def test_object_index_zero_still_uses_forobject_twins():
    """Explicit 0 is the per-object path (WP25-A4), not the legacy slot-0
    shorthand -- never route slot 0 through both surfaces."""
    oi = _OiStub()
    mag.mirror_mag_models([_mtb(), _dipole()], oi, object_index=0)
    assert oi.calls[0] == ("setMaxMtbDipoleForObject", 0, 0.8)
    assert oi.calls[1] == ("setMagneticMomentForObject", 0, 0.5, 0.0, 0.0)
    assert oi.calls[3] == ("subscribeMagForObject", 0, oi.calls[3][2])


# --------------------------------------------------------------------------
# Loud rejects (each raises ValueError pre-mirror)
# --------------------------------------------------------------------------

def test_reject_non_identity_gt_multi_bar():
    eff = _mtb(params=_params(num_mtb=2,
                              gt=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0)))
    with pytest.raises(ValueError, match="non-identity Gt"):
        mag.mirror_mag_models([eff, _dipole()], _OiStub())


def test_reject_non_identity_gt_mixed_bar():
    eff = _mtb(params=_params(gt=(0.0, 0.6, 0.8)))
    with pytest.raises(ValueError, match="non-identity Gt"):
        mag.mirror_mag_models([eff, _dipole()], _OiStub())


def test_reject_three_bar_identity_stays_reject():
    """Literal 3x3-identity Gt (3 bars) stays REJECT: the binding is
    single-bar (PLAN §4.1) and multi-bar clamps diverge (stock per-bar
    |cmd| vs omni |m| <= maxMtbDipole)."""
    eff = _mtb(params=_params(num_mtb=3,
                              gt=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                  1.0)))
    with pytest.raises(ValueError, match="non-identity Gt"):
        mag.mirror_mag_models([eff, _dipole()], _OiStub())


def test_reject_num_mtb_zero():
    eff = _mtb(params=_params(num_mtb=0, gt=(0.0, 0.0, 0.0)))
    with pytest.raises(ValueError, match="non-identity Gt"):
        mag.mirror_mag_models([eff, _dipole()], _OiStub())


def test_reject_mtb_cmd_unlinked():
    with pytest.raises(ValueError, match="mtbCmdInMsg unlinked"):
        mag.mirror_mag_models([_mtb(cmd=_cmd(linked=False)), _dipole()],
                              _OiStub())


def test_reject_mtb_mag_unlinked():
    with pytest.raises(ValueError, match="magInMsg unlinked"):
        mag.mirror_mag_models([_mtb(maglink=_mag_link(linked=False)),
                               _dipole()], _OiStub())


def test_reject_mtb_params_unlinked():
    with pytest.raises(ValueError, match="mtbParamsInMsg unlinked"):
        mag.mirror_mag_models([_mtb(params=_params(linked=False)),
                               _dipole()], _OiStub())


def test_reject_params_not_prewritten():
    with pytest.raises(ValueError,
                       match="cannot verify single-bar identity Gt"):
        mag.mirror_mag_models([_mtb(params=_params(written=False)),
                               _dipole()], _OiStub())


def test_reject_cmd_not_prewritten_time_varying_driver():
    with pytest.raises(ValueError, match="time-varying dipole driver"):
        mag.mirror_mag_models([_mtb(cmd=_cmd(written=False)), _dipole()],
                              _OiStub())


def test_reject_zero_dipole_with_mag_on():
    with pytest.raises(ValueError, match="dipole.*is zero"):
        mag.mirror_mag_models([_mtb(cmd=_cmd(cmds=(0.0,))), _dipole()],
                              _OiStub())


def test_reject_nonpositive_max_dipole():
    with pytest.raises(ValueError, match="maxMtbDipoles"):
        mag.mirror_mag_models([_mtb(params=_params(dmax=(0.0,))), _dipole()],
                              _OiStub())


def test_reject_mtb_without_field_model_for_mag_link():
    with pytest.raises(ValueError, match="mag link REQUIRED"):
        mag.mirror_mag_models([_mtb()], _OiStub())


def test_reject_wmm_coeff_recording():
    wmm = _fake("MagneticFieldWMM", epochDateFractionalYear=2025.0,
                wmmDataFullPath="/tmp/WMM.COF")
    with pytest.raises(ValueError, match="WMM-coeff recording"):
        mag.mirror_mag_models([_mtb(), wmm], _OiStub())
    with pytest.raises(ValueError, match="message-only"):
        mag.mirror_mag_models([wmm], _OiStub())
    with pytest.raises(ValueError, match="no model"):
        mag.mirror_mag_models([wmm], _OiStub())


def test_reject_magnetometer_as_actuator():
    mm = _fake("Magnetometer", magInMsg=None, dcm_SB=None, tamTrue_S=None)
    with pytest.raises(ValueError, match="magnetometer-as-actuator"):
        mag.mirror_mag_models([mm], _OiStub())


def test_reject_unknown_mag_model():
    with pytest.raises(ValueError, match="SUPPORTED_CLASSES"):
        mag.mirror_mag_models([_fake("MagneticFieldNoise")], _OiStub())


def test_reject_two_mtb_effectors():
    with pytest.raises(ValueError, match="registration order is ambiguous"):
        mag.mirror_mag_models([_mtb(), _mtb(), _dipole()], _OiStub())


def test_reject_two_dipole_models():
    with pytest.raises(ValueError, match="envOutMsgs"):
        mag.mirror_mag_models([_dipole(), _dipole()], _OiStub())


def test_reject_dipole_without_env_out_msgs():
    with pytest.raises(ValueError, match="envOutMsgs"):
        mag.mirror_mag_models([_dipole(n_out=0)], _OiStub())


def test_reject_missing_slot0_magfunctor():
    oi = _OiStub(block=("magFieldInMsg",))
    with pytest.raises(ValueError, match="magFieldInMsg"):
        mag.mirror_mag_models([_dipole()], oi)


def test_reject_missing_subscribe_mag_for_object():
    oi = _OiStub(block=("subscribeMagForObject",))
    with pytest.raises(ValueError, match="subscribeMagForObject"):
        mag.mirror_mag_models([_dipole()], oi, object_index=1)


def test_reject_unreadable_params_payload():
    eff = _mtb(params=_Functor(None))
    with pytest.raises(ValueError, match="mtbParamsInMsg payload"):
        mag.mirror_mag_models([eff, _dipole()], _OiStub())
