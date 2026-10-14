"""WP-E (multi-tank fuel depletion): Phase-1 verdict NO-GO, loud-reject retained.

Phase-1 subset-visibility probe (host, 2026-10-14) found the per-tank
thruster-subset links UNOBSERVABLE from Python:

- ``FuelTank.thrDynEffectors`` / ``thrStateEffectors`` are opaque
  ``SwigPyObject`` handles: no ``len`` / iteration / ``size`` / ``getitem``.
- ``ThrusterDynamicEffector`` exposes no tank back-reference (full attr
  census: ``mDotTotal``/``fuelMass`` are computed doubles, not links).
- ``FuelTank`` offers ``addThrusterSet`` but no getter for the linked sets.

Per-tank ``getNameOfMassState`` / ``getR_TB_B`` / ``getFuelLeakRate`` ARE
independent per instance, but without the subset mapping no correct mdot
attribution is possible, so ``_mirror_fuel`` keeps raising on >1 tank
("per-tank thruster subsets unmapped"). These tests pin that discipline:
if a future SDK ever exposes the links, the observability test below
fails loudly and WP-E can be re-opened.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SWAP_PY = REPO / "omniheyoka" / "swap.py"


def _load_swap():
    spec = importlib.util.spec_from_file_location(
        "omniheyoka_swap_wpe", str(SWAP_PY))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _two_tanks():
    ft = pytest.importorskip("Basilisk.simulation.fuelTank")
    t1 = ft.FuelTank()
    t2 = ft.FuelTank()
    t1.setNameOfMassState("tankA")
    t2.setNameOfMassState("tankB")
    t1.setR_TB_B([1.0, 0.0, 0.0])
    t2.setR_TB_B([0.0, 2.0, 0.0])
    t1.setFuelLeakRate(1e-6)
    t2.setFuelLeakRate(2e-6)
    return t1, t2


def test_multitank_mirror_raises():
    """Two stock tanks must raise (subsets unmapped), never model one."""
    swap = _load_swap()
    t1, t2 = _two_tanks()
    with pytest.raises(ValueError, match="exactly one"):
        swap._mirror_fuel([t1, t2], None, [200.0])


def test_subset_links_unobservable():
    """Pin the Phase-1 negative evidence: no host-side subset enumeration."""
    t1, _ = _two_tanks()
    for attr in ("thrDynEffectors", "thrStateEffectors"):
        vec = getattr(t1, attr)
        with pytest.raises(TypeError):
            len(vec)
        with pytest.raises(TypeError):
            iter(vec)
        with pytest.raises(TypeError):
            vec[0]
        assert not hasattr(vec, "size")


def test_per_tank_getters_independent():
    """Name / r_TB / leak read back per instance (mapping is the blocker)."""
    t1, t2 = _two_tanks()
    assert str(t1.getNameOfMassState()) == "tankA"
    assert str(t2.getNameOfMassState()) == "tankB"
    assert [r[0] for r in t1.getR_TB_B()] == [1.0, 0.0, 0.0]
    assert [r[0] for r in t2.getR_TB_B()] == [0.0, 2.0, 0.0]
    assert float(t1.getFuelLeakRate()) == pytest.approx(1e-6)
    assert float(t2.getFuelLeakRate()) == pytest.approx(2e-6)


def test_single_tank_guard_unchanged():
    """Empty-mirror (0) and tank-without-thrusters (loud) behavior retained."""
    swap = _load_swap()
    assert swap._mirror_fuel([], None, []) == 0
    (t1, _) = _two_tanks()
    with pytest.raises(ValueError, match="no mirrored thrusters"):
        swap._mirror_fuel([t1], None, [])
