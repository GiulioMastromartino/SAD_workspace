"""Env-bodies+J2 NO-GO pins: the gap recorded loudly (list.md §A1).

Ledger: audit_bundle/results/env_parity_envbodies.json
(probe env_parity_envbodies.py -- attempted stock-vs-omni legs at matched
dt=0.1 through the WIRED mirror_stock_config path).

This family does NOT reach parity, for REAL reader/host-interface gaps
(the jitter-spike NO-GO precedent: record divergence + cause, keep the
loud behavior, never weaken the reader to force green):

* N-body cell (earth + sun + moon, static ephemeris, task
  GravityEffector): the wired mirror RAISES on host. The census is
  rejected before the N-body link is reached -- the central body's
  concrete law lives in the ``_pyGravityModel`` shadow and is lost across
  the C++ ``addBodiesTo`` copy, so the central-law gate fires first (the
  sibling per-body link gate -- a linked reader whose writer Message is
  unrecoverable, since BSK ``subscribeTo`` needs the writer Message, not
  the reader functor -- is pinned host-free by
  ``test_reject_linked_but_sourceless_link``). The pin asserts the loud
  raise is RECORDED (red here = behavior changed: if a reader fix lands,
  this cell moves to the parity suite). Firing proof: the stock tide
  displacement is macroscopic (ledger tide_effect_m; basis: PLAN §6.1 /
  reader "Sun tide ~1.4e-6 m/s^2" -> ~0.25 m / 600 s analytic order; gate
  1e-2 carries >=25x margin over the measured 0.32).
* J2 cell (central SH maxDeg=2): the wired mirror REJECTS the central
  body loudly -- the concrete gravity law lives in the ``_pyGravityModel``
  Python shadow, and the census bodies iterated from C++ vectors lost it
  (the reader's own docstring anticipates "lost Python-side state"; on
  host it is ALWAYS lost), so the reader refuses to assume point-mass
  (PLAN §1.2). The pin asserts the loud reject: the shadow-lost
  ``unreadable`` signature + the ``_pyGravityModel`` cause + the
  macroscopic stock J2 effect (ledger j2_effect_m ~2.4e3 m, gate >1.0 m
  proves the fixture is live, not vacuous). Historically this was a silent
  drop (~2.3e3 m divergence); that is now a loud reject. Red here =
  silent-green claims or a softened reject; never relax.

Pure-JSON pins (host-free shape): skip (fail open, never fail) when the
probe ledger is absent on this host.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LEDGER = REPO / "audit_bundle" / "results" / "env_parity_envbodies.json"


def _load():
    if not LEDGER.exists():
        pytest.skip("env_parity_envbodies probe not run on this host yet")
    return json.loads(LEDGER.read_text())["payload"]


def test_ledger_no_go_verdict():
    d = _load()
    assert d["status"] == "ok"  # probe ran as designed
    assert "NO-GO" in d["verdict"], f"verdict softened: {d['verdict']}"
    assert d["dt"] == pytest.approx(0.1)


def test_nbody_mirror_raises_loudly():
    """N-body: the wired mirror's host raise is RECORDED (never silent)."""
    n = _load()["cells"]["nbody"]
    assert n["status"] == "mirror-raises-on-host", (
        f"nbody behavior changed: {n['status']} -- move to parity suite")
    assert "mirror_stock_config" in (n["mirror_raise"] or ""), (
        f"raise signature lost: {n['mirror_raise']}")
    assert "unreadable" in (n["mirror_raise"] or ""), (
        f"not the census reject: {n['mirror_raise']}")
    assert "message" in (n["cause"] or "").lower() or \
        "link" in (n["cause"] or "").lower(), "cause not recorded"


def test_nbody_tide_firing_proof():
    """N-body fixture fires: stock sun+moon tide displacement macroscopic."""
    n = _load()["cells"]["nbody"]
    assert n["tide_effect_m"] > 1e-2, (
        f"nbody vacuous tide={n['tide_effect_m']:.3e}")


def test_j2_mirror_rejects_loudly():
    """J2: the wired mirror's host reject of the unreadable central law is
    RECORDED (never a silent point-mass no-action)."""
    j = _load()["cells"]["j2"]
    assert j["status"] == "mirror-raises-on-host", (
        f"j2 behavior changed: {j['status']} -- move to parity suite")
    raise_txt = j["mirror_raise"] or ""
    assert "mirror_stock_config" in raise_txt, (
        f"reject signature lost: {raise_txt}")
    assert "unreadable" in raise_txt, (
        f"not the shadow-lost reject: {raise_txt}")


def test_j2_loud_reject_mechanism_pinned():
    """J2: the unreadable-shadow mechanism is PINNED (census + cause), and
    the stock arm actually feels J2 (fixture is live, not vacuous)."""
    j = _load()["cells"]["j2"]
    assert j["census_shadow"]["iterated_has_py_shadow"] == [False]
    assert "_pyGravityModel" in (j["cause"] or ""), "cause not recorded"
    assert j["j2_effect_m"] > 1.0, (
        f"j2 stock effect vacuous: {j['j2_effect_m']:.3e}")


def test_loadavg_provenance():
    """L9: host load recorded per cell (contention noted, never gated)."""
    for name in ("nbody", "j2"):
        c = _load()["cells"][name]
        assert isinstance(c["loadavg"], list) and len(c["loadavg"]) == 3, name
