"""C++ absolute-value pins (tests_cpp/test_core_values.cpp via build.sh).

Hand-derived expected values evaluated directly against the headers + the
numeric OmniSystem path on double -- no transcribed reference, so a failure
means the code drifted from the math. Covers: hub+gravity absolute accel,
MRP kinematics/DCM, Schur back-substitution, linear3 exact inverse,
drag/SRP TRUE laws, thruster force+torque, offset arm (non-degenerate
geometry), fuel depletion sign. Boundaries (module-side live-c, Stribeck
latch, arc hooks, B/C scatter) stay ledger-covered; see the file header.

Skips cleanly without a compiler (never cited as pass). build.sh owns the
heyoka/Eigen discovery; this file only enforces exit 0 + PASS line.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
CPPDIR = REPO / "tests_cpp"
BUILD = CPPDIR / "build.sh"


def test_cpp_core_values():
    cxx = os.environ.get("CXX") or shutil.which("c++") \
        or shutil.which("clang++") or shutil.which("g++")
    if cxx is None:
        pytest.skip("no C++ compiler on this host")
    if not BUILD.exists():
        pytest.skip("tests_cpp/build.sh absent")
    proc = subprocess.run(["bash", str(BUILD)], capture_output=True,
                          text=True, timeout=900)
    tail = (proc.stdout or "")[-1500:] + (proc.stderr or "")[-500:]
    assert proc.returncode == 0, f"cpp-values failed:\n{tail}"
    assert "FAIL" not in (proc.stdout or ""), f"cpp-values FAIL:\n{tail}"
    assert "PASS:" in (proc.stdout or ""), f"cpp-values no PASS line:\n{tail}"
