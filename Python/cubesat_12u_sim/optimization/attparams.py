"""Parameter space for the attitude-loop tuning sweep.

The loop is the MRP PD running in ``campaign.cpp``: ``t = -K*e - P*fb`` with a
rate/bias observer. Every parameter spans orders of magnitude in effect, so --
like the orbit pipeline -- everything is optimised in natural-log space.

What is searchable, and what is deliberately not:

* ``K`` / ``P`` -- the PD itself. The only two numbers that set closed-loop
  bandwidth and damping.
* ``obs_gain`` / ``obs_bias_gain`` -- the rate observer. Too slow and the
  feedback lags (settle times grow); too fast and gyro noise feeds straight
  into the torque command (fuel grows). That trade-off is exactly what the
  sweep is meant to expose.
* ``ctrl_scale`` -- multiplies the torque request before allocation. Part of
  the loop gain in practice.

Not searched: the integral term (``ACS_KI = -1`` disables it in *both* existing
paths, Basilisk and C++; enabling it would change the controller structure,
not just its tuning), the slew target and sun geometry (mission requirements),
and the phase durations (mission requirements -- the controller must fit inside
them, not resize them).
"""
from __future__ import annotations

import math

from cubesat_12u_sim.core import config as C
from cubesat_12u_sim.optimization.params import Param, ParamSpace


def _p(name, lo, hi, default, unit=""):
    return Param(name, math.log(lo), math.log(hi), math.log(default), unit)


ATT_SPACE = ParamSpace([
    _p("K", 4e-3, 4e-1, C.ACS_K, "rad/s"),
    _p("P", 4e-2, 4e0, C.ACS_P, "N m s"),
    _p("obs_gain", 5.0, 5e2, C.ACS_OBSERVER_GAIN, "1/s"),
    _p("obs_bias_gain", 5e-2, 5e0, C.ACS_OBSERVER_BIAS_GAIN, "1/s"),
    _p("ctrl_scale", 0.2, 20.0, C.THRUSTER_CONTROL_SCALE, "-"),
])

__all__ = ["ATT_SPACE"]
