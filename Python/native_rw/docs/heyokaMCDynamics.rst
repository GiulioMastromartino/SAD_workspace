HeyokaMCDynamics native module
==============================

True compiled C++ ``SysModel`` replacing ``spacecraft`` for the
thruster (RCS-jet) pipeline, mirroring
``heyoka_dynamics.HeyokaSpacecraft`` with stock FSW otherwise untouched.

Messages
--------

- ``ReadFunctor<THRArrayOnTimeCmdMsgPayload> thrOnTimeInMsg`` — thruster
  on-time commands (first 12 slots; duty = onTime/scheduler-step).
- ``Message<SCStatesMsgPayload> scStateOutMsg`` — hub state.

Per tick: throttle duty from on-time, measured 3-level effector
staircase (``{0, 1/6, 5/6, 1}``) unless ``setQuantize(false)``, wrench
via the configured (3, 12) effectiveness matrices, propagation with the
previous tick's wrench (Basilisk mailbox/ZOH semantics), ``SCStatesMsg``
publish.

Configuration (all runtime setters; defaults = 12U bus baseline)
------------------------------------------------------------------

``setR_CN_NInit / setV_CN_NInit / setSigmaBNInit / setOmegaBNInit`` (ICs),
``setMass / setInertia(Ixx..Iyz)`` (inverse derived in C++ at build),
``setForceMatrix / setTorqueMatrix`` (36-entry row-major lists),
``setQuantize / setUseGravityGradient / setTol / setMu``. The packed
inverse inertia is derived in C++ at build; post-``Reset`` property
changes rebuild the integrator rather than going stale. First
``UpdateState`` publishes ICs. ``getSigmaBN / getOmegaBN / getMu``
for verification.

Envelope (by design — extend ``buildMcSystem`` for more)
--------------------------------------------------------

- 12 fixed thrusters, central Kepler + optional gravity gradient,
- body origin == CoM (``r_BN_N`` mirrors ``r_CN_N``),
  ``omegaDot_BN_B`` left zero,
- no effector attachment points (not a ``DynamicObject``).

Reference: detumble 600 s @ 0.01 s, Apple Silicon — stock 9.55 s,
heyoka-python 14.16 s (0.67x), native 9.09 s (1.05x), parity identical
to the validated Python path (9.61e-03 MRP / 5.19e-03 rad/s).
