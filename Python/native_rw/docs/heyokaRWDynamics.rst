HeyokaRWDynamics native module
==============================

True compiled C++ ``SysModel`` replacing ``spacecraft`` +
``reactionWheelStateEffector`` for the ``scenarioAttitudeFeedbackRW``
baseline (750 kg hub, ``I = diag(900, 800, 600)``, 3x Honeywell HR16 on
body axes at 100/200/300 RPM, inertial identity target, ``K = 3.5`` /
``P = 30``).

Messages
--------

- ``ReadFunctor<ArrayMotorTorqueMsgPayload> rwMotorCmdInMsg`` — motor
  torque commands (saturated internally at ``+-uMax``, stock-effector
  behavior; the logged allocator message is unsaturated).
- ``Message<SCStatesMsgPayload> scStateOutMsg`` — hub state.
- ``Message<RWSpeedMsgPayload> rwSpeedOutMsg`` — wheel speeds [rad/s].

Configuration (all runtime setters; defaults = demo baseline)
--------------------------------------------------------------

``setR_CN_NInit / setV_CN_NInit / setSigmaBNInit / setOmegaBNInit /
setWheelSpeedsInitRPM`` (ICs), ``setMass / setInertia(Ixx..Iyz) /
setRotorInertia(J1..J3) / setUMax / setTol / setMu``. The packed inverse
inertia is derived in C++ at ``Reset``. First ``UpdateState`` publishes
ICs (no motor message exists yet); following ticks propagate the
scheduler step derived from message timestamps.

Envelope (by design — extend ``buildRwSystem`` for more)
--------------------------------------------------------

- rigid hub at CoM, 3 body-axis-aligned balanced wheels (no
  jitter/friction/voltage IO),
- central-body point-mass Kepler only (no effector attachment points;
  not a ``DynamicObject``),
- no thrust input.

Reference: 600 s RW demo, Apple Silicon, warm — stock 67 ms, native
20 ms (3.4x) at 7.19e-05 deg / 1.4e-06 deg/s / 0.25 RPM wheels max.
