# SAD_Project module registry

Status legend: **FULL** = typed, functional, parity-tested. **SKEL** = skeleton (init/step/reset, passthrough), refine physics.

## core
- sad_core (FULL) - math, quaternions, DCM, integrator

## dynamics (cpp/dynamics)
- sad_dynamics (FULL) - 6U rigid body, quaternion EOM, orbital gravity, GG torque
- sad_dynamics_heyoka (FULL, optional) - same 13-state plant as an adaptive Heyoka Taylor integrator; build with `SAD_WITH_HEYOKA`
- nbody_gravity, spherical_harmonics, polyhedral_gravity, drag, srp, facet_drag, facet_srp, fuel_slosh, hinged_rigid_body, spinning_body, linear_translation, constraint_effector, ext_force, orbital_elements (SKEL)

## actuators (cpp/actuators)
- sad_thruster (FULL) - 8-jet variable thrust, allocation, fuel
- sad_rw (FULL) - 3 RWs, saturation, first-order
- mtq, vscmg, stepper, ext_forcetorque, constraint, prescribed_motion, fuel_tank, saturation, array_motor, scalar_joint, motor_thermal, motor_voltage (SKEL)

## sensors (cpp/sensors)
- sad_startracker (FULL) - noisy quat + dropout
- sad_imu (FULL) - 9-axis gyro
- magnetometer, css, camera, temp, thermal, encoder, simple_nav, planet_nav (SKEL)

## environment (cpp/environment)
- sad_sun (FULL) - sun ephemeris + sun-pointing quat
- mag_field, atmosphere, eclipse, albedo, planet_ephem, sc_location, ground, space_weather, solar_flux (SKEL)

## estimation (cpp/estimation)
- nav_aggregate, sensor_comm, sunline_est, inertial_est, heading_est, opnav, fdir, ephem_nav, small_body_nav (SKEL)

## control (cpp/control)
- sad_quat_pd (FULL) - quaternion PD detumble/slew/track
- mrp_pd, mrp_feedback, mrp_steering, rate_servo, sun_safe, inertial_3d, pointing, solar_array_point, constrained_maneuver, waypoint, celestial_point (SKEL)

## guidance (cpp/guidance)
- lambert, dv_guidance, orb_elem, hill, thr_force_mapping, thr_firing, thr_momentum, mtb_momentum, torque2dipole, dipole_mapping, rw_allocation, joint_thr_alloc, vscmg_steering, thrust_cm, torque_filter, formation, hinged_motor, att_ref, deadband (SKEL)

## mission (cpp/mission)
- sad_mission_sequencer (FULL) - detumble->slew->track state machine

## simulink (cpp/simulink)
- sad_dynamics_sfun.c (example Level-2 MEX S-function)
