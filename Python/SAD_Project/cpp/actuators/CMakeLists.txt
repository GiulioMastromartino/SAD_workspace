add_library(sad_actuators STATIC
    sad_actuator_thruster.cpp
    sad_actuator_rw.cpp
    mtq.cpp
    vscmg.cpp
    stepper.cpp
    ext_forcetorque.cpp
    constraint.cpp
    prescribed_motion.cpp
    fuel_tank.cpp
    saturation.cpp
    array_motor.cpp
    scalar_joint.cpp
    motor_thermal.cpp
    motor_voltage.cpp
)
target_include_directories(sad_actuators PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/actuators ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_actuators PUBLIC sad_core)
