add_library(sad_control STATIC
    sad_controller_quat_pd.cpp
    mrp_pd.cpp
    mrp_feedback.cpp
    mrp_steering.cpp
    rate_servo.cpp
    sun_safe.cpp
    inertial_3d.cpp
    pointing.cpp
    solar_array_point.cpp
    constrained_maneuver.cpp
    waypoint.cpp
    celestial_point.cpp
)
target_include_directories(sad_control PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/control ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_control PUBLIC sad_core sad_dynamics)
