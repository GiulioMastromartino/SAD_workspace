add_library(sad_sensors STATIC
    sad_sensor_startracker.cpp
    sad_sensor_imu.cpp
    magnetometer.cpp
    css.cpp
    camera.cpp
    temp.cpp
    thermal.cpp
    encoder.cpp
    simple_nav.cpp
    planet_nav.cpp
)
target_include_directories(sad_sensors PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/sensors ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_sensors PUBLIC sad_core)
