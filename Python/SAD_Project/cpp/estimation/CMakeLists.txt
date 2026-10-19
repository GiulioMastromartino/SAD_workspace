add_library(sad_estimation STATIC
    nav_aggregate.cpp
    sensor_comm.cpp
    sunline_est.cpp
    inertial_est.cpp
    heading_est.cpp
    opnav.cpp
    fdir.cpp
    ephem_nav.cpp
    small_body_nav.cpp
)
target_include_directories(sad_estimation PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/estimation ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_estimation PUBLIC sad_core)
