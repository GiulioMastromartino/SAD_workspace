add_library(sad_environment STATIC
    sad_env_sun.cpp
    mag_field.cpp
    atmosphere.cpp
    eclipse.cpp
    albedo.cpp
    planet_ephem.cpp
    sc_location.cpp
    ground.cpp
    space_weather.cpp
    solar_flux.cpp
)
target_include_directories(sad_environment PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/environment ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_environment PUBLIC sad_core)
