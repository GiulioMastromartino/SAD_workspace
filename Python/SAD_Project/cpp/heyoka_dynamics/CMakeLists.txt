add_library(sad_dynamics_heyoka STATIC
    sad_dynamics_heyoka.cpp
    sad_dynamics_heyoka_track.cpp
)
target_include_directories(sad_dynamics_heyoka PUBLIC
    ${CMAKE_SOURCE_DIR}/cpp
    ${CMAKE_SOURCE_DIR}/cpp/core
    ${CMAKE_SOURCE_DIR}/cpp/dynamics
    ${EIGEN_INCLUDE}
)
target_link_libraries(sad_dynamics_heyoka PUBLIC sad_core sad_dynamics heyoka::heyoka)
