add_library(sad_dynamics STATIC
    sad_dynamics.cpp
    nbody_gravity.cpp
    spherical_harmonics.cpp
    polyhedral_gravity.cpp
    drag.cpp
    srp.cpp
    facet_drag.cpp
    facet_srp.cpp
    fuel_slosh.cpp
    hinged_rigid_body.cpp
    spinning_body.cpp
    linear_translation.cpp
    constraint_effector.cpp
    ext_force.cpp
    orbital_elements.cpp
)
target_include_directories(sad_dynamics PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/dynamics ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_dynamics PUBLIC sad_core)
