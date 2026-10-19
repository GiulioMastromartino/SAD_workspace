add_library(sad_guidance STATIC
    lambert.cpp
    dv_guidance.cpp
    orb_elem.cpp
    hill.cpp
    thr_force_mapping.cpp
    thr_firing.cpp
    thr_momentum.cpp
    mtb_momentum.cpp
    torque2dipole.cpp
    dipole_mapping.cpp
    rw_allocation.cpp
    joint_thr_alloc.cpp
    vscmg_steering.cpp
    thrust_cm.cpp
    torque_filter.cpp
    formation.cpp
    hinged_motor.cpp
    att_ref.cpp
    deadband.cpp
)
target_include_directories(sad_guidance PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/guidance ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_guidance PUBLIC sad_core)
