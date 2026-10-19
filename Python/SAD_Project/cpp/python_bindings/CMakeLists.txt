pybind11_add_module(sad_py
    pybind_module.cpp
)
target_include_directories(sad_py PRIVATE ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/core ${CMAKE_SOURCE_DIR}/cpp/dynamics ${CMAKE_SOURCE_DIR}/cpp/actuators ${CMAKE_SOURCE_DIR}/cpp/sensors ${CMAKE_SOURCE_DIR}/cpp/environment ${CMAKE_SOURCE_DIR}/cpp/control ${CMAKE_SOURCE_DIR}/cpp/mission ${EIGEN_INCLUDE})
target_link_libraries(sad_py PRIVATE sad_core sad_dynamics sad_actuators sad_sensors sad_environment sad_control sad_mission sad_mission_batch)
set_target_properties(sad_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python
    BUILD_RPATH "${SAD_ROOT}/../env/lib"
    INSTALL_RPATH "${SAD_ROOT}/../env/lib"
)
