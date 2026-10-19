add_library(sad_mission_batch STATIC
    sad_mission_batch.cpp
)
target_include_directories(sad_mission_batch PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/mission_batch ${CMAKE_SOURCE_DIR}/cpp/core ${CMAKE_SOURCE_DIR}/cpp/dynamics ${CMAKE_SOURCE_DIR}/cpp/actuators ${CMAKE_SOURCE_DIR}/cpp/sensors ${CMAKE_SOURCE_DIR}/cpp/environment ${CMAKE_SOURCE_DIR}/cpp/control ${CMAKE_SOURCE_DIR}/cpp/mission ${EIGEN_INCLUDE})
target_link_libraries(sad_mission_batch PUBLIC sad_core sad_dynamics sad_actuators sad_sensors sad_environment sad_control sad_mission)

if(TARGET sad_dynamics_heyoka)
    target_include_directories(sad_mission_batch PUBLIC ${CMAKE_SOURCE_DIR}/cpp/heyoka_dynamics)
    target_link_libraries(sad_mission_batch PUBLIC sad_dynamics_heyoka)
    target_compile_definitions(sad_mission_batch PRIVATE SAD_WITH_HEYOKA=1)
endif()
