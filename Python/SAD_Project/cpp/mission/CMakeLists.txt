add_library(sad_mission STATIC
    sad_mission_sequencer.cpp
)
target_include_directories(sad_mission PUBLIC ${CMAKE_SOURCE_DIR}/cpp ${CMAKE_SOURCE_DIR}/cpp/mission ${CMAKE_SOURCE_DIR}/cpp/core ${EIGEN_INCLUDE})
target_link_libraries(sad_mission PUBLIC sad_core)
