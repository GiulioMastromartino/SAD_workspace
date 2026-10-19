cmake_minimum_required(VERSION 3.16)
project(SAD_Project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Default to an optimized build. Without this, CMake compiles with NO -O flag
# at all (-O0), which alone costs roughly 3-5x on this workload.
# Override with: cmake -DCMAKE_BUILD_TYPE=Debug ..
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Build type (Release=-O3, Debug=-g, RelWithDebInfo=-O2 -g)" FORCE)
    message(STATUS "SAD_Project: CMAKE_BUILD_TYPE not set, defaulting to Release (-O3)")
endif()

# Paths
set(SAD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BSK_SDK_ROOT "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/env/lib/python3.11/site-packages/bsk_sdk" CACHE PATH "Basilisk SDK root")
set(EIGEN_INCLUDE ${BSK_SDK_ROOT}/include/eigen3)
set(PYBIND11_INCLUDE ${SAD_ROOT}/../env/lib/python3.11/site-packages/pybind11/include)
set(PYTHON_EXECUTABLE "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/env/bin/python" CACHE FILEPATH "Python")
list(APPEND CMAKE_PREFIX_PATH "${SAD_ROOT}/../env")

include_directories(${SAD_ROOT}/cpp ${SAD_ROOT}/cpp/core ${EIGEN_INCLUDE})

# Core library
add_library(sad_core STATIC
    cpp/core/sad_core_dummy.cpp
    cpp/core/sad_timing.cpp
)
# sad_core is header-only; dummy keeps target. Add sources as needed.

# Optional Heyoka Taylor integrator backend.  The RK4 mission kernel remains
# the baseline and does not depend on Heyoka.
option(SAD_WITH_HEYOKA "Build the Heyoka Taylor integrator backend" ON)
set(SAD_HEYOKA_FOUND FALSE)
if(SAD_WITH_HEYOKA)
    find_package(heyoka CONFIG QUIET HINTS ${SAD_ROOT}/../env/lib/cmake/heyoka)
    if(heyoka_FOUND)
        set(SAD_HEYOKA_FOUND TRUE)
        message(STATUS "SAD_Project: Heyoka backend enabled (${heyoka_DIR})")
    else()
        message(WARNING "SAD_Project: Heyoka not found; building RK4-only backend")
    endif()
endif()

# Submodules
add_subdirectory(cpp/dynamics)
add_subdirectory(cpp/actuators)
add_subdirectory(cpp/sensors)
add_subdirectory(cpp/environment)
add_subdirectory(cpp/estimation)
add_subdirectory(cpp/control)
add_subdirectory(cpp/guidance)
add_subdirectory(cpp/mission)
if(SAD_HEYOKA_FOUND)
    add_subdirectory(cpp/heyoka_dynamics)
endif()
add_subdirectory(cpp/mission_batch)

# Python bindings
find_package(pybind11 CONFIG QUIET HINTS ${SAD_ROOT}/../env/lib/python3.11/site-packages/pybind11/share/cmake/pybind11)
if(pybind11_FOUND)
    add_subdirectory(cpp/python_bindings)
endif()

# Tests are Python-side (pytest) driving the pybind module.
