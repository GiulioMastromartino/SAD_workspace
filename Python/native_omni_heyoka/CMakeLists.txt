# ISC License - see native_rw/LICENSE.
#
# Build for native_omni_heyoka, modelled directly on native_rw's (proven)
# recipe: scikit-build-core + bsk-sdk's SWIG helper + heyoka + Eigen.
#
# Differences from native_rw:
#   * module name is omni_heyoka and the package dir is omniheyoka,
#   * the C++ core lives in include/omni/*.hpp (contributors, back-substitution
#     solve, templated MRP) rather than a single hand-written EOM.
cmake_minimum_required(VERSION 3.26)
project(cubesat-omni-heyoka LANGUAGES C CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

# --- locate bsk-sdk from the active Python environment -----------------------
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
    "import bsk_sdk; print(bsk_sdk.cmake_config_dir(), end='')"
  OUTPUT_VARIABLE bsk_sdk_dir
  RESULT_VARIABLE rc
)
if(NOT rc EQUAL 0 OR bsk_sdk_dir STREQUAL "")
  message(FATAL_ERROR "bsk-sdk not found (is it installed in this Python environment?)")
endif()
file(TO_CMAKE_PATH "${bsk_sdk_dir}" bsk_sdk_dir)
set(bsk-sdk_DIR "${bsk_sdk_dir}")
find_package(bsk-sdk CONFIG REQUIRED)

# Where scikit-build-core places wheel outputs
set(EXTENSION_PKG_DIR "${SKBUILD_PLATLIB_DIR}/omniheyoka")

# Record the exact Basilisk version / ABI guard in the wheel (as native_rw does)
bsk_add_extension_compatibility_guard(
  EXTENSION_NAME "omniheyoka"
  OUTPUT_DIR "${EXTENSION_PKG_DIR}"
)

# --- Heyoka headers (same discovery order as the proven rwdrive/native_rw build)
if(DEFINED ENV{HEYOKA_ROOT} AND NOT "$ENV{HEYOKA_ROOT}" STREQUAL "")
  list(APPEND _heyoka_candidates "$ENV{HEYOKA_ROOT}")
endif()
get_filename_component(_pybin_dir "${Python3_EXECUTABLE}" DIRECTORY)
get_filename_component(_pyprefix "${_pybin_dir}" DIRECTORY)
list(APPEND _heyoka_candidates "/opt/homebrew" "${_pyprefix}")

set(HEYOKA_INCLUDE_DIR "")
foreach(_c IN LISTS _heyoka_candidates)
  if(EXISTS "${_c}/include/heyoka/heyoka.hpp")
    set(HEYOKA_INCLUDE_DIR "${_c}/include")
    break()
  endif()
endforeach()
if(HEYOKA_INCLUDE_DIR STREQUAL "")
  message(FATAL_ERROR "heyoka/heyoka.hpp not found; set HEYOKA_ROOT to a prefix "
    "containing include/heyoka (e.g. /opt/homebrew or the conda env)")
endif()
message(STATUS "Heyoka includes: ${HEYOKA_INCLUDE_DIR}")

set(_inc "${HEYOKA_INCLUDE_DIR}")
if(EXISTS "/opt/homebrew/include")
  list(APPEND _inc "/opt/homebrew/include")
endif()
# Boost/Eigen shipped inside bsk-sdk's include tree (works on this machine).
if(EXISTS "${bsk_sdk_DIR}/../../../include/eigen3")
  list(APPEND _inc "${bsk_sdk_DIR}/../../../include/eigen3")
endif()
if(EXISTS "/opt/homebrew/include/eigen3")
  list(APPEND _inc "/opt/homebrew/include/eigen3")
endif()

# --- Build the SWIG module ---------------------------------------------------
bsk_add_swig_module(
  TARGET omni_heyoka
  INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src/omni_heyoka.i"
  SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/omni_spacecraft.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/omni_vec_integrator.cpp"
  INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    ${_inc}
  OUTPUT_DIR "${EXTENSION_PKG_DIR}"
)

# heyoka 44 requires C++23; -O3 only (Taylor math is LLVM-JITed, and
# -ffast-math trips fmt's nan/inf checks).
target_compile_features(omni_heyoka PRIVATE cxx_std_23)
target_compile_options(omni_heyoka PRIVATE -O3)

# --- link libheyoka directly, with an install rpath (no DYLD_* dependence) ---
set(_heyoka_libdirs "")
foreach(_c IN LISTS _heyoka_candidates)
  list(APPEND _heyoka_libdirs "${_c}/lib")
endforeach()
find_library(HEYOKA_LIBRARY NAMES heyoka PATHS ${_heyoka_libdirs} NO_DEFAULT_PATH
  DOC "libheyoka for direct linking (else dynamic_lookup fallback)")
if(HEYOKA_LIBRARY)
  message(STATUS "Heyoka library (linked): ${HEYOKA_LIBRARY}")
  target_link_libraries(omni_heyoka PRIVATE "${HEYOKA_LIBRARY}")
  get_filename_component(_heyoka_libdir "${HEYOKA_LIBRARY}" DIRECTORY)
  set_target_properties(omni_heyoka PROPERTIES
    INSTALL_RPATH "${_heyoka_libdir}"
    BUILD_WITH_INSTALL_RPATH TRUE)
else()
  message(WARNING "libheyoka not found: using dynamic_lookup fallback "
    "(import heyoka before omni_heyoka)")
endif()

# --- ship the top-level cSysModel shim at the wheel root --------------------
# SWIG emits a bare `import cSysModel` for %import "sys_model.i"; without this
# the wrapper cannot resolve Basilisk's SysModel type.
bsk_add_python_module(
  SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/shim/cSysModel.py"
  OUTPUT_DIR "${SKBUILD_PLATLIB_DIR}"
)
