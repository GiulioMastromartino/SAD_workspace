# Native Heyoka RW dynamics Basilisk extension (bsk-sdk, no full-source build).
cmake_minimum_required(VERSION 3.26)
project(cubesat-nativerw LANGUAGES C CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

# Locate bsk-sdk from the active Python environment
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
set(EXTENSION_PKG_DIR "${SKBUILD_PLATLIB_DIR}/nativerw")

# Record the exact Basilisk version / ABI guard in the wheel
bsk_add_extension_compatibility_guard(
  EXTENSION_NAME "nativerw"
  OUTPUT_DIR "${EXTENSION_PKG_DIR}"
)

# --- Heyoka headers (same strategy as the proven rwdrive build) ---
# Candidate prefixes holding include/heyoka/heyoka.hpp; Boost/Eigen come
# along from Homebrew (mirrors cubesat_12u_sim/rwdrive/setup.py).
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
set(_extra_includes "${HEYOKA_INCLUDE_DIR}")
if(EXISTS "/opt/homebrew/include")
  list(APPEND _extra_includes "/opt/homebrew/include")
endif()

# Build the SWIG module.
bsk_add_swig_module(
  TARGET heyokaRWDynamics
  INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/heyokaRWDynamics/heyokaRWDynamics.i"
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/heyokaRWDynamics/heyokaRWDynamics.cpp"
  INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/heyokaRWDynamics"
    ${_extra_includes}
  OUTPUT_DIR "${EXTENSION_PKG_DIR}"
)

# --- Link hygiene: real LC_LOAD_DYLIB + rpath for libheyoka ---
# A published wheel must not depend on dynamic_lookup + import order. Link
# the discovered libheyoka and bake its directory as install rpath so the
# loader resolves it without DYLD_* help. If no library is found (headers
# only), fall back to lookup mode: `import heyoka` before `import nativerw`.
set(_heyoka_libdirs "")
foreach(_c IN LISTS _heyoka_candidates)
  list(APPEND _heyoka_libdirs "${_c}/lib")
endforeach()
find_library(HEYOKA_LIBRARY NAMES heyoka PATHS ${_heyoka_libdirs} NO_DEFAULT_PATH
  DOC "libheyoka for direct linking (else dynamic_lookup fallback)")
if(HEYOKA_LIBRARY)
  message(STATUS "Heyoka library (linked): ${HEYOKA_LIBRARY}")
  target_link_libraries(heyokaRWDynamics PRIVATE "${HEYOKA_LIBRARY}")
  get_filename_component(_heyoka_libdir "${HEYOKA_LIBRARY}" DIRECTORY)
  set_target_properties(heyokaRWDynamics PROPERTIES
    INSTALL_RPATH "${_heyoka_libdir}"
    BUILD_WITH_INSTALL_RPATH TRUE)
else()
  message(WARNING "libheyoka not found: using dynamic_lookup fallback "
    "(import heyoka before nativerw)")
endif()

# heyoka 44 requires C++23 (explicit object params, ranges::to); the SDK
# default (gnu++20) does not suffice. -O3 only: -ffast-math trips fmt's
# nan/inf checks and buys nothing here (Taylor math is LLVM-JITed).
target_compile_features(heyokaRWDynamics PRIVATE cxx_std_23)
target_compile_options(heyokaRWDynamics PRIVATE -O3)

# Ship the package __init__ (compat guard + re-export) alongside the module.
bsk_add_python_module(
  SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/nativerw/__init__.py"
  OUTPUT_DIR "${EXTENSION_PKG_DIR}"
)

# Second module: thruster-pipeline dynamics (same package, same helpers).
bsk_add_swig_module(
  TARGET heyokaMCDynamics
  INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/heyokaMCDynamics/heyokaMCDynamics.i"
  SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/heyokaMCDynamics/heyokaMCDynamics.cpp"
  INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/heyokaMCDynamics"
    ${_extra_includes}
  OUTPUT_DIR "${EXTENSION_PKG_DIR}"
)
target_compile_features(heyokaMCDynamics PRIVATE cxx_std_23)
target_compile_options(heyokaMCDynamics PRIVATE -O3)
if(HEYOKA_LIBRARY)
  target_link_libraries(heyokaMCDynamics PRIVATE "${HEYOKA_LIBRARY}")
  get_filename_component(_heyoka_libdir "${HEYOKA_LIBRARY}" DIRECTORY)
  set_target_properties(heyokaMCDynamics PROPERTIES
    INSTALL_RPATH "${_heyoka_libdir}"
    BUILD_WITH_INSTALL_RPATH TRUE)
endif()

# Ship the top-level cSysModel shim at the wheel root so SWIG-generated
# bare `import cSysModel` resolves (see shim/cSysModel.py).
bsk_add_python_module(
  SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/shim/cSysModel.py"
  OUTPUT_DIR "${SKBUILD_PLATLIB_DIR}"
)
