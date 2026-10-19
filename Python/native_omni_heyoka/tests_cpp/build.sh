#!/usr/bin/env bash
# Build + run the C++ absolute-value pins (tests_cpp/test_core_values.cpp).
# Needs only heyoka + Eigen INCLUDE paths (double-instantiated math links
# against nothing; if the link ever wants heyoka symbols the fallback below
# engages libheyoka). Exit 0 iff the binary reports PASS.
#
#   PY=../../env/bin/python  bash tests_cpp/build.sh   # default PY shown
#   CXX=clang++ HEYOKA_ROOT=/opt/homebrew bash tests_cpp/build.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${DIR}/.."
PY="${PY:-${DIR}/../../env/bin/python}"
[ -x "${PY}" ] || PY="$(command -v python3 || true)"
[ -n "${PY}" ] || { echo "[cpp-values] no python to locate deps" >&2; exit 3; }

# --- heyoka includes (same discovery order as CMakeLists) -------------------
PREFIX="$(cd "$(dirname "${PY}")/.." && pwd)"
HEYOKA_INC=""
for c in "${HEYOKA_ROOT:-}" "${PREFIX}/include" "/opt/homebrew/include"; do
  [ -n "${c}" ] && [ -f "${c}/include/heyoka/heyoka.hpp" ] && HEYOKA_INC="${c}/include" && break
  [ -n "${c}" ] && [ -f "${c}/heyoka/heyoka.hpp" ] && HEYOKA_INC="${c}" && break
done
# Note: candidates above are prefixes except the second test per entry.
if [ -z "${HEYOKA_INC}" ]; then
  for c in "${HEYOKA_ROOT:-}" "${PREFIX}" "/opt/homebrew"; do
    [ -n "${c}" ] && [ -f "${c}/include/heyoka/heyoka.hpp" ] && HEYOKA_INC="${c}/include" && break
  done
fi
[ -n "${HEYOKA_INC}" ] || { echo "[cpp-values] heyoka/heyoka.hpp not found (HEYOKA_ROOT?)" >&2; exit 3; }

# --- Eigen includes (bsk-sdk tree first, like CMakeLists) --------------------
EIGEN_INC=""
SDK_DIR="$("${PY}" -c "import bsk_sdk; print(bsk_sdk.cmake_config_dir(), end='')" 2>/dev/null || true)"
for c in "${SDK_DIR}/../../../include/eigen3" "/opt/homebrew/include/eigen3" "${PREFIX}/include/eigen3"; do
  [ -f "${c}/Eigen/Dense" ] && EIGEN_INC="${c}" && break
done
[ -n "${EIGEN_INC}" ] || { echo "[cpp-values] Eigen/Dense not found" >&2; exit 3; }

# --- Boost includes (heyoka headers need boost/archive/*; CMakeLists gets
# --- these via /opt/homebrew/include on this machine) -----------------------
BOOST_INC=""
for c in "/opt/homebrew/include" "${PREFIX}/include"; do
  [ -f "${c}/boost/archive/binary_iarchive.hpp" ] && BOOST_INC="${c}" && break
done
[ -n "${BOOST_INC}" ] || { echo "[cpp-values] boost/archive headers not found (brew boost?)" >&2; exit 3; }

CXX_BIN="${CXX:-$(command -v c++ || command -v clang++ || command -v g++ || true)}"
[ -n "${CXX_BIN}" ] || { echo "[cpp-values] no C++ compiler" >&2; exit 3; }

mkdir -p "${DIR}/build"
SRC="${DIR}/test_core_values.cpp"
BIN="${DIR}/build/test_core_values"
FLAGS=(-std=c++23 -O2 -Wno-deprecated-declarations "-I${ROOT}/include" "-I${HEYOKA_INC}" "-I${EIGEN_INC}" "-I${BOOST_INC}")

echo "[cpp-values] cxx=${CXX_BIN} heyoka=${HEYOKA_INC} eigen=${EIGEN_INC} boost=${BOOST_INC}"
if "${CXX_BIN}" "${FLAGS[@]}" "${SRC}" -o "${BIN}" 2>"${DIR}/build/link0.err"; then
  echo "[cpp-values] linked without libheyoka (double-only, as designed)"
else
  if grep -qi "heyoka" "${DIR}/build/link0.err"; then
    LIB=""
    for c in "${PREFIX}/lib" "/opt/homebrew/lib"; do
      [ -f "${c}/libheyoka.dylib" ] && LIB="${c}/libheyoka.dylib" && break
      [ -f "${c}/libheyoka.so" ] && LIB="${c}/libheyoka.so" && break
    done
    [ -n "${LIB}" ] || { echo "[cpp-values] link needs heyoka symbols but no libheyoka found" >&2; tail -5 "${DIR}/build/link0.err" >&2; exit 1; }
    echo "[cpp-values] relinking with ${LIB} (+rpath so dyld finds it at runtime)"
    LIBDIR="$(dirname "${LIB}")"
    "${CXX_BIN}" "${FLAGS[@]}" "${SRC}" -o "${BIN}" "${LIB}" "-Wl,-rpath,${LIBDIR}"
  else
    echo "[cpp-values] compile/link failed:" >&2
    tail -20 "${DIR}/build/link0.err" >&2
    exit 1
  fi
fi

"${BIN}"
