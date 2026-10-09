"""Build the cxxdrive pybind11 extensions in place.

Usage (from Python/):
    ./env/bin/python cubesat_12u_sim/cxxdrive/setup.py build_ext --inplace

Builds both ``_cxxdrive`` (full C++ Heyoka campaign driver) and ``_pidbatch``
(fused RTN-PID tuning surrogate; plain C++, no Heyoka dependency). Pass
``--only pidbatch`` to skip the Heyoka module (faster iteration when the
toolchain headers are missing).
"""
import os
import sys

import pybind11
from setuptools import setup
from setuptools.command.build_ext import build_ext


ONLY = None
if "--only" in sys.argv:
    idx = sys.argv.index("--only")
    ONLY = sys.argv[idx + 1]
    del sys.argv[idx:idx + 2]


class BuildExt(build_ext):
    def build_extensions(self):
        import numpy
        import sysconfig
        import os

        extra_include = None
        sp = os.path.join(sysconfig.get_path('purelib'), '..', '..', 'boost_headers', 'include')
        if os.path.isdir(sp):
            extra_include = os.path.normpath(sp)
        if not extra_include:
            for p in ['/opt/homebrew/include', '/usr/local/include']:
                if os.path.isdir(os.path.join(p, 'boost')):
                    extra_include = p
                    break

        for ext in self.extensions:
            ext.extra_compile_args = [
                "-O3", "-std=c++2b", "-ffast-math",
                "-I" + numpy.get_include(),
            ]
            if extra_include:
                ext.extra_compile_args.append("-I" + extra_include)
            if getattr(ext, "libraries", None) == ["nlopt"]:
                # setuptools would drop our rpath otherwise: keep NLopt
                # resolvable at import without touching the conda env.
                import os as _os2
                _nl = _os2.path.expanduser("~/.local/nlopt/lib")
                ext.extra_link_args = ["-Wl,-rpath," + _nl]
            else:
                ext.extra_link_args = []
        super().build_extensions()


try:
    from pybind11.setup_helpers import Pybind11Extension
except ImportError:
    print("pybind11>=2.6 required", file=sys.stderr)
    raise SystemExit(1)

# Source paths are resolved against this file's own directory. The package was
# moved under native/ during the layout restructure, and the literal
# "cubesat_12u_sim/cxxdrive/..." paths these used to carry only resolved when
# the build happened to be run from the right cwd -- otherwise the compile step
# fails with "no such file or directory" on the .cpp.
_HERE = os.path.dirname(os.path.abspath(__file__))


def _src(name: str) -> str:
    return os.path.join(_HERE, name)


ext_modules = []
if ONLY in (None, "cxxdrive"):
    ext_modules.append(Pybind11Extension(
        "cubesat_12u_sim.native.cxxdrive._cxxdrive",
        [_src("campaign.cpp")],
        cxx_std=17,
    ))
if ONLY in (None, "pidbatch"):
    ext_modules.append(Pybind11Extension(
        "cubesat_12u_sim.native.cxxdrive._pidbatch",
        [_src("pid_batch.cpp")],
        cxx_std=17,
    ))
if ONLY in (None, "nmpcbatch"):
    ext_modules.append(Pybind11Extension(
        "cubesat_12u_sim.native.cxxdrive._nmpcbatch",
        [_src("nmpc_batch.cpp")],
        cxx_std=17,
    ))
if ONLY in (None, "nmpcsolve"):
    _nlopt = os.path.expanduser("~/.local/nlopt")
    ext_modules.append(Pybind11Extension(
        "cubesat_12u_sim.native.cxxdrive._nmpcsolve",
        [_src("nmpc_solve.cpp")],
        cxx_std=17,
        include_dirs=[os.path.join(_nlopt, "include")],
        library_dirs=[os.path.join(_nlopt, "lib")],
        libraries=["nlopt"],
        extra_link_args=["-Wl,-rpath," + os.path.join(_nlopt, "lib")],
    ))

setup(
    name="cxxdrive",
    version="0.2.0",
    packages=[],
    ext_modules=ext_modules,
    cmdclass={"build_ext": BuildExt},
)
