"""Build the rwdrive pybind11 extension in place.

Usage (from Python/):  ./env/bin/python cubesat_12u_sim/rwdrive/setup.py build_ext --inplace
"""
import sys

import pybind11
from setuptools import setup
from setuptools.command.build_ext import build_ext


class BuildExt(build_ext):
    def build_extensions(self):
        import numpy
        import sysconfig
        import os

        # Find boost headers (conda env or homebrew)
        boost_include = None
        # Check conda env site-packages
        sp = os.path.join(sysconfig.get_path('purelib'), '..', '..', 'boost_headers', 'include')
        if os.path.isdir(sp):
            boost_include = os.path.normpath(sp)
        # Check homebrew
        if not boost_include:
            for p in ['/opt/homebrew/include', '/usr/local/include']:
                if os.path.isdir(os.path.join(p, 'boost')):
                    boost_include = p
                    break

        for ext in self.extensions:
            ext.extra_compile_args = [
                "-O3", "-std=c++2b", "-ffast-math",
                "-I" + numpy.get_include(),
            ]
            if boost_include:
                ext.extra_compile_args.append("-I" + boost_include)
            ext.extra_link_args = []
        super().build_extensions()


try:
    from pybind11.setup_helpers import Pybind11Extension
except ImportError:
    print("pybind11>=2.6 required", file=sys.stderr)
    raise SystemExit(1)

setup(
    name="rwdrive",
    version="0.1.0",
    packages=[],
    ext_modules=[
        Pybind11Extension(
            "cubesat_12u_sim.rwdrive._rwdrive",
            ["cubesat_12u_sim/rwdrive/rwdrive.cpp"],
            cxx_std=17,
        ),
    ],
    cmdclass={"build_ext": BuildExt},
)
