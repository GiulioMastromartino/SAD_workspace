"""Build the fastloop pybind11 extension in place.

Usage (from Python/):  ./env/bin/python cubesat_12u_sim/fastloop/setup.py build_ext --inplace
"""
import sys

import pybind11
from setuptools import setup
from setuptools.command.build_ext import build_ext


class BuildExt(build_ext):
    def build_extensions(self):
        import numpy

        for ext in self.extensions:
            ext.extra_compile_args = [
                "-O3", "-std=c++17", "-ffast-math",
                "-I" + numpy.get_include(),
            ]
            ext.extra_link_args = []
        super().build_extensions()


try:
    from pybind11.setup_helpers import Pybind11Extension
except ImportError:
    print("pybind11>=2.6 required", file=sys.stderr)
    raise SystemExit(1)

setup(
    name="fastloop",
    version="0.1.0",
    packages=[],
    ext_modules=[
        Pybind11Extension(
            "cubesat_12u_sim.fastloop._fastloop",
            ["cubesat_12u_sim/fastloop/fastloop.cpp"],
            cxx_std=17,
        ),
    ],
    cmdclass={"build_ext": BuildExt},
)
