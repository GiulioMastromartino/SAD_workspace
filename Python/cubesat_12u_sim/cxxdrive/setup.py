"""Build the cxxdrive pybind11 extension in place.

Usage (from Python/):  ./env/bin/python cubesat_12u_sim/cxxdrive/setup.py build_ext --inplace
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

        # Find boost/heyoka/eigen headers (conda env or homebrew).
        # Same recipe as rwdrive/setup.py: campaign.cpp includes
        # <heyoka/heyoka.hpp>, resolved from Homebrew or the conda env.
        extra_include = None
        # Check conda env site-packages
        sp = os.path.join(sysconfig.get_path('purelib'), '..', '..', 'boost_headers', 'include')
        if os.path.isdir(sp):
            extra_include = os.path.normpath(sp)
        # Check homebrew (heyoka + boost + eigen)
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
            ext.extra_link_args = []
        super().build_extensions()


try:
    from pybind11.setup_helpers import Pybind11Extension
except ImportError:
    print("pybind11>=2.6 required", file=sys.stderr)
    raise SystemExit(1)

setup(
    name="cxxdrive",
    version="0.1.0",
    packages=[],
    ext_modules=[
        Pybind11Extension(
            "cubesat_12u_sim.cxxdrive._cxxdrive",
            ["cubesat_12u_sim/cxxdrive/campaign.cpp"],
            cxx_std=17,
        ),
    ],
    cmdclass={"build_ext": BuildExt},
)
