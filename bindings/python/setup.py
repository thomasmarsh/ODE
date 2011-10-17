#!/usr/bin/env python

import os

from distutils.core import setup
from distutils.extension import Extension
from subprocess import check_output,CalledProcessError

try:
    from Cython.Distutils import build_ext
except ImportError:
    raise SystemExit("Requires Cython (http://cython.org/)")

try:
    ode_cflags = check_output(["pkg-config", "--cflags", "ode"]).split()
    ode_libs = check_output(["pkg-config", "--libs", "ode"]).split()
except CalledProcessError:
    raise SystemExit("Failed to find ODE with 'pkg-config'.  Please make sure that it is installed and available on your system path.")

ode_ext = Extension("ode", ["ode.pyx"],
                    extra_compile_args=ode_cflags,
                    extra_link_args=ode_libs)

if __name__ == "__main__":
    setup(
        name="Open Dynamics Engine",
        version="0.1",
        author="Gideon Klompje",
#        author_email="",
#        maintainer="",
#        maintainer_email="",
        url="http://www.ode.org",
        description="Bindings for the Open Dynamics Engine",
        long_description="""A free, industrial quality library for simulating articulated rigid body dynamics - for example ground vehicles, legged creatures, and moving objects in VR environments. It's fast, flexible & robust. Built-in collision detection.""",
#        download_url="https://opende.svn.sourceforge.net/svnroot/opende",
#        classifiers=[],
#        platforms=[],
        license="BSD License, GNU Lesser General Public License (LGPL)",
        cmdclass={"build_ext": build_ext}, ext_modules=[ode_ext])
