import os

from distutils.core import setup
from distutils.extension import Extension

try:
    from Cython.Distutils import build_ext
except ImportError:
    raise SystemExit("Requires Cython (http://cython.org/)")


_dir = os.path.split(__file__)[0] # 'bindings/python' directory
ODE_DLL = os.path.join(_dir, "..", "..", "lib", "DebugSingleDLL", "ode_singled.dll") # TODO: infer from build or environment variable?
INCLUDE_DIR = os.path.join(_dir, "..", "..", "include")

ode_ext = Extension("ode", ["ode.pyx"], extra_objects=[ODE_DLL], include_dirs=[INCLUDE_DIR])


if __name__ == "__main__":
    setup(name="Open Dynamics Engine", cmdclass={"build_ext": build_ext}, ext_modules=[ode_ext])
