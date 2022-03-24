from distutils.core import setup, Extension
from Cython.Build import cythonize
import numpy

setup(ext_modules=cythonize("my_redmarkerdetection.pyx"),  include_dirs=[numpy.get_include()])
setup(ext_modules=cythonize("my_control_car.pyx"),  include_dirs=[numpy.get_include()])