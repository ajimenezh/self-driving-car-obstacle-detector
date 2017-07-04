from distutils.core import setup
from Cython.Build import cythonize
import os

os.environ['CFLAGS'] = '-O3'
setup(name="ros_node", ext_modules=cythonize('ros_node.pyx'),)
