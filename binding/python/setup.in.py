# Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of RBDyn.
#
# SpaceVecAlg is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# SpaceVecAlg is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with SpaceVecAlg.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function
try:
  from setuptools import setup
  from setuptools import Extension
except ImportError:
  from distutils.core import setup
  from distutils.extension import Extension

from Cython.Build import cythonize

import hashlib
import os
import subprocess

from numpy import get_include as numpy_get_include

win32_build = os.name == 'nt'

this_path  = os.path.dirname(os.path.realpath(__file__))
with open(this_path + '/rbdyn/__init__.py', 'w') as fd:
    fd.write('from .rbdyn import *\n')

sha512 = hashlib.sha512()
src_files = ['rbdyn/rbdyn.pyx', 'rbdyn/c_rbdyn.pxd', 'rbdyn/rbdyn.pxd', 'include/rbdyn_wrapper.hpp']
src_files = [ '{}/{}'.format(this_path, f) for f in src_files ]
for f in src_files:
  chunk = 2**16
  with open(f, 'r') as fd:
    while True:
      data = fd.read(chunk)
      if data:
        sha512.update(data.encode('ascii'))
      else:
        break
version_hash = sha512.hexdigest()[:7]

class pkg_config(object):
  def __init__(self):
    self.compile_args = []
    self.include_dirs = [ x for x in '@RBDYN_INCLUDE_DIRECTORIES@'.split(';') if len(x) ]
    self.library_dirs = [ x for x in '@RBDYN_LINK_FLAGS@'.split(';') if len(x) ]
    self.libraries = ['RBDyn']
    rbdyn_location = '@RBDYN_LOCATION@'
    self.library_dirs.append(os.path.dirname(rbdyn_location))
    self.found = True

config = pkg_config()

config.compile_args.append('-std=c++11')
config.include_dirs.append(os.getcwd() + "/include")
if win32_build:
  config.compile_args.append("-DWIN32")

def GenExtension(name, pkg, ):
  pyx_src = name.replace('.', '/')
  cpp_src = pyx_src + '.cpp'
  pyx_src = pyx_src + '.pyx'
  ext_src = pyx_src
  if pkg.found:
    return Extension(name, [ext_src], extra_compile_args = pkg.compile_args, include_dirs = pkg.include_dirs + [numpy_get_include()], library_dirs = pkg.library_dirs, libraries = pkg.libraries)
  else:
    print("Failed to find {}".format(pkg.name))
    return None

extensions = [
  GenExtension('rbdyn.rbdyn', config)
]

extensions = [ x for x in extensions if x is not None ]
packages = ['rbdyn']
data = ['__init__.py', 'c_rbdyn.pxd', 'rbdyn.pxd']

extensions = cythonize(extensions)

setup(
    name = 'rbdyn',
    version='1.0.0-{}'.format(version_hash),
    ext_modules = extensions,
    packages = packages,
    package_data = { 'rbdyn': data }
)
