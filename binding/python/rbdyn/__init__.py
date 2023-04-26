import os

if hasattr(os, "add_dll_directory"):
    os.add_dll_directory("@CMAKE_INSTALL_PREFIX@/bin")

from .rbdyn import *
from . import parsers
