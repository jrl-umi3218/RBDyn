# Copyright (C) 2017 LAAS-CNRS, JRL AIST-CNRS.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

INCLUDE(CheckCCompilerFlag)

# _CHECK_VERSION_SCRIPT_SUPPORT
# -----------------------------
#
# Internal macro to check if version scripts are supported by the current
# linker.
MACRO(_CHECK_VERSION_SCRIPT_SUPPORT)
  SET(CMAKE_REQUIRED_FLAGS "-Wl,--version-script=${PROJECT_SOURCE_DIR}/cmake/version-script-test.lds")
  CHECK_C_COMPILER_FLAG("" HAS_VERSION_SCRIPT_SUPPORT)
  SET(_HAS_VERSION_SCRIPT_SUPPORT ${HAS_VERSION_SCRIPT_SUPPORT} CACHE INTERNAL "Linker supports version scripts")
ENDMACRO(_CHECK_VERSION_SCRIPT_SUPPORT)

#.rst:
# .. command:: ADD_VERSION_SCRIPT(TARGET VERSION_SCRIPT)
#
#   This macro adds a version script to a given target and a link-time
#   dependency between the target and the version script.
#
#   See https://www.gnu.org/software/gnulib/manual/html_node/LD-Version-Scripts.html
#
#   It has no effect on platforms that do not support version script.
#
#   :param TARGET:         Name of the target, the macro does nothing if TARGET is not a
#                          cmake target.
#   :param VERSION_SCRIPT: Version script to add to the library.
#
MACRO(ADD_VERSION_SCRIPT TARGET VERSION_SCRIPT)
  IF(NOT DEFINED _HAS_VERSION_SCRIPT_SUPPORT)
    _CHECK_VERSION_SCRIPT_SUPPORT()
  ENDIF()
  IF(_HAS_VERSION_SCRIPT_SUPPORT)
    IF(TARGET ${TARGET})
      SET_PROPERTY(TARGET ${TARGET} APPEND_STRING PROPERTY
                   LINK_FLAGS " -Wl,--version-script,${VERSION_SCRIPT}")
      SET_TARGET_PROPERTIES(${TARGET} PROPERTIES LINK_DEPENDS ${VERSION_SCRIPT})
    ENDIF()
  ENDIF()
ENDMACRO(ADD_VERSION_SCRIPT)
