# Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
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

# SEARCH_FOR_BOOST
# -----------------
#
# This macro deals with Visual Studio Fortran incompatibilities
# and add detected flags to the pkg-config file automatically.
#
# The components to be detected is controlled by BOOST_COMPONENTS.  If
# this variable is not defined, it defaults to the following component
# list:
# - Filesystem
# - Program_options
# - System
# - Thread
# - Unit_test_framework
#
MACRO(SEARCH_FOR_BOOST)
  SET(Boost_USE_STATIC_LIBS OFF)
  SET(Boost_USE_MULTITHREAD ON)

  IF(NOT BOOST_REQUIRED)
    SET(BOOST_REQUIRED 1.40)
  ENDIF(NOT BOOST_REQUIRED)

  IF(NOT DEFINED BOOST_COMPONENTS)
    SET(BOOST_COMPONENTS
      filesystem system thread program_options unit_test_framework)
  ENDIF(NOT DEFINED BOOST_COMPONENTS)

  FIND_PACKAGE(Boost ${BOOST_REQUIRED} COMPONENTS ${BOOST_COMPONENTS} REQUIRED)

  IF(NOT Boost_FOUND)
    MESSAGE(
      FATAL_ERROR "Failed to detect Boost with the following components:\n"
      ${COMPONENTS})
  ENDIF(NOT Boost_FOUND)

  PKG_CONFIG_APPEND_CFLAGS("-I${Boost_INCLUDE_DIR}")
  PKG_CONFIG_APPEND_LIBRARY_DIR("${Boost_LIBRARY_DIRS}")

  LIST(APPEND LOGGING_WATCHED_VARIABLES
    Boost_USE_MULTITHREADED
    Boost_USE_STATIC_LIBS
    Boost_ADDITIONAL_VERSIONS
    Boost_DEBUG
    Boost_COMPILER
    BOOST_ROOT
    BOOSTROOT
    BOOST_INCLUDEDIR
    BOOST_LIBRARYDIR
    Boost_FOUND
    Boost_INCLUDE_DIRS
    Boost_INCLUDE_DIR
    Boost_LIBRARIES
    Boost_LIBRARY_DIRS
    Boost_VERSION
    Boost_LIB_VERSION
    Boost_MAJOR_VERSION
    Boost_MINOR_VERSION
    Boost_SUBMINOR_VERSION
    Boost_LIB_DIAGNOSTIC_DEFINITIONS
    )
  FOREACH(COMPONENT ${BOOST_COMPONENTS})
    LIST(APPEND LOGGING_WATCHED_VARIABLES
      Boost_${COMPONENT}_FOUND
      Boost_${COMPONENT}_LIBRARY
      Boost_${COMPONENT}_LIBRARY_DEBUG
      Boost_${COMPONENT}_LIBRARY_RELEASE
      )
  ENDFOREACH()
ENDMACRO(SEARCH_FOR_BOOST)
