# Copyright (C) 2008-2016 LAAS-CNRS, JRL AIST-CNRS.
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
    STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)
    LIST(APPEND LOGGING_WATCHED_VARIABLES
      Boost_${UPPERCOMPONENT}_FOUND
      Boost_${UPPERCOMPONENT}_LIBRARY
      Boost_${UPPERCOMPONENT}_LIBRARY_DEBUG
      Boost_${UPPERCOMPONENT}_LIBRARY_RELEASE
      )
  ENDFOREACH()

  # On darwin systems, we must link againt boost_python with unresolved symbols.
  # We then remove boost_python from the global Boost_LIBRARIES list to handle it with specific care.
  IF(Boost_PYTHON_LIBRARY)
    LIST(REMOVE_ITEM Boost_LIBRARIES ${Boost_PYTHON_LIBRARY})
    MESSAGE(WARNING 
            "The boost_python library have been removed from the Boost_LIBRARIES variable.
            \n To link againt boost_python, please use the macro TARGET_LINK_BOOST_PYTHON."
           )
  ENDIF(Boost_PYTHON_LIBRARY)

ENDMACRO(SEARCH_FOR_BOOST)

# TARGET_LINK_BOOST_PYTHON
# ------------------------
#
# Link target againt boost_python library.
# \input target is either a library or an executable
# On darwin systems, boost_python is not linked against any python library.
# This linkage is resolved at execution time via the python interpreter.
# We then need to stipulate that boost_python has unresolved symbols at compile time for a library target.
# Otherwise, for executables we need to link to a specific version of python.
#
MACRO(TARGET_LINK_BOOST_PYTHON target)
  IF(APPLE)
    GET_TARGET_PROPERTY(TARGET_TYPE ${target} TYPE)

    IF(${TARGET_TYPE} MATCHES EXECUTABLE)
      TARGET_LINK_LIBRARIES(${target} ${Boost_PYTHON_LIBRARY})
    ELSE(${TARGET_TYPE} MATCHES EXECUTABLE)
      TARGET_LINK_LIBRARIES(${target} -Wl,-undefined,dynamic_lookup,${Boost_PYTHON_LIBRARIES})
    ENDIF(${TARGET_TYPE} MATCHES EXECUTABLE)

  ELSE(APPLE)
    TARGET_LINK_LIBRARIES(${target} ${Boost_PYTHON_LIBRARY})
  ENDIF(APPLE)
ENDMACRO(TARGET_LINK_BOOST_PYTHON)

# PKG_CONFIG_APPEND_BOOST_LIBS
# ----------------------------
#
# This macro appends Boost libraries to the pkg-config file. A list of Boost
# components is expected, for instance:
#
# PKG_CONFIG_APPEND_BOOST_LIBS(system filesystem)
#
MACRO(PKG_CONFIG_APPEND_BOOST_LIBS)
  IF(NOT APPLE)
    PKG_CONFIG_APPEND_LIBRARY_DIR("${Boost_LIBRARY_DIRS}")
  ENDIF()

  FOREACH(COMPONENT ${ARGN})
    IF(APPLE)
      STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)
      STRING(TOLOWER ${COMPONENT} LOWERCOMPONENT)
      IF("${LOWERCOMPONENT}" MATCHES "python")
        PKG_CONFIG_APPEND_LIBS_RAW(-Wl,-undefined,dynamic_lookup,${Boost_${UPPERCOMPONENT}_LIBRARY})
      ELSE("${LOWERCOMPONENT}" MATCHES "python")
        PKG_CONFIG_APPEND_LIBS_RAW(${Boost_${UPPERCOMPONENT}_LIBRARY})
      ENDIF("${LOWERCOMPONENT}" MATCHES "python")
    ELSE(APPLE)
      STRING(TOLOWER ${COMPONENT} LOWERCOMPONENT)
      PKG_CONFIG_APPEND_LIBS(boost_${LOWERCOMPONENT})
    ENDIF(APPLE)
  ENDFOREACH()
ENDMACRO(PKG_CONFIG_APPEND_BOOST_LIBS)
