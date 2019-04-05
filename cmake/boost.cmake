# Copyright (C) 2008-2018 LAAS-CNRS, JRL AIST-CNRS.
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

#.rst:
# .. variable:: BOOST_COMPONENTS
#
#  Controls the components to be detected.  
#  If this variable is not defined, it defaults to the following component
#  list:
#
#  - Filesystem
#  - Program_options
#  - System
#  - Thread
#  - Unit_test_framework
#
# .. command:: SEARCH_FOR_BOOST
#
#  This macro deals with Visual Studio Fortran incompatibilities
#  and add detected flags to the pkg-config file automatically.
#
#  The components to be detected is controlled by :variable:`BOOST_COMPONENTS`.
#
#  A special treatment must be done for the boost-python component. 
#  For boost >= 1.67.0, FindPython macro should be called first in order
#  to automatically detect the right boost-python component version according
#  to the Python version (2.7 or 3.x).
#
MACRO(SEARCH_FOR_BOOST)
  SET(Boost_USE_STATIC_LIBS OFF)
  SET(Boost_USE_MULTITHREAD ON)

  IF(NOT BOOST_REQUIRED)
    SET(BOOST_REQUIRED 1.40)
  ENDIF(NOT BOOST_REQUIRED)

  # First try to find Boost to get the version
  FIND_PACKAGE(Boost ${BOOST_REQUIRED})

  IF(NOT DEFINED BOOST_COMPONENTS)
    SET(BOOST_COMPONENTS
      filesystem system thread program_options unit_test_framework)
  ENDIF(NOT DEFINED BOOST_COMPONENTS)

  # Check if python is in the list and adjust the version according to the current Python version.
  # This is made mandatory if for Boost version greater than 1.67.0
  LIST(FIND BOOST_COMPONENTS python PYTHON_IN_BOOST_COMPONENTS)
  IF(${PYTHON_IN_BOOST_COMPONENTS} GREATER -1)
    SET(BOOST_COMPONENTS_ ${BOOST_COMPONENTS})
    CHECK_DEBIAN()
    CHECK_NETBSD()
    CHECK_ARCHLINUX()

    IF(Boost_VERSION VERSION_GREATER 106699)
        # Check if Python has been found
        IF(NOT PYTHONLIBS_FOUND)
          MESSAGE(FATAL_ERROR "PYTHON has not been found. You should first call FindPython before calling SEARCH_FOR_BOOST macro.")   
        ENDIF(NOT PYTHONLIBS_FOUND)

        LIST(REMOVE_AT BOOST_COMPONENTS_ ${PYTHON_IN_BOOST_COMPONENTS})
        IF(APPLE OR NETBSD_FOUND OR ARCHLINUX_FOUND)
          SET(BOOST_PYTHON_COMPONENT "python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
        ELSEIF(DEBIAN_FOUND)
          SET(BOOST_PYTHON_COMPONENT "python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
        ELSE(APPLE OR NETBSD_FOUND OR ARCHLINUX_FOUND)
          SET(BOOST_PYTHON_COMPONENT "python${PYTHON_VERSION_MAJOR}")
        ENDIF(APPLE OR NETBSD_FOUND OR ARCHLINUX_FOUND)
        LIST(APPEND BOOST_COMPONENTS_ ${BOOST_PYTHON_COMPONENT})
    ELSE(Boost_VERSION VERSION_GREATER 106699)
      # Check if Python has been found
      IF(NOT PYTHONLIBS_FOUND)
        MESSAGE(FATAL_ERROR "PYTHON has not been found. You should first call FindPython before calling SEARCH_FOR_BOOST macro.")   
      ENDIF(NOT PYTHONLIBS_FOUND)
      IF(${PYTHON_VERSION_MAJOR} EQUAL 3) 
        LIST(REMOVE_AT BOOST_COMPONENTS_ ${PYTHON_IN_BOOST_COMPONENTS})
        IF(APPLE OR NETBSD_FOUND)
          SET(BOOST_PYTHON_COMPONENT "python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
        ELSEIF(DEBIAN_FOUND)
          SET(BOOST_PYTHON_COMPONENT "python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
        ELSE(APPLE OR NETBSD_FOUND)
          SET(BOOST_PYTHON_COMPONENT "python${PYTHON_VERSION_MAJOR}")
        ENDIF(APPLE OR NETBSD_FOUND)
        LIST(APPEND BOOST_COMPONENTS_ ${BOOST_PYTHON_COMPONENT})
      ELSE(${PYTHON_VERSION_MAJOR} EQUAL 3) 
        SET(BOOST_COMPONENTS_ ${BOOST_COMPONENTS})
      ENDIF(${PYTHON_VERSION_MAJOR} EQUAL 3) 
    ENDIF(Boost_VERSION VERSION_GREATER 106699)
  
    SET(BOOST_COMPONENTS ${BOOST_COMPONENTS_})
  ENDIF(${PYTHON_IN_BOOST_COMPONENTS} GREATER -1)

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
    # Detect if component has python as a substring
    STRING(FIND ${COMPONENT} "python" python_comp_pos)

    # If the substring starts with python zero if the result
    # and the test fails. Thus test greater than 1.
    IF(${python_comp_pos} GREATER -1)
      SET(Boost_PYTHON_LIBRARY ${Boost_${UPPERCOMPONENT}_LIBRARY})
      MESSAGE(STATUS "Boost_PYTHON_LIBRARY: ${Boost_PYTHON_LIBRARY}")
      LIST(APPEND Boost_PYTHON_LIBRARIES
	${Boost_PYTHON_LIBRARY})
      LIST(APPEND LOGGING_WATCHED_VARIABLES
	python_comp_pos
	Boost_${UPPERCOMPONENT}_LIBRARY
	Boost_PYTHON_LIBRARY
	)
    ENDIF()

  ENDFOREACH()

  # On darwin systems, we must link againt boost_python with unresolved symbols.
  # We then remove boost_python from the global Boost_LIBRARIES list to handle it with specific care.
  IF(Boost_PYTHON_LIBRARY)
    LIST(REMOVE_ITEM Boost_LIBRARIES ${Boost_PYTHON_LIBRARY})
  ENDIF(Boost_PYTHON_LIBRARY)

ENDMACRO(SEARCH_FOR_BOOST)

#.rst:
# .. command:: TARGET_LINK_BOOST_PYTHON (TARGET)
#
#   Link target againt boost_python library.
#
#   :TARGET: is either a library or an executable
#
#   On darwin systems, boost_python is not linked against any python library.
#   This linkage is resolved at execution time via the python interpreter.
#   We then need to stipulate that boost_python has unresolved symbols at compile time for a library target.
#   Otherwise, for executables we need to link to a specific version of python.
#
MACRO(TARGET_LINK_BOOST_PYTHON target)
  IF(APPLE)
    GET_TARGET_PROPERTY(TARGET_TYPE ${target} TYPE)

    IF(${TARGET_TYPE} MATCHES EXECUTABLE)
      TARGET_LINK_LIBRARIES(${target} ${PUBLIC_KEYWORD} ${Boost_PYTHON_LIBRARY})
    ELSE(${TARGET_TYPE} MATCHES EXECUTABLE)
      TARGET_LINK_LIBRARIES(${target} ${PUBLIC_KEYWORD} -Wl,-undefined,dynamic_lookup,${Boost_PYTHON_LIBRARIES})
    ENDIF(${TARGET_TYPE} MATCHES EXECUTABLE)

  ELSE(APPLE)
    TARGET_LINK_LIBRARIES(${target} ${PUBLIC_KEYWORD} ${Boost_PYTHON_LIBRARY})
  ENDIF(APPLE)
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    Boost_PYTHON_LIBRARIES
    )
ENDMACRO(TARGET_LINK_BOOST_PYTHON)

#.rst:
# .. command:: PKG_CONFIG_APPEND_BOOST_LIBS
#
#   This macro appends Boost libraries to the pkg-config file. A list of Boost
#   components is expected, for instance::
#
#     PKG_CONFIG_APPEND_BOOST_LIBS(system filesystem)
#
MACRO(PKG_CONFIG_APPEND_BOOST_LIBS)
  IF(NOT APPLE)
    PKG_CONFIG_APPEND_LIBRARY_DIR("${Boost_LIBRARY_DIRS}")
  ENDIF()

  FOREACH(COMPONENT ${ARGN})
    STRING(TOUPPER ${COMPONENT} UPPERCOMPONENT)
    STRING(TOLOWER ${COMPONENT} LOWERCOMPONENT)
    IF(APPLE)
      IF("${LOWERCOMPONENT}" MATCHES "python")
        PKG_CONFIG_APPEND_LIBS_RAW(-Wl,-undefined,dynamic_lookup,${Boost_${UPPERCOMPONENT}_LIBRARY})
      ELSE("${LOWERCOMPONENT}" MATCHES "python")
        PKG_CONFIG_APPEND_LIBS_RAW(${Boost_${UPPERCOMPONENT}_LIBRARY})
      ENDIF("${LOWERCOMPONENT}" MATCHES "python")
    ELSE(APPLE)
      GET_FILENAME_COMPONENT(LIB_NAME ${Boost_${UPPERCOMPONENT}_LIBRARY} NAME_WE)
      STRING(REGEX REPLACE "^lib" "" LIB_NAME "${LIB_NAME}")
      PKG_CONFIG_APPEND_LIBS_RAW("-l${LIB_NAME}")
    ENDIF(APPLE)
  ENDFOREACH()
ENDMACRO(PKG_CONFIG_APPEND_BOOST_LIBS)
