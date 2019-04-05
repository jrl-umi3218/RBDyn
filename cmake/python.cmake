# Copyright (C) 2008-2019 LAAS-CNRS, JRL AIST-CNRS, INRIA.
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
# .. command:: FINDPYTHON
#
#  Find python interpreter and python libs.
#  Arguments are passed to the find_package command so
#  refer to `find_package` documentation to learn about valid arguments.
#
#  To specify a specific Python version from the command line,
#  use the command ``FINDPYTHON()``
#  and pass the following arguments to CMake
#  ``-DPYTHON_EXECUTABLE=/usr/bin/python3.5 -DPYTHON_LIBRARY= /usr/lib/x86_64-linux-gnu/libpython3.5m.so.1``
#
#  To specify a specific Python version within the CMakeLists.txt,
#  use the command ``FINDPYTHON(2.7 EXACT REQUIRED)``.
#
#  .. warning::
#    According to the ``FindPythonLibs`` and ``FindPythonInterp``
#    documentation, you could also set ``Python_ADDITIONAL_VERSIONS``.
#    If you do this, you will not have an error if you found two different versions
#    or another version that the requested one.
#

#.rst:
# .. variable:: PYTHON_SITELIB
#
#  Absolute path where Python files will be installed.

IF(CMAKE_VERSION VERSION_LESS "3.2")
    SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/python ${CMAKE_MODULE_PATH})
    MESSAGE(WARNING "CMake versions older than 3.2 do not properly find Python. Custom macros are used to find it.")
ENDIF(CMAKE_VERSION VERSION_LESS "3.2")

MACRO(FINDPYTHON)

FIND_PACKAGE(PythonInterp ${ARGN})
IF (NOT ${PYTHONINTERP_FOUND} STREQUAL TRUE)
   MESSAGE(FATAL_ERROR "Python executable has not been found.")
ENDIF (NOT ${PYTHONINTERP_FOUND} STREQUAL TRUE)
MESSAGE(STATUS "PythonInterp: ${PYTHON_EXECUTABLE}")

# Inform PythonLibs of the required version of PythonInterp
SET(PYTHONLIBS_VERSION_STRING ${PYTHON_VERSION_STRING})
FIND_PACKAGE(PythonLibs ${ARGN})
MESSAGE(STATUS "PythonLibraries: ${PYTHON_LIBRARIES}")
IF (NOT ${PYTHONLIBS_FOUND} STREQUAL TRUE)
   MESSAGE(FATAL_ERROR "Python has not been found.")
ENDIF (NOT ${PYTHONLIBS_FOUND} STREQUAL TRUE)

STRING(REPLACE "." ";" _PYTHONLIBS_VERSION ${PYTHONLIBS_VERSION_STRING})
LIST(GET _PYTHONLIBS_VERSION 0 PYTHONLIBS_VERSION_MAJOR)
LIST(GET _PYTHONLIBS_VERSION 1 PYTHONLIBS_VERSION_MINOR)

IF (NOT ${PYTHON_VERSION_MAJOR} EQUAL ${PYTHONLIBS_VERSION_MAJOR} OR
    NOT ${PYTHON_VERSION_MINOR} EQUAL ${PYTHONLIBS_VERSION_MINOR})
  MESSAGE(FATAL_ERROR "Python interpreter and libraries are in different version: ${PYTHON_VERSION_STRING} vs ${PYTHONLIBS_VERSION_STRING}")
ENDIF (NOT ${PYTHON_VERSION_MAJOR} EQUAL ${PYTHONLIBS_VERSION_MAJOR} OR
       NOT ${PYTHON_VERSION_MINOR} EQUAL ${PYTHONLIBS_VERSION_MINOR})

# Find PYTHON_LIBRARY_DIRS
GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIRS "${PYTHON_LIBRARIES}" PATH)
MESSAGE(STATUS "PythonLibraryDirs: ${PYTHON_LIBRARY_DIRS}")
MESSAGE(STATUS "PythonLibVersionString: ${PYTHONLIBS_VERSION_STRING}")

# Default Python packages directory
SET(PYTHON_PACKAGES_DIR site-packages)

# Use either site-packages (default) or dist-packages (Debian packages) directory
OPTION(PYTHON_DEB_LAYOUT "Enable Debian-style Python package layout" OFF)

IF (PYTHON_DEB_LAYOUT)
  SET(PYTHON_PACKAGES_DIR dist-packages)
ENDIF (PYTHON_DEB_LAYOUT)

EXECUTE_PROCESS(
  COMMAND "${PYTHON_EXECUTABLE}" "-c"
  "import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], '${PYTHON_PACKAGES_DIR}']))"
  OUTPUT_VARIABLE PYTHON_SITELIB
  ERROR_QUIET)

# Remove final \n of the variable PYTHON_SITELIB
STRING(REPLACE "\n" "" PYTHON_SITELIB "${PYTHON_SITELIB}")
NORMALIZE_PATH(PYTHON_SITELIB)

# Get PYTHON_SOABI
SET(PYTHON_SOABI "")
IF(PYTHON_VERSION_MAJOR EQUAL 3)
  EXECUTE_PROCESS(
    COMMAND "${PYTHON_EXECUTABLE}" "-c"
    "from distutils.sysconfig import get_config_var; print('.' + get_config_var('SOABI'))"
    OUTPUT_VARIABLE PYTHON_SOABI)
  STRING(STRIP ${PYTHON_SOABI} PYTHON_SOABI)
ENDIF(PYTHON_VERSION_MAJOR EQUAL 3)

# Log Python variables
LIST(APPEND LOGGING_WATCHED_VARIABLES
  PYTHONINTERP_FOUND
  PYTHONLIBS_FOUND
  PYTHON_LIBRARY_DIRS
  PYTHONLIBS_VERSION_STRING
  PYTHON_EXECUTABLE
  PYTHON_SOABI
  )

ENDMACRO(FINDPYTHON)


#.rst:
# .. command:: DYNAMIC_GRAPH_PYTHON_MODULE ( SUBMODULENAME LIBRARYNAME TARGETNAME INSTALL_INIT_PY=1 SOURCE_PYTHON_MODULE=cmake/dynamic_graph/python-module-py.cc)
#
#   Add a python submodule to dynamic_graph
#  
#   :param SUBMODULENAME: the name of the submodule (can be foo/bar),
#  
#   :param LIBRARYNAME:   library to link the submodule with.
#  
#   :param TARGETNAME:     name of the target: should be different for several
#                   calls to the macro.
#
#   :param INSTALL_INIT_PY: if set to 1 install and generated a __init__.py file.
#                   Set to 1 by default.
#
#   :param SOURCE_PYTHON_MODULE: Location of the cpp file for the python module in the package.
#                   Set to cmake/dynamic_graph/python-module-py.cc by default.
# 
#  .. note::
#    Before calling this macro, set variable NEW_ENTITY_CLASS as
#    the list of new Entity types that you want to be bound.
#    Entity class name should match the name referencing the type
#    in the factory.
#
MACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME)

  # By default the __init__.py file is installed.
  SET(INSTALL_INIT_PY 1)
  SET(SOURCE_PYTHON_MODULE "cmake/dynamic_graph/python-module-py.cc")
    
  # Check if there is optional parameters.
  set(extra_macro_args ${ARGN})
  list(LENGTH extra_macro_args num_extra_args)
  if( ${num_extra_args} GREATER 0)
    list(GET extra_macro_args 0 INSTALL_INIT_PY)
    if( ${num_extra_args} GREATER 1)
      list(GET extra_macro_args 1 SOURCE_PYTHON_MODULE)
    endif(${num_extra_args} GREATER 1)
  endif(${num_extra_args} GREATER 0)
  
  IF(NOT DEFINED PYTHONLIBS_FOUND)
    FINDPYTHON()
  ELSEIF(NOT ${PYTHONLIBS_FOUND} STREQUAL "TRUE")
    MESSAGE(FATAL_ERROR "Python has not been found.")
  ENDIF()

  SET(PYTHON_MODULE ${TARGETNAME})
  # We need to set this policy to old to accept wrap target.
  CMAKE_POLICY(PUSH)
  IF(POLICY CMP0037)
    CMAKE_POLICY(SET CMP0037 OLD)
  ENDIF()

  ADD_LIBRARY(${PYTHON_MODULE}
    MODULE
    ${PROJECT_SOURCE_DIR}/${SOURCE_PYTHON_MODULE})

  FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME})

  SET_TARGET_PROPERTIES(${PYTHON_MODULE}
    PROPERTIES PREFIX ""
    OUTPUT_NAME dynamic_graph/${SUBMODULENAME}/wrap
   )
  CMAKE_POLICY(POP)

  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} "-Wl,--no-as-needed")
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${PUBLIC_KEYWORD} ${LIBRARYNAME} ${PYTHON_LIBRARY})

  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

  #
  # Installation
  #
  SET(PYTHON_INSTALL_DIR ${PYTHON_SITELIB}/dynamic_graph/${SUBMODULENAME})

  INSTALL(TARGETS ${PYTHON_MODULE}
    DESTINATION
    ${PYTHON_INSTALL_DIR})

  SET(ENTITY_CLASS_LIST "")
  FOREACH (ENTITY ${NEW_ENTITY_CLASS})
    SET(ENTITY_CLASS_LIST "${ENTITY_CLASS_LIST}${ENTITY}('')\n")
  ENDFOREACH(ENTITY ${NEW_ENTITY_CLASS})

  # Install if INSTALL_INIT_PY is set to 1
  IF (${INSTALL_INIT_PY} EQUAL 1)

    CONFIGURE_FILE(
      ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/submodule/__init__.py.cmake
      ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME}/__init__.py
      )

    INSTALL(
      FILES ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME}/__init__.py
      DESTINATION ${PYTHON_INSTALL_DIR}
      )
    
  ENDIF(${INSTALL_INIT_PY} EQUAL 1)

ENDMACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME)


#.rst:
# .. command::  PYTHON_INSTALL(MODULE FILE DEST)
#
#  Install a Python file and its associated compiled version.
#
MACRO(PYTHON_INSTALL MODULE FILE DEST)

  PYTHON_BUILD("${MODULE}" "${FILE}")

  INSTALL(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE}/${FILE}"
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}c"
    DESTINATION "${DEST}/${MODULE}")
ENDMACRO()

#.rst:
# .. command:: PYTHON_INSTALL_ON_SITE (MODULE FILE)
#
#  Install a Python file and its associated compiled version in :cmake:variable:`PYTHON_SITELIB`.
#
MACRO(PYTHON_INSTALL_ON_SITE MODULE FILE)

  IF(NOT DEFINED PYTHONLIBS_FOUND)
    FINDPYTHON()
  ELSEIF(NOT ${PYTHONLIBS_FOUND} STREQUAL "TRUE")
    MESSAGE(FATAL_ERROR "Python has not been found.")
  ENDIF()

  PYTHON_INSTALL("${MODULE}" "${FILE}" "${PYTHON_SITELIB}")

ENDMACRO()

# PYTHON_BUILD(MODULE FILE DEST)
# --------------------------------------
#
# Build a Python file from the source directory in the build directory.
#
MACRO(PYTHON_BUILD MODULE FILE)

  FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}")

  # convert "/" to "_"
  STRING(REGEX REPLACE "/" "_" FILE_TARGET_NAME
    "${MODULE}/${FILE}c")

  ADD_CUSTOM_TARGET(${FILE_TARGET_NAME} ALL
    COMMAND
    "${PYTHON_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/cmake/compile.py"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${MODULE}/${FILE}"
  )

  # Tag pyc file as generated.
  SET_SOURCE_FILES_PROPERTIES(
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}c"
    PROPERTIES GENERATED TRUE)

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}c"
    )
ENDMACRO()

# PYTHON_INSTALL_BUILD(MODULE FILE DEST)
# --------------------------------------
#
# Install a Python file residing in the build directory and its
# associated compiled version.
#
MACRO(PYTHON_INSTALL_BUILD MODULE FILE DEST)

  FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}")

  INSTALL(CODE
    "EXECUTE_PROCESS(COMMAND
    \"${PYTHON_EXECUTABLE}\"
    \"${PROJECT_SOURCE_DIR}/cmake/compile.py\"
    \"${CMAKE_CURRENT_BINARY_DIR}\"
    \"${CMAKE_CURRENT_BINARY_DIR}\"
    \"${MODULE}/${FILE}\")
    ")

  # Tag pyc file as generated.
  SET_SOURCE_FILES_PROPERTIES(
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}c"
    PROPERTIES GENERATED TRUE)

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}c"
    )

  INSTALL(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}"
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}c"
    DESTINATION "${DEST}/${MODULE}")
ENDMACRO()

#.rst:
# .. command:: FIND_NUMPY
#
#   Detect numpy module
#

MACRO(FIND_NUMPY)
  # Detect numpy.
  MESSAGE (STATUS "checking for numpy")
  EXECUTE_PROCESS(
    COMMAND "${PYTHON_EXECUTABLE}" "-c"
    "import numpy; print (numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIRS
    ERROR_QUIET)
  IF (NOT NUMPY_INCLUDE_DIRS)
    MESSAGE (FATAL_ERROR "Failed to detect numpy")
  ELSE ()
    STRING(REGEX REPLACE "\n$" "" NUMPY_INCLUDE_DIRS "${NUMPY_INCLUDE_DIRS}")
    MESSAGE (STATUS " NUMPY_INCLUDE_DIRS=${NUMPY_INCLUDE_DIRS}")
  ENDIF()
ENDMACRO()
