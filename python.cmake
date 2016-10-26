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


# FINDPYTHON
# ------------------------------------
#
# Find python interpreter and python libs.
# Arguments are passed to the find_package command so
# refer to find_package documentation to learn about valid arguments.
#
# For instance, the command
# FINDPYTHON(2.7 EXACT REQUIRED)
# will force CMake to find Python2.7
#
# WARNING: According to the FindPythonLibs and FindPythonInterp
# documentation, you could also set Python_ADDITIONAL_VERSIONS.
# If you do this, you will not have an error if you found two different versions
# or another version that the requested one.
#

IF(CMAKE_VERSION VERSION_LESS "3.2")
    SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/python ${CMAKE_MODULE_PATH})
    MESSAGE(WARNING "CMake versions older than 3.2 do not properly find Python. Custom macros are used to find it.")
ENDIF(CMAKE_VERSION VERSION_LESS "3.2")

MACRO(FINDPYTHON)
FIND_PACKAGE(PythonInterp ${ARGN})
IF (NOT ${PYTHONINTERP_FOUND} STREQUAL TRUE)
   MESSAGE(FATAL_ERROR "Python executable has not been found.")
ENDIF (NOT ${PYTHONINTERP_FOUND} STREQUAL TRUE)

FIND_PACKAGE(PythonLibs ${ARGN})
IF (NOT ${PYTHONLIBS_FOUND} STREQUAL TRUE)
   MESSAGE(FATAL_ERROR "Python has not been found.")
ENDIF (NOT ${PYTHONLIBS_FOUND} STREQUAL TRUE)

# Find PYTHON_LIBRARY_DIRS
GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIRS "${PYTHON_LIBRARIES}" PATH)

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
ENDMACRO(FINDPYTHON)


#
# DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME
# ---------------------------
#
# Add a python submodule to dynamic_graph
#
#  SUBMODULENAME : the name of the submodule (can be foo/bar),
#
#  LIBRARYNAME   : library to link the submodule with.
#
#  TARGETNAME    : name of the target: should be different for several
#                  calls to the macro.
#
#  NOTICE : Before calling this macro, set variable NEW_ENTITY_CLASS as
#           the list of new Entity types that you want to be bound.
#           Entity class name should match the name referencing the type
#           in the factory.
#
MACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME)


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
    ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/python-module-py.cc)

  FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME})

  SET_TARGET_PROPERTIES(${PYTHON_MODULE}
    PROPERTIES PREFIX ""
    OUTPUT_NAME dynamic_graph/${SUBMODULENAME}/wrap
   )
  CMAKE_POLICY(POP)

  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} "-Wl,--no-as-needed")
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${LIBRARYNAME} ${PYTHON_LIBRARY})

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

  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/submodule/__init__.py.cmake
    ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME}/__init__.py
    )

  INSTALL(
    FILES ${PROJECT_BINARY_DIR}/src/dynamic_graph/${SUBMODULENAME}/__init__.py
    DESTINATION ${PYTHON_INSTALL_DIR}
    )

ENDMACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME)


# PYTHON_INSTALL(MODULE FILE DEST)
# --------------------------------
#
# Install a Python file and its associated compiled version.
#
MACRO(PYTHON_INSTALL MODULE FILE DEST)

  PYTHON_BUILD("${MODULE}" "${FILE}")

  INSTALL(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/${MODULE}/${FILE}"
    "${CMAKE_CURRENT_BINARY_DIR}/${MODULE}/${FILE}c"
    DESTINATION "${DEST}/${MODULE}")
ENDMACRO()

# PYTHON_INSTALL_ON_SITE (MODULE FILE)
# --------------------------------
#
# Install a Python file and its associated compiled version.
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

# FIND_NUMPY
# ----------
#
# Detect numpy module
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
    MESSAGE (STATUS " NUMPY_INCLUDE_DIRS=${NUMPY_INCLUDE_DIRS}")
  ENDIF()
ENDMACRO()
