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

# _SETUP_PROJECT_HEADER
# ---------------------
#
# This setup CMake to handle headers properly.
#
# 1. The `include` directory in the build and source trees is added
#    to the include search path (see INCLUDE_DIRECTORIES).
#    As always, the build directory has the priority over the source
#    directory in case of conflict.
#
#    However you *should not* have conflicting names
#    for files which are both in the build and source trees.
#    Conflicting names are filenames which differ only by a prefix:
#
#    include/a.h vs _build/include/a.h
#    src/a.h     vs src/foo/a.h
#
#    ...this files makes a project very fragile as the -I ordering
#    will have a lot of importance and may break easily when using
#    tools which may reorder the pre-processor flags such as pkg-config.
#
#
# 2. The headers are installed in the prefix
#    in a way which preserves the directory structure.
#
#    The directory name for header follows the rule:
#    each non alpha-numeric character is replaced by a slash (`/`).
#    In practice, it means that hpp-util will put its header in:
#    ${CMAKE_INSTALL_PREFIX}/include/hpp/util
#
#    This rule has been decided to homogenize headers location, however
#    some packages do not follow this rule (dg-middleware for instance).
#
#    In that case, CUSTOM_HEADER_DIR can be set to override this policy.
#
#    Reminder: breaking the JRL/LAAS policies shoud be done after
#              discussing the issue. You should at least open a ticket
#              or send an e-mail to notify this behavior.
#
MACRO(_SETUP_PROJECT_HEADER)
  # Install project headers.
  IF(DEFINED CUSTOM_HEADER_DIR)
    SET(HEADER_DIR "${CUSTOM_HEADER_DIR}")
  ELSE(DEFINED CUSTOM_HEADER_DIR)
    STRING(REGEX REPLACE "[^a-zA-Z0-9]" "/" HEADER_DIR "${PROJECT_NAME}")
  ENDIF(DEFINED CUSTOM_HEADER_DIR)

  STRING(TOLOWER "${HEADER_DIR}" "HEADER_DIR")

  # Generate config.hh header.
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_"
    PACKAGE_CPPNAME "${PROJECT_NAME}")
  STRING(TOLOWER "${PACKAGE_CPPNAME}" "PACKAGE_CPPNAME_LOWER")
  STRING(TOUPPER "${PACKAGE_CPPNAME}" "PACKAGE_CPPNAME")
  GENERATE_CONFIGURATION_HEADER(
    ${HEADER_DIR} config.hh ${PACKAGE_CPPNAME}
    ${PACKAGE_CPPNAME_LOWER}_EXPORTS)

  # Generate deprecated.hh header.
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/deprecated.hh.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIR}/deprecated.hh
    @ONLY
    )
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIR}/deprecated.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${HEADER_DIR}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    )
  # Generate warning.hh header.
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/warning.hh.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIR}/warning.hh
    @ONLY
    )

  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIR}/warning.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${HEADER_DIR}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    )


  # Generate config.h header.
  # This header, unlike the previous one is *not* installed and is generated
  # in the top-level directory of the build tree.
  # Therefore it must not be inluded by any distributed header.
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
    )

  # Default include directories:
  # - top-level build directory (for generated non-distributed headers).
  # - include directory in the build tree (for generated, distributed headers).
  # - include directory in the source tree (non-generated, distributed headers).
  INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${PROJECT_SOURCE_DIR}/include
    )
ENDMACRO(_SETUP_PROJECT_HEADER)

# GENERATE_CONFIGURATION_HEADER
# -----------------------------
#
# This macro generates a configuration header. Macro parameters may be
# used to customize it.
#
# HEADER_DIR    : where to generate the header
# FILENAME      : how should the file named
# LIBRARY_NAME  : CPP symbol prefix, should match the compiled library name
# EXPORT_SYMBOl : what symbol controls the switch between symbol import/export
FUNCTION(GENERATE_CONFIGURATION_HEADER
    HEADER_DIR FILENAME LIBRARY_NAME EXPORT_SYMBOL)
  # Generate the header.
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/config.hh.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIR}/${FILENAME}
    @ONLY
    )
  # Install it.
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIR}/${FILENAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${HEADER_DIR}
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    )
ENDFUNCTION(GENERATE_CONFIGURATION_HEADER)


# _SETUP_PROJECT_HEADER_FINAlIZE
# ------------------------------
#
# Post-processing of the header management step.
# Install public headers if required.
#
MACRO(_SETUP_PROJECT_HEADER_FINAlIZE)
  # If the header list is set, install it.
  IF(DEFINED ${PROJECT_NAME}_HEADERS)
    FOREACH(FILE ${${PROJECT_NAME}_HEADERS})
      HEADER_INSTALL (${FILE})
    ENDFOREACH(FILE)
  ENDIF(DEFINED ${PROJECT_NAME}_HEADERS)
ENDMACRO(_SETUP_PROJECT_HEADER_FINAlIZE)


# HEADER_INSTALL(FILES)
# -------------------------------
#
# Install a list of headers.
#
MACRO(HEADER_INSTALL FILES)
  FOREACH(FILE ${FILES})
    GET_FILENAME_COMPONENT(DIR "${FILE}" PATH)
    STRING(REGEX REPLACE "${CMAKE_BINARY_DIR}" "" DIR "${DIR}")
    STRING(REGEX REPLACE "${PROJECT_SOURCE_DIR}" "" DIR "${DIR}")
    STRING(REGEX REPLACE "include(/|$)" "" DIR "${DIR}")
    INSTALL(FILES ${FILE}
      DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${DIR}"
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      )
  ENDFOREACH()
ENDMACRO()
