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

INCLUDE(cmake/shared-library.cmake)

FIND_PACKAGE(PkgConfig)

# Additional pkg-config variables whose value will be imported
# during the dependency check.
SET(PKG_CONFIG_ADDITIONAL_VARIABLES bindir pkglibdir datarootdir pkgdatarootdir docdir doxygendocdir)

# _SETUP_PROJECT_PKG_CONFIG
# -------------------------
#
# Prepare pkg-config pc file generation step.
#
MACRO(_SETUP_PROJECT_PKG_CONFIG)
  # Pkg-config related commands.
  SET(_PKG_CONFIG_PREFIX "${CMAKE_INSTALL_PREFIX}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_EXEC_PREFIX "${_PKG_CONFIG_PREFIX}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_FULL_LIBDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_BINDIR "${CMAKE_INSTALL_FULL_BINDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_PKGLIBDIR "${CMAKE_INSTALL_FULL_PKGLIBDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_FULL_INCLUDEDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_DATAROOTDIR "${CMAKE_INSTALL_FULL_DATAROOTDIR}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_PKGDATAROOTDIR "${CMAKE_INSTALL_FULL_DATADIR}" CACHE INTERNAL "")
  IF(_INSTALL_DOC)
    SET(_PKG_CONFIG_DOCDIR "${CMAKE_INSTALL_FULL_DOCDIR}" CACHE INTERNAL "")
    SET(_PKG_CONFIG_DOXYGENDOCDIR "${_PKG_CONFIG_DOCDIR}/doxygen-html" CACHE INTERNAL "")
  ELSE(_INSTALL_DOC)
    SET(_PKG_CONFIG_DOCDIR "" CACHE INTERNAL "")
    SET(_PKG_CONFIG_DOXYGENDOCDIR "" CACHE INTERNAL "")
  ENDIF(_INSTALL_DOC)

  IF(DEFINED PROJECT_DEBUG_POSTFIX)
    IF(DEFINED CMAKE_CONFIGURATION_TYPES)
      SET(_PKG_CONFIG_PROJECT_NAME_NOPOSTFIX "${PROJECT_NAME}" CACHE INTERNAL "")
      SET(_PKG_CONFIG_PROJECT_NAME "${PROJECT_NAME}${PKGCONFIG_POSTFIX}" CACHE INTERNAL "")
    ELSE()
      SET(_PKG_CONFIG_PROJECT_NAME "${PROJECT_NAME}${PKGCONFIG_POSTFIX}" CACHE INTERNAL "")
    ENDIF()
  ELSE()
    SET(_PKG_CONFIG_PROJECT_NAME "${PROJECT_NAME}" CACHE INTERNAL "")
  ENDIF()
  SET(_PKG_CONFIG_DESCRIPTION "${PROJECT_DESCRIPTION}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_URL "${PROJECT_URL}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_VERSION "${PROJECT_VERSION}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_REQUIRES "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_REQUIRES_DEBUG "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_REQUIRES_OPTIMIZED "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CONFLICTS "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS_DEBUG "${LIBDIR_KW}\${libdir}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS_OPTIMIZED "${LIBDIR_KW}\${libdir}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_LIBS_PRIVATE "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CFLAGS "-I\${includedir}" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CFLAGS_DEBUG "" CACHE INTERNAL "")
  SET(_PKG_CONFIG_CFLAGS_OPTIMIZED "" CACHE INTERNAL "")

  SET(PKG_CONFIG_EXTRA "")

  # Where to install the pkg-config file?
  SET(_PKG_CONFIG_DIR "${_PKG_CONFIG_LIBDIR}/pkgconfig" CACHE INTERNAL "")

  # Watch variables.
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    _PKG_CONFIG_FOUND
    PKG_CONFIG_EXECUTABLE
    _PKG_CONFIG_PREFIX
    _PKG_CONFIG_EXEC_PREFIX
    _PKG_CONFIG_LIBDIR
    _PKG_CONFIG_BINDIR
    _PKG_CONFIG_PKGLIBDIR
    _PKG_CONFIG_INCLUDEDIR
    _PKG_CONFIG_DATAROOTDIR
    _PKG_CONFIG_PKGDATAROOTDIR
    _PKG_CONFIG_DOCDIR
    _PKG_CONFIG_DOXYGENDOCDIR
    _PKG_CONFIG_PROJECT_NAME
    _PKG_CONFIG_DESCRIPTION
    _PKG_CONFIG_URL
    _PKG_CONFIG_VERSION
    _PKG_CONFIG_REQUIRES
    _PKG_CONFIG_REQUIRES_DEBUG
    _PKG_CONFIG_REQUIRES_OPTIMIZED
    _PKG_CONFIG_CONFLICTS
    _PKG_CONFIG_LIBS
    _PKG_CONFIG_LIBS_DEBUG
    _PKG_CONFIG_LIBS_OPTIMIZED
    _PKG_CONFIG_LIBS_PRIVATE
    _PKG_CONFIG_CFLAGS
    _PKG_CONFIG_CFLAGS_DEBUG
    _PKG_CONFIG_CFLAGS_OPTIMIZED
    PKG_CONFIG_EXTRA
    )
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG)


# _SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG
# ----------------------------------
#
# Post-processing of the pkg-config step.
#
# The pkg-config file has to be generated at the end to allow end-user
# defined variables replacement.
#
# This macro adds _PKG_CONFIG_LIBS_DEBUG to _PKG_CONFIG_LIBS and
# _PKGCONFIG_CFLAGS_DEBUG to _PKG_CONFIG_CFLAGS
#
MACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG)
  # Setup altered variables
  SET(TEMP_CFLAGS ${_PKG_CONFIG_CFLAGS})
  SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS_DEBUG} ${_PKG_CONFIG_CFLAGS}")
  SET(TEMP_LIBS ${_PKG_CONFIG_LIBS})
  SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS_DEBUG} ${_PKG_CONFIG_LIBS}")
  SET(TEMP_REQUIRES ${_PKG_CONFIG_REQUIRES})
  _ADD_TO_LIST(_PKG_CONFIG_REQUIRES "${_PKG_CONFIG_REQUIRES_DEBUG}" ",")
  CONFIGURE_FILE(
    "${PROJECT_SOURCE_DIR}/cmake/pkg-config.pc.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}${PKGCONFIG_POSTFIX}.pc"
    )
  # Restore altered variables
  SET(_PKG_CONFIG_CFLAGS ${TEMP_CFLAGS})
  SET(_PKG_CONFIG_LIBS ${TEMP_LIBS})
  SET(_PKG_CONFIG_REQUIRES ${TEMP_REQUIRES})

  INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}${PKGCONFIG_POSTFIX}.pc"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG)

# _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED
# ----------------------------------
#
# Post-processing of the pkg-config step.
#
# The pkg-config file has to be generated at the end to allow end-user
# defined variables replacement.
#
# This macro adds _PKG_CONFIG_LIBS_OPTIMIZED to _PKG_CONFIG_LIBS and
# _PKGCONFIG_CFLAGS_OPTIMIZED to _PKG_CONFIG_CFLAGS
#
MACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED)
  # Setup altered variables
  SET(TEMP_PROJECT_NAME ${_PKG_CONFIG_PROJECT_NAME})
  SET(_PKG_CONFIG_PROJECT_NAME ${_PKG_CONFIG_PROJECT_NAME_NOPOSTFIX})
  SET(TEMP_CFLAGS ${_PKG_CONFIG_CFLAGS})
  SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS_OPTIMIZED} ${_PKG_CONFIG_CFLAGS}")
  SET(TEMP_LIBS ${_PKG_CONFIG_LIBS})
  SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS_OPTIMIZED} ${_PKG_CONFIG_LIBS}")
  SET(TEMP_REQUIRES ${_PKG_CONFIG_REQUIRES})
  _ADD_TO_LIST(_PKG_CONFIG_REQUIRES "${_PKG_CONFIG_REQUIRES_OPTIMIZED}" ",")
  # Generate the pkg-config file.
  CONFIGURE_FILE(
    "${PROJECT_SOURCE_DIR}/cmake/pkg-config.pc.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
    )
  # Restore altered variables
  SET(_PKG_CONFIG_PROJECT_NAME ${TEMP_PROJECT_NAME})
  SET(_PKG_CONFIG_CFLAGS ${TEMP_CFLAGS})
  SET(_PKG_CONFIG_LIBS ${TEMP_LIBS})
  SET(_PKG_CONFIG_REQUIRES ${TEMP_REQUIRES})

  INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED)

# _SETUP_PROJECT_PKG_CONFIG_FINALIZE
# ----------------------------------
#
# Post-processing of the pkg-config step.
#
# The pkg-config file has to be generated at the end to allow end-user
# defined variables replacement.
#
MACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE)
  # Single build type generator
  IF(DEFINED CMAKE_BUILD_TYPE)
    STRING(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type)
    IF(${cmake_build_type} MATCHES debug)
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG()
    ELSE()
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED()
    ENDIF()
  # Multiple build types generator
  ELSE()
    IF(DEFINED PROJECT_DEBUG_POSTFIX)
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_DEBUG()
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED()
    ELSE()
      _SETUP_PROJECT_PKG_CONFIG_FINALIZE_OPTIMIZED()
    ENDIF()
  ENDIF()
ENDMACRO(_SETUP_PROJECT_PKG_CONFIG_FINALIZE)


# ADD_DEPENDENCY(PREFIX P_REQUIRED COMPILE_TIME_ONLY PKGCONFIG_STRING)
# ------------------------------------------------
#
# Check for a dependency using pkg-config. Fail if the package cannot
# be found.
#
# P_REQUIRED : if set to 1 the package is required, otherwise it consider
#              as optional.
#              WARNING for optional package:
#              if the package is detected its compile
#              and linking options are still put in the required fields
#              of the generated pc file. Indeed from the binary viewpoint
#              the package becomes required.
#
# COMPILE_TIME_ONLY : if set to 1, the package is only requiered at compile time and won't
#                     appear as a dependency inside the *.pc file. 
#
# PKG_CONFIG_STRING	: string passed to pkg-config to check the version.
#			  Typically, this string looks like:
#                         ``my-package >= 0.5''
#
MACRO(ADD_DEPENDENCY P_REQUIRED COMPILE_TIME_ONLY PKG_CONFIG_STRING PKG_CONFIG_DEBUG_STRING)
  # Retrieve the left part of the equation to get package name.
  STRING(REGEX MATCH "[^<>= ]+" LIBRARY_NAME "${PKG_CONFIG_STRING}")
  # And transform it into a valid variable prefix.
  # 1. replace invalid characters into underscores.
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" PREFIX "${LIBRARY_NAME}")
  # 2. make it uppercase.
  STRING(TOUPPER "${PREFIX}" "PREFIX")
  IF(NOT ${PKG_CONFIG_DEBUG_STRING} STREQUAL "")
    # Retrieve the left part of the equation to get package name.
    STRING(REGEX MATCH "[^<>= ]+" LIBRARY_DEBUG_NAME "${PKG_CONFIG_DEBUG_STRING}")
    # And transform it into a valid variable prefix.
    # 1. replace invalid characters into underscores.
    STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" ${PREFIX}_DEBUG "${LIBRARY_DEBUG_NAME}")
    # 2. make it uppercase.
    STRING(TOUPPER "${PREFIX}_DEBUG" "${PREFIX}_DEBUG")
  ENDIF()

  # Force redetection each time CMake is launched.
  # Rationale: these values are *NEVER* manually set, so information is never
  # lost by overriding them. Moreover, changes in the pkg-config files are
  # not seen as long as the cache is not destroyed, even if the .pc file
  # is changed. This is a BAD behavior.
  SET(${PREFIX}_FOUND 0)
  IF(DEFINED ${PREFIX}_DEBUG)
    SET(${PREFIX}_DEBUG_FOUND 0)
  ENDIF()

  # This makes the debug dependency optional when building in release and
  # vice-versa, this only applies to single build type generators
  SET(PP_REQUIRED ${P_REQUIRED}) # Work-around macro limitation
  IF(DEFINED ${PREFIX}_DEBUG)
    SET(P_DEBUG_REQUIRED ${P_REQUIRED})
    IF(${P_REQUIRED})
      # Single build type generators
      IF(DEFINED CMAKE_BUILD_TYPE)
        STRING(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type)
        IF("${cmake_build_type}" MATCHES "debug")
          SET(PP_REQUIRED 0)
        ELSE()
          SET(P_DEBUG_REQUIRED 0)
        ENDIF()
      ENDIF()
    ENDIF()
  ENDIF()

  # Search for the package.
  IF(${PP_REQUIRED})
    MESSAGE(STATUS "${PKG_CONFIG_STRING} is required.")
    PKG_CHECK_MODULES("${PREFIX}" REQUIRED "${PKG_CONFIG_STRING}")
  ELSE(${PP_REQUIRED})
    MESSAGE(STATUS "${PKG_CONFIG_STRING} is optional.")
    PKG_CHECK_MODULES("${PREFIX}" "${PKG_CONFIG_STRING}")
  ENDIF(${PP_REQUIRED})

  # Search for the debug package
  IF(DEFINED ${PREFIX}_DEBUG)
    IF(${P_DEBUG_REQUIRED})
      MESSAGE(STATUS "${PKG_CONFIG_DEBUG_STRING} is required")
      PKG_CHECK_MODULES("${PREFIX}_DEBUG" REQUIRED "${PKG_CONFIG_DEBUG_STRING}")
    ELSE(${P_DEBUG_REQUIRED})
      MESSAGE(STATUS "${PKG_CONFIG_DEBUG_STRING} is optional")
      PKG_CHECK_MODULES("${PREFIX}_DEBUG" "${PKG_CONFIG_DEBUG_STRING}")
    ENDIF(${P_DEBUG_REQUIRED})
  ENDIF()

  # Fix for ld >= 2.24.90: -l:/some/absolute/path.so is no longer supported.
  # See shared-library.cmake.
  IF(UNIX AND NOT ${LD_VERSION} VERSION_LESS "2.24.90")
    STRING(REPLACE ":/" "/" "${PREFIX}_LIBRARIES" "${${PREFIX}_LIBRARIES}")
    STRING(REPLACE "-l:/" "/" "${PREFIX}_LDFLAGS" "${${PREFIX}_LDFLAGS}")

    IF(DEFINED ${PREFIX}_DEBUG)
      STRING(REPLACE ":/" "/" "${PREFIX}_DEBUG_LIBRARIES" "${${PREFIX}_DEBUG_LIBRARIES}")
      STRING(REPLACE "-l:/" "/" "${PREFIX}_DEBUG_LDFLAGS" "${${PREFIX}_DEBUG_LDFLAGS}")
    ENDIF()
  ENDIF()

  # Watch variables.
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    ${PREFIX}_FOUND
    ${PREFIX}_LIBRARIES
    ${PREFIX}_LIBRARY_DIRS
    ${PREFIX}_LDFLAGS
    ${PREFIX}_LDFLAGS_OTHER
    ${PREFIX}_INCLUDE_DIRS
    ${PREFIX}_CFLAGS
    ${PREFIX}_CFLAGS_OTHER
    ${PREFIX}
    ${PREFIX}_STATIC
    ${PREFIX}_VERSION
    ${PREFIX}_PREFIX
    ${PREFIX}_INCLUDEDIR
    ${PREFIX}_LIBDIR
    ${PREFIX}_PKGLIBDIR
    ${PREFIX}_BINDIR
    ${PREFIX}_DATAROOTDIR
    ${PREFIX}_PKGDATAROOTDIR
    ${PREFIX}_DOCDIR
    ${PREFIX}_DOXYGENDOCDIR
    )
  IF(DEFINED ${PREFIX}_DEBUG)
    LIST(APPEND LOGGING_WATCHED_VARIABLES
      ${PREFIX}_DEBUG_FOUND
      ${PREFIX}_DEBUG_LIBRARIES
      ${PREFIX}_DEBUG_LIBRARY_DIRS
      ${PREFIX}_DEBUG_LDFLAGS
      ${PREFIX}_DEBUG_LDFLAGS_OTHER
      ${PREFIX}_DEBUG_INCLUDE_DIRS
      ${PREFIX}_DEBUG_CFLAGS
      ${PREFIX}_DEBUG_CFLAGS_OTHER
      ${PREFIX}_DEBUG
      ${PREFIX}_DEBUG_STATIC
      ${PREFIX}_DEBUG_VERSION
      ${PREFIX}_DEBUG_PREFIX_DEBUG
      ${PREFIX}_DEBUG_INCLUDEDIR
      ${PREFIX}_DEBUG_LIBDIR
      ${PREFIX}_DEBUG_PKGLIBDIR
      ${PREFIX}_DEBUG_BINDIR
      ${PREFIX}_DEBUG_DATAROOTDIR
      ${PREFIX}_DEBUG_PKGDATAROOTDIR
      ${PREFIX}_DEBUG_DOCDIR
      ${PREFIX}_DEBUG_DOXYGENDOCDIR
      )
  ENDIF()

  # Get the values of additional variables.
  FOREACH(VARIABLE ${PKG_CONFIG_ADDITIONAL_VARIABLES})
    # Upper-case version of the variable for CMake variable generation.
    STRING(TOUPPER "${VARIABLE}" "VARIABLE_UC")
    EXECUTE_PROCESS(
      COMMAND "${PKG_CONFIG_EXECUTABLE}"
      "--variable=${VARIABLE}" "${LIBRARY_NAME}"
      OUTPUT_VARIABLE "${PREFIX}_${VARIABLE_UC}"
      ERROR_QUIET)
    STRING(REPLACE "\n" "" "${PREFIX}_${VARIABLE_UC}" "${${PREFIX}_${VARIABLE_UC}}")
    # Watch additional variables.
    LIST(APPEND LOGGING_WATCHED_VARIABLES ${PREFIX}_${VARIABLE_UC})
    IF(DEFINED ${PREFIX}_DEBUG)
      EXECUTE_PROCESS(
        COMMAND "${PKG_CONFIG_EXECUTABLE}"
        "--variable=${VARIABLE}" "${LIBRARY_DEBUG_NAME}"
        OUTPUT_VARIABLE "${PREFIX}_DEBUG_${VARIABLE_UC}"
        ERROR_QUIET)
      STRING(REPLACE "\n" "" "${PREFIX}_DEBUG_${VARIABLE_UC}" "${${PREFIX}_DEBUG_${VARIABLE_UC}}")
      LIST(APPEND LOGGING_WATCHED_VARIABLES ${PREFIX}_DEBUG_${VARIABLE_UC})
    ENDIF()
  ENDFOREACH(VARIABLE)

  #FIXME: spaces are replaced by semi-colon by mistakes, revert the change.
  #I cannot see why CMake is doing that...
  STRING(REPLACE ";" " " PKG_CONFIG_STRING "${PKG_CONFIG_STRING}")
  IF(DEFINED ${PREFIX}_DEBUG)
    STRING(REPLACE ";" " " PKG_CONFIG_DEBUG_STRING "${PKG_CONFIG_DEBUG_STRING}")
  ENDIF()

  # Add the package to the dependency list if found and if dependency
  # is triggered not only for documentation
  IF((${${PREFIX}_FOUND}) AND (NOT ${COMPILE_TIME_ONLY}))
    IF(DEFINED PROJECT_DEBUG_POSTFIX AND DEFINED ${PREFIX}_DEBUG)
      _ADD_TO_LIST(_PKG_CONFIG_REQUIRES_DEBUG "${PKG_CONFIG_DEBUG_STRING}" ",")
      _ADD_TO_LIST(_PKG_CONFIG_REQUIRES_OPTIMIZED "${PKG_CONFIG_STRING}" ",")
    ELSE()
      # Warn the user in case he/she is using alternative libraries for debug but no postfix
      IF(NOT DEFINED PROJECT_DEBUG_POSTFIX AND DEFINED ${PREFIX}_DEBUG)
        MESSAGE(AUTHOR_WARNING
          "You are linking with different libraries in debug mode but the
           generated .pc cannot reflect that, it will default to release flags. Consider
           setting PROJECT_DEBUG_POSTFIX to generate different libraries and pc files in
           debug mode.")
      ENDIF()

      _ADD_TO_LIST(_PKG_CONFIG_REQUIRES "${PKG_CONFIG_STRING}" ",")
    ENDIF()
  ENDIF()

  # Add the package to the cmake dependency list
  # if cpack has been included.
  # This is likely to disappear when Ubuntu 8.04 will
  # disappear.
  IF(COMMAND ADD_CMAKE_DEPENDENCY)
    ADD_CMAKE_DEPENDENCY(${PKG_CONFIG_STRING})
  ENDIF(COMMAND ADD_CMAKE_DEPENDENCY)

  IF(${${PREFIX}_FOUND})
   MESSAGE(STATUS
    "Pkg-config module ${LIBRARY_NAME} v${${PREFIX}_VERSION}"
    " has been detected with success.")
  ENDIF()
  IF(DEFINED ${PREFIX}_DEBUG AND "${${PREFIX}_DEBUG_FOUND}")
   MESSAGE(STATUS
    "Pkg-config module ${LIBRARY_DEBUG_NAME} v${${PREFIX}_DEBUG_VERSION}"
    " has been detected with success.")
  ENDIF()

ENDMACRO(ADD_DEPENDENCY)

# ADD_REQUIRED_DEPENDENCY(PREFIX PKGCONFIG_STRING)
# ------------------------------------------------
#
# Check for a dependency using pkg-config. Fail if the package cannot
# be found.
#
# PKG_CONFIG_STRING	: string passed to pkg-config to check the version.
#			  Typically, this string looks like:
#                         ``my-package >= 0.5''
#
# An optional argument can be passed to define an alternate PKG_CONFIG_STRING
# for debug builds. It should follow the same rule as PKG_CONFIG_STRING.
#
MACRO(ADD_REQUIRED_DEPENDENCY PKG_CONFIG_STRING)
  SET(PKG_CONFIG_DEBUG_STRING "")
  FOREACH(ARG ${ARGN})
    SET(PKG_CONFIG_DEBUG_STRING ${ARG})
  ENDFOREACH()
  ADD_DEPENDENCY(1 0 ${PKG_CONFIG_STRING} "${PKG_CONFIG_DEBUG_STRING}")
ENDMACRO(ADD_REQUIRED_DEPENDENCY)

# ADD_OPTIONAL_DEPENDENCY(PREFIX PKGCONFIG_STRING)
# ------------------------------------------------
#
# Check for a dependency using pkg-config. Quiet if the package cannot
# be found.
#
# PKG_CONFIG_STRING	: string passed to pkg-config to check the version.
#			  Typically, this string looks like:
#                         ``my-package >= 0.5''
#
# An optional argument can be passed to define an alternate PKG_CONFIG_STRING
# for debug builds. It should follow the same rule as PKG_CONFIG_STRING.
#
MACRO(ADD_OPTIONAL_DEPENDENCY PKG_CONFIG_STRING)
  SET(PKG_CONFIG_DEBUG_STRING "")
  FOREACH(ARG ${ARGN})
    SET(PKG_CONFIG_DEBUG_STRING ${ARG})
  ENDFOREACH()
  ADD_DEPENDENCY(0 0 ${PKG_CONFIG_STRING} "${PKG_CONFIG_DEBUG_STRING}")
ENDMACRO(ADD_OPTIONAL_DEPENDENCY)

# ADD_COMPILE_DEPENDENCY(PREFIX PKGCONFIG_STRING)
# ------------------------------------------------
#
# Check for a dependency using pkg-config. Fail if the package cannot be found.
# The package won't appear as depency inside the *.pc file of the PROJECT.
#
#
# PKG_CONFIG_STRING : string passed to pkg-config to check the version.
#       Typically, this string looks like:
#                         ``my-package >= 0.5''
#
# An optional argument can be passed to define an alternate PKG_CONFIG_STRING
# for debug builds. It should follow the same rule as PKG_CONFIG_STRING.
#
MACRO(ADD_COMPILE_DEPENDENCY PKG_CONFIG_STRING)
  SET(PKG_CONFIG_DEBUG_STRING "")
  FOREACH(ARG ${ARGN})
    SET(PKG_CONFIG_DEBUG_STRING ${ARG})
  ENDFOREACH()
  ADD_DEPENDENCY(1 1 ${PKG_CONFIG_STRING} "${PKG_CONFIG_DEBUG_STRING}")
ENDMACRO(ADD_COMPILE_DEPENDENCY)


# ADD_DOC_DEPENDENCY(PREFIX PKGCONFIG_STRING)
# ------------------------------------------------
#
# Check for a dependency using pkg-config. Do not express dependency in
# "requires" field.
#
# PKG_CONFIG_STRING	: string passed to pkg-config to check the version.
#			  Typically, this string looks like:
#                         ``my-package >= 0.5''
#
# An optional argument can be passed to define an alternate PKG_CONFIG_STRING
# for debug builds. It should follow the same rule as PKG_CONFIG_STRING.
#
MACRO(ADD_DOC_DEPENDENCY PKG_CONFIG_STRING)
  ADD_COMPILE_DEPENDENCY(${PKG_CONFIG_STRING})
ENDMACRO(ADD_DOC_DEPENDENCY)

# PKG_CONFIG_APPEND_LIBRARY_DIR
# -----------------------------
#
# This macro adds library directories in a portable way
# into the CMake file.
MACRO(PKG_CONFIG_APPEND_LIBRARY_DIR DIRS)
  FOREACH(DIR ${DIRS})
    IF(DIR)
      SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBDIR_KW}${DIR}" CACHE INTERNAL "")
    ENDIF(DIR)
  ENDFOREACH(DIR ${DIRS})
ENDMACRO(PKG_CONFIG_APPEND_LIBRARY_DIR DIR)

# PKG_CONFIG_APPEND_CFLAGS_DEBUG
# ------------------------
#
# This macro adds CFLAGS in a portable way into the pkg-config file of the debug library
# As such the macro fails if PROJECT_DEBUG_POSTFIX is not set
#
MACRO(PKG_CONFIG_APPEND_CFLAGS_DEBUG FLAGS)
  IF(NOT DEFINED PROJECT_DEBUG_POSTFIX)
    MESSAGE(FATAL_ERROR "You are trying to use PKG_CONFIG_APPEND_CFLAGS_DEBUG on a package that does not have a debug library")
  ENDIF()
  FOREACH(FLAG ${FLAGS})
    IF(FLAG)
      SET(_PKG_CONFIG_CFLAGS_DEBUG "${_PKG_CONFIG_CFLAGS_DEBUG} ${FLAG}" CACHE INTERNAL "")
    ENDIF(FLAG)
  ENDFOREACH(FLAG ${FLAGS})
ENDMACRO(PKG_CONFIG_APPEND_CFLAGS_DEBUG FLAGS)

# PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED
# ------------------------
#
# This macro adds CFLAGS in a portable way into the pkg-config file of the OPTIMIZED library
# As such the macro fails if PROJECT_OPTIMIZED_POSTFIX is not set
#
MACRO(PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED FLAGS)
  IF(NOT DEFINED PROJECT_DEBUG_POSTFIX)
    MESSAGE(FATAL_ERROR "You are trying to use PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED on a package that does not have a debug library")
  ENDIF()
  FOREACH(FLAG ${FLAGS})
    IF(FLAG)
      SET(_PKG_CONFIG_CFLAGS_OPTIMIZED "${_PKG_CONFIG_CFLAGS_OPTIMIZED} ${FLAG}" CACHE INTERNAL "")
    ENDIF(FLAG)
  ENDFOREACH(FLAG ${FLAGS})
ENDMACRO(PKG_CONFIG_APPEND_CFLAGS_OPTIMIZED FLAGS)

# PKG_CONFIG_APPEND_CFLAGS
# ------------------------
#
# This macro adds CFLAGS in a portable way into the pkg-config file.
#
MACRO(PKG_CONFIG_APPEND_CFLAGS FLAGS)
  FOREACH(FLAG ${FLAGS})
    IF(FLAG)
      SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS} ${FLAG}" CACHE INTERNAL "")
    ENDIF(FLAG)
  ENDFOREACH(FLAG ${FLAGS})
ENDMACRO(PKG_CONFIG_APPEND_CFLAGS)


# PKG_CONFIG_APPEND_LIBS_RAW
# ----------------------------
#
# This macro adds raw value in the "Libs:" into the pkg-config file.
#
# Exception for mac OS X:
# In addition to the classical static and dynamic libraries (handled like
# unix does), mac systems can link against frameworks.
# Frameworks are directories gathering headers, libraries, shared resources...
#
# The syntax used to link with a framework is particular, hence a filter is
# added to convert the absolute path to a framework (e.g. /Path/to/Sample.framework)
# into the correct flags (-F/Path/to/ -framework Sample).
#
MACRO(PKG_CONFIG_APPEND_LIBS_RAW LIBS)
  FOREACH(LIB ${LIBS})
    IF(LIB)
      IF( APPLE AND ${LIB} MATCHES "\\.framework")
	    GET_FILENAME_COMPONENT(framework_PATH ${LIB} PATH)
	    GET_FILENAME_COMPONENT(framework_NAME ${LIB} NAME_WE)
        SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} -F${framework_PATH} -framework ${framework_NAME}" CACHE INTERNAL "")
      ELSE( APPLE AND ${LIB} MATCHES "\\.framework")
        SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIB}" CACHE INTERNAL "")
      ENDIF( APPLE AND ${LIB} MATCHES "\\.framework")
    ENDIF(LIB)
  ENDFOREACH(LIB ${LIBS})
  STRING(REPLACE "\n" "" _PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS}")
ENDMACRO(PKG_CONFIG_APPEND_LIBS_RAW)


# PKG_CONFIG_APPEND_LIBS
# ----------------------
#
# This macro adds libraries in a portable way into the pkg-config
# file.
#
# Library prefix and suffix is automatically added.
#
MACRO(PKG_CONFIG_APPEND_LIBS LIBS)
  FOREACH(LIB ${LIBS})
    IF(LIB)
      # Check if this project is building this library
      IF(TARGET ${LIB})
        # Single build type generator
        IF(DEFINED CMAKE_BUILD_TYPE)
          SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBINCL_KW}${LIB}${PKGCONFIG_POSTFIX}${LIB_EXT}" CACHE INTERNAL "")
        # Multiple build types generator
        ELSE()
          SET(_PKG_CONFIG_LIBS_DEBUG "${_PKG_CONFIG_LIBS_DEBUG} ${LIBINCL_KW}${LIB}${PKGCONFIG_POSTFIX}${LIB_EXT}" CACHE INTERNAL "")
          SET(_PKG_CONFIG_LIBS_OPTIMIZED "${_PKG_CONFIG_LIBS_OPTIMIZED} ${LIBINCL_KW}${LIB}${LIB_EXT}" CACHE INTERNAL "")
        ENDIF()
      ELSE()
        IF(IS_ABSOLUTE ${LIB})
          SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBINCL_ABSKW}${LIB}" CACHE INTERNAL "")
        ELSE()
          SET(_PKG_CONFIG_LIBS "${_PKG_CONFIG_LIBS} ${LIBINCL_KW}${LIB}${LIB_EXT}" CACHE INTERNAL "")
        ENDIF()
      ENDIF()
    ENDIF(LIB)
  ENDFOREACH(LIB ${LIBS})
ENDMACRO(PKG_CONFIG_APPEND_LIBS)


# For internal use only.
# PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(TARGET DEPENDENCY)
# --------------------------------------------
#
# For user look at PKG_CONFIG_USE_COMPILE_DEPENDENCY
#
# This macro changes the target properties to properly search for
# headers  against the required shared libraries
# when using a dependency detected through pkg-config.
#
# I.e. PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(my-binary my-package)
#
MACRO(PKG_CONFIG_USE_LCOMPILE_DEPENDENCY TARGET PREFIX)

  # COMPILE_OPTION variable appeared only since CMake 2.8.12
  IF(${CMAKE_VERSION} VERSION_LESS 2.8.12)
    SET(COMPILE_OPTIONS_NAME COMPILE_FLAGS)
  ELSE()
    SET(COMPILE_OPTIONS_NAME COMPILE_OPTIONS)
  ENDIF()

  GET_TARGET_PROPERTY(CFLAGS ${TARGET} ${COMPILE_OPTIONS_NAME})
  IF(NOT CFLAGS)
    SET(CFLAGS "")
  ENDIF()

  IF(DEFINED ${PREFIX}_DEBUG_FOUND)
    FOREACH(FLAG ${${PREFIX}_DEBUG_CFLAGS_OTHER})
      LIST(APPEND CFLAGS "$<$<CONFIG:Debug>:${FLAG}>")
    ENDFOREACH()
    FOREACH(FLAG ${${PREFIX}_CFLAGS_OTHER})
      LIST(APPEND CFLAGS "$<$<NOT:$<CONFIG:Debug>>:${FLAG}>")
    ENDFOREACH()
  ELSE()
    FOREACH(FLAG ${${PREFIX}_CFLAGS_OTHER})
      LIST(APPEND CFLAGS "${FLAG}")
    ENDFOREACH()
  ENDIF()

  IF(${CMAKE_VERSION} VERSION_LESS 2.8.12)
    # This cast from LIST to STRING is mandatory for old CMake versions,
    # but harmful for CMake 2.8.12, and useless for 3.2.2 and above
    STRING(REPLACE ";" " " CFLAGS "${CFLAGS}")
  ENDIF(${CMAKE_VERSION} VERSION_LESS 2.8.12)

  SET_TARGET_PROPERTIES(${TARGET} PROPERTIES ${COMPILE_OPTIONS_NAME} "${CFLAGS}")

  # Include/libraries paths seems to be filtered on Linux, add paths
  # again.
  INCLUDE_DIRECTORIES(SYSTEM ${${PREFIX}_INCLUDE_DIRS})
  IF(DEFINED ${PREFIX}_DEBUG_FOUND)
    INCLUDE_DIRECTORIES(SYSTEM ${${PREFIX}_DEBUG_INCLUDE_DIRS})
  ENDIF()

ENDMACRO(PKG_CONFIG_USE_LCOMPILE_DEPENDENCY)

# Internal use only.
# _PKG_CONFIG_MANIPULATE_LDFLAGS(TARGET PREFIX CONFIG IS_GENERAL IS_DEBUG)
#
MACRO(_PKG_CONFIG_MANIPULATE_LDFLAGS TARGET PREFIX CONFIG IS_GENERAL IS_DEBUG)
  # Make sure we do not override previous flags
  GET_TARGET_PROPERTY(LDFLAGS ${TARGET} LINK_FLAGS${CONFIG})

  # If there were no previous flags, get rid of the XYFLAGS-NOTFOUND
  # in the variables.
  IF(NOT LDFLAGS)
    SET(LDFLAGS "")
  ENDIF()

  # Transform semi-colon seperated list in to space separated list.
  FOREACH(FLAG ${${PREFIX}_LDFLAGS})
    SET(LDFLAGS "${LDFLAGS} ${FLAG}")
  ENDFOREACH()

  # Update the flags.
  SET_TARGET_PROPERTIES(${TARGET}
    PROPERTIES LINK_FLAGS${CONFIG} "${LDFLAGS}")

  IF(UNIX AND NOT APPLE)
    IF(${IS_GENERAL})
      TARGET_LINK_LIBRARIES(${TARGET} ${${PREFIX}_LDFLAGS})
      TARGET_LINK_LIBRARIES(${TARGET} ${${PREFIX}_LDFLAGS_OTHER})
    ELSEIF(${IS_DEBUG})
      FOREACH(FLAG ${${PREFIX}_LDFLAGS})
        TARGET_LINK_LIBRARIES(${TARGET} debug ${FLAG})
      ENDFOREACH()
      FOREACH(FLAG ${${PREFIX}_LDFLAGS_OTHER})
        TARGET_LINK_LIBRARIES(${TARGET} debug ${FLAG})
      ENDFOREACH()
    ELSE()
      FOREACH(FLAG ${${PREFIX}_LDFLAGS})
        TARGET_LINK_LIBRARIES(${TARGET} optimized ${FLAG})
      ENDFOREACH()
      FOREACH(FLAG ${${PREFIX}_LDFLAGS_OTHER})
        TARGET_LINK_LIBRARIES(${TARGET} optimized ${FLAG})
      ENDFOREACH()
    ENDIF()
  ENDIF(UNIX AND NOT APPLE)
ENDMACRO(_PKG_CONFIG_MANIPULATE_LDFLAGS TARGET PREFIX CONFIG IS_GENERAL IS_DEBUG)

# Internal use only.
# PKG_CONFIG_USE_LLINK_DEPENDENCY(TARGET DEPENDENCY)
# --------------------------------------------
#
# For user look at PKG_CONFIG_USE_LINK_DEPENDENCY
#
# This macro changes the target properties to properly search for
# the required shared libraries
# when using a dependency detected through pkg-config.
#
# I.e. PKG_CONFIG_USE_LLINK_DEPENDENCY(my-binary my-package)
#
MACRO(PKG_CONFIG_USE_LLINK_DEPENDENCY TARGET PREFIX)

  IF(NOT DEFINED ${PREFIX}_DEBUG_FOUND)
    _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} ${PREFIX} "" 1 0)
  ELSE()
    # Single build type generators
    IF(DEFINED CMAKE_BUILD_TYPE)
      STRING(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type)
      IF("${cmake_build_type}" MATCHES "debug")
        _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} "${PREFIX}_DEBUG" "" 1 0)
      ELSE()
        _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} ${PREFIX} "" 1 0)
      ENDIF()
    # Multiple build types generators
    ELSE()
      FOREACH(config ${CMAKE_CONFIGURATION_TYPES})
        STRING(TOUPPER "_${config}" config_in)
        IF(${config_in} MATCHES "_DEBUG")
          _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} "${PREFIX}_DEBUG" "${config_in}" 0 1)
        ELSE()
          _PKG_CONFIG_MANIPULATE_LDFLAGS(${TARGET} "${PREFIX}" "${config_in}" 0 0)
        ENDIF()
      ENDFOREACH()
    ENDIF()
  ENDIF()

  # Include/libraries paths seems to be filtered on Linux, add paths
  # again.
  LINK_DIRECTORIES(${${PREFIX}_LIBRARY_DIRS})
  IF(DEFINED ${PREFIX}_DEBUG_FOUND)
    LINK_DIRECTORIES(${${PREFIX}_DEBUG_LIBRARY_DIRS})
  ENDIF()

ENDMACRO(PKG_CONFIG_USE_LLINK_DEPENDENCY)

MACRO(BUILD_PREFIX_FOR_PKG DEPENDENCY PREFIX)

  # Transform the dependency into a valid variable prefix.
  # 1. replace invalid characters into underscores.
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" LPREFIX "${DEPENDENCY}")
  # 2. make it uppercase.
  STRING(TOUPPER "${LPREFIX}" "LPREFIX")

  # Make sure we search for a previously detected package.
  IF(NOT DEFINED ${LPREFIX}_FOUND)
    MESSAGE(FATAL_ERROR
      "The package ${DEPENDENCY} has not been detected correctly.\n"
      "Have you called ADD_REQUIRED_DEPENDENCY/ADD_OPTIONAL_DEPENDENCY?")
  ENDIF()
  IF(NOT (${LPREFIX}_FOUND OR ${LPREFIX}_DEBUG_FOUND))
    MESSAGE(FATAL_ERROR
      "The package ${DEPENDENCY} has not been found.")
  ENDIF()

  SET(${PREFIX} ${LPREFIX})

ENDMACRO(BUILD_PREFIX_FOR_PKG)

# PKG_CONFIG_USE_DEPENDENCY(TARGET DEPENDENCY)
# --------------------------------------------
#
# This macro changes the target properties to properly search for
# headers, libraries and link against the required shared libraries
# when using a dependency detected through pkg-config.
#
# I.e. PKG_CONFIG_USE_DEPENDENCY(my-binary my-package)
#
MACRO(PKG_CONFIG_USE_DEPENDENCY TARGET DEPENDENCY)
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)
  PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(${TARGET} ${PREFIX})
  PKG_CONFIG_USE_LLINK_DEPENDENCY(${TARGET} ${PREFIX})
ENDMACRO(PKG_CONFIG_USE_DEPENDENCY TARGET DEPENDENCY)


# PKG_CONFIG_USE_COMPILE_DEPENDENCY(TARGET DEPENDENCY)
# --------------------------------------------
#
# This macro changes the target properties to properly search for
# headers  against the required shared libraries
# when using a dependency detected through pkg-config.
#
# I.e. PKG_CONFIG_USE_COMPILE_DEPENDENCY(my-binary my-package)
MACRO(PKG_CONFIG_USE_COMPILE_DEPENDENCY TARGET DEPENDENCY)
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)
  PKG_CONFIG_USE_LCOMPILE_DEPENDENCY(${TARGET} ${PREFIX})
ENDMACRO(PKG_CONFIG_USE_COMPILE_DEPENDENCY TARGET DEPENDENCY)

# PKG_CONFIG_USE_LINK_DEPENDENCY(TARGET DEPENDENCY)
# --------------------------------------------
#
# This macro changes the target properties to properly search for
# the required shared libraries
# when using a dependency detected through pkg-config.
#
# I.e. PKG_CONFIG_USE_LINK_DEPENDENCY(my-binary my-package)
MACRO(PKG_CONFIG_USE_LINK_DEPENDENCY TARGET DEPENDENCY)
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)
  PKG_CONFIG_USE_LLINK_DEPENDENCY(${TARGET} ${PREFIX})
ENDMACRO(PKG_CONFIG_USE_LINK_DEPENDENCY TARGET DEPENDENCY)


# PKG_CONFIG_ADD_COMPILE_OPTIONS(COMPILE_OPTIONS DEPENDENCY)
# ----------------------------------------------------------
#
# This macro adds the compile-time options for a given pkg-config
# dependency to a given semi-colon-separated list. This can be
# used to provide options to CUDA_ADD_LIBRARY for instance, since
# it does not support SET_TARGET_PROPERTIES...
#
# I.e. PKG_CONFIG_ADD_COMPILE_OPTIONS(${MY_OPTIONS} my-package)
MACRO(PKG_CONFIG_ADD_COMPILE_OPTIONS COMPILE_OPTIONS DEPENDENCY)
  BUILD_PREFIX_FOR_PKG(${DEPENDENCY} PREFIX)

  # If there were no previous options
  IF(NOT ${COMPILE_OPTIONS})
    SET(${COMPILE_OPTIONS} "")
  ENDIF()

  # Append flags
  FOREACH(FLAG ${${PREFIX}_CFLAGS_OTHER})
    LIST(APPEND COMPILE_OPTIONS "${FLAG}")
  ENDFOREACH()
ENDMACRO(PKG_CONFIG_ADD_COMPILE_OPTIONS COMPILE_OPTIONS DEPENDENCY)
