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

# _SETUP_PROJECT_DOCUMENTATION
# ----------------------------
#
# Look for Doxygen, add a custom rule to generate the documentation
# and install the documentation properly.
#
MACRO(_SETUP_PROJECT_DOCUMENTATION)
  # Search for Doxygen.
  FIND_PACKAGE(Doxygen)

  IF(NOT DOXYGEN_FOUND)
    MESSAGE(FATAL_ERROR "Failed to find Doxygen.")
  ENDIF(NOT DOXYGEN_FOUND)

  # Search for Perl.
  FIND_PROGRAM(PERL perl DOC "the Perl interpreter")
  IF(NOT PERL)
    MESSAGE(SEND_ERROR "Failed to find Perl.")
  ENDIF(NOT PERL)

  # Generate variable to be substitued in Doxyfile.in
  # for dot use.
  IF(DOXYGEN_DOT_FOUND)
    SET(HAVE_DOT YES)
  ELSE(DOXYGEN_DOT_FOUND)
    SET(HAVE_DOT NO)
  ENDIF(DOXYGEN_DOT_FOUND)

  # Teach CMake how to generate the documentation.
  IF(MSVC)
    # FIXME: it is impossible to trigger documentation installation
    # at install, so put the target in ALL instead.
    ADD_CUSTOM_TARGET(doc ALL
      COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
      WORKING_DIRECTORY doc
      COMMENT "Generating Doxygen documentation"
      )
  ELSE(MSVC)
    ADD_CUSTOM_TARGET(doc
      COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
      WORKING_DIRECTORY doc
      COMMENT "Generating Doxygen documentation"
      )

    INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_MAKE_PROGRAM} doc)")
  ENDIF(MSVC)

  ADD_CUSTOM_COMMAND(
    OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME}.doxytag
    ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen-html
    COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
    WORKING_DIRECTORY doc
    COMMENT "Generating Doxygen documentation"
    )

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME}.doxytag
    ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen.log
    ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen-html
    )

  # Install generated files.
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME}.doxytag
    DESTINATION ${CMAKE_INSTALL_DOCDIR}/doxygen-html)
  INSTALL(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen-html
    DESTINATION ${CMAKE_INSTALL_DOCDIR})

  IF(EXISTS ${PROJECT_SOURCE_DIR}/doc/pictures)
    INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/doc/pictures
      DESTINATION ${CMAKE_INSTALL_DOCDIR}/doxygen-html)
  ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/doc/pictures)

  LIST(APPEND LOGGING_WATCHED_VARIABLES
    DOXYGEN_SKIP_DOT
    DOXYGEN_EXECUTABLE
    DOXYGEN_FOUND
    DOXYGEN_DOT_EXECUTABLE
    DOXYGEN_DOT_FOUND
    DOXYGEN_DOT_PATH
    )
ENDMACRO(_SETUP_PROJECT_DOCUMENTATION)

# _SETUP_PROJECT_DOCUMENTATION_FINALIZE
# -------------------------------------
#
# Post-processing for the documentation generation macro.
#
# Doxyfile.extra and Doxyfile files are generated at the end to allow
# the replacement of user-defined variables.
#
MACRO(_SETUP_PROJECT_DOCUMENTATION_FINALIZE)
  # Generate Doxyfile.extra.
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/doc/Doxyfile.extra.in
    ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile.extra
    @ONLY
    )
  # Generate Doxyfile.
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/doxygen/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
    @ONLY
    )
  FILE(STRINGS ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile.extra doxyfile_extra)
  FOREACH(x ${doxyfile_extra})
    FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile ${x} "\n")
  ENDFOREACH(x in doxyfile_extra)
ENDMACRO(_SETUP_PROJECT_DOCUMENTATION_FINALIZE)