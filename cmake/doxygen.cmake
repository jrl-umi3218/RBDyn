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
# Available user options (to be set before calling SETUP_PROJECT):
#   DOXYGEN_DOT_IMAGE_FORMAT: format for dot images. Defaults to "svg".
#   DOXYGEN_USE_MATHJAX: use MathJax to render LaTeX equations. Defaults to "NO".
MACRO(_SETUP_PROJECT_DOCUMENTATION)

  # Search for Doxygen.
  FIND_PACKAGE(Doxygen)

  IF(NOT DOXYGEN_FOUND)
    MESSAGE(WARNING "Failed to find Doxygen, documentation will not be generated.")
  ELSE(NOT DOXYGEN_FOUND)
    # Generate variable to be substitued in Doxyfile.in
    # for dot use.
    IF(DOXYGEN_DOT_FOUND)
      SET(HAVE_DOT YES)
    ELSE(DOXYGEN_DOT_FOUND)
      SET(HAVE_DOT NO)
    ENDIF(DOXYGEN_DOT_FOUND)

    # Dot support.
    IF(NOT DEFINED DOXYGEN_DOT_IMAGE_FORMAT)
      SET(DOXYGEN_DOT_IMAGE_FORMAT "svg")
    ENDIF()

    # MathJax support.
    IF(NOT DEFINED DOXYGEN_USE_MATHJAX)
      SET(DOXYGEN_USE_MATHJAX "NO")
    ENDIF()

    # HTML style sheet configuration
    IF(NOT DEFINED DOXYGEN_USE_TEMPLATE_CSS)
      SET(DOXYGEN_USE_TEMPLATE_CSS "YES")
    ENDIF()

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

      IF(INSTALL_DOCUMENTATION)
        INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_MAKE_PROGRAM} doc)")
      ENDIF(INSTALL_DOCUMENTATION)
    ENDIF(MSVC)

    IF (DOXYGEN_USE_TEMPLATE_CSS)
      ADD_CUSTOM_COMMAND(
        OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/doc/header.html
        ${CMAKE_CURRENT_BINARY_DIR}/doc/footer.html
        ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen.css
        COMMAND ${DOXYGEN_EXECUTABLE} -w html
        ${CMAKE_CURRENT_BINARY_DIR}/doc/header.html
        ${CMAKE_CURRENT_BINARY_DIR}/doc/footer.html
        ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen.css
        WORKING_DIRECTORY doc
        COMMENT "Generating Doxygen template files"
        )
      ADD_CUSTOM_TARGET(generate-template-css
        DEPENDS
        ${CMAKE_CURRENT_BINARY_DIR}/doc/header.html
        ${CMAKE_CURRENT_BINARY_DIR}/doc/footer.html
        ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen.css
        )
      ADD_DEPENDENCIES(doc generate-template-css)
    ELSE (DOXYGEN_USE_TEMPLATE_CSS)
      FILE (COPY
        ${PROJECT_SOURCE_DIR}/cmake/doxygen/doxygen.css
        DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}/doc/
        )
    ENDIF (DOXYGEN_USE_TEMPLATE_CSS)

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

    # Install MathJax minimal version.
    IF(${DOXYGEN_USE_MATHJAX} STREQUAL "YES")
      FILE(COPY ${PROJECT_SOURCE_DIR}/cmake/doxygen/MathJax
           DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen-html)
    ENDIF()

    # Install generated files.
    IF(INSTALL_DOCUMENTATION)
      INSTALL(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/doc/${PROJECT_NAME}.doxytag
        DESTINATION ${CMAKE_INSTALL_DOCDIR}/doxygen-html)
      INSTALL(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen-html
        DESTINATION ${CMAKE_INSTALL_DOCDIR})

      IF(EXISTS ${PROJECT_SOURCE_DIR}/doc/pictures)
        INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/doc/pictures
          DESTINATION ${CMAKE_INSTALL_DOCDIR}/doxygen-html)
      ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/doc/pictures)
    ENDIF(INSTALL_DOCUMENTATION)

    LIST(APPEND LOGGING_WATCHED_VARIABLES
      DOXYGEN_SKIP_DOT
      DOXYGEN_EXECUTABLE
      DOXYGEN_FOUND
      DOXYGEN_DOT_EXECUTABLE
      DOXYGEN_DOT_FOUND
      DOXYGEN_DOT_PATH
      DOXYGEN_DOT_IMAGE_FORMAT
      DOXYGEN_USE_MATHJAX
      DOXYGEN_USE_TEMPLATE_CSS
      )
  ENDIF(NOT DOXYGEN_FOUND)
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
  IF(DOXYGEN_FOUND)
    IF(NOT ${DOXYGEN_USE_MATHJAX} STREQUAL "YES")
      # Make sure latex, dvips and gs are available
      FIND_PROGRAM(LATEX latex DOC "LaTeX compiler")
      FIND_PROGRAM(DVIPS dvips DOC "DVI to PostScript converter")
      FIND_PROGRAM(GS gs DOC "GhostScript interpreter")

      IF(NOT (LATEX AND GS AND DVIPS))
        MESSAGE(STATUS "Failed to find latex/dvips/gs, will use MathJax backend.")
        SET(DOXYGEN_USE_MATHJAX "YES")
      ENDIF()
    ENDIF()

    IF(${DOXYGEN_USE_MATHJAX} STREQUAL "YES")
      MESSAGE(STATUS "Doxygen rendering: using MathJax backend")
      SET(DOXYGEN_HEADER_NAME "header-mathjax.html")
    ELSE()
      MESSAGE(STATUS "Doxygen rendering: using LaTeX backend")
      SET(DOXYGEN_HEADER_NAME "header.html")
    ENDIF()

    IF(INSTALL_DOCUMENTATION)
      # Find doxytag files
      # To ignore this list of tag files, add to doc/Doxyfile.extra.in
      # TAGFILES =
      SET(PKG_REQUIRES ${_PKG_CONFIG_REQUIRES})
      _ADD_TO_LIST(PKG_REQUIRES "${_PKG_CONFIG_COMPILE_TIME_REQUIRES}" ",")
      STRING(REPLACE "," ";" PKG_REQUIRES "${PKG_REQUIRES}")
      FOREACH(PKG_CONFIG_STRING ${PKG_REQUIRES})
        _PARSE_PKG_CONFIG_STRING(${PKG_CONFIG_STRING} LIBRARY_NAME PREFIX)
        # If DOXYGENDOCDIR is specified, add a doc path.
        IF( DEFINED ${PREFIX}_DOXYGENDOCDIR
            AND EXISTS ${${PREFIX}_DOXYGENDOCDIR}/${LIBRARY_NAME}.doxytag)
          FILE(RELATIVE_PATH DEP_DOCDIR ${_PKG_CONFIG_DOXYGENDOCDIR} ${${PREFIX}_DOXYGENDOCDIR})

          SET(DOXYTAG_ENTRIES "${DOXYTAG_ENTRIES} \"${${PREFIX}_DOXYGENDOCDIR}/${LIBRARY_NAME}.doxytag\"=\"${DEP_DOCDIR}\"")
        ENDIF()
      ENDFOREACH()
    ENDIF()

    IF(EXISTS ${PROJECT_SOURCE_DIR}/include)
      SET (DOXYGEN_INCLUDE_PATH "${DOXYGEN_INCLUDE_PATH} \"${PROJECT_SOURCE_DIR}/include\"")
      SET (DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${PROJECT_SOURCE_DIR}/include\"")
    ENDIF()
    IF(EXISTS ${PROJECT_SOURCE_DIR}/src)
      SET (DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${PROJECT_SOURCE_DIR}/src\"")
    ENDIF()
    IF(EXISTS ${PROJECT_SOURCE_DIR}/tests)
      SET (DOXYGEN_EXAMPLE_PATH "${DOXYGEN_EXAMPLE_PATH} \"${PROJECT_SOURCE_DIR}/tests\"")
    ENDIF()
    SET (DOXYGEN_INCLUDE_PATH "${DOXYGEN_INCLUDE_PATH} \"${CMAKE_BINARY_DIR}/include\"")

    # Generate Doxyfile.extra.
    IF(EXISTS ${PROJECT_SOURCE_DIR}/doc/Doxyfile.extra.in)
      CONFIGURE_FILE(
        ${PROJECT_SOURCE_DIR}/doc/Doxyfile.extra.in
        ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile.extra
        @ONLY
        )
    ELSE()
      CONFIGURE_FILE(
        ${PROJECT_SOURCE_DIR}/cmake/doxygen/Doxyfile.extra.in
        ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile.extra
        @ONLY
        )
    ENDIF()
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
  ENDIF(DOXYGEN_FOUND)
ENDMACRO(_SETUP_PROJECT_DOCUMENTATION_FINALIZE)
