# Copyright (C) 2016 LAAS-CNRS, JRL AIST-CNRS.
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

# GENERATE_URDF_FILE FILENAME EXTENSION
# ------------------------------------
#
# Generate urdf ${CMAKE_CURRENT_BINARY_DIR}/${FILENAME}.${EXTENSION}
# from xacro ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro file.
#
# To trigger generation, use:
# ADD_CUSTOM_TARGET (generate_urdf_files DEPENDS ${ALL_GENERATED_URDF})
#
# FILENAME : XACRO filename without the extension
# EXTENSION : desired extension of the output file, e.g. "urdf" or "srdf"
#
# Note : If ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro does not exists,
#        the macros tries to configure file ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in
#
MACRO(GENERATE_URDF_FILE FILENAME EXTENSION)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS xacro)

  IF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)
    IF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in)
      MESSAGE(FATAL_ERROR "cannot find \"${FILENAME}.xacro\" or \"${FILENAME}.xacro.in\"")
    ENDIF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in)

    SET(_XACRO_FILE_ ${CMAKE_CURRENT_BINARY_DIR}/${FILENAME}.xacro)
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro.in
      ${_XACRO_FILE_} @ONLY)
    # MESSAGE("Configuring ${FILENAME}.xacro.in")
  ELSE (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)
    SET(_XACRO_FILE_ ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)
  ENDIF (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${FILENAME}.xacro)

  SET (OUTPUT_FILE ${FILENAME}.${EXTENSION})
  STRING (TOUPPER ${EXTENSION} EXT_UPPER)

  ADD_CUSTOM_COMMAND(
    OUTPUT ${OUTPUT_FILE}
    COMMAND ${_xacro_py}
    ARGS -o ${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_FILE} ${_XACRO_FILE_}
    MAIN_DEPENDENCY ${_XACRO_FILE_}
    COMMENT "Generating ${EXT_UPPER} from ${_XACRO_FILE_}"
    )
  LIST(APPEND ALL_GENERATED_URDF ${OUTPUT_FILE})

  # Clean generated files.
  SET_PROPERTY(
    DIRECTORY APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES
    ${OUTPUT_FILE}
    )

  LIST(APPEND LOGGING_WATCHED_VARIABLES ALL_GENERATED_URDF)
ENDMACRO(GENERATE_URDF_FILE FILENAME CONFIGURE)
