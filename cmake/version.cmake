# Copyright (C) 2008-2018 LAAS-CNRS, JRL AIST-CNRS, INRIA.
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
# .. command:: VERSION_COMPUTE
#
#  Deduce automatically the version number.
#  This mechanism makes sure that version number is always up-to-date and
#  coherent (i.e. strictly increasing as commits are made).
#
#  There is two cases:
#
#  - the software comes from a release (stable version). In this case, the
#    software is retrieved through a tarball which does not contain the ``.git``
#    directory. Hence, there is no way to search in the Git history to generate
#    the version number.
#    In this case, a ``.version`` file is put at the top-directory of the source
#    tree which contains the project version. Read the file to retrieve the
#    version number.
#
#  - the softwares comes from git (possibly unstable version).
#    ``git describe`` is used to retrieve the version number
#    (see 'man git-describe'). This tool generates a version number from the git
#    history. The version number follows this pattern:
#
#      ``TAG[-N-SHA1][-dirty]``
#
#    - ``TAG``: last matching tag (i.e. last signed tag starting with v, i.e. v0.1)
#    - ``N``: number of commits since the last maching tag
#    - ``SHA1``: sha1 of the current commit
#    - ``-dirty``: added if the workig directory is dirty (there is some uncommitted
#      changes).
#
#    For stable releases, i.e. the current commit is a matching tag, ``-N-SHA1`` is
#    omitted. If the HEAD is on the signed tag v0.1, the version number will be
#    0.1.
#
#    If the HEAD is two commits after v0.5 and the last commit is 034f6d...
#    The version number will be:
#
#    - ``0.5-2-034f`` if there is no uncommitted changes,
#    - ``0.5-2-034f-dirty`` if there is some uncommitted changes.
#
MACRO(VERSION_COMPUTE)
  SET(PROJECT_STABLE False)

  # Check if a version is embedded in the project.
  IF(EXISTS ${PROJECT_SOURCE_DIR}/.version)
    # Yes, use it. This is a stable version.
    FILE(STRINGS .version PROJECT_VERSION)
    SET(PROJECT_STABLE True)
  ELSE(EXISTS ${PROJECT_SOURCE_DIR}/.version)
    # No, there is no '.version' file. Deduce the version from git.

    # Search for git.
    FIND_PROGRAM(GIT git)
    IF(NOT GIT)
      MESSAGE("Warning: failed to compute the version number, git not found.")
      SET(PROJECT_VERSION UNKNOWN)
    ENDIF()

    # Run describe: search for *signed* tags starting with v, from the HEAD and
    # display only the first four characters of the commit id.
    EXECUTE_PROCESS(
      COMMAND ${GIT} describe --tags --abbrev=4 --match=v* HEAD
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      RESULT_VARIABLE GIT_DESCRIBE_RESULT
      OUTPUT_VARIABLE GIT_DESCRIBE_OUTPUT
      ERROR_VARIABLE GIT_DESCRIBE_ERROR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )

    # Run diff-index to check whether the tree is clean or not.
    EXECUTE_PROCESS(
      COMMAND ${GIT} diff-index --name-only HEAD
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
      RESULT_VARIABLE GIT_DIFF_INDEX_RESULT
      OUTPUT_VARIABLE GIT_DIFF_INDEX_OUTPUT
      ERROR_VARIABLE GIT_DIFF_INDEX_ERROR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )

    # Check if the tree is clean.
    IF(NOT GIT_DIFF_INDEX_RESULT AND NOT GIT_DIFF_INDEX_OUTPUT)
      SET(PROJECT_DIRTY False)
    ELSE()
      SET(PROJECT_DIRTY True)
    ENDIF()

    # Check if git describe worked and store the returned version number.
    IF(GIT_DESCRIBE_RESULT)
      MESSAGE(AUTHOR_WARNING
        "Warning: failed to compute the version number,"
        " 'git describe' failed:\n"
        "\t" ${GIT_DESCRIBE_ERROR})
      SET(PROJECT_VERSION UNKNOWN)
    ELSE()
      # Get rid of the tag prefix to generate the final version.
      STRING(REGEX REPLACE "^v" "" PROJECT_VERSION "${GIT_DESCRIBE_OUTPUT}")
      IF(NOT PROJECT_VERSION)
	      MESSAGE(AUTHOR_WARNING
	        "Warning: failed to compute the version number,"
          "'git describe' returned an empty string.")
        SET(PROJECT_VERSION UNKNOWN)
      ENDIF()

      # If there is a dash in the version number, it is an unstable release,
      # otherwise it is a stable release.
      # I.e. 1.0, 2, 0.1.3 are stable but 0.2.4-1-dg43 is unstable.
      STRING(REGEX MATCH "-" PROJECT_STABLE "${PROJECT_VERSION}")
      IF(NOT PROJECT_STABLE STREQUAL -)
        SET(PROJECT_STABLE True)
      ELSE()
        SET(PROJECT_STABLE False)
      ENDIF()
    ENDIF()

    # Append dirty if the project is dirty.
    IF(PROJECT_DIRTY)
      SET(PROJECT_VERSION "${PROJECT_VERSION}-dirty")
    ENDIF()
  ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/.version)
  
  # Set PROJECT_VERSION_{MAJOR,MINOR,PATCH} variables
  IF(PROJECT_VERSION)
    # Compute the major, minor and patch version of the project
    IF(NOT DEFINED PROJECT_VERSION_MAJOR AND
       NOT DEFINED PROJECT_VERSION_MINOR AND
       NOT DEFINED PROJECT_VERSION_PATCH)
      IF(PROJECT_VERSION MATCHES UNKNOWN)
        SET(PROJECT_VERSION_MAJOR UNKNOWN)
        SET(PROJECT_VERSION_MINOR UNKNOWN)
        SET(PROJECT_VERSION_PATCH UNKNOWN)
      ELSE()
        # Extract the version from PROJECT_VERSION
        string(REPLACE "." ";" _PROJECT_VERSION_LIST "${PROJECT_VERSION}")
        list(LENGTH _PROJECT_VERSION_LIST SIZE)
        IF(${SIZE} GREATER 0)
          list(GET _PROJECT_VERSION_LIST 0 PROJECT_VERSION_MAJOR)
        ENDIF()
        IF(${SIZE} GREATER 1)
          list(GET _PROJECT_VERSION_LIST 1 PROJECT_VERSION_MINOR)
        ENDIF()
        IF(${SIZE} GREATER 2)
          string(REPLACE "-" ";" _PROJECT_VERSION_LIST "${_PROJECT_VERSION_LIST}")
          list(GET _PROJECT_VERSION_LIST 2 PROJECT_VERSION_PATCH)
        ENDIF()
      ENDIF()
    ENDIF()
  ENDIF()

ENDMACRO()
