# Copyright (C) 2008-2014,2018 LAAS-CNRS, JRL AIST-CNRS.
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
# .. variable:: DISABLE_TESTS
#
#   Boolean variable to configure unit test compilation declared with
#   :command:`ADD_UNIT_TEST`.
#
#   A target *build_tests* is added to compile the unit-tests.
#   In all cases, ``make all && make test`` compiles and runs the unit-tests.
#
#   * if ``OFF`` (default), the unit-tests are compiled with target *all*,
#     as usual.
#   * if ``ON``, a unit-test called *ctest_build_tests* is added.
#     It is equivalent to the command ``make build_tests``.
#     All unit-test added with :command:`ADD_UNIT_TEST` will be executed
#     after unit-test *ctest_build_tests* completed.
#
#     Thus, the unit-tests are not compiled with target *all* but with target *test*.
#     unit-test  is added and all tests added with
IF(NOT DEFINED DISABLE_TESTS)
  SET(DISABLE_TESTS OFF)
ENDIF(NOT DEFINED DISABLE_TESTS)

ADD_CUSTOM_TARGET(build_tests)
IF(DISABLE_TESTS)
   ADD_TEST(ctest_build_tests "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target build_tests)
ENDIF()

#.rst:
# .. command:: ADD_UNIT_TEST (NAME SOURCE)
#
#   The behaviour of this function depends on :variable:`DISABLE_TESTS` option.
#
MACRO(ADD_UNIT_TEST NAME SOURCE)
  IF(DISABLE_TESTS)
    ADD_EXECUTABLE(${NAME} EXCLUDE_FROM_ALL ${SOURCE})
  ELSE(DISABLE_TESTS)
    ADD_EXECUTABLE(${NAME} ${SOURCE})
  ENDIF(DISABLE_TESTS)

  ADD_DEPENDENCIES(build_tests ${NAME})

  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})
  # Support definition of DYLD_LIBRARY_PATH for OSX systems
  IF(APPLE)
    SET_TESTS_PROPERTIES(${NAME} PROPERTIES ENVIRONMENT "DYLD_LIBRARY_PATH=$ENV{DYLD_LIBRARY_PATH}")
    SET_TESTS_PROPERTIES(${NAME} PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}")
  ENDIF(APPLE)

  IF(DISABLE_TESTS)
    SET_TESTS_PROPERTIES(${NAME} PROPERTIES DEPENDS ctest_build_tests)
  ENDIF(DISABLE_TESTS)
ENDMACRO(ADD_UNIT_TEST NAME SOURCE)

#.rst:
# .. command:: ADD_PYTHON_UNIT_TEST (NAME SOURCE [MODULE_PATH])
#
#   Add a test called `NAME` that runs an equivalent of ``python ${SOURCE}``,
#   optionnaly with a `PYTHONPATH` set to `CMAKE_BINARY_DIR/MODULE_PATH` if `MODULE_PATH` is set.
#   `SOURCE` is relative to `PROJECT_SOURCE_DIR`
#
#   .. note:: :command:`FINDPYTHON` should have been called first.
#
MACRO(ADD_PYTHON_UNIT_TEST NAME SOURCE)
  ADD_TEST(NAME ${NAME} COMMAND ${PYTHON_EXECUTABLE} "${PROJECT_SOURCE_DIR}/${SOURCE}")
  SET(PYTHONPATH)

  SET(MODULE_PATH "${ARGN}")  # ARGN is not a variable
  IF(MODULE_PATH)
    LIST(APPEND PYTHONPATH "${CMAKE_BINARY_DIR}/${MODULE_PATH}")
  ENDIF(MODULE_PATH)
  
  IF(DEFINED ENV{PYTHONPATH})
    LIST(APPEND PYTHONPATH $ENV{PYTHONPATH})
  ENDIF(DEFINED ENV{PYTHONPATH})

  SET(ENV_VARIABLES "PYTHONPATH=${PYTHONPATH}")
  IF(APPLE)
    LIST(APPEND ENV_VARIABLES "LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}")
    LIST(APPEND ENV_VARIABLES "DYLD_LIBRARY_PATH=$ENV{DYLD_LIBRARY_PATH}")
  ENDIF(APPLE)
  SET_TESTS_PROPERTIES(${NAME} PROPERTIES ENVIRONMENT "${ENV_VARIABLES}")
ENDMACRO(ADD_PYTHON_UNIT_TEST NAME SOURCE)

# DEFINE_UNIT_TEST(NAME LIB)
# ----------------------
#
# Compile a program and add it as a test
#
MACRO(DEFINE_UNIT_TEST NAME LIB)
  ADD_UNIT_TEST(${NAME} ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} ${PUBLIC_KEYWORD} ${LIB})
ENDMACRO(DEFINE_UNIT_TEST)
