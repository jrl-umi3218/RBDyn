#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1.3)

set(CMAKE_CXX_STANDARD 11)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(cmake/base.cmake)
include(cmake/cython/cython.cmake)
include(cmake/msvc-specific.cmake)

set(PROJECT_NAME RBDyn)
set(PROJECT_DESCRIPTION "...")
set(PROJECT_URL "https://github.com/jrl-umi3218/RBDyn")
set(PROJECT_DEBUG_POSTFIX "_d")

# Disable -Werror on Unix for now.
set(CXX_DISABLE_WERROR True)

setup_project()
set(PROJECT_VERSION 1.1.0)

option(DISABLE_TESTS "Disable unit tests." OFF)
option(BENCHMARKS "Generate benchmarks." OFF)

add_project_dependency(SpaceVecAlg REQUIRED)

# For MSVC, set local environment variable to enable finding the built dll
# of the main library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif()

add_subdirectory(src)

if(NOT ${DISABLE_TESTS} OR ${BENCHMARKS})
  add_subdirectory(tests)
endif()

if(${PYTHON_BINDING})
  add_subdirectory(binding/python)
endif()

setup_project_finalize()
setup_project_package_finalize()
