cmake_minimum_required(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME RBDyn)
SET(PROJECT_DESCRIPTION "...")
SET(PROJECT_URL "https://github.com/jorisv/RBDyn")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

option(PYTHON_BINDING "Generate python binding." ON)
option(UNIT_TESTS "Generate unit tests." ON)
option(BENCHMARK "Generate benchmark." OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")

#########################
# External dependencies
# set(BOOST_COMPONENTS unit_test_framework) 
# SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("SpaceVecAlg")

#########################
# 

add_subdirectory(src)

add_subdirectory(tests)

if(${PYTHON_BINDING})
  add_subdirectory(binding/python)
endif()

# Add dependency towards the library in the pkg-config file.
PKG_CONFIG_APPEND_LIBS(RBDyn)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

