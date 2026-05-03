#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(PROJECT_NAME RBDyn)
set(PROJECT_DESCRIPTION "...")
set(PROJECT_URL "")
set(PROJECT_DEBUG_POSTFIX "_d")
set(PROJECT_VERSION 1.9.3)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

option(BUILD_RBDYN_PARSERS "Build URDF and YAML parsers" ON)

if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

include(cmake/get-jrl-cmakemodules.cmake)

set(CXX_DISABLE_WERROR True)
project(
  RBDyn
  LANGUAGES C CXX
  VERSION "${PROJECT_VERSION}"
  HOMEPAGE_URL "https://github.com/jrl-umi3218/RBDyn"
  DESCRIPTION
    "RBDyn provides a set of classes and functions to model the dynamics of rigid body systems."
)

option(BENCHMARKS "Generate benchmarks." OFF)

jrl_find_package(SpaceVecAlg CONFIG REQUIRED)

if(BUILD_RBDYN_PARSERS)
  jrl_find_package(yaml-cpp 0.5.0 REQUIRED NO_MODULE)
  if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
  else()
    set(YAML_CPP_TARGET yaml-cpp)
  endif()

  find_package(tinyxml2 QUIET NO_MODULE)
  if(NOT ${tinyxml2_FOUND})
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                          "${PROJECT_SOURCE_DIR}/CMakeModules")
    find_package(RBDyn_TinyXML2 REQUIRED)
    install(FILES "${PROJECT_SOURCE_DIR}/CMakeModules/FindRBDyn_TinyXML2.cmake"
            DESTINATION "${CONFIG_INSTALL_DIR}")
    set(PACKAGE_EXTRA_MACROS
        "list(APPEND CMAKE_MODULE_PATH \${CMAKE_CURRENT_LIST_DIR})
find_package(RBDyn_TinyXML2 REQUIRED)")
  else()
    jrl_find_package(tinyxml2 CONFIG REQUIRED)
  endif()

  # Note: technically we don't need filesystem but it is likely to be here and
  # CMake <= 3.5.0 needs at least one component to define Boost::boost
  jrl_find_package(Boost REQUIRED COMPONENTS filesystem)
endif()

# For MSVC, set local environment variable to enable finding the built dll of
# the main library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif()

add_subdirectory(src)

if(BUILD_TESTING OR BENCHMARKS)
  add_subdirectory(tests)
endif()

if(PYTHON_BINDING OR NANOBIND_BINDINGS)
  add_subdirectory(binding)
endif()

# Export the package files (headers, targets, config files, etc.)
jrl_export_package()

# Print summaries
jrl_print_dependencies_summary()
