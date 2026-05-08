#
# Copyright 2012-2026 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# XXX how to support this?
set(PROJECT_DEBUG_POSTFIX "_d")

if(NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
endif()

include(cmake/get-jrl-cmakemodules.cmake)
jrl_option(NANOBIND_BINDINGS "Build nanobind bindings" OFF)
jrl_option(
  NANOBIND_DOCUMENTATION
  "Build nanobind documentation"
  OFF
  CONDITION
  NANOBIND_BINDINGS
  FALLBACK
  OFF
)
jrl_option(BUILD_RBDYN_PARSERS "Build URDF and YAML parsers" ON)
jrl_option(BENCHMARKS "Generate benchmarks." OFF)
jrl_option(BUILD_DOCUMENTATION "Whether doxygen documentation should be built"
           OFF
)
jrl_option(
  INSTALL_DOCUMENTATION
  "Generate and install the doxygen documentation"
  OFF
  CONDITION
  BUILD_DOCUMENTATION
  FALLBACK
  OFF
)

set(CXX_DISABLE_WERROR True)
project(
    RBDyn
    LANGUAGES C CXX
    VERSION "1.9.3"
    HOMEPAGE_URL "https://github.com/jrl-umi3218/RBDyn"
    DESCRIPTION
        "RBDyn provides a set of classes and functions to model the dynamics of rigid body systems."
)
# Configure default binary and install directories, build type, etc.
jrl_configure_defaults()

jrl_find_package(SpaceVecAlg CONFIG REQUIRED)

if(BUILD_RBDYN_PARSERS)
    jrl_find_package(yaml-cpp 0.5.0 REQUIRED NO_MODULE)
    if(TARGET yaml-cpp::yaml-cpp)
        set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
    else()
        set(YAML_CPP_TARGET yaml-cpp)
    endif()

    find_package(tinyxml2 QUIET NO_MODULE)
    # FIXME how to do this with jrl-cmakemodulesv2 is this still required?
    # if(NOT ${tinyxml2_FOUND})
    #     set(CMAKE_MODULE_PATH
    #         ${CMAKE_MODULE_PATH}
    #         "${PROJECT_SOURCE_DIR}/CMakeModules"
    #     )
    #     find_package(RBDyn_TinyXML2 REQUIRED)
    #     install(
    #         FILES "${PROJECT_SOURCE_DIR}/CMakeModules/FindRBDyn_TinyXML2.cmake"
    #         DESTINATION "${CONFIG_INSTALL_DIR}"
    #     )
    #     set(PACKAGE_EXTRA_MACROS
    #         "list(APPEND
    #     CMAKE_MODULE_PATH \${CMAKE_CURRENT_LIST_DIR}) find_package(RBDyn_TinyXML2 REQUIRED)"
    #     )
    # else()
    jrl_find_package(tinyxml2 CONFIG REQUIRED)
    # endif()

    # Note: technically we don't need filesystem but it is likely to be here and
    # CMake <= 3.5.0 needs at least one component to define Boost::boost
    jrl_find_package(Boost REQUIRED COMPONENTS filesystem)
endif()

add_subdirectory(src)

if(BUILD_TESTING OR BENCHMARKS)
    add_subdirectory(tests)
endif()

if(BUILD_DOCUMENTATION)
    jrl_find_package(Doxygen REQUIRED COMPONENTS dot)
    # Configure Doxygen
    set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/share/doc/rbdyn/doxygen)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
    doxygen_add_docs(
        doc
        ${PROJECT_SOURCE_DIR}
        ALL
        COMMENT "Generating API documentation with Doxygen"
    )

    if(INSTALL_DOCUMENTATION)
        install(
            DIRECTORY ${CMAKE_BINARY_DIR}/share/doc/rbdyn/doxygen/html
            DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/rbdyn/doxygen
            COMPONENT doc
        )
    endif()
endif()

if(NANOBIND_BINDINGS)
    if(NANOBIND_DOCUMENTATION)
        include(cmake/get-sphinx-cmake.cmake)
        add_subdirectory(binding/nanobind)
    endif()
endif()

if(PYTHON_BINDING)
    add_subdirectory(binding/python)
endif()

# Export the package files (headers, targets, config files, etc.)
jrl_export_package()

# Print summaries jrl_print_dependencies_summary()
