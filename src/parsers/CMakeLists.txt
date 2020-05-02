#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

add_project_dependency(yaml-cpp 0.5.0 REQUIRED)

find_package(tinyxml2 QUIET)
if(NOT ${tinyxml2_FOUND})
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules")
  find_package(RBDyn_TinyXML2 REQUIRED)
  install(FILES "${PROJECT_SOURCE_DIR}/CMakeModules/FindRBDyn_TinyXML2.cmake"
          DESTINATION "${CONFIG_INSTALL_DIR}")
  set(PACKAGE_EXTRA_MACROS "include(\${CMAKE_CURRENT_LIST_DIR}/FindRBDyn_TinyXML2.cmake)" CACHE INTERNAL "")
else()
  add_project_dependency(tinyxml2 REQUIRED)
endif()

# Note: technically we don't need system but it is likely to be here and
# CMake <= 3.5.0 needs at least one component to define Boost::boost
add_project_dependency(Boost REQUIRED COMPONENTS system)

set(SOURCES common.cpp urdf.cpp to_urdf.cpp yaml.cpp to_yaml.cpp)
set(HEADERS RBDyn/parsers/api.h RBDyn/parsers/common.h RBDyn/parsers/urdf.h RBDyn/parsers/yaml.h)

add_library(RBDynParsers SHARED ${SOURCES} ${HEADERS})
add_library(RBDyn::Parsers ALIAS RBDynParsers)
target_include_directories(RBDynParsers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../include> $<INSTALL_INTERFACE:include>)
target_link_libraries(RBDynParsers PUBLIC RBDyn tinyxml2::tinyxml2 yaml-cpp Boost::boost)
set_target_properties(RBDynParsers PROPERTIES COMPILE_FLAGS "-DRBDYN_PARSERS_EXPORTS")
set_target_properties(RBDynParsers PROPERTIES CXX_STANDARD 11)
set_target_properties(RBDynParsers PROPERTIES EXPORT_NAME Parsers)
set_target_properties(RBDynParsers PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION_FULL})

install(
    TARGETS RBDynParsers
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
)
install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DESTINATION}/parsers)


add_executable(urdf_yaml_converter urdf_yaml_conv.cpp)
target_link_libraries(urdf_yaml_converter PUBLIC RBDyn::Parsers)
set_target_properties(urdf_yaml_converter PROPERTIES COMPILE_FLAGS "-DRBDYN_PARSERS_DLLIMPORT")

install(
    TARGETS urdf_yaml_converter
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
)