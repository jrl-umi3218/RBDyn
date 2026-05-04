#
# Copyright 2012-2026 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(SOURCES common.cpp urdf.cpp to_urdf.cpp yaml.cpp to_yaml.cpp)
set(HEADERS src/parsers/RBDyn/parsers/api.h src/parsers/RBDyn/parsers/common.h
            src/parsers/RBDyn/parsers/urdf.h src/parsers/RBDyn/parsers/yaml.h)

add_library(RBDynParsers SHARED)
add_library(RBDyn::Parsers ALIAS RBDynParsers)
target_sources(RBDynParsers PRIVATE ${SOURCES})
jrl_target_headers(RBDynParsers PUBLIC HEADERS ${HEADERS} BASE_DIRS src/parsers)
target_include_directories(
  RBDynParsers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                      $<INSTALL_INTERFACE:include>)
target_link_libraries(RBDynParsers PUBLIC RBDyn tinyxml2::tinyxml2
                                          ${YAML_CPP_TARGET} Boost::boost)
if(BUILD_SHARED_LIBS)
  set_target_properties(RBDynParsers PROPERTIES COMPILE_FLAGS
                                                "-DRBDYN_PARSERS_EXPORTS")
else()
  target_compile_definitions(RBDynParsers PUBLIC "-DRBDYN_PARSERS_STATIC")
endif()
set_target_properties(RBDynParsers PROPERTIES CXX_STANDARD 11)
set_target_properties(RBDynParsers PROPERTIES EXPORT_NAME Parsers)

jrl_add_export_component(NAME RBDynParsers TARGETS RBDynParsers)

# install( TARGETS RBDynParsers EXPORT "${TARGETS_EXPORT_NAME}" LIBRARY
# DESTINATION "${CMAKE_INSTALL_LIBDIR}" ARCHIVE DESTINATION
# "${CMAKE_INSTALL_LIBDIR}" RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
# install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DESTINATION}/parsers)

add_executable(urdf_yaml_converter urdf_yaml_conv.cpp)
target_link_libraries(urdf_yaml_converter PUBLIC RBDyn::Parsers)

install(
  TARGETS urdf_yaml_converter
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
