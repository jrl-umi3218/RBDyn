#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(SOURCES common.cpp urdf.cpp to_urdf.cpp yaml.cpp to_yaml.cpp)
set(HEADERS RBDyn/parsers/api.h RBDyn/parsers/common.h RBDyn/parsers/urdf.h
            RBDyn/parsers/yaml.h)

add_library(RBDynParsers ${SOURCES} ${HEADERS})
add_library(RBDyn::Parsers ALIAS RBDynParsers)
target_include_directories(
  RBDynParsers
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../include>
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
set_target_properties(RBDynParsers PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR}
                                              VERSION ${PROJECT_VERSION_FULL})

install(
  TARGETS RBDynParsers
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DESTINATION}/parsers)

add_executable(urdf_yaml_converter urdf_yaml_conv.cpp)
target_link_libraries(urdf_yaml_converter PUBLIC RBDyn::Parsers)

install(
  TARGETS urdf_yaml_converter
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
