#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(SOURCES
    MultiBodyGraph.cpp
    MultiBody.cpp
    MultiBodyConfig.cpp
    FK.cpp
    FV.cpp
    FA.cpp
    Jacobian.cpp
    ID.cpp
    IK.cpp
    IS.cpp
    FD.cpp
    NumericalIntegration.cpp
    CoM.cpp
    Momentum.cpp
    ZMP.cpp
    IDIM.cpp
    VisServo.cpp
    Coriolis.cpp)
set(HEADERS
    src/RBDyn/RBDyn/Body.h
    src/RBDyn/RBDyn/Joint.h
    src/RBDyn/RBDyn/MultiBodyGraph.h
    src/RBDyn/RBDyn/MultiBody.h
    src/RBDyn/RBDyn/MultiBodyConfig.h
    src/RBDyn/RBDyn/FK.h
    src/RBDyn/RBDyn/FV.h
    src/RBDyn/RBDyn/FA.h
    src/RBDyn/RBDyn/Jacobian.h
    src/RBDyn/RBDyn/ID.h
    src/RBDyn/RBDyn/IK.h
    src/RBDyn/RBDyn/IS.h
    src/RBDyn/RBDyn/FD.h
    src/RBDyn/RBDyn/NumericalIntegration.h
    src/RBDyn/RBDyn/EulerIntegration.h
    src/RBDyn/RBDyn/CoM.h
    src/RBDyn/RBDyn/Momentum.h
    src/RBDyn/RBDyn/ZMP.h
    src/RBDyn/RBDyn/IDIM.h
    src/RBDyn/RBDyn/VisServo.h
    src/RBDyn/RBDyn/util.hh
    src/RBDyn/RBDyn/util.hxx
    src/RBDyn/RBDyn/Coriolis.h)

set(ALL_RBDYN_TARGETS)
add_library(RBDyn SHARED)
target_sources(RBDyn PRIVATE ${SOURCES})
jrl_target_headers(RBDyn PUBLIC HEADERS ${HEADERS} BASE_DIRS src/RBDyn)
list(APPEND ALL_RBDYN_TARGETS RBDyn)
target_include_directories(
  RBDyn
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(RBDyn PUBLIC SpaceVecAlg::SpaceVecAlg)
if(BUILD_SHARED_LIBS)
  set_target_properties(RBDyn PROPERTIES COMPILE_FLAGS "-Drbdyn_EXPORTS")
else()
  target_compile_definitions(RBDyn PUBLIC "-DRBDYN_STATIC")
endif()
set_target_properties(RBDyn PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR}
                                       VERSION ${PROJECT_VERSION})
set_target_properties(RBDyn PROPERTIES CXX_STANDARD 11)

jrl_target_generate_config_header(RBDyn PUBLIC VERSION ${PROJECT_VERSION})
jrl_target_generate_deprecated_header(RBDyn PUBLIC)
jrl_add_export_component(NAME RBDyn TARGETS RBDyn)
# install( TARGETS RBDyn EXPORT "${TARGETS_EXPORT_NAME}" LIBRARY DESTINATION
# "${CMAKE_INSTALL_LIBDIR}" ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
# RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}") install(FILES ${HEADERS}
# DESTINATION ${INCLUDE_INSTALL_DESTINATION})
