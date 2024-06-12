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
    RBDyn/Body.h
    RBDyn/Joint.h
    RBDyn/MultiBodyGraph.h
    RBDyn/MultiBody.h
    RBDyn/MultiBodyConfig.h
    RBDyn/FK.h
    RBDyn/FV.h
    RBDyn/FA.h
    RBDyn/Jacobian.h
    RBDyn/ID.h
    RBDyn/IK.h
    RBDyn/IS.h
    RBDyn/FD.h
    RBDyn/NumericalIntegration.h
    RBDyn/EulerIntegration.h
    RBDyn/CoM.h
    RBDyn/Momentum.h
    RBDyn/ZMP.h
    RBDyn/IDIM.h
    RBDyn/VisServo.h
    RBDyn/util.hh
    RBDyn/util.hxx
    RBDyn/Coriolis.h)

add_library(RBDyn ${SOURCES} ${HEADERS})
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

install(
  TARGETS RBDyn
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DESTINATION})
