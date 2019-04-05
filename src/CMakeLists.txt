#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(SOURCES MultiBodyGraph.cpp MultiBody.cpp MultiBodyConfig.cpp
  FK.cpp FV.cpp FA.cpp Jacobian.cpp ID.cpp IK.cpp IS.cpp FD.cpp EulerIntegration.cpp
  CoM.cpp Momentum.cpp ZMP.cpp IDIM.cpp VisServo.cpp Coriolis.cpp)
set(HEADERS RBDyn/Body.h RBDyn/Joint.h RBDyn/MultiBodyGraph.h RBDyn/MultiBody.h RBDyn/MultiBodyConfig.h
  RBDyn/FK.h RBDyn/FV.h RBDyn/FA.h RBDyn/Jacobian.h RBDyn/ID.h RBDyn/IK.h RBDyn/IS.h RBDyn/FD.h RBDyn/EulerIntegration.h RBDyn/CoM.h
  RBDyn/Momentum.h RBDyn/ZMP.h RBDyn/IDIM.h RBDyn/VisServo.h RBDyn/util.hh RBDyn/util.hxx RBDyn/Coriolis.h)

add_library(RBDyn SHARED ${SOURCES} ${HEADERS})
target_include_directories(RBDyn PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>)
target_link_libraries(RBDyn PUBLIC SpaceVecAlg::SpaceVecAlg)
set_target_properties(RBDyn PROPERTIES COMPILE_FLAGS "-Drbdyn_EXPORTS")
set_target_properties(RBDyn PROPERTIES SOVERSION 1 VERSION 1.1.0)

install(
    TARGETS RBDyn
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
)
install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DESTINATION})
