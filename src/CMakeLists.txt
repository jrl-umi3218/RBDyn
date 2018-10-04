# Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of RBDyn.
#
# RBDyn is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# RBDyn is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

set(SOURCES MultiBodyGraph.cpp MultiBody.cpp MultiBodyConfig.cpp
  FK.cpp FV.cpp FA.cpp Jacobian.cpp ID.cpp IK.cpp IS.cpp FD.cpp EulerIntegration.cpp
  CoM.cpp Momentum.cpp ZMP.cpp IDIM.cpp VisServo.cpp Coriolis.cpp)
set(HEADERS RBDyn/Body.h RBDyn/Joint.h RBDyn/MultiBodyGraph.h RBDyn/MultiBody.h RBDyn/MultiBodyConfig.h
  RBDyn/FK.h RBDyn/FV.h RBDyn/FA.h RBDyn/Jacobian.h RBDyn/ID.h RBDyn/IK.h RBDyn/IS.h RBDyn/FD.h RBDyn/EulerIntegration.h RBDyn/CoM.h
  RBDyn/Momentum.h RBDyn/ZMP.h RBDyn/IDIM.h RBDyn/VisServo.h RBDyn/util.hh RBDyn/util.hxx RBDyn/Coriolis.h)

add_library(RBDyn SHARED ${SOURCES} ${HEADERS})
target_link_libraries(RBDyn SpaceVecAlg::SpaceVecAlg)
set_target_properties(RBDyn PROPERTIES COMPILE_FLAGS "-Drbdyn_EXPORTS")
set_target_properties(RBDyn PROPERTIES SOVERSION 0 VERSION 0.9.0)

install(
    TARGETS RBDyn
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${INCLUDE_INSTALL_DIR}"
)
install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DESTINATION})
