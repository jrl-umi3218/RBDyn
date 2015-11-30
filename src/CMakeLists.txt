# Copyright 2012-2016 CNRS-UM LIRMM, CNRS-AIST JRL
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
  CoM.cpp Momentum.cpp ZMP.cpp IDIM.cpp VisServo.cpp)
set(HEADERS Body.h Joint.h MultiBodyGraph.h MultiBody.h MultiBodyConfig.h
  FK.h FV.h FA.h Jacobian.h ID.h IK.h IS.h FD.h EulerIntegration.h CoM.h
  Momentum.h ZMP.h IDIM.h VisServo.h)

add_library(RBDyn SHARED ${SOURCES} ${HEADERS})
PKG_CONFIG_USE_DEPENDENCY(RBDyn SpaceVecAlg)
set_target_properties(RBDyn PROPERTIES COMPILE_FLAGS "-Drbdyn_EXPORTS")
set_target_properties(RBDyn PROPERTIES SOVERSION 0 VERSION 0.9.0)

set(INSTALL_PATH include/RBDyn)
install(FILES ${HEADERS} DESTINATION ${INSTALL_PATH})
install(TARGETS RBDyn 
                      RUNTIME DESTINATION bin
                      LIBRARY DESTINATION lib
                      ARCHIVE DESTINATION lib
)
