set(SOURCES MultiBodyGraph.cpp MultiBody.cpp MultiBodyConfig.cpp
  FK.cpp FV.cpp Jacobian.cpp ID.cpp FD.cpp EulerIntegration.cpp CoM.cpp)
set(HEADERS Body.h Joint.h MultiBodyGraph.h MultiBody.h MultiBodyConfig.h
  FK.h FV.h Jacobian.h ID.h FD.h EulerIntegration.h CoM.h)

pkg_check_modules(EIGEN3 REQUIRED eigen3)
find_path(SVA_INCLUDE_DIRS SpaceVecAlg/SpaceVecAlg
                           /usr/include
                           /usr/local/include)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${SVA_INCLUDE_DIRS}/SpaceVecAlg)

add_library(RBDyn SHARED ${SOURCES} ${HEADERS})

set(INSTALL_PATH include/RBDyn)
install(FILES ${HEADERS} DESTINATION ${INSTALL_PATH})
install(TARGETS RBDyn DESTINATION "lib")
