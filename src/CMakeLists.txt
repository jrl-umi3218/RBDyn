set(SOURCES MultiBodyGraph.cpp MultiBody.cpp MultiBodyConfig.cpp
  FK.cpp FV.cpp FA.cpp Jacobian.cpp ID.cpp FD.cpp EulerIntegration.cpp CoM.cpp
  Momentum.cpp ZMP.cpp)
set(HEADERS Body.h Joint.h MultiBodyGraph.h MultiBody.h MultiBodyConfig.h
  FK.h FV.h FA.h Jacobian.h ID.h FD.h EulerIntegration.h CoM.h
  Momentum.h ZMP.h)

add_library(RBDyn SHARED ${SOURCES} ${HEADERS})
PKG_CONFIG_USE_DEPENDENCY(RBDyn SpaceVecAlg)

set(INSTALL_PATH include/RBDyn)
install(FILES ${HEADERS} DESTINATION ${INSTALL_PATH})
install(TARGETS RBDyn DESTINATION "lib")
