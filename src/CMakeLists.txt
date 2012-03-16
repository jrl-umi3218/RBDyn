set(SOURCES MultiBodyGraph.cpp MultiBody.cpp)
set(HEADERS Body.h Joint.h MultiBodyGraph.h MultiBody.h)

pkg_check_modules(EIGEN3 REQUIRED eigen3)
find_path(SVA_INCLUDE_DIRS SpaceVecAlg/SpaceVecAlg
                           /usr/include
                           /usr/local/include)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${SVA_INCLUDE_DIRS}/SpaceVecAlg)

add_library(RBDyn SHARED ${SOURCES} ${HEADERS})

