ENABLE_TESTING()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)

pkg_check_modules(EIGEN3 REQUIRED eigen3)
find_path(SVA_INCLUDE_DIRS SpaceVecAlg/SpaceVecAlg
                           /usr/include
                           /usr/local/include)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${SVA_INCLUDE_DIRS}/SpaceVecAlg)
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories(${Boost_INCLUDE_DIRS})

add_executable(BodyTest BodyTest.cpp)
target_link_libraries(BodyTest ${Boost_LIBRARIES} RBDyn)
add_test(BodyTestUnit BodyTest)

add_executable(JointTest JointTest.cpp)
target_link_libraries(JointTest ${Boost_LIBRARIES} RBDyn)
add_test(JointTestUnit JointTest)

add_executable(MultiBodyTest MultiBodyTest.cpp)
target_link_libraries(MultiBodyTest ${Boost_LIBRARIES} RBDyn)
add_test(MultiBodyTestUnit MultiBodyTest)

add_executable(AlgoTest AlgoTest.cpp)
target_link_libraries(AlgoTest ${Boost_LIBRARIES} RBDyn)
add_test(AlgoTestUnit AlgoTest)
