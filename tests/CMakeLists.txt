ENABLE_TESTING()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories(${Boost_INCLUDE_DIRS})

macro(addUnitTest name)
  add_executable(${name}  ${name}.cpp)
  target_link_libraries(${name} ${Boost_LIBRARIES} RBDyn)
  PKG_CONFIG_USE_DEPENDENCY(${name} SpaceVecAlg)
  add_test(${name}Unit ${name})
endmacro(addUnitTest)

addUnitTest ("BodyTest")
addUnitTest ("JointTest")
addUnitTest ("MultiBodyTest")
addUnitTest ("AlgoTest")
addUnitTest ("JacobianTest")
addUnitTest ("DynamicsTest")
addUnitTest ("CoMTest")

