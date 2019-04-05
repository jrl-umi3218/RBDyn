#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

enable_testing()

find_package(Boost REQUIRED COMPONENTS unit_test_framework system)

set(HEADERS XXXarm.h XYZarm.h XYZSarm.h Tree30Dof.h SSSarm.h)

macro(addUnitTest name)
  if(NOT ${DISABLE_TESTS})
    add_executable(${name} ${name}.cpp ${HEADERS})
    target_link_libraries(${name} PRIVATE RBDyn Boost::unit_test_framework Boost::system)
    target_compile_definitions(${name} PRIVATE -DBOOST_TEST_DYN_LINK -DTESTS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(${name}Unit ${name})
    # Adding a project configuration file (for MSVC only)
    generate_msvc_dot_user_file(${name})
  endif()
endmacro(addUnitTest)

macro(addBenchmark name)
  add_executable(${name} ${name}.cpp ${HEADERS})
  target_link_libraries(${name} PUBLIC RBDyn benchmark)
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${name})
endmacro(addBenchmark)

addUnitTest("BodyTest")
addUnitTest("JointTest")
addUnitTest("MultiBodyTest")
addUnitTest("AlgoTest")
addUnitTest("JacobianTest")
addUnitTest("DynamicsTest")
addUnitTest("CoMTest")
addUnitTest("MomentumTest")
addUnitTest("IDIMTest")
addUnitTest("InverseStaticsTest")
addUnitTest("IntegrationTest")
addUnitTest("ExpandTest")
addUnitTest("CoriolisTest")

if(${BENCHMARKS})
  option(BENCHMARK_ENABLE_TESTING "Enable testing of the benchmark library." OFF)
  if(NOT WIN32)
    add_definitions(-Wno-error)
  endif()
  add_subdirectory(benchmark EXCLUDE_FROM_ALL)
  addBenchmark("JacobianBench")
  addBenchmark("DynamicsBench")
endif()
