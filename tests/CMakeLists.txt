#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

enable_testing()

find_package(Boost REQUIRED COMPONENTS unit_test_framework system filesystem)

set(HEADERS XXXarm.h XYZarm.h XYZSarm.h Tree30Dof.h SSSarm.h)

macro(addUnitTest name)
  if(${BUILD_TESTING})
    add_executable(${name} ${name}.cpp ${HEADERS})
    target_link_libraries(
      ${name} PRIVATE RBDyn Boost::unit_test_framework Boost::system
                      Boost::filesystem Boost::disable_autolinking)
    target_compile_definitions(
      ${name} PRIVATE -DBOOST_TEST_DYN_LINK
                      -DTESTS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(${name}Unit ${name})
    # Adding a project configuration file (for MSVC only)
    generate_msvc_dot_user_file(${name})
  endif()
endmacro(addUnitTest)

macro(addParserUnitTest name)
  if(${BUILD_TESTING} AND ${BUILD_RBDYN_PARSERS})
    add_executable(${name} ${name}.cpp ${HEADERS})
    target_link_libraries(
      ${name} PRIVATE RBDynParsers Boost::unit_test_framework Boost::system
                      Boost::filesystem Boost::disable_autolinking)
    target_compile_definitions(
      ${name} PRIVATE -DBOOST_TEST_DYN_LINK
                      -DTESTS_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(${name}Unit ${name})
    # Adding a project configuration file (for MSVC only)
    generate_msvc_dot_user_file(${name})
  endif()
endmacro(addParserUnitTest)

macro(addBenchmark name)
  add_executable(${name} ${name}.cpp ${HEADERS})
  target_link_libraries(${name} PUBLIC RBDyn benchmark)
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${name})
endmacro(addBenchmark)

addunittest("BodyTest")
addunittest("JointTest")
addunittest("MultiBodyTest")
addunittest("AlgoTest")
addunittest("JacobianTest")
addunittest("DynamicsTest")
addunittest("CoMTest")
addunittest("MomentumTest")
addunittest("IDIMTest")
addunittest("InverseStaticsTest")
addunittest("IntegrationTest")
addunittest("ExpandTest")
addunittest("CoriolisTest")
addparserunittest("URDFParserTest")
addparserunittest("URDFOutputTest")
addparserunittest("YAMLParserTest")
addparserunittest("YAMLOutputTest")

if(${BENCHMARKS})
  option(BENCHMARK_ENABLE_TESTING "Enable testing of the benchmark library."
         OFF)
  if(NOT WIN32)
    add_definitions(-Wno-error)
  endif()
  add_subdirectory(benchmark EXCLUDE_FROM_ALL)
  addbenchmark("JacobianBench")
  addbenchmark("DynamicsBench")
endif()
