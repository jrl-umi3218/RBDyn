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

ENABLE_TESTING()

set(BOOST_COMPONENTS unit_test_framework timer system filesystem)
SEARCH_FOR_BOOST()
add_definitions(-DBOOST_TEST_DYN_LINK)

add_definitions(-DTESTS_DATA_DIR="${PROJECT_SOURCE_DIR}/tests")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR} ${Boost_LIBRARY_DIRS})

set(HEADERS XXXarm.h XYZarm.h XYZSarm.h Tree30Dof.h SSSarm.h)

macro(addUnitTest name)
  if(NOT ${DISABLE_TESTS})
    add_executable(${name} ${name}.cpp ${HEADERS})
    if(MSVC)
      # for msvc, we let boost auto-link do the work
	    target_link_libraries(${name} PUBLIC RBDyn)
    else(MSVC)
      target_link_libraries(${name} PUBLIC ${Boost_LIBRARIES} RBDyn)
    endif(MSVC)
    PKG_CONFIG_USE_DEPENDENCY(${name} SpaceVecAlg)
    add_test(${name}Unit ${name})
	# Adding a project configuration file (for MSVC only)
    GENERATE_MSVC_DOT_USER_FILE(${name})
  endif()
endmacro(addUnitTest)

macro(addBenchmark name)
  if(${BENCHMARKS})
    add_executable(${name} ${name}.cpp ${HEADERS})
    if(MSVC)
      # for msvc, we let boost auto-link do the work
	    target_link_libraries(${name} PUBLIC RBDyn)
    else(MSVC)
      target_link_libraries(${name} PUBLIC ${Boost_LIBRARIES} RBDyn)
    endif(MSVC)
    PKG_CONFIG_USE_DEPENDENCY(${name} SpaceVecAlg)
	# Adding a project configuration file (for MSVC only)
    GENERATE_MSVC_DOT_USER_FILE(${name})
  endif()
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

addBenchmark("JacobianBench")
addBenchmark("DynamicsBench")
