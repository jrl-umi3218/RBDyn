cmake_minimum_required(VERSION 2.8)
project(RBDyn)

option(PYTHON_BINDING "Generate python binding." ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")

find_package(PkgConfig REQUIRED)

add_subdirectory(src)
add_subdirectory(tests)

if(${PYTHON_BINDING})
	add_subdirectory(binding/python)
endif()
