# Copyright (C) 2018 LAAS-CNRS
# Authors: Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

INCLUDE(cmake/hpp/doc.cmake)

#.rst:
# .. ifmode:: hpp
#
# .. variable:: HPP_DEBUG
#
#   Enable logging of debug output in log files.
#
# .. variable:: HPP_BENCHMARK
#
#   Enable logging of benchmark output in log files.

#.rst:
# .. ifmode:: hpp
#
# .. command:: SETUP_HPP_PROJECT
#
#   Initialize a HPP project. Calls :cmake:command:`SETUP_PROJECT`.
MACRO(SETUP_HPP_PROJECT)

  IF (NOT DEFINED PROJECT_ORG)
    SET(PROJECT_ORG "humanoid-path-planner")
  ENDIF (NOT DEFINED PROJECT_ORG)

  IF (NOT DEFINED PROJECT_URL)
    SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")
  ENDIF (NOT DEFINED PROJECT_URL)

  SETUP_PROJECT()
  _SETUP_PROJECT_HPP_DOCUMENTATION()

  # Activate hpp-util logging if requested
  SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
  IF (HPP_DEBUG)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
  ENDIF()
  # Activate hpp-util logging if requested
  SET (HPP_BENCHMARK FALSE CACHE BOOL "trigger hpp-util benchmark output")
  IF (HPP_BENCHMARK)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
  ENDIF()
ENDMACRO(SETUP_HPP_PROJECT)

#.rst:
# .. ifmode:: hpp
#
# .. command:: SETUP_HPP_PROJECT_FINALIZE
#
#   To be called at the end of the CMakeLists.txt to
#   finalize the HPP project setup.
#   Calls :cmake:command:`SETUP_PROJECT_FINALIZE`.
MACRO(SETUP_HPP_PROJECT_FINALIZE)
  SETUP_PROJECT_FINALIZE()
  _SETUP_PROJECT_HPP_DOCUMENTATION()
ENDMACRO(SETUP_HPP_PROJECT_FINALIZE)
