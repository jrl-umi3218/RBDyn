# Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# SEARCH_FOR_EIGEN
# -----------------
#
# This macro gets eigen include path from pkg-config file, and adds it include directories.

# If the variable Eigen_REQUIRED is not defined before calling
#  the method SEARCH_FOR_EIGEN, the version chosen is the default one.
MACRO(SEARCH_FOR_EIGEN)

  SET(_Eigen_FOUND 0)
  IF(NOT Eigen_REQUIRED)
    SET(Eigen_REQUIRED "eigen3 >= 3.0.0")
  ENDIF(NOT Eigen_REQUIRED)
  PKG_CHECK_MODULES(_Eigen REQUIRED ${Eigen_REQUIRED})
  
  IF(NOT ${_Eigen_FOUND})
    MESSAGE(FATAL_ERROR "Check that package Eigen is installed in a directory pointed out by PKG_CONFIG_PATH.")
  ENDIF(NOT ${_Eigen_FOUND})
  
  SET(${PROJECT_NAME}_CXX_FLAGS "${${PROJECT_NAME}_CXX_FLAGS} ${_Eigen_CFLAGS}")
  SET(${PROJECT_NAME}_LINK_FLAGS "${${PROJECT_NAME}_LINK_FLAGS} ${_Eigen_LDFLAGS}")

  STRING(REGEX REPLACE "-I" "" Eigen_INCLUDE_DIR ${_Eigen_CFLAGS})
  INCLUDE_DIRECTORIES(SYSTEM ${Eigen_INCLUDE_DIR} )
ENDMACRO(SEARCH_FOR_EIGEN)
