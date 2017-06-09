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

MACRO(GET_RBDYN_PROPERTY PROPERTY)
  GET_TARGET_PROPERTY(RBDYN_${PROPERTY} RBDyn ${PROPERTY})
  IF(NOT RBDYN_${PROPERTY})
    SET(RBDYN_${PROPERTY} "")
  ENDIF()
ENDMACRO()
GET_RBDYN_PROPERTY(COMPILE_FLAGS)
GET_RBDYN_PROPERTY(INCLUDE_DIRECTORIES)
GET_RBDYN_PROPERTY(LINK_FLAGS)
GET_RBDYN_PROPERTY(LINK_LIBRARIES)
GET_RBDYN_PROPERTY(LOCATION)
CONFIGURE_FILE(setup.in.py ${CMAKE_CURRENT_SOURCE_DIR}/setup.py)
GET_FILENAME_COMPONENT(RBDYN_LOCATION_DIRECTORY ${RBDYN_LOCATION} DIRECTORY)

# Build the bindings locally at build time for test purposes
ADD_CUSTOM_TARGET(rbdyn-python-bindings ALL
  COMMAND python setup.py build_ext --inplace
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating local RBDyn Python bindings"
  DEPENDS include/rbdyn_wrapper.hpp rbdyn/c_rbdyn.pxd rbdyn/rbdyn.pxd rbdyn/rbdyn.pyx
)

if(NOT ${DISABLE_TESTS})
  IF(NOT DEFINED PYTHON_DEB_ROOT)
    ADD_TEST(NAME PythonBindingsTest
      COMMAND nosetests ${CMAKE_CURRENT_SOURCE_DIR}
      WORKING_DIRECTORY ${RBDYN_LOCATION_DIRECTORY}
    )
  ENDIF()
endif()

SET(PIP_EXTRA_OPTIONS "")
IF(${PYTHON_BINDING_USER_INSTALL})
  SET(PIP_EXTRA_OPTIONS "--user")
ENDIF()
# Install the bindings
IF(DEFINED PYTHON_DEB_ROOT)
  ADD_CUSTOM_TARGET(install-rbdyn-python-bindings
    COMMAND python setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb && python3 setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Install SpaceVecAlg Python bindings (Debian package)"
  )
ELSE()
  ADD_CUSTOM_TARGET(install-rbdyn-python-bindings
    COMMAND pip install . ${PIP_EXTRA_OPTIONS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Install RBDyn Python bindings"
  )
ENDIF()
INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_MAKE_PROGRAM} install-rbdyn-python-bindings)")
