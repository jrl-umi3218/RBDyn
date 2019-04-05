#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

SET(SETUP_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>/")
IF(DEFINED CMAKE_BUILD_TYPE)
  FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
ELSE()
  FOREACH(CFG ${CMAKE_CONFIGURATION_TYPES})
    FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CFG}")
  ENDFOREACH()
ENDIF()
CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/setup.in.py" "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")
FILE(GENERATE
      OUTPUT "${SETUP_LOCATION}/setup.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")

SET(PYTHON "python")
SET(PIP "pip")
IF(${PYTHON_BINDING_FORCE_PYTHON3})
  SET(PYTHON "python3")
  SET(PIP "pip3")
ENDIF()

# Build the bindings locally at build time for test purposes
ADD_CUSTOM_TARGET(rbdyn-python-bindings ALL
  COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" ${PYTHON} setup.py build_ext --inplace
  COMMENT "Generating local RBDyn Python bindings"
  DEPENDS "include/rbdyn_wrapper.hpp" "rbdyn/c_rbdyn.pxd" "rbdyn/rbdyn.pxd" "rbdyn/rbdyn.pyx"
  SOURCES "include/rbdyn_wrapper.hpp" "rbdyn/c_rbdyn.pxd" "rbdyn/rbdyn.pxd" "rbdyn/rbdyn.pyx"
)
ADD_DEPENDENCIES(rbdyn-python-bindings RBDyn)

IF(NOT ${DISABLE_TESTS})
  IF(NOT DEFINED PYTHON_DEB_ROOT)
    ADD_TEST(NAME PythonBindingsTest
      COMMAND ${CMAKE_COMMAND} -E chdir "$<TARGET_FILE_DIR:RBDyn>" nosetests ${SETUP_LOCATION}
    )
  ENDIF()
ENDIF()

SET(PIP_EXTRA_OPTIONS "")
IF(${PYTHON_BINDING_USER_INSTALL})
  SET(PIP_EXTRA_OPTIONS "--user")
ENDIF()
# Install the bindings
IF(DEFINED PYTHON_DEB_ROOT)
  ADD_CUSTOM_TARGET(install-rbdyn-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" python setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb && python3 setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb
    COMMENT "Install SpaceVecAlg Python bindings (Debian package)"
  )
ELSE()
  ADD_CUSTOM_TARGET(install-rbdyn-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" ${PIP} install . ${PIP_EXTRA_OPTIONS}
    COMMENT "Install RBDyn Python bindings"
  )
ENDIF()

INSTALL(CODE "EXECUTE_PROCESS(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --config \${CMAKE_INSTALL_CONFIG_NAME} --target install-rbdyn-python-bindings)")
