#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(SETUP_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>/")
if(DEFINED CMAKE_BUILD_TYPE)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
else()
  foreach(CFG ${CMAKE_CONFIGURATION_TYPES})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CFG}")
  endforeach()
endif()
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/setup.in.py" "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")
file(GENERATE
      OUTPUT "${SETUP_LOCATION}/setup.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")

set(PYTHON "python")
set(PIP "pip")
if(${PYTHON_BINDING_FORCE_PYTHON3})
  set(PYTHON "python3")
  set(PIP "pip3")
endif()

# Build the bindings locally at build time for test purposes
add_custom_target(rbdyn-python-bindings ALL
  COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" ${PYTHON} setup.py build_ext --inplace
  COMMENT "Generating local RBDyn Python bindings"
  DEPENDS "include/rbdyn_wrapper.hpp" "rbdyn/c_rbdyn.pxd" "rbdyn/rbdyn.pxd" "rbdyn/rbdyn.pyx"
  SOURCES "include/rbdyn_wrapper.hpp" "rbdyn/c_rbdyn.pxd" "rbdyn/rbdyn.pxd" "rbdyn/rbdyn.pyx"
)
add_dependencies(rbdyn-python-bindings RBDyn)

if(NOT ${DISABLE_TESTS})
  if(WIN32)
    set(PATH_SEP ";")
  else()
    set(PATH_SEP ":")
  endif()
  add_test(NAME PythonBindingsTest
    COMMAND ${CMAKE_COMMAND} -E env LD_LIBRARY_PATH=$<TARGET_FILE_DIR:RBDyn>${PATH_SEP}$ENV{LD_LIBRARY_PATH} nosetests -s ${SETUP_LOCATION}
  )
endif()

set(PIP_EXTRA_OPTIONS "")
if(${PYTHON_BINDING_USER_INSTALL})
  set(PIP_EXTRA_OPTIONS "--user")
endif()
# Install the bindings
if(DEFINED PYTHON_DEB_ROOT)
  add_custom_target(install-rbdyn-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" python setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb && python3 setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb
    COMMENT "Install SpaceVecAlg Python bindings (Debian package)"
  )
else()
  add_custom_target(install-rbdyn-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" ${PIP} install . ${PIP_EXTRA_OPTIONS}
    COMMENT "Install RBDyn Python bindings"
  )
endif()

install(CODE "EXECUTE_PROCESS(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --config \${CMAKE_INSTALL_CONFIG_NAME} --target install-rbdyn-python-bindings)")
