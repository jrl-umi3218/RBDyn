INCLUDE(../../cmake/python.cmake)

# Look for Python 2.7
SET(Python_ADDITIONAL_VERSIONS 2.7)
FINDPYTHON()

## Define PYTHON_DISTLIB
EXECUTE_PROCESS(
  COMMAND "${PYTHON_EXECUTABLE}" "-c"
  "import sys, os; print os.sep.join(['lib', 'python' + sys.version[:3], 'dist-packages'])"
  OUTPUT_VARIABLE PYTHON_DISTLIB
  ERROR_QUIET)
# Remove final \n of the variable PYTHON_DISTLIB
STRING(REPLACE "\n" "" PYTHON_DISTLIB "${PYTHON_DISTLIB}")

# create the package in build dir for testing purpose
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/rbdyn)
configure_file(__init__.py ${CMAKE_CURRENT_BINARY_DIR}/rbdyn/__init__.py COPYONLY)

set(OUTPUT_BINDING ${CMAKE_CURRENT_BINARY_DIR}/rbdyn.cpp)

# generate python binding code
add_custom_command (
  OUTPUT ${OUTPUT_BINDING}
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate.py ${OUTPUT_BINDING}
  DEPENDS generate.py
)

# build the library
set(SOURCES ${OUTPUT_BINDING})
include_directories(.)
include_directories(../../src)
include_directories(${PYTHON_INCLUDE_DIRS})

add_library(_rbdyn SHARED ${SOURCES})
PKG_CONFIG_USE_DEPENDENCY(_rbdyn SpaceVecAlg)
target_link_libraries(_rbdyn RBDyn)
set_target_properties(_rbdyn PROPERTIES PREFIX "")

# install rules
set(INSTALL_PATH "${PYTHON_DISTLIB}/rbdyn/")
install(TARGETS _rbdyn DESTINATION ${INSTALL_PATH})
install(FILES __init__.py DESTINATION ${INSTALL_PATH})
