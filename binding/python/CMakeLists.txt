#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(RBDYN_BINDINGS_MODULES rbdyn.rbdyn)
set(RBDYN_BINDINGS_EXPORT_SOURCES rbdyn/c_rbdyn.pxd rbdyn/rbdyn.pxd)
set(RBDYN_BINDINGS_PRIVATE_SOURCES
    include/rbdyn_wrapper.hpp rbdyn/rbdyn.pyx rbdyn/c_rbdyn_private.pxd
    tests/test_rbdyn_pickle.py)
set(RBDYN_BINDINGS_TARGETS RBDyn)

if(${BUILD_RBDYN_PARSERS})
  list(APPEND RBDYN_BINDINGS_MODULES rbdyn.parsers.parsers)
  list(APPEND RBDYN_BINDINGS_EXPORT_SOURCES rbdyn/parsers/c_parsers.pxd
       rbdyn/parsers/parsers.pxd rbdyn/parsers/__init__.py)
  list(APPEND RBDYN_BINDINGS_PRIVATE_SOURCES include/rbdyn_parsers_wrapper.hpp
       rbdyn/parsers/parsers.pyx rbdyn/parsers/c_parsers_private.pxd
       tests/test_rbdyn_parsers.py)
  list(APPEND RBDYN_BINDINGS_TARGETS RBDyn::Parsers)
  if(NOT BUILD_SHARED_LIBS)
    list(APPEND RBDYN_BINDINGS_TARGETS ${YAML_CPP_TARGET} tinyxml2::tinyxml2)
  endif()
endif()

make_cython_bindings(
  rbdyn
  TARGETS
  ${RBDYN_BINDINGS_TARGETS}
  MODULES
  ${RBDYN_BINDINGS_MODULES}
  EXPORT_SOURCES
  ${RBDYN_BINDINGS_EXPORT_SOURCES}
  PRIVATE_SOURCES
  ${RBDYN_BINDINGS_PRIVATE_SOURCES}
  GENERATE_SOURCES
  rbdyn/__init__.py)
