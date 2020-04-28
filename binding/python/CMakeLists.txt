#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(RBDYN_BINDINGS_MODULES rbdyn.rbdyn)
set(RBDYN_BINDINGS_EXPORT_SOURCES rbdyn/c_rbdyn.pxd rbdyn/rbdyn.pxd rbdyn/__init__.py)
set(RBDYN_BINDINGS_PRIVATE_SOURCES include/rbdyn_wrapper.hpp rbdyn/rbdyn.pyx rbdyn/c_rbdyn_private.pxd tests/test_rbdyn_pickle.py)

if(${BUILD_PARSERS})
  list(APPEND RBDYN_BINDINGS_MODULES rbdyn.parsers.parsers)
  list(APPEND RBDYN_BINDINGS_EXPORT_SOURCES rbdyn/parsers/c_parsers.pxd rbdyn/parsers/parsers.pxd rbdyn/parsers/__init__.py)
  list(APPEND RBDYN_BINDINGS_PRIVATE_SOURCES include/rbdyn_parsers_wrapper.hpp rbdyn/parsers/parsers.pyx rbdyn/parsers/c_parsers_private.pxd tests/test_rbdyn_parsers.py)
endif()

ADD_CYTHON_BINDINGS(rbdyn
  TARGETS RBDyn RBDyn::Parsers
  MODULES ${RBDYN_BINDINGS_MODULES}
  EXPORT_SOURCES ${RBDYN_BINDINGS_EXPORT_SOURCES}
  PRIVATE_SOURCES ${RBDYN_BINDINGS_PRIVATE_SOURCES}
)
