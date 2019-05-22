#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

ADD_CYTHON_BINDINGS(rbdyn
  TARGETS RBDyn
  EXPORT_SOURCES rbdyn/c_rbdyn.pxd rbdyn/rbdyn.pxd rbdyn/__init__.py
  PRIVATE_SOURCES include/rbdyn_wrapper.hpp rbdyn/rbdyn.pyx rbdyn/c_rbdyn_private.pxd
                  tests/test_rbdyn_pickle.py
)
