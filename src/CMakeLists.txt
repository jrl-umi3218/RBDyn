#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

add_subdirectory(RBDyn)
if(BUILD_RBDYN_PARSERS)
  add_subdirectory(parsers)
endif()
