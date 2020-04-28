#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find TinyXML2
# in standard prefixes and in ${TinyXML2_PREFIX}
# Once done this will define
#  TinyXML2_FOUND - System has TinyXML
#  TinyXML2_INCLUDE_DIRS - The TinyXML include directories
#  TinyXML2_LIBRARIES - The libraries needed to use TinyXML
#  TinyXML2_DEFINITIONS - Compiler switches required for using TinyXML
# Define TinyXML2::TinyXML2 if needed

find_path(TinyXML2_INCLUDE_DIR
  NAMES tinyxml2.h
  PATHS ${TinyXML2_PREFIX}
  PATH_SUFFIXES include/tinyxml2
  )
find_library(TinyXML2_LIBRARY
  NAMES tinyxml2
  PATHS ${TinyXML2_PREFIX}
  )

set(TinyXML2_LIBRARIES ${TinyXML2_LIBRARY})
set(TinyXML2_INCLUDE_DIRS ${TinyXML2_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TinyXML2_LIBRARY TinyXML2_INCLUDE_DIR)
mark_as_advanced(TinyXML2_INCLUDE_DIR TinyXML2_LIBRARY)
if(TinyXML2_FOUND AND NOT TARGET tinyxml2::tinyxml2)
  add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
  set_target_properties(tinyxml2::tinyxml2 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${TinyXML2_INCLUDE_DIRS}
    IMPORTED_LOCATION ${TinyXML2_LIBRARY}
    )
endif()
