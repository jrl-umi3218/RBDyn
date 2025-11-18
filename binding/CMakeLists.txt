if(NANOBIND_BINDINGS)
  add_custom_target(
    rbdyn-nanobind-bindings
    COMMAND pip install --no-build-isolation -ve .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Building and installing nanobind bindings for mc_rtc (incremental)"
  )
  # add_dependencies(rbdyn-nanobind-bindings install)
  add_custom_target(
    force-rbdyn-nanobind-bindings
    COMMAND pip install .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Installing nanobind bindings for mc_rtc (isolated)")
  # add_dependencies(rbdyn-nanobind-bindings ${ALL_RBDYN_TARGETS})
  # add_dependencies(force-rbdyn-nanobind-bindings ${ALL_RBDYN_TARGETS})
elseif(PYTHON_BINDING)
  add_subdirectory(binding/python)
endif()
