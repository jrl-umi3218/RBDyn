if(NANOBIND_BINDINGS)
  if(NOT PYTHON_EXECUTABLE)
    find_package(
      Python3
      COMPONENTS Interpreter
      REQUIRED)
    set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
  endif()
  add_custom_target(
    RBDyn-nanobind-bindings
    COMMAND ${PYTHON_EXECUTABLE} -m pip install --no-build-isolation -ve .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Building and installing nanobind bindings for mc_rtc (incremental)"
  )
  # add_dependencies(rbdyn-nanobind-bindings install)
  add_custom_target(
    force-RBDyn-nanobind-bindings
    COMMAND ${PYTHON_EXECUTABLE} -m pip install .
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/nanobind
    COMMENT "Installing nanobind bindings for mc_rtc (isolated)")
  # add_dependencies(rbdyn-nanobind-bindings ${ALL_RBDYN_TARGETS})
  # add_dependencies(force-rbdyn-nanobind-bindings ${ALL_RBDYN_TARGETS})
elseif(PYTHON_BINDING)
  add_subdirectory(binding/python)
endif()
