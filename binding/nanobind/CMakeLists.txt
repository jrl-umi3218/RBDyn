message(STATUS "[${PROJECT_NAME}] Building nanobind Python bindings")
jrl_find_python(3.8 REQUIRED COMPONENTS Interpreter Development.Module)
jrl_find_nanobind(2.5.0 CONFIG QUIET)

if(nanobind_ROOT AND NOT nanobind_FOUND)
    message(WARNING "nanobind found at ${nanobind_ROOT} but too old")
endif()

# On Ubuntu 24.04, the nanobind-dev package ships nanobind 1.9.2. We require
# >=2.5.0 for nanobind_add_stub, NB_SUPPRESS_WARNINGS, and visitor pattern (c++)
if(NOT nanobind_FOUND)
    set(nanobind_GIT_REPOSITORY "https://github.com/wjakob/nanobind.git")
    set(nanobind_GIT_TAG "v2.10.2")

    message(
        STATUS
        "nanobind: fallback to FetchContent from ${nanobind_GIT_REPOSITORY} (tag: ${nanobind_GIT_TAG})"
    )

    include(FetchContent)
    FetchContent_Declare(
        nanobind
        GIT_REPOSITORY ${nanobind_GIT_REPOSITORY}
        GIT_TAG ${nanobind_GIT_TAG}
        GIT_SHALLOW True
    )
    FetchContent_MakeAvailable(nanobind)
endif()

jrl_find_package(nanoeigenpy CONFIG REQUIRED)

# We are now ready to compile the actual extension module
nanobind_add_module(
  # Name of the extension
  _rbdyn
  # Target the stable ABI for Python 3.12+, which reduces the number of binary
  # wheels that must be built. This does nothing on older Python versions
  STABLE_ABI
  # Build libnanobind statically and merge it into the extension (which itself
  # remains a shared library)
  #
  # If your project builds multiple extensions, you can replace this flag by
  # NB_SHARED to conserve space by reusing a shared libnanobind across libraries
  NB_STATIC
  LTO
  # Source code goes here
  src/rbdyn_module/rbdyn_module.cpp
  src/rbdyn_module/bind_MultiBodyConfig.cpp
)
jrl_check_python_module_name(_rbdyn)
target_link_libraries(_rbdyn PRIVATE RBDyn)
# FIXME: { in nix this is needed otherwise fixupPhase fails with RPATH errors
# see flake.nix for corresponding cmake flags fix
set_target_properties(
    _rbdyn
    PROPERTIES
        BUILD_RPATH "${CMAKE_BINARY_DIR}/lib"
        INSTALL_RPATH "$ORIGIN:$ORIGIN/../.."
)
# } FIXME
jrl_target_set_default_compile_options(_rbdyn PRIVATE)
jrl_target_set_output_directory(_rbdyn OUTPUT_DIRECTORY
                                ${CMAKE_BINARY_DIR}/lib/site-packages/rbdyn
)

jrl_python_generate_init_py(
  _rbdyn OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/site-packages/rbdyn/__init__.py
)

# Display output of stub generation
set(CMAKE_EXECUTE_PROCESS_COMMAND_ECHO STDOUT)
set(MOD_DIR "${CMAKE_BINARY_DIR}/lib/site-packages/${MODULE}")

nanobind_add_stub(
  _rbdyn_stub
  MODULE
  rbdyn._rbdyn
  OUTPUT
  ${CMAKE_BINARY_DIR}/lib/site-packages/rbdyn/_rbdyn.pyi
  PYTHON_PATH
  ${CMAKE_BINARY_DIR}/lib/site-packages
  DEPENDS
  _rbdyn
  VERBOSE
)

jrl_python_compile_all(DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/rbdyn
                       VERBOSE
)

jrl_python_compute_install_dir(python_install_dir)
install(
    DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/rbdyn
    DESTINATION ${python_install_dir}
    FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.pyc"
    PATTERN "*.pyi"
    PATTERN "*.typed"
    PATTERN "*.so"
    PATTERN "*.pyd"
)
