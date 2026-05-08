find_package(Sphinx REQUIRED)

if(Sphinx_FOUND)
    message(STATUS "Found Sphinx (version: ${Sphinx_VERSION}) package.")
    return()
endif()

message(STATUS "Sphinx not found, falling back to FetchContent")

include(FetchContent)

# Declare the external resource
FetchContent_Declare(
    sphinx_cmake
    URL
        https://github.com/python-cmake/sphinx-cmake/archive/refs/tags/1.1.0.tar.gz
)

# Make the content available
FetchContent_MakeAvailable(sphinx_cmake)
list(APPEND CMAKE_MODULE_PATH "${sphinx_cmake_SOURCE_DIR}/cmake")

find_package(Sphinx REQUIRED)
