# -*- coding: utf-8 -*-
#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from conans import python_requires, tools
import os

base = python_requires("Eigen3ToPython/latest@multi-contact/dev")

def get_default_options():
    out = base.get_default_options()
    out.update({'*:shared': True})
    return out

class RBDynConan(base.Eigen3ToPythonConan):
    name = "RBDyn"
    version = "1.7.0"
    description = "Model the dynamics of rigid body systems"
    topics = ("robotics", "dynamics", "eigen", "python")
    url = "https://github.com/jrl-umi3218/RBDyn"
    homepage = "https://github.com/jrl-umi3218/RBDyn"
    author = "Pierre Gergondet <pierre.gergondet@gmail.com>"
    license = "BSD-2-Clause"
    exports = ["LICENSE"]
    exports_sources = ["CMakeLists.txt", "conan/*", "binding/*", "cmake/*", "doc/*", "src/*"]
    generators = ["cmake_paths"]
    settings = "os", "arch", "compiler", "build_type"
    options = {
            "python2_version": [None, "2.7"],
            "python3_version": [None, "3.3", "3.4", "3.5", "3.6", "3.7", "3.8"]
    }
    default_options = get_default_options()

    requires = (
        "SpaceVecAlg/latest@multi-contact/dev",
        "boost/1.73.0",
        "yaml-cpp/0.6.3@multi-contact/3rd-party",
        "tinyxml2/8.0.0@multi-contact/3rd-party"
    )

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
        self.env_info.LD_LIBRARY_PATH.append(os.path.join(self.package_folder, 'lib'))
        self.env_info.PYTHONPATH.append(self._extra_python_path())

    def source(self):
        base.Eigen3ToPythonConan.source(self)
        pattern = 'include(CMakeFindDependencyMacro)'
        replacement = '''if(CONAN_BOOST_ROOT)
  set(BOOST_ROOT "${{CONAN_BOOST_ROOT}}")
else()
  set(BOOST_ROOT "${{PACKAGE_PREFIX_DIR}}")
endif()
set(Boost_NO_SYSTEM_PATHS ON)
list(APPEND CMAKE_MODULE_PATH "${{CMAKE_CURRENT_LIST_DIR}}")
{}'''.format(pattern)
        tools.replace_in_file('cmake/Config.cmake.in', pattern, replacement)
        pattern = 'add_subdirectory(src)'
        replacement = '''list(APPEND CMAKE_MODULE_PATH "${{CMAKE_CURRENT_LIST_DIR}}/conan")
{}
install(FILES conan/FindBoost.cmake DESTINATION lib/cmake/RBDyn)'''.format(pattern)
        tools.replace_in_file('CMakeListsOriginal.txt', pattern, replacement)

    def package(self):
        cmake = self._configure_cmake()
        cmake.definitions['INSTALL_DOCUMENTATION'] = False
        cmake.configure()
        cmake.install()
