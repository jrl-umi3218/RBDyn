# -*- coding: utf-8 -*-
#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from conans import python_requires, tools
import os

base = python_requires("Eigen3ToPython/1.0.0@gergondet/stable")


class RBDynConan(base.Eigen3ToPythonConan):
    name = "RBDyn"
    version = "1.1.0"
    description = "Model the dynamics of rigid body systems"
    topics = ("robotics", "dynamics", "eigen", "python")
    url = "https://github.com/jrl-umi3218/RBDyn"
    homepage = "https://github.com/jrl-umi3218/RBDyn"
    author = "Pierre Gergondet <pierre.gergondet@gmail.com>"
    license = "BSD-2-Clause"
    exports = ["LICENSE"]
    exports_sources = ["CMakeLists.txt", "conan/CMakeLists.txt", "binding/*", "cmake/*", "doc/*", "src/*"]
    generators = "cmake"
    settings = "os", "arch", "compiler", "build_type"
    options = { "python_version": ["2.7", "3.3", "3.4", "3.5", "3.6", "3.7"] }
    default_options = { "python_version": base.get_python_version() }

    requires = (
        "SpaceVecAlg/1.1.0@gergondet/stable"
    )

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
        self.env_info.LD_LIBRARY_PATH.append(os.path.join(self.package_folder, 'lib'))
        self.env_info.PYTHONPATH.append(self._extra_python_path())
