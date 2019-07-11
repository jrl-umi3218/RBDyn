from conans import ConanFile, CMake, tools
import os
import subprocess

class RBDynTestConan(ConanFile):
    requires = "RBDyn/1.1.0@gergondet/stable"
    settings = "os", "arch", "compiler", "build_type"
    generators = "cmake"

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def test(self):
        if not tools.cross_building(self.settings):
            os.chdir("bin")
            self.run(".%sexample" % os.sep)
            subprocess.check_call(['python', os.path.join(os.path.dirname(__file__), 'test.py')])
