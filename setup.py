from setuptools import setup
from setuptools.command.install import install
import shutil
import os

class CustomInstallCommand(install):
    def run(self):
        super().run()
        # Copy your prebuilt .so file to the correct location
        built_file = [x for x in os.listdir("build/pyglomap") if x.endswith(".so")][0]
        shutil.copy(f"build/pyglomap/{built_file}", self.install_lib)

# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
    name="pyglomap",
    version="1.0.0",
    author="Linfei Pan",
    author_email="linfei.pan@inf.ethz.ch",
    description="Pybind11 bindings for GLOMAP",
    packages=["pyglomap"],
    cmdclass={"install": CustomInstallCommand},
    zip_safe=False,
)