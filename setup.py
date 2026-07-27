import os
import re

from Cython.Build import cythonize
from setuptools import find_packages, setup
from setuptools.command.build_py import build_py as _build_py

this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, "README.md"), "r", encoding="utf-8") as f:
    long_description = f.read()


with open(os.path.join(this_directory, "sensor", "__init__.py"), "r", encoding="utf-8") as f:
    version = re.search(r'^__version__\s*=\s*["\']([^"\']+)["\']', f.read(), re.M).group(1)


class build_py(_build_py):

    def find_package_modules(self, package, package_dir):
        modules = super().find_package_modules(package, package_dir)
        return [m for m in modules if m[1] == "__init__"]


ext_modules = cythonize(
    ["sensor/**/*.py"],
    exclude=["**/__init__.py"],
    language_level="3",
)

setup(
    name="sensor-sdk",
    version=version,
    description="Python sdk for Synchroni",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/oymotion/SynchroniSDKPython",
    author="Martin Ye",
    author_email="yecq_82@hotmail.com",
    packages=find_packages(include=["sensor", "sensor.*"]),
    ext_modules=ext_modules,
    cmdclass={"build_py": build_py},
    install_requires=[
        "numpy",
        "setuptools",
        "bleak>=3.0.2",
        "flatbuffers>=25.0.0",
        "bumble==0.0.233",
        "libusb1",
        "libusb-package",
    ],
    package_data={"sensor": ["tools/*"]},
    zip_safe=False,
    python_requires=">=3.10.0",
)
