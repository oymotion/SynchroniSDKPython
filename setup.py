import os

from setuptools import setup, find_packages

this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, "README.md"), "r", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="sensor-sdk",
    version="0.0.37",
    description="Python sdk for Synchroni",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/oymotion/SynchroniSDKPython",
    author="Martin Ye",
    author_email="yecq_82@hotmail.com",
    packages=find_packages(),
    install_requires=["numpy", "setuptools", "bleak"],
    package_data={"sensor": []},
    zip_safe=True,
    python_requires=">=3.9.0",
)
