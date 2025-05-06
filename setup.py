from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension
import os

# Path to the build directory where libOpenDrive.dylib is located
build_dir = os.path.abspath("build")

ext_modules = [
    Pybind11Extension(
        "opendrive_bindings",
        ["bindings/bindings.cpp"],
        include_dirs=[
            "include",  # libOpenDRIVE headers
            os.path.join(build_dir, "_deps/pugixml-src/src"),  # pugixml headers
        ],
        library_dirs=[build_dir],  # Where libOpenDrive.dylib is located
        libraries=["OpenDrive"],  # Correct library name (without 'lib' prefix)
        extra_link_args=["-Wl,-rpath," + build_dir],  # Add rpath for macOS
    ),
]

setup(
    name="opendrive_bindings",
    version="0.0.1",
    description="Python bindings for libOpenDRIVE",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Your Name",
    author_email="your.email@example.com",
    url="https://github.com/your-username/libOpenDRIVE",
    license="MIT",  # Adjust based on libOpenDRIVE license
    ext_modules=ext_modules,
    install_requires=["pybind11>=2.9"],
)
