from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension
import os

# Path to the build directory where e.g. libOpenDrive.dylib is located
build_dir = os.path.abspath("build")

ext_modules = [
    Pybind11Extension(
        "opendrive",
        [
            "bindings/bindings.cpp",
            "bindings/opendrivemap.cpp",
            "bindings/junction.cpp",
            "bindings/lane.cpp",
            "bindings/lanesection.cpp",
            "bindings/road.cpp",
            "bindings/refline.cpp",
            "bindings/mesh.cpp",
            "bindings/math.cpp",
            "bindings/routinggraph.cpp",
            "bindings/roadmark.cpp",
            "bindings/roadnetworkmesh.cpp",
            "bindings/roadobject.cpp",
            "bindings/lanevalidityrecord.cpp",
            "bindings/roadsignal.cpp",
            "bindings/xml_node.cpp",
        ],
        include_dirs=[
            "include",  # libOpenDRIVE headers
            os.path.join(build_dir, "_deps/pugixml-src/src"),  # pugixml headers
        ],
        define_macros=[("NDEBUG", None)],
        library_dirs=[build_dir],  # Where libOpenDrive.dylib is located
        libraries=["OpenDrive"],  # Correct library name (without 'lib' prefix)
        extra_link_args=["-Wl,-rpath," + build_dir],  # Add rpath for macOS
    ),
]

setup(
    name="opendrive",
    version="0.0.1",
    description="Python bindings for libOpenDRIVE",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Christian Contreras",
    author_email="chrisjcc.physics@gmail.com",
    url="https://github.com/pageldev/libOpenDRIVE",
    license="Apache License",
    ext_modules=ext_modules,
    install_requires=["pybind11>=2.9"],
)
