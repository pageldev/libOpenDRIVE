from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension
import os

# Path to the build directory where libOpenDrive.dylib is located
build_dir = os.path.abspath("build")

ext_modules = [
    Pybind11Extension(
        "opendrive_bindings",
        [
            "bindings/bindings.cpp",
            "bindings/opendrivemap.cpp",
            "bindings/junction.cpp",
            "bindings/lane.cpp",
            "bindings/lanesection.h",
            #"src/Geometries/Arc.cpp",
            #"src/Geometries/CubicSpline.cpp",
            #"src/Geometries/Line.cpp",
            #"src/Geometries/ParamPoly3.cpp",
            #"src/Geometries/RoadGeometry.cpp",
            #"src/Geometries/Spiral.cpp",
            #"src/Geometries/Spiral/odrSpiral.cpp",
            #"src/Junction.cpp",
            #"src/Lane.cpp",
            #"src/LaneSection.cpp",
            #"src/Log.cpp",
            #"src/Mesh.cpp",
            #"src/OpenDriveMap.cpp",
            #"src/RefLine.cpp",
            #"src/Road.cpp",
            #"src/RoadMark.cpp",
            #"src/RoadNetworkMesh.cpp",
            #"src/RoadObject.cpp",
            #"src/RoadSignal.cpp",
            #"src/RoutingGraph.cpp",
            #"src/pugixml.cpp",
        ],
        include_dirs=[
            "include",  # libOpenDRIVE headers
            #"src",
            os.path.join(build_dir, "_deps/pugixml-src/src"),  # pugixml headers
        ],
        define_macros=[("NDEBUG", None)],
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
