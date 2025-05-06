#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // For std::vector, std::map, etc.
#include <OpenDriveMap.h>

namespace py = pybind11;

PYBIND11_MODULE(opendrive_bindings, m) {
    m.doc() = "Python bindings for libOpenDRIVE";
    // Add class bindings here
}
