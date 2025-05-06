#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // For std::vector, std::map, etc.
#include <OpenDriveMap.h>

namespace py = pybind11;

void init_opendrivemap(py::module_ &);

PYBIND11_MODULE(opendrive_bindings, m) {
    m.doc() = "Python bindings for libOpenDRIVE";

    init_opendrivemap(m);
    // Add class bindings here
}
