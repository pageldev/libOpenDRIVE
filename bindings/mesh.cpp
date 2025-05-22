#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Mesh.h"

namespace py = pybind11;
using namespace odr;

void init_mesh(py::module_ &m) {
    // Bind Mesh3D
    py::class_<Mesh3D>(m, "Mesh3D")
        .def(py::init<>())
        .def("add_mesh", &Mesh3D::add_mesh,
             py::arg("other"), "Add another Mesh3D to this mesh")
        .def("get_obj", &Mesh3D::get_obj,
             "Get the mesh as an OBJ format string")
        .def_readwrite("vertices", &Mesh3D::vertices,
                       "List of 3D vertices")
        .def_readwrite("indices", &Mesh3D::indices,
                       "List of triangle indices")
        .def_readwrite("normals", &Mesh3D::normals,
                       "List of 3D normals")
        .def_readwrite("st_coordinates", &Mesh3D::st_coordinates,
                       "List of 2D texture coordinates");
}

PYBIND11_MODULE(mesh, m) {
    init_mesh(m);
}
