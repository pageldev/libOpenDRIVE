#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "RefLine.h"

namespace py = pybind11;
using namespace odr;

void init_refline(py::module_ &m) {
    py::class_<RefLine>(m, "RefLine")
        .def(py::init<std::string, double>(),
             py::arg("road_id"), py::arg("length"))
        .def("get_geometries", py::overload_cast<>(&RefLine::get_geometries, py::const_),
             "Get a set of const RoadGeometry pointers")
        .def("get_geometries", py::overload_cast<>(&RefLine::get_geometries),
             "Get a set of non-const RoadGeometry pointers")
        .def("get_geometry_s0", &RefLine::get_geometry_s0,
             py::arg("s"), "Get the s0 value for the geometry at position s")
        .def("get_geometry", py::overload_cast<const double>(&RefLine::get_geometry, py::const_),
             py::arg("s"), "Get a const RoadGeometry pointer at position s")
        .def("get_geometry", py::overload_cast<const double>(&RefLine::get_geometry),
             py::arg("s"), "Get a non-const RoadGeometry pointer at position s")
        .def("get_xyz", &RefLine::get_xyz,
             py::arg("s"), "Get the 3D coordinates at position s")
        .def("get_grad", &RefLine::get_grad,
             py::arg("s"), "Get the gradient at position s")
        .def("get_line", &RefLine::get_line,
             py::arg("s_start"), py::arg("s_end"), py::arg("eps"),
             "Get a 3D line from s_start to s_end with given epsilon")
        .def("match", &RefLine::match,
             py::arg("x"), py::arg("y"), "Match a 2D point to the reference line")
        .def("approximate_linear", &RefLine::approximate_linear,
             py::arg("eps"), py::arg("s_start"), py::arg("s_end"),
             "Approximate the reference line linearly between s_start and s_end")
        .def_readwrite("road_id", &RefLine::road_id)
        .def_readwrite("length", &RefLine::length)
        .def_readwrite("elevation_profile", &RefLine::elevation_profile)
        .def("get_s0_to_geometry", [](const RefLine& self) {
            std::map<double, odr::RoadGeometry*> result;
            for (const auto& [s0, geom_ptr] : self.s0_to_geometry) {
                result[s0] = geom_ptr.get();
            }
            return result;
        }, "Return map of s0 to raw RoadGeometry pointers");
}

PYBIND11_MODULE(refline, m) {
    init_refline(m);
}
