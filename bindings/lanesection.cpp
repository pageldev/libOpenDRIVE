#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <LaneSection.h>

namespace py = pybind11;

void init_lanesection(py::module_ &m) {
    // Bind LaneSection
    py::class_<odr::LaneSection>(m, "LaneSection")
        .def(py::init<std::string, double>(),
             py::arg("road_id"), py::arg("s0"),
             "Constructs a LaneSection with road ID and start position s0")
        .def_readwrite("road_id", &odr::LaneSection::road_id, "Road ID")
        .def_readwrite("s0", &odr::LaneSection::s0, "Start position (s-coordinate)")
        .def_readwrite("id_to_lane", &odr::LaneSection::id_to_lane, "Map of lane IDs to Lane objects")
        .def("get_lanes", &odr::LaneSection::get_lanes,
             py::return_value_policy::move,
             "Returns a list of lanes")
        .def("get_lane_id", &odr::LaneSection::get_lane_id,
             py::arg("s"), py::arg("t"),
             "Returns the lane ID at position (s, t), favoring the inner lane on boundaries")
        .def("get_lane", py::overload_cast<int>(&odr::LaneSection::get_lane, py::const_),
             py::arg("id"),
             py::return_value_policy::copy,
             "Returns the lane with the given ID")
        .def("get_lane", py::overload_cast<double, double>(&odr::LaneSection::get_lane, py::const_),
             py::arg("s"), py::arg("t"),
             py::return_value_policy::copy,
             "Returns the lane at position (s, t)");
}
