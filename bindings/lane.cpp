#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Lane.h>

namespace py = pybind11;

// Forward declarations for opaque types
namespace odr {
struct CubicSpline;
struct RoadMark;
struct RoadMarkGroup;
}

void init_lane(py::module_ &m) {
    // Bind HeightOffset
    py::class_<odr::HeightOffset>(m, "HeightOffset")
        .def(py::init<double, double>(),
             py::arg("inner"), py::arg("outer"),
             "Constructs a HeightOffset with inner and outer offsets")
        .def_readwrite("inner", &odr::HeightOffset::inner, "Inner height offset")
        .def_readwrite("outer", &odr::HeightOffset::outer, "Outer height offset");

    // Bind LaneKey
    py::class_<odr::LaneKey>(m, "LaneKey")
        .def(py::init<std::string, double, int>(),
             py::arg("road_id"), py::arg("lanesection_s0"), py::arg("lane_id"),
             "Constructs a LaneKey with road ID, lane section s0, and lane ID")
        .def_readwrite("road_id", &odr::LaneKey::road_id, "Road ID")
        .def_readwrite("lanesection_s0", &odr::LaneKey::lanesection_s0, "Lane section s0")
        .def_readwrite("lane_id", &odr::LaneKey::lane_id, "Lane ID")
        .def("to_string", &odr::LaneKey::to_string, "Returns a string representation of the LaneKey")
        .def(" __eq__", [](const odr::LaneKey& a, const odr::LaneKey& b) { return a == b; }, "Equality comparison");

    // Bind opaque types
    py::class_<odr::CubicSpline>(m, "CubicSpline");

    // Bind Lane
    py::class_<odr::Lane>(m, "Lane")
        .def(py::init<std::string, double, int, bool, std::string>(),
             py::arg("road_id"), py::arg("lanesection_s0"), py::arg("id"),
             py::arg("level"), py::arg("type"),
             "Constructs a Lane with road ID, lane section s0, ID, level, and type")
        .def_readwrite("key", &odr::Lane::key, "Lane key")
        .def_readwrite("id", &odr::Lane::id, "Lane ID")
        .def_readwrite("level", &odr::Lane::level, "Level flag")
        .def_readwrite("predecessor", &odr::Lane::predecessor, "Predecessor lane ID")
        .def_readwrite("successor", &odr::Lane::successor, "Successor lane ID")
        .def_readwrite("type", &odr::Lane::type, "Lane type")
        .def_readwrite("lane_width", &odr::Lane::lane_width, "Cubic spline for lane width")
        .def_readwrite("outer_border", &odr::Lane::outer_border, "Cubic spline for outer border")
        .def_readwrite("s_to_height_offset", &odr::Lane::s_to_height_offset,
                       "Map of s-coordinates to height offsets")
        .def_readwrite("roadmark_groups", &odr::Lane::roadmark_groups,
                       "Set of road mark groups")
        .def("get_roadmarks", &odr::Lane::get_roadmarks,
             py::arg("s_start"), py::arg("s_end"),
             py::return_value_policy::move,
             "Returns road marks between s_start and s_end");
}
