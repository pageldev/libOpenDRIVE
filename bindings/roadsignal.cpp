#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <RoadSignal.h>
#include <Mesh.h>
#include <LaneValidityRecord.h>

namespace py = pybind11;

void init_roadsignal(py::module_ &m) {
    // Bind RoadSignal
    py::class_<odr::RoadSignal>(m, "RoadSignal")
        .def(py::init<std::string, std::string, std::string, double, double, bool, double, double, double, double, double, double, double, std::string, std::string, std::string, std::string, std::string, std::string>(),
             py::arg("road_id"), py::arg("id"), py::arg("name"), py::arg("s0"), py::arg("t0"),
             py::arg("is_dynamic"), py::arg("zOffset"), py::arg("value"), py::arg("height"),
             py::arg("width"), py::arg("hOffset"), py::arg("pitch"), py::arg("roll"),
             py::arg("orientation"), py::arg("country"), py::arg("type"), py::arg("subtype"),
             py::arg("unit"), py::arg("text"),
             "Constructs a RoadSignal with specified properties")
        .def_static("get_box", &odr::RoadSignal::get_box,
                    py::arg("width"), py::arg("length"), py::arg("height"),
                    py::return_value_policy::move,
                    "Generates a box Mesh3D")
        .def_readwrite("road_id", &odr::RoadSignal::road_id, "Road ID")
        .def_readwrite("id", &odr::RoadSignal::id, "Signal ID")
        .def_readwrite("name", &odr::RoadSignal::name, "Signal name")
        .def_readwrite("s0", &odr::RoadSignal::s0, "Start s-coordinate")
        .def_readwrite("t0", &odr::RoadSignal::t0, "Start t-coordinate")
        .def_readwrite("is_dynamic", &odr::RoadSignal::is_dynamic, "Dynamic flag")
        .def_readwrite("zOffset", &odr::RoadSignal::zOffset, "Z offset")
        .def_readwrite("value", &odr::RoadSignal::value, "Signal value")
        .def_readwrite("height", &odr::RoadSignal::height, "Height")
        .def_readwrite("width", &odr::RoadSignal::width, "Width")
        .def_readwrite("hOffset", &odr::RoadSignal::hOffset, "Heading offset")
        .def_readwrite("pitch", &odr::RoadSignal::pitch, "Pitch")
        .def_readwrite("roll", &odr::RoadSignal::roll, "Roll")
        .def_readwrite("orientation", &odr::RoadSignal::orientation, "Orientation")
        .def_readwrite("country", &odr::RoadSignal::country, "Country code")
        .def_readwrite("type", &odr::RoadSignal::type, "Signal type")
        .def_readwrite("subtype", &odr::RoadSignal::subtype, "Signal subtype")
        .def_readwrite("unit", &odr::RoadSignal::unit, "Unit of value")
        .def_readwrite("text", &odr::RoadSignal::text, "Signal text")
        .def_readwrite("lane_validities", &odr::RoadSignal::lane_validities, "List of LaneValidityRecord objects");
}
