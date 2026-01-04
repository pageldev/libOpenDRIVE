#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <RoadMark.h>

namespace py = pybind11;

void init_roadmark(py::module_ &m) {
    // Expose constants
    m.attr("ROADMARK_WEIGHT_STANDARD_WIDTH") = odr::ROADMARK_WEIGHT_STANDARD_WIDTH;
    m.attr("ROADMARK_WEIGHT_BOLD_WIDTH") = odr::ROADMARK_WEIGHT_BOLD_WIDTH;

    // Bind RoadMarksLine
    py::class_<odr::RoadMarksLine>(m, "RoadMarksLine")
        .def(py::init<std::string, double, int, double, double, double, double, double, double, std::string, std::string>(),
             py::arg("road_id"), py::arg("lanesection_s0"), py::arg("lane_id"), py::arg("group_s0"),
             py::arg("width"), py::arg("length"), py::arg("space"), py::arg("t_offset"), py::arg("s_offset"),
             py::arg("name"), py::arg("rule"),
             "Constructs a RoadMarksLine with specified properties")
        .def_readwrite("road_id", &odr::RoadMarksLine::road_id, "Road ID")
        .def_readwrite("lanesection_s0", &odr::RoadMarksLine::lanesection_s0, "Lane section s0")
        .def_readwrite("lane_id", &odr::RoadMarksLine::lane_id, "Lane ID")
        .def_readwrite("group_s0", &odr::RoadMarksLine::group_s0, "Group start s-coordinate")
        .def_readwrite("width", &odr::RoadMarksLine::width, "Width of the line")
        .def_readwrite("length", &odr::RoadMarksLine::length, "Length of the line")
        .def_readwrite("space", &odr::RoadMarksLine::space, "Spacing between lines")
        .def_readwrite("t_offset", &odr::RoadMarksLine::t_offset, "Lateral offset")
        .def_readwrite("s_offset", &odr::RoadMarksLine::s_offset, "Longitudinal offset")
        .def_readwrite("name", &odr::RoadMarksLine::name, "Name of the line")
        .def_readwrite("rule", &odr::RoadMarksLine::rule, "Rule (e.g., no passing)");

    // Bind RoadMarkGroup
    py::class_<odr::RoadMarkGroup>(m, "RoadMarkGroup")
        .def(py::init<std::string, double, int, double, double, double, std::string, std::string, std::string, std::string, std::string>(),
             py::arg("road_id"), py::arg("lanesection_s0"), py::arg("lane_id"), py::arg("width"),
             py::arg("height"), py::arg("s_offset"), py::arg("type"), py::arg("weight"),
             py::arg("color"), py::arg("material"), py::arg("lane_change"),
             "Constructs a RoadMarkGroup with specified properties")
        .def_readwrite("road_id", &odr::RoadMarkGroup::road_id, "Road ID")
        .def_readwrite("lanesection_s0", &odr::RoadMarkGroup::lanesection_s0, "Lane section s0")
        .def_readwrite("lane_id", &odr::RoadMarkGroup::lane_id, "Lane ID")
        .def_readwrite("width", &odr::RoadMarkGroup::width, "Width of the group")
        .def_readwrite("height", &odr::RoadMarkGroup::height, "Height of the group")
        .def_readwrite("s_offset", &odr::RoadMarkGroup::s_offset, "Longitudinal offset")
        .def_readwrite("type", &odr::RoadMarkGroup::type, "Type (e.g., solid, broken)")
        .def_readwrite("weight", &odr::RoadMarkGroup::weight, "Weight (e.g., standard, bold)")
        .def_readwrite("color", &odr::RoadMarkGroup::color, "Color of the group")
        .def_readwrite("material", &odr::RoadMarkGroup::material, "Material of the group")
        .def_readwrite("lane_change", &odr::RoadMarkGroup::lane_change, "Lane change rule")
        .def_readwrite("roadmark_lines", &odr::RoadMarkGroup::roadmark_lines, "Set of RoadMarksLine objects");

    // Bind RoadMark
    py::class_<odr::RoadMark>(m, "RoadMark")
        .def(py::init<std::string, double, int, double, double, double, double, double, std::string>(),
             py::arg("road_id"), py::arg("lanesection_s0"), py::arg("lane_id"), py::arg("group_s0"),
             py::arg("s_start"), py::arg("s_end"), py::arg("t_offset"), py::arg("width"), py::arg("type"),
             "Constructs a RoadMark with specified properties")
        .def_readwrite("road_id", &odr::RoadMark::road_id, "Road ID")
        .def_readwrite("lanesection_s0", &odr::RoadMark::lanesection_s0, "Lane section s0")
        .def_readwrite("lane_id", &odr::RoadMark::lane_id, "Lane ID")
        .def_readwrite("group_s0", &odr::RoadMark::group_s0, "Group start s-coordinate")
        .def_readwrite("s_start", &odr::RoadMark::s_start, "Start s-coordinate")
        .def_readwrite("s_end", &odr::RoadMark::s_end, "End s-coordinate")
        .def_readwrite("t_offset", &odr::RoadMark::t_offset, "Lateral offset")
        .def_readwrite("width", &odr::RoadMark::width, "Width of the mark")
        .def_readwrite("type", &odr::RoadMark::type, "Type (e.g., solid, broken)");
}
