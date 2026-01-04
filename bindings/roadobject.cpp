#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <RoadObject.h>
#include <Mesh.h>
#include <LaneValidityRecord.h>
#include <Math.hpp>

namespace py = pybind11;

void init_roadobject(py::module_ &m) {
    // Bind RoadObjectRepeat
    py::class_<odr::RoadObjectRepeat>(m, "RoadObjectRepeat")
        .def(py::init<double, double, double, double, double, double, double, double, double, double, double>(),
             py::arg("s0"), py::arg("length"), py::arg("distance"), py::arg("t_start"), py::arg("t_end"),
             py::arg("width_start"), py::arg("width_end"), py::arg("height_start"), py::arg("height_end"),
             py::arg("z_offset_start"), py::arg("z_offset_end"),
             "Constructs a RoadObjectRepeat with specified properties")
        .def_readwrite("s0", &odr::RoadObjectRepeat::s0, "Start s-coordinate")
        .def_readwrite("length", &odr::RoadObjectRepeat::length, "Length of repetition")
        .def_readwrite("distance", &odr::RoadObjectRepeat::distance, "Distance between repetitions")
        .def_readwrite("t_start", &odr::RoadObjectRepeat::t_start, "Start t-coordinate")
        .def_readwrite("t_end", &odr::RoadObjectRepeat::t_end, "End t-coordinate")
        .def_readwrite("width_start", &odr::RoadObjectRepeat::width_start, "Start width")
        .def_readwrite("width_end", &odr::RoadObjectRepeat::width_end, "End width")
        .def_readwrite("height_start", &odr::RoadObjectRepeat::height_start, "Start height")
        .def_readwrite("height_end", &odr::RoadObjectRepeat::height_end, "End height")
        .def_readwrite("z_offset_start", &odr::RoadObjectRepeat::z_offset_start, "Start z-offset")
        .def_readwrite("z_offset_end", &odr::RoadObjectRepeat::z_offset_end, "End z-offset");

    // Bind RoadObjectCorner::Type enum
    py::enum_<odr::RoadObjectCorner::Type>(m, "RoadObjectCornerType")
        .value("Type_Local_RelZ", odr::RoadObjectCorner::Type::Type_Local_RelZ, "Z relative to road’s reference line")
        .value("Type_Local_AbsZ", odr::RoadObjectCorner::Type::Type_Local_AbsZ, "Absolute z value")
        .value("Type_Road", odr::RoadObjectCorner::Type::Type_Road, "Road-based")
        .export_values();

    // Bind RoadObjectCorner
    py::class_<odr::RoadObjectCorner>(m, "RoadObjectCorner")
        .def(py::init<int, odr::Vec3D, double, odr::RoadObjectCorner::Type>(),
             py::arg("id"), py::arg("pt"), py::arg("height"), py::arg("type"),
             "Constructs a RoadObjectCorner with ID, point, height, and type")
        .def_readwrite("id", &odr::RoadObjectCorner::id, "Corner ID")
        .def_readwrite("pt", &odr::RoadObjectCorner::pt, "3D point")
        .def_readwrite("height", &odr::RoadObjectCorner::height, "Height")
        .def_readwrite("type", &odr::RoadObjectCorner::type, "Corner type");

    // Bind RoadObjectOutline
    py::class_<odr::RoadObjectOutline>(m, "RoadObjectOutline")
        .def(py::init<int, std::string, std::string, bool, bool>(),
             py::arg("id"), py::arg("fill_type"), py::arg("lane_type"), py::arg("outer"), py::arg("closed"),
             "Constructs a RoadObjectOutline with specified properties")
        .def_readwrite("id", &odr::RoadObjectOutline::id, "Outline ID")
        .def_readwrite("fill_type", &odr::RoadObjectOutline::fill_type, "Fill type")
        .def_readwrite("lane_type", &odr::RoadObjectOutline::lane_type, "Lane type")
        .def_readwrite("outer", &odr::RoadObjectOutline::outer, "Outer flag")
        .def_readwrite("closed", &odr::RoadObjectOutline::closed, "Closed flag")
        .def_readwrite("outline", &odr::RoadObjectOutline::outline, "List of RoadObjectCorner objects");

    // Bind RoadObject
    py::class_<odr::RoadObject>(m, "RoadObject")
        .def(py::init<std::string, std::string, double, double, double, double, double, double, double, double, double, double, double, std::string, std::string, std::string, std::string, bool>(),
             py::arg("road_id"), py::arg("id"), py::arg("s0"), py::arg("t0"), py::arg("z0"),
             py::arg("length"), py::arg("valid_length"), py::arg("width"), py::arg("radius"),
             py::arg("height"), py::arg("hdg"), py::arg("pitch"), py::arg("roll"),
             py::arg("type"), py::arg("name"), py::arg("orientation"), py::arg("subtype"),
             py::arg("is_dynamic"),
             "Constructs a RoadObject with specified properties")
        .def_static("get_cylinder", &odr::RoadObject::get_cylinder,
                    py::arg("eps"), py::arg("radius"), py::arg("height"),
                    py::return_value_policy::move,
                    "Generates a cylinder Mesh3D")
        .def_static("get_box", &odr::RoadObject::get_box,
                    py::arg("width"), py::arg("length"), py::arg("height"),
                    py::return_value_policy::move,
                    "Generates a box Mesh3D")
        .def_readwrite("road_id", &odr::RoadObject::road_id, "Road ID")
        .def_readwrite("id", &odr::RoadObject::id, "Object ID")
        .def_readwrite("type", &odr::RoadObject::type, "Object type")
        .def_readwrite("name", &odr::RoadObject::name, "Object name")
        .def_readwrite("orientation", &odr::RoadObject::orientation, "Orientation")
        .def_readwrite("subtype", &odr::RoadObject::subtype, "Subtype")
        .def_readwrite("s0", &odr::RoadObject::s0, "Start s-coordinate")
        .def_readwrite("t0", &odr::RoadObject::t0, "Start t-coordinate")
        .def_readwrite("z0", &odr::RoadObject::z0, "Start z-coordinate")
        .def_readwrite("length", &odr::RoadObject::length, "Length")
        .def_readwrite("valid_length", &odr::RoadObject::valid_length, "Valid length")
        .def_readwrite("width", &odr::RoadObject::width, "Width")
        .def_readwrite("radius", &odr::RoadObject::radius, "Radius")
        .def_readwrite("height", &odr::RoadObject::height, "Height")
        .def_readwrite("hdg", &odr::RoadObject::hdg, "Heading")
        .def_readwrite("pitch", &odr::RoadObject::pitch, "Pitch")
        .def_readwrite("roll", &odr::RoadObject::roll, "Roll")
        .def_readwrite("is_dynamic", &odr::RoadObject::is_dynamic, "Dynamic flag")
        .def_readwrite("repeats", &odr::RoadObject::repeats, "List of RoadObjectRepeat objects")
        .def_readwrite("outlines", &odr::RoadObject::outlines, "List of RoadObjectOutline objects")
        .def_readwrite("lane_validities", &odr::RoadObject::lane_validities, "List of LaneValidityRecord objects");
}
