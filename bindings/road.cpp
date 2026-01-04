// bindings/road.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "XmlNode.h"
#include "Road.h"

namespace py = pybind11;
using namespace odr;

void init_road(py::module_ &m) {
    // Bind Crossfall::Side enum
    py::enum_<Crossfall::Side>(m, "CrossfallSide")
        .value("Side_Both", Crossfall::Side::Side_Both)
        .value("Side_Left", Crossfall::Side::Side_Left)
        .value("Side_Right", Crossfall::Side::Side_Right)
        .export_values();

    // Bind Crossfall struct
    py::class_<Crossfall, CubicSpline>(m, "Crossfall")
        .def(py::init<>())
        .def("get_crossfall", &Crossfall::get_crossfall)
        .def_readwrite("sides", &Crossfall::sides);

    // Bind RoadLink::Type enum
    py::enum_<RoadLink::Type>(m, "RoadLinkType")
        .value("Type_None", RoadLink::Type::Type_None)
        .value("Type_Road", RoadLink::Type::Type_Road)
        .value("Type_Junction", RoadLink::Type::Type_Junction)
        .export_values();

    // Bind RoadLink struct
    py::class_<RoadLink, XmlNode, std::shared_ptr<RoadLink>>(m, "RoadLink")
        .def(py::init<>())
        .def(py::init<std::string, RoadLink::Type, RoadLink::ContactPoint>())
        .def_readwrite("id", &RoadLink::id)
        .def_readwrite("type", &RoadLink::type)
        .def_readwrite("contact_point", &RoadLink::contact_point);

    // Bind RoadNeighbor struct
    py::class_<RoadNeighbor, XmlNode, std::shared_ptr<RoadNeighbor>>(m, "RoadNeighbor")
        .def(py::init<std::string, std::string, std::string>())
        .def_readwrite("id", &RoadNeighbor::id)
        .def_readwrite("side", &RoadNeighbor::side)
        .def_readwrite("direction", &RoadNeighbor::direction);

    // Bind SpeedRecord struct
    py::class_<SpeedRecord, XmlNode, std::shared_ptr<SpeedRecord>>(m, "SpeedRecord")
        .def(py::init<std::string, std::string>())
        .def_readwrite("max", &SpeedRecord::max)
        .def_readwrite("unit", &SpeedRecord::unit);

    // Bind Road class
    py::class_<Road, XmlNode, std::shared_ptr<Road>>(m, "Road")
        .def(py::init<std::string, double, std::string, std::string, bool>(),
             py::arg("id"), py::arg("length"), py::arg("junction"), py::arg("name"), py::arg("left_hand_traffic") = false)
        .def("get_lanesections", &Road::get_lanesections)
        .def("get_road_objects", &Road::get_road_objects)
        .def("get_road_signals", &Road::get_road_signals)
        .def("get_lanesection_s0", &Road::get_lanesection_s0)
        .def("get_lanesection", &Road::get_lanesection)
        .def("get_lanesection_end", py::overload_cast<const LaneSection&>(&Road::get_lanesection_end, py::const_))
        .def("get_lanesection_end", py::overload_cast<const double>(&Road::get_lanesection_end, py::const_))
        .def("get_lanesection_length", py::overload_cast<const LaneSection&>(&Road::get_lanesection_length, py::const_))
        .def("get_lanesection_length", py::overload_cast<const double>(&Road::get_lanesection_length, py::const_))
        .def("get_xyz", &Road::get_xyz, py::arg("s"), py::arg("t"), py::arg("h"),
             py::arg("e_s") = nullptr, py::arg("e_t") = nullptr, py::arg("e_h") = nullptr)
        .def("get_surface_pt", &Road::get_surface_pt, py::arg("s"), py::arg("t"), py::arg("vn") = nullptr)
        .def("get_lane_border_line", py::overload_cast<const Lane&, const double, const double, const double, const bool>(
                                         &Road::get_lane_border_line, py::const_),
             py::arg("lane"), py::arg("s_start"), py::arg("s_end"), py::arg("eps"), py::arg("outer") = true)
        .def("get_lane_border_line", py::overload_cast<const Lane&, const double, const bool>(
                                         &Road::get_lane_border_line, py::const_),
             py::arg("lane"), py::arg("eps"), py::arg("outer") = true)
        .def("get_lane_mesh", py::overload_cast<const Lane&, const double, const double, const double, std::vector<uint32_t>*>(
                                  &Road::get_lane_mesh, py::const_),
             py::arg("lane"), py::arg("s_start"), py::arg("s_end"), py::arg("eps"), py::arg("outline_indices") = nullptr)
        .def("get_lane_mesh", py::overload_cast<const Lane&, const double, std::vector<uint32_t>*>(
                                  &Road::get_lane_mesh, py::const_),
             py::arg("lane"), py::arg("eps"), py::arg("outline_indices") = nullptr)
        .def("get_roadmark_mesh", &Road::get_roadmark_mesh)
        .def("get_road_signal_mesh", &Road::get_road_signal_mesh)
        .def("get_road_object_mesh", &Road::get_road_object_mesh)
        .def("approximate_lane_border_linear", py::overload_cast<const Lane&, const double, const double, const double, const bool>(
                                                  &Road::approximate_lane_border_linear, py::const_),
             py::arg("lane"), py::arg("s_start"), py::arg("s_end"), py::arg("eps"), py::arg("outer") = true)
        .def("approximate_lane_border_linear", py::overload_cast<const Lane&, const double, const bool>(
                                                  &Road::approximate_lane_border_linear, py::const_),
             py::arg("lane"), py::arg("eps"), py::arg("outer") = true)
        .def_readwrite("length", &Road::length)
        .def_readwrite("id", &Road::id)
        .def_readwrite("junction", &Road::junction)
        .def_readwrite("name", &Road::name)
        .def_readwrite("left_hand_traffic", &Road::left_hand_traffic)
        .def_readwrite("predecessor", &Road::predecessor)
        .def_readwrite("successor", &Road::successor)
        .def_readwrite("neighbors", &Road::neighbors)
        .def_readwrite("lane_offset", &Road::lane_offset)
        .def_readwrite("superelevation", &Road::superelevation)
        .def_readwrite("crossfall", &Road::crossfall)
        .def_readonly("ref_line", &Road::ref_line)
        .def_readwrite("s_to_lanesection", &Road::s_to_lanesection)
        .def_readwrite("s_to_type", &Road::s_to_type)
        .def_readwrite("s_to_speed", &Road::s_to_speed)
        .def_readwrite("id_to_object", &Road::id_to_object)
        .def_readwrite("id_to_signal", &Road::id_to_signal);
}

PYBIND11_MODULE(road, m) {
    init_road(m);
}
