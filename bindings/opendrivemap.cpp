// bindings/opendrivemap.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <OpenDriveMap.h>
#include <Road.h>
#include <Junction.h>
#include <Mesh.h>
#include <RoadNetworkMesh.h>
#include <RoutingGraph.h>
#include <pugixml.hpp> // Include pugixml header for xml_parse_result

namespace py = pybind11;

void init_opendrivemap(py::module_ &m) {
    // Bind pugi::xml_parse_result
    py::class_<pugi::xml_parse_result>(m, "XmlParseResult")
        .def_readonly("status", &pugi::xml_parse_result::status, "Status of XML parsing")
        .def_readonly("offset", &pugi::xml_parse_result::offset, "Offset of parsing error, if any")
        .def_readonly("encoding", &pugi::xml_parse_result::encoding, "Encoding of the parsed document")
        .def("description", &pugi::xml_parse_result::description, "Returns a description of the parsing error")
        .def("__bool__", [](const pugi::xml_parse_result &result) { return result.status == pugi::xml_parse_status::status_ok; },
             "Returns true if parsing was successful");

    // Bind OpenDriveMap
    py::class_<odr::OpenDriveMap>(m, "OpenDriveMap")
        .def(py::init<const std::string &, bool, bool, bool, bool, bool, bool, bool>(),
             py::arg("xodr_file"),
             py::arg("center_map") = false,
             py::arg("with_road_objects") = true,
             py::arg("with_lateral_profile") = true,
             py::arg("with_lane_height") = true,
             py::arg("abs_z_for_for_local_road_obj_outline") = false,
             py::arg("fix_spiral_edge_cases") = true,
             py::arg("with_road_signals") = true,
             "Constructs an OpenDriveMap from an XODR file with configuration options")
        .def("get_road", &odr::OpenDriveMap::get_road, py::arg("id"), py::return_value_policy::copy,
             "Returns the road with the specified ID")
        .def("get_roads", &odr::OpenDriveMap::get_roads, py::return_value_policy::copy,
             "Returns a list of roads")
        .def("get_junction", &odr::OpenDriveMap::get_junction, py::arg("id"), py::return_value_policy::copy,
             "Returns the junction with the specified ID")
        .def("get_junctions", &odr::OpenDriveMap::get_junctions, py::return_value_policy::copy,
             "Returns a list of junctions")
        .def("get_road_network_mesh", &odr::OpenDriveMap::get_road_network_mesh, py::arg("eps"),
             py::return_value_policy::move, "Returns the road network mesh")
        .def("get_routing_graph", &odr::OpenDriveMap::get_routing_graph, py::return_value_policy::move,
             "Returns the routing graph")
        .def_readwrite("proj4", &odr::OpenDriveMap::proj4, "Projection string")
        .def_readwrite("x_offs", &odr::OpenDriveMap::x_offs, "X offset")
        .def_readwrite("y_offs", &odr::OpenDriveMap::y_offs, "Y offset")
        .def_readonly("xodr_file", &odr::OpenDriveMap::xodr_file, "Path to the XODR file")
        .def_readonly("id_to_road", &odr::OpenDriveMap::id_to_road, "Map of road IDs to Road objects")
        .def_readonly("id_to_junction", &odr::OpenDriveMap::id_to_junction, "Map of junction IDs to Junction objects")
        .def_readonly("xml_parse_result", &odr::OpenDriveMap::xml_parse_result, "Result of XML parsing");
}
