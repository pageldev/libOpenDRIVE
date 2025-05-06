#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <OpenDriveMap.h>
#include <Road.h>
#include <Junction.h>
#include <Mesh.h>
#include <RoadNetworkMesh.h>
#include <RoutingGraph.h>

namespace py = pybind11;

// Forward declarations for opaque types
namespace odr {
struct LanesMesh;
struct RoadmarksMesh;
struct RoadObjectsMesh;
struct RoadSignalsMesh;
class RoutingGraph;
}

void init_opendrivemap(py::module_ &m) {
    // Bind Mesh3D
    py::class_<odr::Mesh3D>(m, "Mesh3D")
        .def(py::init<>())
        .def_readwrite("vertices", &odr::Mesh3D::vertices, "Array of vertex coordinates (x, y, z)")
        .def_readwrite("indices", &odr::Mesh3D::indices, "Array of triangle indices")
        .def_readwrite("normals", &odr::Mesh3D::normals, "Array of vertex normals (x, y, z)")
        .def_readwrite("st_coordinates", &odr::Mesh3D::st_coordinates, "Array of texture coordinates (s, t)");

    // Bind LanesMesh, RoadmarksMesh, RoadObjectsMesh, RoadSignalsMesh as opaque types
    py::class_<odr::LanesMesh>(m, "LanesMesh")
        .def(py::init<>());
    py::class_<odr::RoadmarksMesh>(m, "RoadmarksMesh")
        .def(py::init<>());
    py::class_<odr::RoadObjectsMesh>(m, "RoadObjectsMesh")
        .def(py::init<>());
    py::class_<odr::RoadSignalsMesh>(m, "RoadSignalsMesh")
        .def(py::init<>());

    // Bind RoadNetworkMesh
    py::class_<odr::RoadNetworkMesh>(m, "RoadNetworkMesh")
        .def(py::init<>())
        .def("get_mesh", &odr::RoadNetworkMesh::get_mesh, py::return_value_policy::move, "Returns the combined 3D mesh")
        .def_readwrite("lanes_mesh", &odr::RoadNetworkMesh::lanes_mesh, "Mesh for lanes")
        .def_readwrite("roadmarks_mesh", &odr::RoadNetworkMesh::roadmarks_mesh, "Mesh for road marks")
        .def_readwrite("road_objects_mesh", &odr::RoadNetworkMesh::road_objects_mesh, "Mesh for road objects")
        .def_readwrite("road_signals_mesh", &odr::RoadNetworkMesh::road_signals_mesh, "Mesh for road signals");

    // Forward-declare Road, Junction, and RoutingGraph as opaque types
    py::class_<odr::Road>(m, "Road");
    py::class_<odr::Junction>(m, "Junction");
    py::class_<odr::RoutingGraph>(m, "RoutingGraph");

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
        .def_readonly("id_to_junction", &odr::OpenDriveMap::id_to_junction, "Map of junction IDs to Junction objects");
}
