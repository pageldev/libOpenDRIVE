#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <RoadNetworkMesh.h>

namespace py = pybind11;

void init_roadnetworkmesh(py::module_ &m) {
    // Bind RoadsMesh
    py::class_<odr::RoadsMesh, odr::Mesh3D>(m, "RoadsMesh")
        .def(py::init<>(), "Constructs an empty RoadsMesh")
        .def("get_road_id", &odr::RoadsMesh::get_road_id,
             py::arg("vert_idx"),
             "Returns the road ID for the given vertex index")
        .def("get_idx_interval_road", &odr::RoadsMesh::get_idx_interval_road,
             py::arg("vert_idx"),
             py::return_value_policy::move,
             "Returns the index interval for the road at the given vertex index")
        .def_readwrite("road_start_indices", &odr::RoadsMesh::road_start_indices,
                       "Map of vertex indices to road IDs")
        .def_readwrite("vertices", &odr::Mesh3D::vertices, py::return_value_policy::reference,
                       "List of 3D vertices");

    // Bind LanesMesh
    py::class_<odr::LanesMesh, odr::RoadsMesh>(m, "LanesMesh")
        .def(py::init<>(), "Constructs an empty LanesMesh")
        .def("get_lanesec_s0", &odr::LanesMesh::get_lanesec_s0,
             py::arg("vert_idx"),
             "Returns the lane section s0 for the given vertex index")
        .def("get_lane_id", &odr::LanesMesh::get_lane_id,
             py::arg("vert_idx"),
             "Returns the lane ID for the given vertex index")
        .def("get_idx_interval_lanesec", &odr::LanesMesh::get_idx_interval_lanesec,
             py::arg("vert_idx"),
             py::return_value_policy::move,
             "Returns the index interval for the lane section at the given vertex index")
        .def("get_idx_interval_lane", &odr::LanesMesh::get_idx_interval_lane,
             py::arg("vert_idx"),
             py::return_value_policy::move,
             "Returns the index interval for the lane at the given vertex index")
        .def("get_lane_outline_indices", &odr::LanesMesh::get_lane_outline_indices,
             py::return_value_policy::move,
             "Returns the lane outline indices")
        .def_readwrite("lanesec_start_indices", &odr::LanesMesh::lanesec_start_indices,
                       "Map of vertex indices to lane section s0 values")
        .def_readwrite("lane_start_indices", &odr::LanesMesh::lane_start_indices,
                       "Map of vertex indices to lane IDs");

    // Bind RoadmarksMesh
    py::class_<odr::RoadmarksMesh, odr::LanesMesh>(m, "RoadmarksMesh")
        .def(py::init<>(), "Constructs an empty RoadmarksMesh")
        .def("get_roadmark_type", &odr::RoadmarksMesh::get_roadmark_type,
             py::arg("vert_idx"),
             "Returns the roadmark type for the given vertex index")
        .def("get_idx_interval_roadmark", &odr::RoadmarksMesh::get_idx_interval_roadmark,
             py::arg("vert_idx"),
             py::return_value_policy::move,
             "Returns the index interval for the roadmark at the given vertex index")
        .def("get_roadmark_outline_indices", &odr::RoadmarksMesh::get_roadmark_outline_indices,
             py::return_value_policy::move,
             "Returns the roadmark outline indices")
        .def_readwrite("roadmark_type_start_indices", &odr::RoadmarksMesh::roadmark_type_start_indices,
                       "Map of vertex indices to roadmark types");

    // Bind RoadObjectsMesh
    py::class_<odr::RoadObjectsMesh, odr::RoadsMesh>(m, "RoadObjectsMesh")
        .def(py::init<>(), "Constructs an empty RoadObjectsMesh")
        .def("get_road_object_id", &odr::RoadObjectsMesh::get_road_object_id,
             py::arg("vert_idx"),
             "Returns the road object ID for the given vertex index")
        .def("get_idx_interval_road_object", &odr::RoadObjectsMesh::get_idx_interval_road_object,
             py::arg("vert_idx"),
             py::return_value_policy::move,
             "Returns the index interval for the road object at the given vertex index")
        .def_readwrite("road_object_start_indices", &odr::RoadObjectsMesh::road_object_start_indices,
                       "Map of vertex indices to road object IDs");

    // Bind RoadSignalsMesh
    py::class_<odr::RoadSignalsMesh, odr::RoadsMesh>(m, "RoadSignalsMesh")
        .def(py::init<>(), "Constructs an empty RoadSignalsMesh")
        .def("get_road_signal_id", &odr::RoadSignalsMesh::get_road_signal_id,
             py::arg("vert_idx"),
             "Returns the road signal ID for the given vertex index")
        .def("get_idx_interval_signal", &odr::RoadSignalsMesh::get_idx_interval_signal,
             py::arg("vert_idx"),
             py::return_value_policy::move,
             "Returns the index interval for the signal at the given vertex index")
        .def_readwrite("road_signal_start_indices", &odr::RoadSignalsMesh::road_signal_start_indices,
                       "Map of vertex indices to road signal IDs");

    // Bind RoadNetworkMesh
    py::class_<odr::RoadNetworkMesh>(m, "RoadNetworkMesh")
        .def(py::init<>(), "Constructs an empty RoadNetworkMesh")
        .def("get_mesh", &odr::RoadNetworkMesh::get_mesh,
             py::return_value_policy::copy,
             "Returns the combined Mesh3D")
        .def_readwrite("lanes_mesh", &odr::RoadNetworkMesh::lanes_mesh, "Lanes mesh")
        .def_readwrite("roadmarks_mesh", &odr::RoadNetworkMesh::roadmarks_mesh, "Roadmarks mesh")
        .def_readwrite("road_objects_mesh", &odr::RoadNetworkMesh::road_objects_mesh, "Road objects mesh")
        .def_readwrite("road_signals_mesh", &odr::RoadNetworkMesh::road_signals_mesh, "Road signals mesh");
}
