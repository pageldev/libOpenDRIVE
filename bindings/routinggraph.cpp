#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <RoutingGraph.h>

namespace py = pybind11;

void init_routinggraph(py::module_ &m) {
    // Bind RoutingGraphEdge
    py::class_<odr::RoutingGraphEdge>(m, "RoutingGraphEdge")
        .def(py::init<odr::LaneKey, odr::LaneKey, double>(),
             py::arg("from"), py::arg("to"), py::arg("length"),
             "Constructs a RoutingGraphEdge with from and to LaneKeys and length")
        .def_readwrite("from", &odr::RoutingGraphEdge::from, "Source LaneKey")
        .def_readwrite("to", &odr::RoutingGraphEdge::to, "Destination LaneKey")
        .def_readwrite("weight", &odr::RoutingGraphEdge::weight, "Edge weight")
        .def(" __eq__", [](const odr::RoutingGraphEdge& a, const odr::RoutingGraphEdge& b) {
            return a.from == b.from && a.to == b.to && a.weight == b.weight;
        }, "Equality comparison");

    // Bind WeightedLaneKey
    py::class_<odr::WeightedLaneKey, odr::LaneKey>(m, "WeightedLaneKey")
        .def(py::init<const odr::LaneKey&, double>(),
             py::arg("lane_key"), py::arg("weight"),
             "Constructs a WeightedLaneKey from a LaneKey and weight")
        .def(py::init<std::string, double, int, double>(),
             py::arg("road_id"), py::arg("lanesection_s0"), py::arg("lane_id"), py::arg("weight"),
             "Constructs a WeightedLaneKey with road ID, lane section s0, lane ID, and weight")
        .def_readwrite("weight", &odr::WeightedLaneKey::weight, "Weight of the lane key")
        .def(" __eq__", [](const odr::WeightedLaneKey& a, const odr::WeightedLaneKey& b) {
            return static_cast<odr::LaneKey>(a) == static_cast<odr::LaneKey>(b) && a.weight == b.weight;
        }, "Equality comparison");

    // Bind RoutingGraph
    py::class_<odr::RoutingGraph>(m, "RoutingGraph")
        .def(py::init<>(), "Constructs an empty RoutingGraph")
        .def("add_edge", &odr::RoutingGraph::add_edge,
             py::arg("edge"),
             "Adds an edge to the graph")
        .def("get_lane_successors", &odr::RoutingGraph::get_lane_successors,
             py::arg("lane_key"),
             py::return_value_policy::move,
             "Returns the successor LaneKeys for the given LaneKey")
        .def("get_lane_predecessors", &odr::RoutingGraph::get_lane_predecessors,
             py::arg("lane_key"),
             py::return_value_policy::move,
             "Returns the predecessor LaneKeys for the given LaneKey")
        .def("shortest_path", &odr::RoutingGraph::shortest_path,
             py::arg("from"), py::arg("to"),
             py::return_value_policy::move,
             "Computes the shortest path between two LaneKeys")
        .def_readwrite("edges", &odr::RoutingGraph::edges, "Set of RoutingGraphEdges")
        .def_readwrite("lane_key_to_successors", &odr::RoutingGraph::lane_key_to_successors,
                       "Map of LaneKeys to their successor WeightedLaneKeys")
        .def_readwrite("lane_key_to_predecessors", &odr::RoutingGraph::lane_key_to_predecessors,
                       "Map of LaneKeys to their predecessor WeightedLaneKeys");
}
