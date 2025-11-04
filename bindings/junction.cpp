#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "XmlNode.h"
#include <Junction.h>


namespace py = pybind11;

void init_junction(py::module_ &m) {
    // Bind JunctionLaneLink
    py::class_<odr::JunctionLaneLink>(m, "JunctionLaneLink")
        .def(py::init<int, int>(),
             py::arg("from"), py::arg("to"),
             "Constructs a JunctionLaneLink with from and to lane IDs")
        .def_readwrite("from", &odr::JunctionLaneLink::from, "Source lane ID")
        .def_readwrite("to", &odr::JunctionLaneLink::to, "Destination lane ID");

    // Bind JunctionConnection
    py::class_<odr::JunctionConnection>(m, "JunctionConnection")
        .def(py::init<std::string, std::string, std::string, odr::JunctionConnection::ContactPoint>(),
             py::arg("id"), py::arg("incoming_road"), py::arg("connecting_road"), py::arg("contact_point"),
             "Constructs a JunctionConnection with ID, roads, and contact point")
        .def_readwrite("id", &odr::JunctionConnection::id, "Connection ID")
        .def_readwrite("incoming_road", &odr::JunctionConnection::incoming_road, "Incoming road ID")
        .def_readwrite("connecting_road", &odr::JunctionConnection::connecting_road, "Connecting road ID")
        .def_readwrite("contact_point", &odr::JunctionConnection::contact_point, "Contact point (None, Start, End)")
        .def_readwrite("lane_links", &odr::JunctionConnection::lane_links, "Set of lane links");

    // Bind JunctionPriority
    py::class_<odr::JunctionPriority>(m, "JunctionPriority")
        .def(py::init<std::string, std::string>(),
             py::arg("high"), py::arg("low"),
             "Constructs a JunctionPriority with high and low priority road IDs")
        .def_readwrite("high", &odr::JunctionPriority::high, "High priority road ID")
        .def_readwrite("low", &odr::JunctionPriority::low, "Low priority road ID");

    // Bind JunctionController
    py::class_<odr::JunctionController>(m, "JunctionController")
        .def(py::init<std::string, std::string, std::uint32_t>(),
             py::arg("id"), py::arg("type"), py::arg("sequence"),
             "Constructs a JunctionController with ID, type, and sequence")
        .def_readwrite("id", &odr::JunctionController::id, "Controller ID")
        .def_readwrite("type", &odr::JunctionController::type, "Controller type")
        .def_readwrite("sequence", &odr::JunctionController::sequence, "Controller sequence number");

    // Bind Junction
    py::class_<odr::Junction>(m, "Junction")
        .def(py::init<std::string, std::string>(),
             py::arg("name"), py::arg("id"),
             "Constructs a Junction with name and ID")
        .def_readwrite("name", &odr::Junction::name, "Junction name")
        .def_readwrite("id", &odr::Junction::id, "Junction ID")
        .def_readwrite("id_to_connection", &odr::Junction::id_to_connection, "Map of connection IDs to JunctionConnection objects")
        .def_readwrite("id_to_controller", &odr::Junction::id_to_controller, "Map of controller IDs to JunctionController objects")
        .def_readwrite("priorities", &odr::Junction::priorities, "Set of JunctionPriority objects");
}
