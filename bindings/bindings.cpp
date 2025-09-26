#include "bindings.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <OpenDriveMap.h>
#include <Road.h> // For RoadLink::ContactPoint

namespace py = pybind11;

PYBIND11_MODULE(opendrive, m) {
    m.doc() = "Python bindings for libOpenDRIVE";

    // RoadLink depends on XmlNode
    init_xml_node(m);

    // Bind ContactPoint enum
    py::enum_<odr::RoadLink::ContactPoint>(m, "ContactPoint")
        .value("None", odr::RoadLink::ContactPoint::ContactPoint_None)
        .value("Start", odr::RoadLink::ContactPoint::ContactPoint_Start)
        .value("End", odr::RoadLink::ContactPoint::ContactPoint_End)
        .export_values();

    // Initialize other modules
    init_opendrivemap(m);
    init_mesh(m);
    init_junction(m);
    init_lane(m);
    init_lanesection(m);
    init_road(m);
    init_refline(m);
    init_routinggraph(m);
    init_roadmark(m);
    init_roadnetworkmesh(m);
    init_roadobject(m);
    init_lanevalidityrecord(m);
    init_roadsignal(m);
    init_math(m);
}
