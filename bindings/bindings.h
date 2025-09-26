#pragma once
#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_opendrivemap(py::module_ &);
void init_mesh(py::module_ &);
void init_junction(py::module_ &);
void init_lane(py::module_ &);
void init_lanesection(py::module_ &);
void init_road(py::module_ &);
void init_routinggraph(py::module_ &);
void init_roadmark(py::module_ &);
void init_roadnetworkmesh(py::module_ &);
void init_roadobject(py::module_ &);
void init_lanevalidityrecord(py::module_ &);
void init_refline(py::module_ &);
void init_roadsignal(py::module_ &);
void init_math(py::module_ &);
void init_xml_node(py::module_ &m);
