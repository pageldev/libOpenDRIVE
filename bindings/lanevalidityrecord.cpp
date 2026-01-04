#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <LaneValidityRecord.h>

namespace py = pybind11;

void init_lanevalidityrecord(py::module_ &m) {
    // Bind LaneValidityRecord
    py::class_<odr::LaneValidityRecord>(m, "LaneValidityRecord")
        .def(py::init<int, int>(),
             py::arg("from_lane"), py::arg("to_lane"),
             "Constructs a LaneValidityRecord with from_lane and to_lane")
        .def_readwrite("from_lane", &odr::LaneValidityRecord::from_lane, "Starting lane ID")
        .def_readwrite("to_lane", &odr::LaneValidityRecord::to_lane, "Ending lane ID");
}
