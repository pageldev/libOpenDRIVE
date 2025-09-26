#include <pybind11/pybind11.h>
#include "XmlNode.h"

namespace py = pybind11;
using namespace odr;

void init_xml_node(py::module_ &m)
{
    //py::class_<XmlNode>(m, "XmlNode");
    py::class_<XmlNode, std::shared_ptr<XmlNode>>(m, "XmlNode");
}
