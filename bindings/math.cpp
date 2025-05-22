#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Math.hpp"

namespace py = pybind11;
using namespace odr;

void init_math(py::module_ &m) {
    // Bind Vec1D (std::array<double, 1>)
    py::class_<Vec1D>(m, "Vec1D")
        .def(py::init([](double x) { return Vec1D{x}; }))
        .def("__getitem__", [](const Vec1D& v, size_t i) { return v.at(i); })
        .def("__setitem__", [](Vec1D& v, size_t i, double value) { v.at(i) = value; })
        .def("__len__", [](const Vec1D&) { return 1; })
        .def("__repr__", [](const Vec1D& v) { return "Vec1D(" + std::to_string(v[0]) + ")"; });

    // Bind Vec2D (std::array<double, 2>)
    py::class_<Vec2D>(m, "Vec2D")
        .def(py::init([](double x, double y) { return Vec2D{x, y}; }))
        .def_property("x", [](const Vec2D& v) { return v[0]; }, [](Vec2D& v, double x) { v[0] = x; }, "X coordinate")
        .def_property("y", [](const Vec2D& v) { return v[1]; }, [](Vec2D& v, double y) { v[1] = y; }, "Y coordinate")
        .def("__getitem__", [](const Vec2D& v, size_t i) { return v.at(i); })
        .def("__setitem__", [](Vec2D& v, size_t i, double value) { v.at(i) = value; })
        .def("__len__", [](const Vec2D&) { return 2; })
        .def("__repr__", [](const Vec2D& v) {
            return "Vec2D(" + std::to_string(v[0]) + ", " + std::to_string(v[1]) + ")";
        });

    // Bind Vec3D (std::array<double, 3>)
    py::class_<Vec3D>(m, "Vec3D")
        .def(py::init([](double x, double y, double z) { return Vec3D{x, y, z}; }))
        .def_property("x", [](const Vec3D& v) { return v[0]; }, [](Vec3D& v, double x) { v[0] = x; }, "X coordinate")
        .def_property("y", [](const Vec3D& v) { return v[1]; }, [](Vec3D& v, double y) { v[1] = y; }, "Y coordinate")
        .def_property("z", [](const Vec3D& v) { return v[2]; }, [](Vec3D& v, double z) { v[2] = z; }, "Z coordinate")
        .def("__getitem__", [](const Vec3D& v, size_t i) { return v.at(i); })
        .def("__setitem__", [](Vec3D& v, size_t i, double value) { v.at(i) = value; })
        .def("__len__", [](const Vec3D&) { return 3; })
        .def("__repr__", [](const Vec3D& v) {
            return "Vec3D(" + std::to_string(v[0]) + ", " + std::to_string(v[1]) + ", " + std::to_string(v[2]) + ")";
        });

    // Bind Line3D (std::vector<Vec3D>)
    py::class_<Line3D>(m, "Line3D")
        .def(py::init<>())
        .def("push_back", [](Line3D& line, const Vec3D& v) { line.push_back(v); },
             py::arg("v"), "Add a Vec3D to the line")
        .def("clear", &Line3D::clear, "Clear all points in the line")
        .def("__getitem__", [](const Line3D& line, size_t i) { return line.at(i); })
        .def("__setitem__", [](Line3D& line, size_t i, const Vec3D& v) { line.at(i) = v; })
        .def("__len__", [](const Line3D& line) { return line.size(); })
        .def("__repr__", [](const Line3D& line) {
            std::string s = "Line3D([";
            for (size_t i = 0; i < line.size(); ++i) {
                s += (i > 0 ? ", " : "") + std::to_string(line[i][0]) + ", " +
                     std::to_string(line[i][1]) + ", " + std::to_string(line[i][2]);
            }
            return s + "])";
        });

    // Bind Mat3D (std::array<std::array<double, 3>, 3>)
    py::class_<Mat3D>(m, "Mat3D")
        .def(py::init<>())
        .def("__getitem__", [](const Mat3D& m, size_t i) { return m.at(i); })
        .def("__setitem__", [](Mat3D& m, size_t i, const std::array<double, 3>& v) { m.at(i) = v; })
        .def("__len__", [](const Mat3D&) { return 3; })
        .def("__repr__", [](const Mat3D& m) {
            std::string s = "Mat3D([";
            for (size_t i = 0; i < 3; ++i) {
                if (i > 0) s += ", ";
                s += "[" + std::to_string(m[i][0]) + ", " +
                     std::to_string(m[i][1]) + ", " + std::to_string(m[i][2]) + "]";
            }
            return s + "])";
        });

    // Bind Math functions
    m.def("sign", &sign<double>, py::arg("val"), "Return the sign of a value (-1, 0, or 1)");
    m.def("add", py::overload_cast<const Vec2D&, const Vec2D&>(&add<double, 2>),
          py::arg("a"), py::arg("b"), "Add two Vec2D vectors");
    m.def("add", py::overload_cast<const Vec3D&, const Vec3D&>(&add<double, 3>),
          py::arg("a"), py::arg("b"), "Add two Vec3D vectors");
    m.def("add", py::overload_cast<const double&, const Vec2D&>(&add<double, 2>),
          py::arg("scalar"), py::arg("a"), "Add a scalar to a Vec2D");
    m.def("add", py::overload_cast<const double&, const Vec3D&>(&add<double, 3>),
          py::arg("scalar"), py::arg("a"), "Add a scalar to a Vec3D");
    m.def("sub", py::overload_cast<const Vec2D&, const Vec2D&>(&sub<double, 2>),
          py::arg("a"), py::arg("b"), "Subtract two Vec2D vectors");
    m.def("sub", py::overload_cast<const Vec3D&, const Vec3D&>(&sub<double, 3>),
          py::arg("a"), py::arg("b"), "Subtract two Vec3D vectors");
    m.def("sub", py::overload_cast<const double&, const Vec2D&>(&sub<double, 2>),
          py::arg("scalar"), py::arg("a"), "Subtract a Vec2D from a scalar");
    m.def("sub", py::overload_cast<const double&, const Vec3D&>(&sub<double, 3>),
          py::arg("scalar"), py::arg("a"), "Subtract a Vec3D from a scalar");
    m.def("mut", py::overload_cast<const double&, const Vec2D&>(&mut<double, 2>),
          py::arg("scalar"), py::arg("a"), "Multiply a Vec2D by a scalar");
    m.def("mut", py::overload_cast<const double&, const Vec3D&>(&mut<double, 3>),
          py::arg("scalar"), py::arg("a"), "Multiply a Vec3D by a scalar");
    m.def("euclDistance", [](const Vec2D& a, const Vec2D& b) { return euclDistance<double, 2>(a, b); },
          py::arg("a"), py::arg("b"), "Compute Euclidean distance between two Vec2D vectors");
    m.def("euclDistance", [](const Vec3D& a, const Vec3D& b) { return euclDistance<double, 3>(a, b); },
          py::arg("a"), py::arg("b"), "Compute Euclidean distance between two Vec3D vectors");
    m.def("squaredNorm", py::overload_cast<const Vec2D&>(&squaredNorm<double, 2>),
          py::arg("v"), "Compute squared norm of a Vec2D");
    m.def("squaredNorm", py::overload_cast<const Vec3D&>(&squaredNorm<double, 3>),
          py::arg("v"), "Compute squared norm of a Vec3D");
    m.def("norm", py::overload_cast<const Vec2D&>(&norm<double, 2>),
          py::arg("v"), "Compute norm of a Vec2D");
    m.def("norm", py::overload_cast<const Vec3D&>(&norm<double, 3>),
          py::arg("v"), "Compute norm of a Vec3D");
    m.def("normalize", py::overload_cast<const Vec2D&>(&normalize<double, 2>),
          py::arg("v"), "Normalize a Vec2D");
    m.def("normalize", py::overload_cast<const Vec3D&>(&normalize<double, 3>),
          py::arg("v"), "Normalize a Vec3D");
    m.def("crossProduct", &crossProduct<double>,
          py::arg("a"), py::arg("b"), "Compute cross product of two Vec3D vectors");
    m.def("MatVecMultiplication", &MatVecMultiplication<double, 3>,
          py::arg("m"), py::arg("v"), "Multiply a Mat3D by a Vec3D");
    m.def("EulerAnglesToMatrix", &EulerAnglesToMatrix<double>,
          py::arg("r_x"), py::arg("r_y"), py::arg("r_z"), "Convert Euler angles to a Mat3D rotation matrix");
}

PYBIND11_MODULE(opendrive_math, m) {
    init_math(m);
}
