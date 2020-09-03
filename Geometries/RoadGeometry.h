#pragma once

#include <array>
#include <map>
#include <string>

namespace odr
{

template <typename T, size_t Dim, typename std::enable_if_t<(Dim > 1)> * = nullptr, typename std::enable_if_t<std::is_arithmetic<T>::value> * = nullptr>
using Point = std::array<T, Dim>;

using Point2D = Point<double, 2>;
using Point3D = Point<double, 3>;

struct Box2D
{
    Box2D(Point2D min, Point2D max);
    double get_distance(const Point2D &pt);

    Point2D min, max;
    Point2D center;
    double width, height;
};

enum class Geometry_type
{
    Line,
    Spiral,
    Arc,
    ParamPoly3
};

const std::map<Geometry_type, std::string> geometry_type2str = {
    {Geometry_type::Line, "line"},
    {Geometry_type::Spiral, "spiral"},
    {Geometry_type::Arc, "arc"},
    {Geometry_type::ParamPoly3, "paramPoly3"}};

struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type);
    virtual ~RoadGeometry();

    virtual Point2D get_point(double s, double t = 0) const = 0;
    virtual Box2D   get_bbox() const = 0;

    Geometry_type type;

    double s0;
    double x0;
    double y0;
    double hdg0;
    double length;
};

} // namespace odr
