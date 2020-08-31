#pragma once

#include <map>
#include <string>

namespace odr
{

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

template <typename T>
struct Point2D
{
    T x, y;
};

template <typename T>
struct Point3D
{
    T x, y, z;
};

template <typename T>
struct Box2D
{
    Point2D<T> min, max;
};

struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type);
    virtual ~RoadGeometry();

    virtual Point2D<double> get_point(double s, double t = 0) const = 0;
    virtual Box2D<double>   get_bbox() const = 0;

    Geometry_type type;

    double s0;
    double x0;
    double y0;
    double hdg0;
    double length;
};

} // namespace odr
