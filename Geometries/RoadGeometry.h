#pragma once

#include <map>
#include <string>

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

struct Point2D
{
    double x, y;
};

struct Box2D
{
    Point2D min, max;
};

struct Point3D
{
    double x, y, z;
};

struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type);
    virtual ~RoadGeometry();
    virtual Point2D get_point(double s, double t = 0) const = 0;

    Geometry_type type;
    double s0;
    double x0;
    double y0;
    double hdg0;
    double length;
};
