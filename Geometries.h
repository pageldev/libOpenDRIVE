#pragma once

#include <map>
#include <string>


enum class Geometry_type { Line, Spiral, Arc, ParamPoly3 };

const std::map<Geometry_type, std::string> geometry_type2str = {
    { Geometry_type::Line , "line" },
    { Geometry_type::Spiral , "spiral" },
    { Geometry_type::Arc , "arc" },
    { Geometry_type::ParamPoly3 , "paramPoly3" }
};


struct Point3D {
    double x, y, z;
};


struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type);
    virtual ~RoadGeometry();
    virtual Point3D get_point(double s, double t = 0) = 0;

    Geometry_type type;
    double s0;
    double x0;
    double y0;
    double hdg0;
    double length;
};


struct Line : public RoadGeometry
{
    Line(double s0, double x0, double y0, double hdg0, double length);
    Point3D get_point(double s, double t = 0) override;
};


struct Spiral : public RoadGeometry
{
    Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end);
    Point3D get_point(double s, double t = 0) override;
    
    double curv_start;
    double curv_end;
    double c_dot;
};


struct Arc : public RoadGeometry
{
    Arc(double s0, double x0, double y0, double hdg0, double length, double curvature);
    Point3D get_point(double s, double t = 0) override;
    
    double curvature;
};


struct ParamPoly3 : public RoadGeometry
{
    ParamPoly3(double s0, double x0, double y0, double hdg0, double length
        , double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV);
    Point3D get_point(double s, double t = 0) override;

    double aU, bU, cU, dU, aV, bV, cV, dV;
};