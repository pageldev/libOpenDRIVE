#pragma once

#include "Spiral/odrSpiral.h"

#include <utility>
#include <cmath>
#include <cstring>


enum class Geometry_type { line, spiral, arc, paramPoly3};

struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type);
    virtual ~RoadGeometry();
    virtual std::pair<double, double> get_point(double s, double t = 0) = 0;

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
    std::pair<double, double> get_point(double s, double t = 0) override;
};


struct Spiral : public RoadGeometry
{
    Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end);
    std::pair<double, double> get_point(double s, double t = 0) override;
    
    double curv_start;
    double curv_end;
    double c_dot;
};


struct Arc : public RoadGeometry
{
    Arc(double s0, double x0, double y0, double hdg0, double length, double curvature);
    std::pair<double, double> get_point(double s, double t = 0) override;
    
    double curvature;
};


struct ParamPoly3 : public RoadGeometry
{
    ParamPoly3(double s0, double x0, double y0, double hdg0, double length
        , double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV);
    std::pair<double, double> get_point(double s, double t = 0) override;

    double aU, bU, cU, dU, aV, bV, cV, dV;
};