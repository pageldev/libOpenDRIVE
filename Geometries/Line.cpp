#include "Line.h"
#include <cmath>

Line::Line(double s0, double x0, double y0, double hdg0, double length)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Line)
{
}

Point2D Line::get_point(double s, double t)
{
    double xt = (std::cos(hdg0) * (s - s0)) - (std::sin(hdg0) * t) + x0;
    double yt = (std::sin(hdg0) * (s - s0)) + (std::cos(hdg0) * t) + y0;
    return Point2D{xt, yt};
}