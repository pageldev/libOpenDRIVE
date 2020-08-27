#include "Arc.h"
#include <cmath>

Arc::Arc(double s0, double x0, double y0, double hdg0, double length, double curvature)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Arc), curvature(curvature)
{
}

Point2D Arc::get_point(double s, double t) const
{
    double angle_at_s = (s - s0) * curvature - M_PI / 2;
    double r = 1 / curvature;
    double xs = (r - t) * std::cos(angle_at_s);
    double ys = (r - t) * std::sin(angle_at_s) + r;
    double xt = (std::cos(hdg0) * xs) - (std::sin(hdg0) * ys) + x0;
    double yt = (std::sin(hdg0) * xs) + (std::cos(hdg0) * ys) + y0;
    return Point2D{xt, yt};
}