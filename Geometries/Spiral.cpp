#include "Spiral.h"
#include "Spiral/odrSpiral.h"
#include <cmath>

Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Spiral), curv_start(curv_start), curv_end(curv_end)
{
    this->c_dot = (curv_end - curv_start) / length;
}

Point2D Spiral::get_point(double s, double t)
{
    double s0_spiral = curv_start / c_dot;
    double x0_spiral, y0_spiral, a0_spiral;
    odrSpiral(s0_spiral, c_dot, &x0_spiral, &y0_spiral, &a0_spiral);
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - s0 + s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral);

    double tx = t * std::cos(as_spiral + M_PI / 2);
    double ty = t * std::sin(as_spiral + M_PI / 2);
    double hdg = hdg0 - a0_spiral;

    double xt = (std::cos(hdg) * (xs_spiral - x0_spiral + tx)) - (std::sin(hdg) * (ys_spiral - y0_spiral + ty)) + x0;
    double yt = (std::sin(hdg) * (xs_spiral - x0_spiral + tx)) + (std::cos(hdg) * (ys_spiral - y0_spiral + ty)) + y0;
    return Point2D{xt, yt};
}