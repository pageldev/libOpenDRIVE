#include "Line.h"
#include <cmath>

Line::Line(double s0, double x0, double y0, double hdg0, double length)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Line)
{
}

Point2D Line::get_point(double s, double t) const
{
    double xt = (std::cos(hdg0) * (s - s0)) - (std::sin(hdg0) * t) + x0;
    double yt = (std::sin(hdg0) * (s - s0)) + (std::cos(hdg0) * t) + y0;
    return Point2D{xt, yt};
}

Box2D Line::get_bbox() const
{
    Point2D pt0 = this->get_point(s0, 0);
    Point2D pt1 = this->get_point(s0 + length, 0);

    Box2D bbox;
    bbox.min.x = std::min(pt0.x, pt1.x);
    bbox.min.y = std::min(pt0.y, pt1.y);
    bbox.max.x = std::max(pt0.x, pt1.x);
    bbox.max.y = std::max(pt0.y, pt1.y);

    return bbox;
}