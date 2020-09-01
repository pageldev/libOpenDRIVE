#include "Line.h"
#include "Utils.h"

#include <cmath>

namespace odr
{

Line::Line(double s0, double x0, double y0, double hdg0, double length)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Line)
{
}

Point2D<double> Line::get_point(double s, double t) const
{
    double xt = (std::cos(hdg0) * (s - s0)) - (std::sin(hdg0) * t) + x0;
    double yt = (std::sin(hdg0) * (s - s0)) + (std::cos(hdg0) * t) + y0;
    return Point2D<double>{xt, yt};
}

Box2D<double> Line::get_bbox() const
{
    return get_bbox_for_s_values<double>({s0, s0 + length}, std::bind(&Line::get_point, this, std::placeholders::_1, std::placeholders::_2));
}

} // namespace odr