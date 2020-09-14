#include "Line.h"
#include "Utils.hpp"

#include <cmath>

namespace odr
{

Line::Line(double s0, double x0, double y0, double hdg0, double length)
    : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Line)
{
}

void Line::update()
{
    this->bounding_box = get_bbox_for_s_values<double>({s0, s0 + length}, std::bind(&Line::get_point, this, std::placeholders::_1, std::placeholders::_2));
}

Vec2D Line::get_point(double s, double t) const
{
    const double xt = (std::cos(hdg0) * (s - s0)) - (std::sin(hdg0) * t) + x0;
    const double yt = (std::sin(hdg0) * (s - s0)) + (std::cos(hdg0) * t) + y0;
    return Vec2D{xt, yt};
}

double Line::project(double x, double y) const
{
    const Vec2D  pt_l0{{x - x0, y - y0}};
    const Vec2D  l1l0{{std::cos(hdg0) * length, std::sin(hdg0) * length}};
    const double dot = pt_l0[0] * l1l0[0] + pt_l0[1] * l1l0[1];
    const double s = std::max(std::min(dot / (length * length), 1.0), 0.0);
    return s;
}

Vec2D Line::get_grad(double s) const
{
    return {{std::cos(hdg0), std::sin(hdg0)}};
}

} // namespace odr