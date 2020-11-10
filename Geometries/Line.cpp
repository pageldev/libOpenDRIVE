#include "Line.h"
#include "Utils.hpp"

#include <cmath>

namespace odr
{

Line::Line(double s0, double x0, double y0, double hdg0, double length, std::shared_ptr<Road> road)
    : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Line, road)
{
    this->update();
}

void Line::update()
{
    this->bounding_box = get_bbox_for_s_values<double>({s0, s0 + length}, std::bind(&Line::get_xy, this, std::placeholders::_1));
}

Vec2D Line::get_xy(double s) const
{
    const double x = (std::cos(hdg0) * (s - s0)) + x0;
    const double y = (std::sin(hdg0) * (s - s0)) + y0;
    return Vec2D{x, y};
}

Vec2D Line::get_grad(double s) const
{
    return {{std::cos(hdg0), std::sin(hdg0)}};
}

} // namespace odr