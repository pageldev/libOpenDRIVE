#include "libodr/Geometries/Line.h"
#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Math.hpp"

#include <cmath>

namespace odr
{

Line::Line(double s, double x, double y, double hdg, double length) : RoadGeometry(s, x, y, hdg, length) {}

std::unique_ptr<RoadGeometry> Line::clone() const
{
    return std::make_unique<Line>(*this);
}

Vec2D Line::get_xy(double s) const
{
    const double x_s = (std::cos(hdg) * (s - this->s)) + x;
    const double y_s = (std::sin(hdg) * (s - this->s)) + y;
    return Vec2D{x_s, y_s};
}

Vec2D Line::derivative([[maybe_unused]] double s) const
{
    return {{std::cos(hdg), std::sin(hdg)}};
}

} // namespace odr
