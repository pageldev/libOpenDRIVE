#include "libodr/Geometries/Spiral.h"
#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Geometries/Spiral/odrSpiral.h"
#include "libodr/Math.hpp"
#include "libodr/Utils.hpp"

#include <cmath>

namespace odr
{

Spiral::Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end) :
    RoadGeometry(s, x, y, hdg, length), curv_start(curv_start), curv_end(curv_end)
{
    require_or_throw(!std::isnan(curv_start), "curvStart must not be NaN");
    require_or_throw(!std::isnan(curv_end), "curvEnd must not be NaN");

    this->c_dot = (curv_end - curv_start) / length;
    this->spiral_s_start = curv_start / c_dot;
    this->spiral_s_end = curv_end / c_dot;
    spiral_s_origin = curv_start / c_dot;
    odrSpiral(spiral_s_origin, c_dot, &spiral_x_origin, &spiral_y_origin, &spiral_hdg_origin);
}

std::unique_ptr<RoadGeometry> Spiral::clone() const
{
    return std::make_unique<Spiral>(*this);
}

Vec2D Spiral::get_xy(double s) const
{
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - this->s + spiral_s_origin, c_dot, &xs_spiral, &ys_spiral, &as_spiral);

    const double hdg_rotation = hdg - spiral_hdg_origin;
    const double xt = (std::cos(hdg_rotation) * (xs_spiral - spiral_x_origin)) - (std::sin(hdg_rotation) * (ys_spiral - spiral_y_origin)) + x;
    const double yt = (std::sin(hdg_rotation) * (xs_spiral - spiral_x_origin)) + (std::cos(hdg_rotation) * (ys_spiral - spiral_y_origin)) + y;
    return Vec2D{xt, yt};
}

Vec2D Spiral::derivative(double s) const
{
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - this->s + spiral_s_origin, c_dot, &xs_spiral, &ys_spiral, &as_spiral);
    const double hdg_s = as_spiral + hdg - spiral_hdg_origin;
    const double dx = std::cos(hdg_s);
    const double dy = std::sin(hdg_s);
    return {{dx, dy}};
}

} // namespace odr
