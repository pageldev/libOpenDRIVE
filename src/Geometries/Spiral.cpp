#include "libodr/Geometries/Spiral.h"
#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Geometries/Spiral/odrSpiral.h"
#include "libodr/Math.hpp"
#include "libodr/Utils.hpp"

#include <cmath>

namespace odr
{

Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end) :
    RoadGeometry(s0, x0, y0, hdg0, length), curv_start(curv_start), curv_end(curv_end)
{
    require_or_throw(!std::isnan(curv_start), "curvStart is NaN");
    require_or_throw(!std::isnan(curv_end), "curvEnd is NaN");

    this->c_dot = (curv_end - curv_start) / length;
    this->s_start = curv_start / c_dot;
    this->s_end = curv_end / c_dot;
    s0_spiral = curv_start / c_dot;
    odrSpiral(s0_spiral, c_dot, &x0_spiral, &y0_spiral, &a0_spiral);
}

std::unique_ptr<RoadGeometry> Spiral::clone() const
{
    return std::make_unique<Spiral>(*this);
}

Vec2D Spiral::get_xy(double s) const
{
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - s0 + s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral);

    const double hdg = hdg0 - a0_spiral;
    const double xt = (std::cos(hdg) * (xs_spiral - x0_spiral)) - (std::sin(hdg) * (ys_spiral - y0_spiral)) + x0;
    const double yt = (std::sin(hdg) * (xs_spiral - x0_spiral)) + (std::cos(hdg) * (ys_spiral - y0_spiral)) + y0;
    return Vec2D{xt, yt};
}

Vec2D Spiral::derivative(double s) const
{
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - s0 + s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral);
    const double hdg = as_spiral + hdg0 - a0_spiral;
    const double dx = std::cos(hdg);
    const double dy = std::sin(hdg);
    return {{dx, dy}};
}

std::set<double> Spiral::approximate_linear(double eps) const
{
    require_or_throw(std::isfinite(eps) && eps > 0, "eps must be finite and > 0");

    // TODO: properly implement
    std::set<double> s_vals;
    for (double s = s0; s < (s0 + length); s += (10 * eps))
        s_vals.insert(s);
    s_vals.insert(s0 + length);

    return s_vals;
}

} // namespace odr
