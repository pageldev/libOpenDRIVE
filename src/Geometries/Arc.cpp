#include "Geometries/Arc.h"
#include "Geometries/RoadGeometry.h"

#include <cmath>

namespace odr
{
Arc::Arc(double s0, double x0, double y0, double hdg0, double length, double curvature) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType_Arc), curvature(curvature)
{
}

std::unique_ptr<RoadGeometry> Arc::clone() const { return std::make_unique<Arc>(*this); }

Vec2D Arc::get_xy(double s) const
{
    const double angle_at_s = (s - s0) * curvature - M_PI / 2;
    const double r = 1 / curvature;
    const double xs = r * (std::cos(hdg0 + angle_at_s) - std::sin(hdg0)) + x0;
    const double ys = r * (std::sin(hdg0 + angle_at_s) + std::cos(hdg0)) + y0;
    return Vec2D{xs, ys};
}

Vec2D Arc::get_grad(double s) const
{
    const double dx = std::sin((M_PI / 2) - curvature * (s - s0) - hdg0);
    const double dy = std::cos((M_PI / 2) - curvature * (s - s0) - hdg0);
    return {{dx, dy}};
}

std::set<double> Arc::approximate_linear(double eps) const
{
    // TODO: properly implement
    const double     s_step = 0.01 / std::abs(this->curvature); // sample at approx. every 1°
    std::set<double> s_vals;
    for (double s = s0; s < (s0 + length); s += s_step)
        s_vals.insert(s);
    s_vals.insert(s0 + length);

    return s_vals;
}

} // namespace odr
