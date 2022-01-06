#define _USE_MATH_DEFINES
#include <cmath>

#include "Arc.h"
#include "Math.hpp"
#include "RoadGeometry.h"
#include "Utils.hpp"

#include <array>
#include <functional>
#include <initializer_list>
#include <vector>

namespace odr
{
Arc::Arc(double s0, double x0, double y0, double hdg0, double length, double curvature) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Arc), curvature(curvature)
{
}

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
    std::set<double> s_vals;
    for (double s = s0; s < (s0 + length); s += (10 * eps))
        s_vals.insert(s);
    s_vals.insert(s0 + length);

    return s_vals;
}

} // namespace odr