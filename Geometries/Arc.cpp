#include "Arc.h"
#include "Utils.hpp"

#include <cmath>
#include <functional>
#include <vector>

namespace odr
{

Arc::Arc(double s0, double x0, double y0, double hdg0, double length, double curvature)
    : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Arc), curvature(curvature)
{
}

void Arc::update()
{
    const int n_border = std::floor(std::abs(hdg0 / M_PI)) + 2;

    std::vector<double> s_extremas{s0, s0 + length};
    for (bool is_x : {true, false})
    {
        for (int n = -n_border; n < n_border; n++)
        {
            const double s_extrema = is_x ? -((-2 * curvature * s0 + 2 * hdg0 - 2 * M_PI * n + M_PI) / (2 * curvature)) : (curvature * s0 - hdg0 + M_PI * n) / (curvature);
            if (std::isnan(s_extrema) || s_extrema < s0 || s_extrema > (s0 + length))
                continue;
            s_extremas.push_back(s_extrema);
        }
    }

    this->bounding_box = get_bbox_for_s_values<double>(s_extremas, std::bind(&Arc::get_point, this, std::placeholders::_1, std::placeholders::_2));
}

Vec2D Arc::get_point(double s, double t) const
{
    const double angle_at_s = (s - s0) * curvature - M_PI / 2;
    const double r = 1 / curvature;
    const double xs = (r - t) * std::cos(angle_at_s);
    const double ys = (r - t) * std::sin(angle_at_s) + r;
    const double xt = (std::cos(hdg0) * xs) - (std::sin(hdg0) * ys) + x0;
    const double yt = (std::sin(hdg0) * xs) + (std::cos(hdg0) * ys) + y0;
    return Vec2D{xt, yt};
}

double Arc::project(double x, double y) const
{
    return 0;
}

Vec2D Arc::get_grad(double s) const
{
    return {{0, 0}};
}

} // namespace odr