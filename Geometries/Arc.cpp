#include "Arc.h"
#include "Math.hpp"
#include "RoadGeometry.h"
#include "Utils.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <vector>

namespace odr
{
Arc::Arc(double s0, double x0, double y0, double hdg0, double length, double curvature) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Arc), curvature(curvature)
{
    this->update();
}

void Arc::update()
{
    const int n_border = std::floor(std::abs(hdg0 / M_PI)) + 2;

    std::vector<double> s_extremas{s0, s0 + length};
    for (bool is_x : {true, false})
    {
        for (int n = -n_border; n < n_border; n++)
        {
            const double s_extrema =
                is_x ? -((-2 * curvature * s0 + 2 * hdg0 - 2 * M_PI * n + M_PI) / (2 * curvature)) : (curvature * s0 - hdg0 + M_PI * n) / (curvature);
            if (std::isnan(s_extrema) || s_extrema < s0 || s_extrema > (s0 + length))
                continue;
            s_extremas.push_back(s_extrema);
        }
    }

    this->bounding_box = get_bbox_for_s_values<double>(s_extremas, std::bind(&Arc::get_xy, this, std::placeholders::_1));
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

} // namespace odr