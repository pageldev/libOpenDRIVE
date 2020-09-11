#include "Spiral.h"
#include "Spiral/odrSpiral.h"
#include "Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace odr
{

Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end)
    : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Spiral), curv_start(curv_start), curv_end(curv_end)
{
    this->c_dot = (curv_end - curv_start) / length;
    this->s_start = curv_start / c_dot;
    this->s_end = curv_end / c_dot;
}

Vec2D Spiral::get_point(double s, double t) const
{
    double s0_spiral = curv_start / c_dot;
    double x0_spiral, y0_spiral, a0_spiral;
    odrSpiral(s0_spiral, c_dot, &x0_spiral, &y0_spiral, &a0_spiral);
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - s0 + s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral);

    double tx = t * std::cos(as_spiral + M_PI / 2);
    double ty = t * std::sin(as_spiral + M_PI / 2);
    double hdg = hdg0 - a0_spiral;

    double xt = (std::cos(hdg) * (xs_spiral - x0_spiral + tx)) - (std::sin(hdg) * (ys_spiral - y0_spiral + ty)) + x0;
    double yt = (std::sin(hdg) * (xs_spiral - x0_spiral + tx)) + (std::cos(hdg) * (ys_spiral - y0_spiral + ty)) + y0;
    return Vec2D{xt, yt};
}

Box2D Spiral::get_bbox() const
{
    const std::function<double(int)> f_s_x_extrema_1 = [&](const int n) { return ((std::sqrt(curv_start * curv_start + c_dot * (-2 * hdg0 - 2 * M_PI * n + M_PI)) - curv_start) / c_dot) + s0; };
    const std::function<double(int)> f_s_x_extrema_2 = [&](const int n) { return (-(std::sqrt(curv_start * curv_start + c_dot * (-2 * hdg0 - 2 * M_PI * n + M_PI)) + curv_start) / c_dot) + s0; };
    const std::function<double(int)> f_s_y_extrema_1 = [&](const int n) { return (-(std::sqrt(curv_start * curv_start + 2 * c_dot * (-hdg0 - M_PI * n)) + curv_start) / c_dot) + s0; };
    const std::function<double(int)> f_s_y_extrema_2 = [&](const int n) { return ((std::sqrt(curv_start * curv_start + 2 * c_dot * (-hdg0 - M_PI * n)) - curv_start) / c_dot) + s0; };

    std::array<const std::function<double(int)> *, 2> f_s_extremas;

    std::vector<double> s_extremas{s0, s0 + length};
    for (bool is_x : {true, false})
    {
        const double n_end = is_x ? (2 * curv_start * length + c_dot * length * length - M_PI) / (2 * M_PI) : (2 * curv_start * length + c_dot * length * length) / (2 * M_PI);
        if (is_x)
            f_s_extremas = {&f_s_x_extrema_1, &f_s_x_extrema_2};
        else
            f_s_extremas = {&f_s_y_extrema_1, &f_s_y_extrema_2};
        for (const std::function<double(int)> *f_s_extrema : f_s_extremas)
        {
            for (int n = std::floor(-std::abs(n_end)) - 1; n < std::ceil(std::abs(n_end)) + 1; n++)
            {
                if (std::isnan((*f_s_extrema)(n)) || (*f_s_extrema)(n) < s0 || (*f_s_extrema)(n) > (s0 + length))
                    continue;
                s_extremas.push_back((*f_s_extrema)(n));
            }
        }
    }

    return get_bbox_for_s_values<double>(s_extremas, std::bind(&Spiral::get_point, this, std::placeholders::_1, std::placeholders::_2));
}

double Spiral::project(double x, double y) const
{
    return 0;
}

Vec2D Spiral::get_grad(double s) const
{
    return {{0, 0}};
}

} // namespace odr