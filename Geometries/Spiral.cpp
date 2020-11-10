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

Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end, std::shared_ptr<Road> road)
    : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::Spiral, road), curv_start(curv_start), curv_end(curv_end)
{
    this->update();
}

void Spiral::update()
{
    this->c_dot = (curv_end - curv_start) / length;
    this->s_start = curv_start / c_dot;
    this->s_end = curv_end / c_dot;

    s0_spiral = curv_start / c_dot;
    odrSpiral(s0_spiral, c_dot, &x0_spiral, &y0_spiral, &a0_spiral);

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

    this->bounding_box = get_bbox_for_s_values<double>(s_extremas, std::bind(&Spiral::get_xy, this, std::placeholders::_1));
}

Vec2D Spiral::get_xy(double s) const
{
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - s0 + s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral);

    double hdg = hdg0 - a0_spiral;
    double xt = (std::cos(hdg) * (xs_spiral - x0_spiral)) - (std::sin(hdg) * (ys_spiral - y0_spiral)) + x0;
    double yt = (std::sin(hdg) * (xs_spiral - x0_spiral)) + (std::cos(hdg) * (ys_spiral - y0_spiral)) + y0;
    return Vec2D{xt, yt};
}

Vec2D Spiral::get_grad(double s) const
{
    const double h1 = std::cos(hdg0);
    const double h2 = std::sin(hdg0);
    const double a = std::sqrt(M_PI / std::abs(c_dot));
    const double dx = h1 * std::cos((M_PI * (s - s0 + s0_spiral) * (s - s0 + s0_spiral)) / (2 * a * a)) - h2 * std::sin((M_PI * (s - s0 + s0_spiral) * (s - s0 + s0_spiral)) / (2 * a * a));
    const double dy = h2 * std::cos((M_PI * (s - s0 + s0_spiral) * (s - s0 + s0_spiral)) / (2 * a * a)) + h1 * std::sin((M_PI * (s - s0 + s0_spiral) * (s - s0 + s0_spiral)) / (2 * a * a));

    return {{dx, dy}};
}

} // namespace odr