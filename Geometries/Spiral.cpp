#include "Spiral.h"
#include "Spiral/odrSpiral.h"
#include "Utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Spiral), curv_start(curv_start), curv_end(curv_end)
{
    this->c_dot = (curv_end - curv_start) / length;
    this->s_start = curv_start / c_dot;
    this->s_end = curv_end / c_dot;
}

Point2D Spiral::get_point(double s, double t) const
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
    return Point2D{xt, yt};
}

Box2D Spiral::get_bbox()
{
    double s_min = std::min(s_start, s_end);
    double s_max = std::max(s_start, s_end);

    double n_pi_s_min = sign(s_min) * ((s_min * s_min * std::abs(c_dot)) / (2 * M_PI));
    double next_n_pi_half = n_pi_s_min - std::fmod(n_pi_s_min, 0.5);
    double n_pi_s_max = sign(s_max) * ((s_max * s_max * std::abs(c_dot)) / (2 * M_PI));

    std::vector<double> s_vals {s_min, s_max};
    for (double n_pi = next_n_pi_half; n_pi < n_pi_s_max; n_pi += 0.5)
    {
        if(n_pi == 0)
            continue;
        s_vals.push_back(sign(n_pi) * std::sqrt((2 * std::abs(n_pi) * M_PI) / std::abs(c_dot)));
    }
}