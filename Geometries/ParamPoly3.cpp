#include "ParamPoly3.h"

#include <cmath>
#include <vector>

ParamPoly3::ParamPoly3(double s0, double x0, double y0, double hdg0, double length, double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::ParamPoly3), aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV)
{
}

Point2D ParamPoly3::get_point(double s, double t) const
{
    double p = (s - s0) / length;
    double xs = aU + bU * p + cU * p * p + dU * p * p * p;
    double ys = aV + bV * p + cV * p * p + dV * p * p * p;
    double xs_dp = bU + 2 * cU * p + 3 * dU * p * p;
    double ys_dp = bV + 2 * cV * p + 3 * dV * p * p;
    double x_offs = xs - ((t * ys_dp) / std::sqrt(xs_dp * xs_dp + ys_dp * ys_dp));
    double y_offs = ys + ((t * xs_dp) / std::sqrt(xs_dp * xs_dp + ys_dp * ys_dp));
    double xt = (std::cos(hdg0) * x_offs) - (std::sin(hdg0) * y_offs) + x0;
    double yt = (std::sin(hdg0) * x_offs) + (std::cos(hdg0) * y_offs) + y0;
    return Point2D{xt, yt};
}

Box2D ParamPoly3::get_bbox() const
{
    const double p_x_extrema_1 = ((-1 / 2) * std::sqrt(std::pow((2 * cV * std::sin(hdg0) - 2 * cU * std::cos(hdg0)), 2) - 4.0 * (bV * std::sin(hdg0) - bU * std::cos(hdg0)) * (3 * dV * std::sin(hdg0) - 3 * dU * std::cos(hdg0))) - cV * std::sin(hdg0) + cU * std::cos(hdg0)) / (3 * (dV * std::sin(hdg0) - dU * std::cos(hdg0)));
    const double p_x_extrema_2 = ((1 / 2) * std::sqrt(std::pow((2 * cV * std::sin(hdg0) - 2 * cU * std::cos(hdg0)), 2) - 4 * (bV * std::sin(hdg0) - bU * std::cos(hdg0)) * (3 * dV * std::sin(hdg0) - 3 * dU * std::cos(hdg0))) - cV * std::sin(hdg0) + cU * std::cos(hdg0)) / (3 * (dV * std::sin(hdg0) - dU * std::cos(hdg0)));
    const double p_y_extrema_1 = ((-1 / 2) * std::sqrt(std::pow((2 * cU * std::sin(hdg0) + 2 * cV * std::cos(hdg0)), 2) - 4 * (bU * std::sin(hdg0) + bV * std::cos(hdg0)) * (3 * dU * std::sin(hdg0) + 3 * dV * std::cos(hdg0))) + cU * (-std::sin(hdg0)) - cV * std::cos(hdg0)) / (3 * (dU * std::sin(hdg0) + dV * std::cos(hdg0)));
    const double p_y_extrema_2 = ((1 / 2) * std::sqrt(std::pow((2 * cU * std::sin(hdg0) + 2 * cV * std::cos(hdg0)), 2) - 4 * (bU * std::sin(hdg0) + bV * std::cos(hdg0)) * (3 * dU * std::sin(hdg0) + 3 * dV * std::cos(hdg0))) + cU * (-std::sin(hdg0)) - cV * std::cos(hdg0)) / (3 * (dU * std::sin(hdg0) + dV * std::cos(hdg0)));

    std::vector<double> s_extremas{s0, s0 + length};
    for (const double p_extrema : {p_x_extrema_1, p_x_extrema_2, p_y_extrema_1, p_y_extrema_2})
    {
        const double s_extrema = p_extrema * length + s0;
        if (std::isnan(s_extrema) || s_extrema < s0 || s_extrema > (s0 + length))
            continue;
        s_extremas.push_back(s_extrema);
    }

    Box2D bbox;
    bbox.min = this->get_point(s_extremas.at(0), 0.0);
    bbox.max = this->get_point(s_extremas.at(0), 0.0);
    for (const double s : s_extremas)
    {
        Point2D pt_2d = this->get_point(s, 0.0);
        bbox.min.x = std::min(bbox.min.x, pt_2d.x);
        bbox.min.y = std::min(bbox.min.y, pt_2d.y);
        bbox.max.x = std::max(bbox.max.x, pt_2d.x);
        bbox.max.y = std::max(bbox.max.y, pt_2d.y);
    }

    return bbox;
}