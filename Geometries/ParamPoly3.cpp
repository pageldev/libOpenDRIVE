#include "ParamPoly3.h"
#include "Utils.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <vector>

namespace odr
{

ParamPoly3::ParamPoly3(double s0, double x0, double y0, double hdg0, double length, double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV)
    : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::ParamPoly3), aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV)
{
}

Vec2D ParamPoly3::get_point(double s, double t) const
{
    const double p = (s - s0) / length;
    const double xs = aU + bU * p + cU * p * p + dU * p * p * p;
    const double ys = aV + bV * p + cV * p * p + dV * p * p * p;
    const double xs_dp = bU + 2 * cU * p + 3 * dU * p * p;
    const double ys_dp = bV + 2 * cV * p + 3 * dV * p * p;
    const double x_offs = xs - ((t * ys_dp) / std::sqrt(xs_dp * xs_dp + ys_dp * ys_dp));
    const double y_offs = ys + ((t * xs_dp) / std::sqrt(xs_dp * xs_dp + ys_dp * ys_dp));
    const double xt = (std::cos(hdg0) * x_offs) - (std::sin(hdg0) * y_offs) + x0;
    const double yt = (std::sin(hdg0) * x_offs) + (std::cos(hdg0) * y_offs) + y0;

    return Vec2D{xt, yt};
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

    return get_bbox_for_s_values<double>(s_extremas, std::bind(&ParamPoly3::get_point, this, std::placeholders::_1, std::placeholders::_2));
}

double ParamPoly3::project(double x, double y) const
{
    const double h1 = std::cos(hdg0);
    const double h2 = std::sin(hdg0);

    /* derivative of point to poly distance function f_dist'[p] = a5*p^5 + a4*p^4 + a3*p^3 + a2*p^2 + a1*p + a0 */
    const double a0 = 2 * aU * bU * h1 * h1 + 2 * aV * bV * h1 * h1 + 2 * aU * bU * h2 * h2 + 2 * aV * bV * h2 * h2 + 2 * bU * h1 * x0 - 2 * bV * h2 * x0 - 2 * bU * h1 * x + 2 * bV * h2 * x + 2 * bV * h1 * y0 + 2 * bU * h2 * y0 - 2 * bV * h1 * y - 2 * bU * h2 * y;
    const double a1 = 2 * bU * bU * h1 * h1 + 2 * bV * bV * h1 * h1 + 4 * aU * cU * h1 * h1 + 4 * aV * cV * h1 * h1 + 2 * bU * bU * h2 * h2 + 2 * bV * bV * h2 * h2 + 4 * aU * cU * h2 * h2 + 4 * aV * cV * h2 * h2 + 4 * cU * h1 * x0 - 4 * cV * h2 * x0 - 4 * cU * h1 * x + 4 * cV * h2 * x + 4 * cV * h1 * y0 + 4 * cU * h2 * y0 - 4 * cV * h1 * y - 4 * cU * h2 * y;
    const double a2 = 6 * bU * cU * h1 * h1 + 6 * bV * cV * h1 * h1 + 6 * aU * dU * h1 * h1 + 6 * aV * dV * h1 * h1 + 6 * bU * cU * h2 * h2 + 6 * bV * cV * h2 * h2 + 6 * aU * dU * h2 * h2 + 6 * aV * dV * h2 * h2 + 6 * dU * h1 * x0 - 6 * dV * h2 * x0 - 6 * dU * h1 * x + 6 * dV * h2 * x + 6 * dV * h1 * y0 + 6 * dU * h2 * y0 - 6 * dV * h1 * y - 6 * dU * h2 * y;
    const double a3 = 4 * cU * cU * h1 * h1 + 4 * cV * cV * h1 * h1 + 8 * bU * dU * h1 * h1 + 8 * bV * dV * h1 * h1 + 4 * cU * cU * h2 * h2 + 4 * cV * cV * h2 * h2 + 8 * bU * dU * h2 * h2 + 8 * bV * dV * h2 * h2;
    const double a4 = 10 * cU * dU * h1 * h1 + 10 * cV * dV * h1 * h1 + 10 * cU * dU * h2 * h2 + 10 * cV * dV * h2 * h2;
    const double a5 = 6 * dU * dU * h1 * h1 + 6 * dV * dV * h1 * h1 + 6 * dU * dU * h2 * h2 + 6 * dV * dV * h2 * h2;

    Eigen::Matrix<double, 5, 5> CompanionMat;
    CompanionMat << 0, 0, 0, 0, (-a0 / a5),
        1, 0, 0, 0, (-a1 / a5),
        0, 1, 0, 0, (-a2 / a5),
        0, 0, 1, 0, (-a3 / a5),
        0, 0, 0, 1, (-a4 / a5);
    Eigen::EigenSolver<Eigen::Matrix<double, 5, 5>> solver(CompanionMat);

    /* eigenvalues of companion matrix are roots of polynomial */
    auto   eigenvals = solver.eigenvalues();
    double min_dist_sqr = std::numeric_limits<double>::max();
    double min_p = 0.0;
    for (size_t idx = 0; idx < 5; idx++)
    {
        const double p = eigenvals[idx].real();
        if ((p < 0.0) || (p > 1.0))
            continue;
        const double dist_x = h1 * (aU + bU * p + cU * p * p + dU * p * p * p) - h2 * (aV + bV * p + cV * p * p + dV * p * p * p) + x0 - x;
        const double dist_y = h2 * (aU + bU * p + cU * p * p + dU * p * p * p) + h1 * (aV + bV * p + cV * p * p + dV * p * p * p) + y0 - y;
        const double dist_sqr = std::pow(dist_x, 2) + std::pow(dist_y, 2);
        if (dist_sqr < min_dist_sqr)
        {
            min_dist_sqr = dist_sqr;
            min_p = p;
        }
    }

    return min_p * length + s0;
}

Vec2D ParamPoly3::get_grad(double s) const
{
    const double h1 = std::cos(hdg0);
    const double h2 = std::sin(hdg0);
    const double p = (s - s0) / length;

    const double f_dot_x = h1 * (bU + 2 * cU * p + 3 * dU * p * p) - h2 * (bV + 2 * cV * p + 3 * dV * p * p);
    const double f_dot_y = h2 * (bU + 2 * cU * p + 3 * dU * p * p) + h1 * (bV + 2 * cV * p + 3 * dV * p * p);

    return {{f_dot_x, f_dot_y}};
}

} // namespace odr