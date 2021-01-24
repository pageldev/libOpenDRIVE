#include "ParamPoly3.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <vector>

namespace odr
{
ParamPoly3::ParamPoly3(double s0,
                       double x0,
                       double y0,
                       double hdg0,
                       double length,
                       double aU,
                       double bU,
                       double cU,
                       double dU,
                       double aV,
                       double bV,
                       double cV,
                       double dV,
                       bool   pRange_normalized) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::ParamPoly3),
    aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV), pRange_normalized(pRange_normalized)
{
    if (!pRange_normalized)
    {
        this->bU = bU * length;
        this->bV = bV * length;
        this->cU = cU * length * length;
        this->cV = cV * length * length;
        this->dU = dU * length * length * length;
        this->dV = dV * length * length * length;
    }
}

Vec2D ParamPoly3::get_xy(double s) const
{
    const double p = (s - s0) / length;
    const double xs = aU + bU * p + cU * p * p + dU * p * p * p;
    const double ys = aV + bV * p + cV * p * p + dV * p * p * p;
    const double xt = (std::cos(hdg0) * xs) - (std::sin(hdg0) * ys) + x0;
    const double yt = (std::sin(hdg0) * xs) + (std::cos(hdg0) * ys) + y0;

    return Vec2D{xt, yt};
}

Vec2D ParamPoly3::get_grad(double s) const
{
    const double h1 = std::cos(hdg0);
    const double h2 = std::sin(hdg0);
    const double p = (s - s0) / length;
    const double dx = h1 * (bU + 2 * cU * p + 3 * dU * p * p) - h2 * (bV + 2 * cV * p + 3 * dV * p * p);
    const double dy = h2 * (bU + 2 * cU * p + 3 * dU * p * p) + h1 * (bV + 2 * cV * p + 3 * dV * p * p);

    return {{dx, dy}};
}

std::set<double> ParamPoly3::approximate_linear(double eps) const
{
    /* get control points */
    const Vec2D pA = {aU, aV};
    const Vec2D pB = {(bU / 3) + aU, (bV / 3) + aV};
    const Vec2D pC = {(cU / 3) + 2 * pB[0] - pA[0], (cU / 3) + 2 * pB[1] - pA[1]};
    const Vec2D pD = {dU + 3 * pC[0] - 3 * pB[0] + pA[0], dV + 3 * pC[1] - 3 * pB[1] + pA[1]};

    /* approximate cubic bezier by splitting into quadratic ones */
    const double seg_size = std::pow(eps / ((1.0 / 54.0) * std::sqrt(dU * dU + dV * dV)), (1.0 / 3.0));

    std::vector<std::array<double, 2>> seg_intervals;
    for (double p = 0; p < 1.0; p += seg_size)
        seg_intervals.push_back({p, std::min(p + seg_size, 1.0)});

    if (1.0 - (seg_intervals.back().at(1)) < 1e-6)
        seg_intervals.back().at(1) = 1.0;
    else
        seg_intervals.push_back({seg_intervals.back().at(1), 1.0});

    std::vector<double> p_vals;
    for (const std::array<double, 2>& seg_intrvl : seg_intervals)
    {
        /* get sub-cubic bezier for interval */
        const double& p0 = seg_intrvl.at(0);
        const double& p1 = seg_intrvl.at(1);

        const std::array<Vec2D, 4> c_pts_sub = subdivide_cubic_bezier<double, 2>(p0, p1, {pA, pB, pC, pD});

        /* approximate sub-cubic bezier by two quadratic ones */
        const Vec2D pB_quad_0 = {(1.0 - 0.75) * c_pts_sub[0][0] + 0.75 * c_pts_sub[1][0], (1.0 - 0.75) * c_pts_sub[0][1] + 0.75 * c_pts_sub[1][1]};
        const Vec2D pB_quad_1 = {(1.0 - 0.75) * c_pts_sub[3][0] + 0.75 * c_pts_sub[2][0], (1.0 - 0.75) * c_pts_sub[3][1] + 0.75 * c_pts_sub[2][1]};
        const Vec2D pM_quad = {(1.0 - 0.5) * pB_quad_0[0] + 0.5 * pB_quad_1[0], (1.0 - 0.5) * pB_quad_0[1] + 0.5 * pB_quad_1[1]};

        /* linear approximate the two quadratic bezier */
        for (const double& p_sub : approximate_linear_quad_bezier<double, 2>({c_pts_sub[0], pB_quad_0, pM_quad}, eps))
            p_vals.push_back(p0 + p_sub * (p1 - p0) * 0.5);
        p_vals.pop_back();
        for (const double& p_sub : approximate_linear_quad_bezier<double, 2>({pM_quad, pB_quad_1, c_pts_sub[3]}, eps))
            p_vals.push_back(p0 + (p1 - p0) * 0.5 + p_sub * (p1 - p0) * 0.5);
        p_vals.pop_back();
    }
    p_vals.push_back(1.0);

    std::set<double> s_vals;
    for (const double& p : p_vals)
        s_vals.insert(p * length + s0);

    return s_vals;
}

} // namespace odr