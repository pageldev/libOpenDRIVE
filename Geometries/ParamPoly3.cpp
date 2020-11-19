#include "ParamPoly3.h"
#include "Math.hpp"

#include <cmath>
#include <functional>
#include <vector>

namespace odr
{
ParamPoly3::ParamPoly3(double                s0,
                       double                x0,
                       double                y0,
                       double                hdg0,
                       double                length,
                       double                aU,
                       double                bU,
                       double                cU,
                       double                dU,
                       double                aV,
                       double                bV,
                       double                cV,
                       double                dV,
                       std::shared_ptr<Road> road) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType::ParamPoly3, road),
    aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV)
{
    this->update();
}

void ParamPoly3::update()
{
    const double p_x_extrema_1 =
        ((-1 / 2) * std::sqrt(std::pow((2 * cV * std::sin(hdg0) - 2 * cU * std::cos(hdg0)), 2) -
                              4.0 * (bV * std::sin(hdg0) - bU * std::cos(hdg0)) * (3 * dV * std::sin(hdg0) - 3 * dU * std::cos(hdg0))) -
         cV * std::sin(hdg0) + cU * std::cos(hdg0)) /
        (3 * (dV * std::sin(hdg0) - dU * std::cos(hdg0)));
    const double p_x_extrema_2 =
        ((1 / 2) * std::sqrt(std::pow((2 * cV * std::sin(hdg0) - 2 * cU * std::cos(hdg0)), 2) -
                             4 * (bV * std::sin(hdg0) - bU * std::cos(hdg0)) * (3 * dV * std::sin(hdg0) - 3 * dU * std::cos(hdg0))) -
         cV * std::sin(hdg0) + cU * std::cos(hdg0)) /
        (3 * (dV * std::sin(hdg0) - dU * std::cos(hdg0)));
    const double p_y_extrema_1 =
        ((-1 / 2) * std::sqrt(std::pow((2 * cU * std::sin(hdg0) + 2 * cV * std::cos(hdg0)), 2) -
                              4 * (bU * std::sin(hdg0) + bV * std::cos(hdg0)) * (3 * dU * std::sin(hdg0) + 3 * dV * std::cos(hdg0))) +
         cU * (-std::sin(hdg0)) - cV * std::cos(hdg0)) /
        (3 * (dU * std::sin(hdg0) + dV * std::cos(hdg0)));
    const double p_y_extrema_2 =
        ((1 / 2) * std::sqrt(std::pow((2 * cU * std::sin(hdg0) + 2 * cV * std::cos(hdg0)), 2) -
                             4 * (bU * std::sin(hdg0) + bV * std::cos(hdg0)) * (3 * dU * std::sin(hdg0) + 3 * dV * std::cos(hdg0))) +
         cU * (-std::sin(hdg0)) - cV * std::cos(hdg0)) /
        (3 * (dU * std::sin(hdg0) + dV * std::cos(hdg0)));

    std::vector<double> s_extremas{s0, s0 + length};
    for (const double p_extrema : {p_x_extrema_1, p_x_extrema_2, p_y_extrema_1, p_y_extrema_2})
    {
        const double s_extrema = p_extrema * length + s0;
        if (std::isnan(s_extrema) || s_extrema < s0 || s_extrema > (s0 + length))
            continue;
        s_extremas.push_back(s_extrema);
    }

    this->bounding_box = get_bbox_for_s_values<double>(s_extremas, std::bind(&ParamPoly3::get_xy, this, std::placeholders::_1));
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

} // namespace odr