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
}

Vec2D ParamPoly3::get_xy(double s) const
{
    double p = (s - s0);
    if (this->pRange_normalized)
        p /= length;

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

    double p = (s - s0);
    if (this->pRange_normalized)
        p /= length;

    const double dx = h1 * (bU + 2 * cU * p + 3 * dU * p * p) - h2 * (bV + 2 * cV * p + 3 * dV * p * p);
    const double dy = h2 * (bU + 2 * cU * p + 3 * dU * p * p) + h1 * (bV + 2 * cV * p + 3 * dV * p * p);

    return {{dx, dy}};
}

} // namespace odr