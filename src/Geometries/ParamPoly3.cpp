#include "libodr/Geometries/ParamPoly3.h"
#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Math.hpp"
#include "libodr/Utils.hpp"

#include <array>
#include <cmath>
#include <map>

namespace odr
{

ParamPoly3::ParamPoly3(double s,
                       double x,
                       double y,
                       double hdg,
                       double length,
                       double aU,
                       double bU,
                       double cU,
                       double dU,
                       double aV,
                       double bV,
                       double cV,
                       double dV,
                       PRange p_range) :
    RoadGeometry(s, x, y, hdg, length), aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV), p_range(p_range)
{
    require_or_throw(!std::isnan(aU), "aU must not be NaN");
    require_or_throw(!std::isnan(bU), "bU must not be NaN");
    require_or_throw(!std::isnan(cU), "cU must not be NaN");
    require_or_throw(!std::isnan(dU), "dU must not be NaN");
    require_or_throw(!std::isnan(aV), "aV must not be NaN");
    require_or_throw(!std::isnan(bV), "bV must not be NaN");
    require_or_throw(!std::isnan(cV), "cV must not be NaN");
    require_or_throw(!std::isnan(dV), "dV must not be NaN");

    if (p_range == PRange::ArcLength) // normalize
    {
        this->bU = bU * length;
        this->bV = bV * length;
        this->cU = cU * length * length;
        this->cV = cV * length * length;
        this->dU = dU * length * length * length;
        this->dV = dV * length * length * length;
    }

    const std::array<Vec2D, 4> coefficients = {{{this->aU, this->aV}, {this->bU, this->bV}, {this->cU, this->cV}, {this->dU, this->dV}}};
    this->cubic_bezier = CubicBezier2D(CubicBezier2D::get_control_points(coefficients));

    this->cubic_bezier.arclen_t[length] = 1.0;
    this->cubic_bezier.valid_length = length;
}

std::unique_ptr<RoadGeometry> ParamPoly3::clone() const
{
    return std::make_unique<ParamPoly3>(*this);
}

Vec2D ParamPoly3::get_xy(double s) const
{
    const double p = this->cubic_bezier.get_t(s - this->s);
    const Vec2D  pt = this->cubic_bezier.evaluate(p);

    const double x_t = (std::cos(hdg) * pt[0]) - (std::sin(hdg) * pt[1]) + x;
    const double y_t = (std::sin(hdg) * pt[0]) + (std::cos(hdg) * pt[1]) + y;

    return Vec2D{x_t, y_t};
}

Vec2D ParamPoly3::derivative(double s) const
{
    const double p = this->cubic_bezier.get_t(s - this->s);
    const Vec2D  dxy = this->cubic_bezier.derivative(p);

    const double h1 = std::cos(hdg);
    const double h2 = std::sin(hdg);
    const double dx = h1 * dxy[0] - h2 * dxy[1];
    const double dy = h2 * dxy[0] + h1 * dxy[1];

    return {{dx, dy}};
}

} // namespace odr
