#pragma once

#include "CubicBezier.hpp"
#include "RoadGeometry.h"

namespace odr
{
class Road;

struct ParamPoly3 : public RoadGeometry
{
    ParamPoly3(double s0,
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
               bool   pRange_normalized = true);

    Vec2D get_xy(double s) const override;
    Vec2D get_grad(double s) const override;

    std::set<double> approximate_linear(double eps) const override;

    double aU = 0, bU = 0, cU = 0, dU = 0, aV = 0, bV = 0, cV = 0, dV = 0;
    bool   pRange_normalized = true;

    CubicBezier2D cubic_bezier;
};

} // namespace odr