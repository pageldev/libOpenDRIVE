#pragma once

#include "Math.hpp"
#include "RoadGeometry.h"

#include <memory>

namespace odr
{
class Road;

struct Arc : public RoadGeometry
{
    Arc(double s0, double x0, double y0, double hdg0, double length, double curvature);

    Vec2D get_xy(double s) const override;
    Vec2D get_grad(double s) const override;

    std::set<double> approximate_linear(double eps) const override;

    double curvature = 0;
};

} // namespace odr