#pragma once

#include "Math.hpp"
#include "RoadGeometry.h"

#include <memory>

namespace odr
{
class Road;

struct Spiral : public RoadGeometry
{
    Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end);

    Vec2D get_xy(double s) const override;
    Vec2D get_grad(double s) const override;

    std::set<double> approximate_linear(double eps) const override;

    double curv_start = 0, curv_end = 0;
    double s_start = 0, s_end = 0;
    double c_dot = 0;

private:
    double s0_spiral = 0, x0_spiral = 0, y0_spiral = 0, a0_spiral = 0;
};

} // namespace odr