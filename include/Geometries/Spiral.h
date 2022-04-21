#pragma once
#include "Math.hpp"
#include "RoadGeometry.h"

#include <memory>
#include <set>

namespace odr
{

struct Spiral : public RoadGeometry
{
    Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end);

    std::unique_ptr<RoadGeometry> clone() const override;

    Vec2D get_xy(double s) const override;
    Vec2D get_grad(double s) const override;

    std::set<double> approximate_linear(double eps) const override;

    double curv_start = 0;
    double curv_end = 0;
    double s_start = 0;
    double s_end = 0;
    double c_dot = 0;

private:
    double s0_spiral = 0;
    double x0_spiral = 0;
    double y0_spiral = 0;
    double a0_spiral = 0;
};

} // namespace odr