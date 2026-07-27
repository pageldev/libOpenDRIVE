#pragma once
#include "RoadGeometry.h"
#include "libodr/Math.hpp"

#include <memory>
#include <set>

namespace odr
{

struct Arc : public RoadGeometry
{
    Arc(double s, double x, double y, double hdg, double length, double curvature);

    std::unique_ptr<RoadGeometry> clone() const override;

    Vec2D get_xy(double s) const override;
    Vec2D derivative(double s) const override;

    std::set<double> approximate_linear(double eps) const override;

    double curvature = 0;
};

} // namespace odr
