#pragma once
#include "Math.hpp"

#include <memory>
#include <set>

namespace odr
{

struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length);
    virtual ~RoadGeometry() = default;

    virtual std::unique_ptr<RoadGeometry> clone() const = 0;

    virtual Vec2D get_xy(double s) const = 0;
    virtual Vec2D derivative(double s) const = 0;

    virtual std::set<double> approximate_linear(double eps) const = 0;

    double s0 = 0;
    double x0 = 0;
    double y0 = 0;
    double hdg0 = 0;
    double length = 0;
};

} // namespace odr
