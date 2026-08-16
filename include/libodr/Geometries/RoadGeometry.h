#pragma once
#include "libodr/Math.hpp"

#include <memory>
#include <set>

namespace odr
{

struct RoadGeometry
{
    RoadGeometry(double s, double x, double y, double hdg, double length);
    virtual ~RoadGeometry() = default;

    virtual std::unique_ptr<RoadGeometry> clone() const = 0;

    virtual Vec2D get_xy(double s) const = 0;
    virtual Vec2D derivative(double s) const = 0;

    double s = 0;
    double x = 0;
    double y = 0;
    double hdg = 0;
    double length = 0;
};

} // namespace odr
