#pragma once
#include "Math.hpp"

#include <memory>
#include <set>

namespace odr
{

enum class GeometryType
{
    Line,
    Spiral,
    Arc,
    ParamPoly3
};

struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, GeometryType type);
    virtual ~RoadGeometry() = default;

    virtual std::unique_ptr<RoadGeometry> clone() const = 0;

    virtual Vec2D get_xy(double s) const = 0;
    virtual Vec2D get_grad(double s) const = 0;

    virtual std::set<double> approximate_linear(double eps) const = 0;

    double       s0 = 0;
    double       x0 = 0;
    double       y0 = 0;
    double       hdg0 = 0;
    double       length = 0;
    GeometryType type;
};

} // namespace odr
