#pragma once

#include "Math.hpp"
#include "Utils.hpp"
#include "XmlNode.h"

#include <memory>
#include <set>

namespace odr
{
class Road;

enum class GeometryType
{
    Line,
    Spiral,
    Arc,
    ParamPoly3
};

struct RoadGeometry : public XmlNode
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, GeometryType type);
    virtual ~RoadGeometry() = default;

    virtual Vec2D get_xy(double s) const = 0;
    virtual Vec2D get_grad(double s) const = 0;

    virtual std::set<double> approximate_linear(double eps) const = 0;

    double s0 = 0;
    double x0 = 0;
    double y0 = 0;
    double hdg0 = 0;
    double length = 0;

    GeometryType type;
};

using ConstRoadGeometrySet = std::set<std::shared_ptr<const RoadGeometry>, SharedPtrCmp<const RoadGeometry, double, &RoadGeometry::s0>>;
using RoadGeometrySet = std::set<std::shared_ptr<RoadGeometry>, SharedPtrCmp<RoadGeometry, double, &RoadGeometry::s0>>;

} // namespace odr
