#pragma once

#include "Geometries/CubicSpline.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"

#include <map>
#include <memory>
#include <set>

namespace odr
{

struct RefLine
{
    RefLine(double length);
    virtual ~RefLine() = default;

    ConstRoadGeometrySet get_geometries() const;
    RoadGeometrySet      get_geometries();

    std::shared_ptr<const RoadGeometry> get_geometry(double s) const;

    Vec3D  get_xyz(double s) const;
    Vec3D  get_grad(double s) const;
    Line3D get_line(double s_start, double s_end, double eps) const;
    double match(double x, double y) const;

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    double      length = 0;
    CubicSpline elevation_profile;

    std::map<double, std::shared_ptr<RoadGeometry>> s0_to_geometry;
};

} // namespace odr