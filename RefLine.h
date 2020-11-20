#pragma once

#include "Geometries/CubicSpline.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"

#include <map>
#include <memory>

namespace odr
{
struct RoadGeometry;

struct RefLine
{
    RefLine(double length);
    RoadGeometrySet get_geometries();

    std::shared_ptr<const RoadGeometry> get_geometry(double s) const;

    Vec3D  get_xyz(double s) const;
    Vec3D  get_grad(double s) const;
    double match(double x, double y) const;

    double      length;
    CubicSpline elevation_profile;

    std::map<double, std::shared_ptr<RoadGeometry>> s0_to_geometry;
};

} // namespace odr