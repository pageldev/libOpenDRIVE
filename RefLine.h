#pragma once

#include "Geometries/CubicSpline.h"
#include "Geometries/RoadGeometry.h"

#include <map>
#include <memory>

namespace odr
{

struct RefLine
{
    RefLine(double length);

    Vec3D                         get_xyz(double s) const;
    Vec3D                         get_grad(double s) const;
    double                        match(double x, double y) const;
    std::shared_ptr<RoadGeometry> get_geometry(double s) const;

    double      length;
    CubicSpline elevation_profile;

    std::map<double, std::shared_ptr<RoadGeometry>> geometries;
};

} // namespace odr