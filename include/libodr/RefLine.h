#pragma once
#include "libodr/Geometries/CubicSpline.h"
#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Math.hpp"

#include <map>
#include <memory>
#include <set>

namespace odr
{

struct RefLine
{
    RefLine(double length);
    RefLine(const RefLine& other);
    RefLine(RefLine&& other) = default;

    RefLine& operator=(RefLine&& other) = default;

    std::optional<double> get_geometry_s(double s) const;
    const RoadGeometry*   get_geometry(double s) const;
    RoadGeometry*         get_geometry(double s);

    Vec3D  get_xyz(double s) const;
    Vec3D  derivative(double s) const;
    double match(double x, double y) const;

    double length;

    CubicProfile elevation_profile;

    std::map<double, std::unique_ptr<RoadGeometry>> s_to_geometry;
};

} // namespace odr
