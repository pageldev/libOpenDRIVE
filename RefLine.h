#pragma once

#include "Geometries/RoadGeometry.h"

#include <map>
#include <memory>

namespace odr
{

struct ElevationProfile
{
    ElevationProfile(double s0, double a, double b, double c, double d);
    double get(double s) const;
    double get_grad(double s) const;

    double s0, a, b, c, d;
};

struct RefLine
{
    RefLine(double length);

    Vec3D get_xyz(double s) const;
    Vec3D get_grad(double s) const;

    double match(double x, double y) const;
    double get_z(double s) const;
    double get_z_grad(double s) const;

    double length;

    std::shared_ptr<RoadGeometry>                       get_geometry(double s) const;
    std::map<double, std::shared_ptr<RoadGeometry>>     geometries;
    std::map<double, std::shared_ptr<ElevationProfile>> elevation_profiles;
};

} // namespace odr