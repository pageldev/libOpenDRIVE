#pragma once

#include "Geometries/RoadGeometry.h"

#include <map>
#include <memory>

namespace odr
{

struct ElevationProfile
{
    ElevationProfile(double s0, double a, double b, double c, double d);
    double get_elevation(double s) const;
    double get_grad(double s) const;

    double s0, a, b, c, d;
};

struct RefLine
{
    RefLine() = default;
    Vec3D get_point(double s, double t = 0, double t_offset = 0) const;
    Vec3D get_grad(double s) const;

    double get_elevation(double s) const;
    double get_elevation_grad(double s) const;
    double project(double x, double y) const;

    std::shared_ptr<RoadGeometry> get_geometry(double s) const;

    std::map<double, std::shared_ptr<RoadGeometry>>     geometries;
    std::map<double, std::shared_ptr<ElevationProfile>> elevation_profiles;
};

} // namespace odr