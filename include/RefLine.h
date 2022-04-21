#pragma once
#include "Geometries/CubicSpline.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>

namespace odr
{

struct RefLine
{
    RefLine(std::string road_id, double length);
    RefLine(const RefLine& other);

    std::set<const RoadGeometry*> get_geometries() const;
    std::set<RoadGeometry*>       get_geometries();

    double              get_geometry_s0(double s) const;
    const RoadGeometry* get_geometry(double s) const;
    RoadGeometry*       get_geometry(double s);

    Vec3D            get_xyz(double s) const;
    Vec3D            get_grad(double s) const;
    Line3D           get_line(double s_start, double s_end, double eps) const;
    double           match(double x, double y) const;
    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    std::string road_id = "";
    double      length = 0;
    CubicSpline elevation_profile;

    std::map<double, std::unique_ptr<RoadGeometry>> s0_to_geometry;
};

} // namespace odr