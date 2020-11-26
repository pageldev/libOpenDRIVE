#include "RefLine.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <functional>
#include <utility>

namespace odr
{
RefLine::RefLine(double length) : length(length) {}

ConstRoadGeometrySet RefLine::get_geometries() const
{
    ConstRoadGeometrySet geometries;
    for (const auto& s0_geometry : this->s0_to_geometry)
        geometries.insert(s0_geometry.second);

    return geometries;
}

RoadGeometrySet RefLine::get_geometries()
{
    RoadGeometrySet geometries;
    for (const auto& s0_geometry : this->s0_to_geometry)
        geometries.insert(s0_geometry.second);

    return geometries;
}

std::shared_ptr<const RoadGeometry> RefLine::get_geometry(double s) const
{
    if (this->s0_to_geometry.size() > 0)
    {
        auto target_geom_iter = this->s0_to_geometry.upper_bound(s);
        if (target_geom_iter != s0_to_geometry.begin())
            target_geom_iter--;
        return target_geom_iter->second;
    }
    return nullptr;
}

Vec3D RefLine::get_xyz(double s) const
{
    std::shared_ptr<const RoadGeometry> geom = this->get_geometry(s);

    Vec2D pt_xy{0, 0};
    if (geom)
        pt_xy = geom->get_xy(s);

    return Vec3D{pt_xy[0], pt_xy[1], this->elevation_profile.get(s)};
}

Vec3D RefLine::get_grad(double s) const
{
    std::shared_ptr<const RoadGeometry> geom = this->get_geometry(s);

    Vec2D d_xy{0, 0};
    if (geom)
        d_xy = geom->get_grad(s);

    return Vec3D{d_xy[0], d_xy[1], this->elevation_profile.get_grad(s)};
}

double RefLine::match(double x, double y) const
{
    std::function<double(double)> f_dist = [&](const double s) {
        const Vec3D pt = this->get_xyz(s);
        return euclDistance(Vec2D{pt[0], pt[1]}, {x, y});
    };
    return golden_section_search<double>(f_dist, 0.0, length, 1e-2);
}

} // namespace odr