#include "libodr/RefLine.h"
#include "libodr/Math.hpp"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace odr
{

RefLine::RefLine(double length) : length(length) {}

RefLine::RefLine(const RefLine& other) : length(other.length), elevation_profile(other.elevation_profile)
{
    for (const auto& [s, geometry] : other.s_to_geometry)
        this->s_to_geometry.emplace(s, geometry->clone());
}

const RoadGeometry* RefLine::get_geometry(double s) const
{
    if (this->s_to_geometry.empty())
        return nullptr;

    auto target_geom_iter = this->s_to_geometry.upper_bound(s); // first element > s
    if (target_geom_iter != this->s_to_geometry.begin())
        target_geom_iter--;
    return target_geom_iter->second.get();
}

RoadGeometry* RefLine::get_geometry(double s)
{
    RoadGeometry* road_geometry = const_cast<RoadGeometry*>(static_cast<const RefLine&>(*this).get_geometry(s));
    return road_geometry;
}

Vec3D RefLine::get_xyz(double s) const
{
    const RoadGeometry* geom = this->get_geometry(s);
    require_or_throw(geom != nullptr, "reference line has no geometry at s {}", s);
    const Vec2D pt_xy = geom->get_xy(s);
    return Vec3D{pt_xy[0], pt_xy[1], this->elevation_profile.evaluate(s).value_or(0.0)};
}

Vec3D RefLine::derivative(double s) const
{
    const RoadGeometry* geom = this->get_geometry(s);
    require_or_throw(geom != nullptr, "reference line has no geometry at s {}", s);
    const Vec2D d_xy = geom->derivative(s);
    return Vec3D{d_xy[0], d_xy[1], this->elevation_profile.derivative(s).value_or(0.0)};
}

double RefLine::match(double x, double y) const
{
    std::function<double(double)> f_dist = [&](double s)
    {
        const Vec3D pt = this->get_xyz(s);
        return euclDistance(Vec2D{pt[0], pt[1]}, {x, y});
    };
    return golden_section_search<double>(f_dist, 0.0, length, 1e-2);
}

} // namespace odr
