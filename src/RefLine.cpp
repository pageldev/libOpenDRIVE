#include "libodr/RefLine.h"
#include "libodr/Math.hpp"
#include "libodr/Sampler.h"
#include "libodr/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
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
    if (length <= 0.0)
        return 0.0;

    const std::set<double> samples = RefLineSampler(*this).get_polyline_s_samples(0.0, length, 1e-2);
    double                 best_s = 0.0;
    double                 best_dist = std::numeric_limits<double>::infinity();

    auto  prev = samples.begin();
    Vec3D prev_pt = get_xyz(*prev);
    for (auto it = std::next(prev); it != samples.end(); ++it)
    {
        const Vec3D  pt = get_xyz(*it);
        const double dx = pt[0] - prev_pt[0];
        const double dy = pt[1] - prev_pt[1];
        const double length_sq = dx * dx + dy * dy;
        const double fraction = length_sq == 0.0 ? 0.0 : std::clamp(((x - prev_pt[0]) * dx + (y - prev_pt[1]) * dy) / length_sq, 0.0, 1.0);
        const double s = *prev + fraction * (*it - *prev);
        const Vec3D  candidate = get_xyz(s);
        const double dist = std::hypot(candidate[0] - x, candidate[1] - y);
        if (dist < best_dist)
        {
            best_dist = dist;
            best_s = s;
        }
        prev = it;
        prev_pt = pt;
    }
    return best_s;
}

} // namespace odr
