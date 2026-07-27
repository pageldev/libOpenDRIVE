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

std::set<const RoadGeometry*> RefLine::get_geometries() const
{
    std::set<const RoadGeometry*> geometries;
    for (const auto& [_, geometry] : this->s_to_geometry)
        geometries.insert(geometry.get());
    return geometries;
}

std::set<RoadGeometry*> RefLine::get_geometries()
{
    std::set<RoadGeometry*> geometries;
    for (auto& [s, geometry] : this->s_to_geometry)
        geometries.insert(geometry.get());
    return geometries;
}

std::optional<double> RefLine::get_geometry_s(double s) const
{
    if (this->s_to_geometry.empty())
        return std::nullopt;
    auto target_geom_iter = this->s_to_geometry.upper_bound(s); // first element > s
    if (target_geom_iter != s_to_geometry.begin())
        target_geom_iter--;
    return target_geom_iter->first;
}

const RoadGeometry* RefLine::get_geometry(double s) const
{
    const std::optional<double> s_geometry = this->get_geometry_s(s);
    if (!s_geometry)
        return nullptr;
    return this->s_to_geometry.at(*s_geometry).get();
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

Line3D RefLine::get_line(double s_start, double s_end, double eps) const
{
    std::set<double> s_samples = this->approximate_linear(eps, s_start, s_end);

    Line3D out_line;
    for (const double s : s_samples)
        out_line.push_back(this->get_xyz(s));
    return out_line;
}

std::set<double> RefLine::approximate_linear(double eps, double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s_to_geometry.empty())
        return {};

    s_start = std::min(s_start, s_end);
    s_end = std::max(s_start, s_end);

    auto geometry_end_iter = this->s_to_geometry.lower_bound(s_end);     // first element >= s
    auto geometry_start_iter = this->s_to_geometry.upper_bound(s_start); // first element > s
    if (geometry_start_iter != s_to_geometry.begin())
        geometry_start_iter--;

    std::vector<double> s_samples{s_start};
    for (auto geometry_iter = geometry_start_iter; geometry_iter != geometry_end_iter; geometry_iter++)
    {
        const std::set<double> s_samples_geometry = geometry_iter->second->approximate_linear(eps);
        if (s_samples_geometry.size() < 2)
            throw std::runtime_error("expected at least two sample points");
        for (const double s : s_samples_geometry)
        {
            if (s > s_start && s < s_end)
                s_samples.push_back(s);
        }
        if (std::next(geometry_iter) != geometry_end_iter)
            s_samples.pop_back();
    }

    std::set<double> s_samples_elevation = this->elevation_profile.approximate_linear(eps, s_start, s_end);
    for (const double s : s_samples_elevation)
    {
        if (s > s_start && s < s_end)
            s_samples.push_back(s);
    }

    s_samples.push_back(s_end);

    std::set<double> s_sample_set(s_samples.begin(), s_samples.end());
    return s_sample_set;
}

} // namespace odr
