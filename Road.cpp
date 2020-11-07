#include "Road.h"

#include <iostream>

#include "Lanes.h"

namespace odr
{

ElevationProfile::ElevationProfile(double s0, double a, double b, double c,
                                   double d)
    : s0(s0), a(a), b(b), c(c), d(d) {}

double ElevationProfile::get_elevation(const double s) const
{
    const double ds = s - this->s0;
    return (a + b * ds + c * ds * ds + d * ds * ds * ds);
}

Road::Road(double length, int id, int junction)
    : length(length), id(id), junction(junction) {}

void Road::add_lane_section(std::shared_ptr<LaneSection> lane_section)
{
    if (lane_section->road)
    {
        std::cerr << "Error - lane section was already associated with a road"
                  << std::endl;
    }
    lane_section->road = shared_from_this();
    this->lane_sections[lane_section->s0] = lane_section;
}

Vec3D Road::get_refline_point(const double s, const double t, const bool with_offset) const
{
    double offset = 0;
    if (with_offset && this->lane_offsets.size() > 0)
    {
        std::map<double, std::shared_ptr<LaneOffset>>::const_iterator
            target_lane_offset_iter = this->lane_offsets.upper_bound(s);

        if (target_lane_offset_iter != lane_offsets.begin())
        {
            target_lane_offset_iter--;
        }
        offset = (*target_lane_offset_iter).second->get_offset(s);
    }

    std::map<double, std::shared_ptr<RoadGeometry>>::const_iterator target_geom_iter =
        this->geometries.upper_bound(s);

    if (target_geom_iter != geometries.begin())
    {
        target_geom_iter--;
    }
    Vec2D plan_view_pt = (*target_geom_iter).second->get_point(s, t + offset);

    const double z = this->get_elevation(s);
    return Vec3D{plan_view_pt[0], plan_view_pt[1], z};
}

double Road::get_elevation(const double s) const
{
    double elev = 0;
    if (this->elevation_profiles.size() > 0)
    {
        std::map<double, std::shared_ptr<ElevationProfile>>::const_iterator
            target_elev_iter = this->elevation_profiles.upper_bound(s);
        if (target_elev_iter != elevation_profiles.begin())
        {
            target_elev_iter--;
        }
        elev = (*target_elev_iter).second->get_elevation(s);
    }

    return elev;
}

double Road::project(double x, double y) const
{
    double s = 0;
    double min_dist_sqr = -1;
    for (const std::pair<const double, std::shared_ptr<RoadGeometry>> &geom : geometries)
    {
        double const s_proj = geom.second->project(x, y);
        const Vec3D  refl_pt = get_refline_point(s_proj, 0.0, false);
        const double dist_sqr = get_dist_sqr(refl_pt, {x, y, refl_pt[2]});
        if (min_dist_sqr < 0 || dist_sqr < min_dist_sqr)
        {
            min_dist_sqr = dist_sqr;
            s = s_proj;
        }
    }
    return s;
}

} // namespace odr