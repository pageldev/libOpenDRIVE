#include "RefLine.h"
#include "Utils.hpp"

namespace odr
{

ElevationProfile::ElevationProfile(double s0, double a, double b, double c, double d)
    : s0(s0), a(a), b(b), c(c), d(d) {}

double ElevationProfile::get_elevation(double s) const
{
    const double ds = s - this->s0;
    return (a + b * ds + c * ds * ds + d * ds * ds * ds);
}

double ElevationProfile::get_grad(double s) const
{
    return (b + 2 * c * s + 3 * d * s * s);
}

Vec3D RefLine::get_point(double s, double t, double t_offset) const
{
    Vec2D pt_xy{0, 0};
    if (this->geometries.size() > 0)
    {
        auto target_geom_iter = this->geometries.upper_bound(s);
        if (target_geom_iter != geometries.begin())
            target_geom_iter--;
        pt_xy = target_geom_iter->second->get_point(s, t + t_offset);
    }

    double elev = 0;
    if (this->elevation_profiles.size() > 0)
    {
        auto target_elev_iter = this->elevation_profiles.upper_bound(s);
        if (target_elev_iter != elevation_profiles.begin())
            target_elev_iter--;
        elev = target_elev_iter->second->get_elevation(s);
    }

    return Vec3D{pt_xy[0], pt_xy[1], elev};
}

double RefLine::project(double x, double y) const
{
    double s = 0;
    double min_dist_sqr = -1;
    for (const std::pair<const double, std::shared_ptr<RoadGeometry>> &geom : geometries)
    {
        double const s_proj = geom.second->project(x, y);
        const Vec3D  refl_pt = this->get_point(s_proj, 0.0, false);
        const double dist_sqr = get_dist_sqr(refl_pt, {x, y, refl_pt[2]});
        if (min_dist_sqr < 0 || dist_sqr < min_dist_sqr)
        {
            min_dist_sqr = dist_sqr;
            s = s_proj;
        }
    }
    return s;
}

Vec3D RefLine::get_grad(double s) const
{
    Vec2D d_xy{0, 0};
    if (this->geometries.size() > 0)
    {
        auto target_geom_iter = this->geometries.upper_bound(s);
        if (target_geom_iter != geometries.begin())
            target_geom_iter--;
        d_xy = target_geom_iter->second->get_grad(s);
    }

    double d_z = 0;
    if (this->elevation_profiles.size() > 0)
    {
        auto target_elev_iter = this->elevation_profiles.upper_bound(s);
        if (target_elev_iter != elevation_profiles.begin())
            target_elev_iter--;
        d_z = target_elev_iter->second->get_grad(s);
    }

    return Vec3D{d_xy[0], d_xy[1], d_z};
}

} // namespace odr