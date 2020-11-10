#include "RefLine.h"
#include "Utils.hpp"

namespace odr
{

ElevationProfile::ElevationProfile(double s0, double a, double b, double c, double d)
    : s0(s0), a(a), b(b), c(c), d(d) {}

double ElevationProfile::get(double s) const
{
    const double ds = s - this->s0;
    return (a + b * ds + c * ds * ds + d * ds * ds * ds);
}

double ElevationProfile::get_grad(double s) const
{
    return (b + 2 * c * s + 3 * d * s * s);
}

Vec3D RefLine::get_xyz(double s) const
{
    std::shared_ptr<RoadGeometry> geom = this->get_geometry(s);

    Vec2D pt_xy{0, 0};
    if (geom)
        pt_xy = geom->get_xy(s);

    return Vec3D{pt_xy[0], pt_xy[1], this->get_z(s)};
}

Vec3D RefLine::get_grad(double s) const
{
    std::shared_ptr<RoadGeometry> geom = this->get_geometry(s);

    Vec2D d_xy{0, 0};
    if (geom)
        d_xy = geom->get_grad(s);

    return Vec3D{d_xy[0], d_xy[1], this->get_z_grad(s)};
}

double RefLine::get_z(double s) const
{
    double elev = 0;
    if (this->elevation_profiles.size() > 0)
    {
        auto target_elev_iter = this->elevation_profiles.upper_bound(s);
        if (target_elev_iter != elevation_profiles.begin())
            target_elev_iter--;
        elev = target_elev_iter->second->get(s);
    }
    return elev;
}

double RefLine::get_z_grad(double s) const
{
    double d_z = 0;
    if (this->elevation_profiles.size() > 0)
    {
        auto target_elev_iter = this->elevation_profiles.upper_bound(s);
        if (target_elev_iter != elevation_profiles.begin())
            target_elev_iter--;
        d_z = target_elev_iter->second->get_grad(s);
    }
    return d_z;
}

std::shared_ptr<RoadGeometry> RefLine::get_geometry(double s) const
{
    std::shared_ptr<RoadGeometry> geom = nullptr;
    if (this->geometries.size() > 0)
    {
        auto target_geom_iter = this->geometries.upper_bound(s);
        if (target_geom_iter != geometries.begin())
            target_geom_iter--;
        geom = target_geom_iter->second;
    }
    return geom;
}

} // namespace odr