#include "RefLine.h"
#include "Utils.hpp"

namespace odr
{

RefLine::RefLine(double length)
    : length(length)
{
}

Vec3D RefLine::get_xyz(double s) const
{
    std::shared_ptr<RoadGeometry> geom = this->get_geometry(s);

    Vec2D pt_xy{0, 0};
    if (geom)
        pt_xy = geom->get_xy(s);

    return Vec3D{pt_xy[0], pt_xy[1], this->elevation_profile.get(s)};
}

Vec3D RefLine::get_grad(double s) const
{
    std::shared_ptr<RoadGeometry> geom = this->get_geometry(s);

    Vec2D d_xy{0, 0};
    if (geom)
        d_xy = geom->get_grad(s);

    return Vec3D{d_xy[0], d_xy[1], this->elevation_profile.get_grad(s)};
}

double RefLine::match(double x, double y) const
{
    std::function<double(double)> f_dist = [&](const double s) { const Vec3D pt = this->get_xyz(s); return euclDistance(Vec2D{pt[0], pt[1]}, {x,y}); };
    return golden_section_search<double>(f_dist, 0.0, length, 1e-2);
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