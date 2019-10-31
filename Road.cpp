#include "Road.h"
#include "Lanes.h"


ElevationProfile::ElevationProfile(double s0, double a, double b, double c, double d)
    : s0(s0), a(a), b(b), c(c), d(d)
{  }

double ElevationProfile::get_elevation(double s)
{
    double ds = s - this->s0;
    return (a + b*ds + c*ds*ds + d*ds*ds*ds);
}

Road::Road(double length, int id, int junction, std::set<std::shared_ptr<RoadGeometry>, PtrCompareS0<RoadGeometry>> geometries )
    : length(length), id(id), junction(junction), geometries(geometries)
{  }

Point3D Road::get_refline_point(double s, double t)
{
    std::set<std::shared_ptr<RoadGeometry>, PtrCompareS0<RoadGeometry>>::iterator target_geom_iter 
        = this->geometries.upper_bound(std::make_shared<Line>(s, 0.0, 0.0, 0.0, 0.0));
    if( target_geom_iter != geometries.begin() ) {
        target_geom_iter--;
    }
    Point2D plan_view_pt = (*target_geom_iter)->get_point(s, t);

    double z = 0;
    if( this->elevation_profiles.size() > 0 ) {
        std::set<std::shared_ptr<ElevationProfile>, PtrCompareS0<ElevationProfile>>::iterator target_elev_iter 
            = this->elevation_profiles.upper_bound(std::make_shared<ElevationProfile>(s, 0.0, 0.0, 0.0, 0.0));
        if( target_elev_iter != elevation_profiles.begin() ) {
            target_elev_iter--;
        }
        z = (*target_elev_iter)->get_elevation(s);
    }

    return Point3D {plan_view_pt.x, plan_view_pt.y, z};
}