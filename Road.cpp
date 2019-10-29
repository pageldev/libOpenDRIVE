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

Road::Road(double length, int id, int junction, std::set<std::shared_ptr<RoadGeometry>, PtrCompareS0<RoadGeometry>> geometries)
    : length(length), id(id), junction(junction), geometries(geometries)
{  }

Point3D Road::get_refline_point(double s, double t)
{
    std::set<std::shared_ptr<RoadGeometry>, PtrCompareS0<RoadGeometry>>::iterator target_geom_iter 
        = this->geometries.upper_bound(std::make_shared<Line>(s, 0.0, 0.0, 0.0, 0.0));
    if( target_geom_iter != geometries.begin() ) {
        target_geom_iter--;
    }
    return (*target_geom_iter)->get_point(s, t);
}