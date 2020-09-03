#include "RoadGeometry.h"

#include <cmath>
#include <cstdlib>

namespace odr
{

Box2D::Box2D(Point2D min, Point2D max)
    : min(min), max(max)
{
    center[0] = (min[0] + max[0]) / 2;
    center[1] = (min[1] + max[1]) / 2;
    width = max[0] - min[0];
    height = max[1] - min[1];
}

double Box2D::get_distance(const Point2D &pt)
{
    const double dist_x = std::abs(center[0] - pt[0]) - (width / 2);
    const double dist_y = std::abs(center[1] - pt[1]) - (height / 2);

    double dist = 0.0;
    if (dist_x < 0 && dist_y < 0)
        dist = std::abs(std::max(dist_x, dist_y));
    else if (dist_x < 0)
        dist = dist_y;
    else if (dist_y < 0)
        dist = dist_x;
    else
        dist = std::sqrt(dist_x * dist_x + dist_y * dist_y);

    return dist;
}

RoadGeometry::RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type)
    : s0(s0), x0(x0), y0(y0), hdg0(hdg0), length(length), type(type)
{
}

RoadGeometry::~RoadGeometry(){};

} // namespace odr