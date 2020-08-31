#include "RoadGeometry.h"

namespace odr
{

RoadGeometry::RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type)
    : s0(s0), x0(x0), y0(y0), hdg0(hdg0), length(length), type(type)
{
}

RoadGeometry::~RoadGeometry(){};

} // namespace odr