#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Utils.hpp"

#include <cmath>

namespace odr
{

RoadGeometry::RoadGeometry(double s0, double x0, double y0, double hdg0, double length) : s0(s0), x0(x0), y0(y0), hdg0(hdg0), length(length)
{
    require_or_throw(s0 >= 0, "s must be greater than or equal to 0 (got {})", s0);
    require_or_throw(!std::isnan(x0), "x must not be NaN");
    require_or_throw(!std::isnan(y0), "y must not be NaN");
    require_or_throw(!std::isnan(hdg0), "heading must not be NaN");
    require_or_throw(length > 0, "length must be greater than 0 (got {})", length);
}

} // namespace odr
