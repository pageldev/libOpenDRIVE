#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Utils.hpp"

#include <cmath>

namespace odr
{

RoadGeometry::RoadGeometry(double s, double x, double y, double hdg, double length) : s(s), x(x), y(y), hdg(hdg), length(length)
{
    require_or_throw(s >= 0, "s must be greater than or equal to 0 (got {})", s);
    require_or_throw(!std::isnan(x), "x must not be NaN");
    require_or_throw(!std::isnan(y), "y must not be NaN");
    require_or_throw(!std::isnan(hdg), "heading must not be NaN");
    require_or_throw(length > 0, "length must be greater than 0 (got {})", length);
}

} // namespace odr
