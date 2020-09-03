#pragma once
#include "RoadGeometry.h"

namespace odr
{

struct Line : public RoadGeometry
{
    Line(double s0, double x0, double y0, double hdg0, double length);

    Point2D get_point(double s, double t = 0) const override;
    Box2D   get_bbox() const override;
};

} // namespace odr