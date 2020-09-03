#pragma once
#include "RoadGeometry.h"

namespace odr
{

struct Arc : public RoadGeometry
{
    Arc(double s0, double x0, double y0, double hdg0, double length, double curvature);

    Point2D get_point(double s, double t = 0) const override;
    Box2D   get_bbox() const override;

    double curvature;
};

} // namespace odr