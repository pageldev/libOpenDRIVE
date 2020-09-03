#pragma once
#include "RoadGeometry.h"

namespace odr
{

struct ParamPoly3 : public RoadGeometry
{
    ParamPoly3(double s0, double x0, double y0, double hdg0, double length, double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV);

    Point2D get_point(double s, double t = 0) const override;
    Box2D   get_bbox() const override;

    double aU, bU, cU, dU, aV, bV, cV, dV;
};

} // namespace odr