#pragma once
#include "RoadGeometry.h"

struct Spiral : public RoadGeometry
{
    Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end);
    Point2D get_point(double s, double t = 0) override;

    double curv_start;
    double curv_end;
    double c_dot;
};