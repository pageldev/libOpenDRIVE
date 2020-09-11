#pragma once
#include "RoadGeometry.h"

namespace odr
{

struct Line : public RoadGeometry
{
    Line(double s0, double x0, double y0, double hdg0, double length);
    void update() override;

    Vec2D  get_point(double s, double t = 0) const override;
    double project(double x, double y) const override;
    Vec2D  get_grad(double s) const override;
};

} // namespace odr