#pragma once
#include "RoadGeometry.h"

namespace odr
{

struct Spiral : public RoadGeometry
{
    Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end);
    void update() override;

    Vec2D  get_point(double s, double t = 0) const override;
    double project(double x, double y) const override;
    Vec2D  get_grad(double s) const override;

    double curv_start, curv_end;
    double s_start, s_end;
    double c_dot;

private:    
    double s0_spiral, x0_spiral, y0_spiral, a0_spiral;
};

} // namespace odr