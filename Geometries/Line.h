#pragma once
#include "RoadGeometry.h"

namespace odr
{

struct Line : public RoadGeometry
{
    Line(double s0, double x0, double y0, double hdg0, double length, std::shared_ptr<Road> road);
    void update() override;

    Vec2D  get_xy(double s) const override;
    Vec2D  get_grad(double s) const override;
};

} // namespace odr