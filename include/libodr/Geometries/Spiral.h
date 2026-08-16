#pragma once
#include "RoadGeometry.h"
#include "libodr/Math.hpp"

#include <memory>
#include <set>

namespace odr
{

struct Spiral : public RoadGeometry
{
    Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end);

    std::unique_ptr<RoadGeometry> clone() const override;

    Vec2D get_xy(double s) const override;
    Vec2D derivative(double s) const override;

    double curv_start;
    double curv_end;
    double spiral_s_start; // internal s (curv_start/c_dot) != road s
    double spiral_s_end;
    double c_dot;

private:
    double spiral_s_origin;
    double spiral_x_origin;
    double spiral_y_origin;
    double spiral_hdg_origin;
};

} // namespace odr
