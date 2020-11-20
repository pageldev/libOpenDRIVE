#pragma once

#include "RoadGeometry.h"

namespace odr
{
class Road;

struct ParamPoly3 : public RoadGeometry
{
    ParamPoly3(double                s0,
               double                x0,
               double                y0,
               double                hdg0,
               double                length,
               double                aU,
               double                bU,
               double                cU,
               double                dU,
               double                aV,
               double                bV,
               double                cV,
               double                dV,
               std::shared_ptr<Road> road);
    void update() override;

    Vec2D get_xy(double s) const override;
    Vec2D get_grad(double s) const override;

    double aU, bU, cU, dU, aV, bV, cV, dV;
};

} // namespace odr