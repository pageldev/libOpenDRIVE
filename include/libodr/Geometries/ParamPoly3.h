#pragma once
#include "RoadGeometry.h"
#include "libodr/CubicBezier.hpp"
#include "libodr/Math.hpp"

#include <memory>
#include <set>

namespace odr
{

struct ParamPoly3 : public RoadGeometry
{
    enum class PRange
    {
        Normalized,
        ArcLength
    };

    ParamPoly3(double s,
               double x,
               double y,
               double hdg,
               double length,
               double aU,
               double bU,
               double cU,
               double dU,
               double aV,
               double bV,
               double cV,
               double dV,
               PRange p_range = PRange::Normalized);

    std::unique_ptr<RoadGeometry> clone() const override;

    Vec2D get_xy(double s) const override;
    Vec2D derivative(double s) const override;

    double        aU = 0, bU = 0, cU = 0, dU = 0, aV = 0, bV = 0, cV = 0, dV = 0;
    PRange        p_range = PRange::Normalized;
    CubicBezier2D cubic_bezier;
};

} // namespace odr
