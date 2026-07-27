#include "libodr/Geometries/Arc.h"
#include "libodr/Geometries/RoadGeometry.h"
#include "libodr/Utils.hpp"

#include <cmath>

namespace odr
{
Arc::Arc(double s, double x, double y, double hdg, double length, double curvature) : RoadGeometry(s, x, y, hdg, length), curvature(curvature)
{
    require_or_throw(!std::isnan(curvature), "curvature must not be NaN");
}

std::unique_ptr<RoadGeometry> Arc::clone() const
{
    return std::make_unique<Arc>(*this);
}

Vec2D Arc::get_xy(double s) const
{
    const double angle_at_s = (s - this->s) * curvature - M_PI / 2;
    const double r = 1 / curvature;
    const double x_s = r * (std::cos(hdg + angle_at_s) - std::sin(hdg)) + x;
    const double y_s = r * (std::sin(hdg + angle_at_s) + std::cos(hdg)) + y;
    return Vec2D{x_s, y_s};
}

Vec2D Arc::derivative(double s) const
{
    const double dx = std::sin((M_PI / 2) - curvature * (s - this->s) - hdg);
    const double dy = std::cos((M_PI / 2) - curvature * (s - this->s) - hdg);
    return {{dx, dy}};
}

std::set<double> Arc::approximate_linear([[maybe_unused]] double eps) const
{
    // TODO: properly implement
    const double     s_step = 0.01 / std::abs(this->curvature); // sample at approx. every 1°
    std::set<double> s_samples;
    for (double s_sample = s; s_sample < (s + length); s_sample += s_step)
        s_samples.insert(s_sample);
    s_samples.insert(s + length);

    return s_samples;
}

} // namespace odr
