#include "ParamPoly3.h"
#include <cmath>

ParamPoly3::ParamPoly3(double s0, double x0, double y0, double hdg0, double length, double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::ParamPoly3), aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV)
{
}

Point2D ParamPoly3::get_point(double s, double t) const
{
    double p = (s - s0) / length;
    double xs = aU + bU * p + cU * p * p + dU * p * p * p;
    double ys = aV + bV * p + cV * p * p + dV * p * p * p;
    double xs_dp = bU + 2 * cU * p + 3 * dU * p * p;
    double ys_dp = bV + 2 * cV * p + 3 * dV * p * p;
    double x_offs = xs - ((t * ys_dp) / std::sqrt(xs_dp * xs_dp + ys_dp * ys_dp));
    double y_offs = ys + ((t * xs_dp) / std::sqrt(xs_dp * xs_dp + ys_dp * ys_dp));
    double xt = (std::cos(hdg0) * x_offs) - (std::sin(hdg0) * y_offs) + x0;
    double yt = (std::sin(hdg0) * x_offs) + (std::cos(hdg0) * y_offs) + y0;
    return Point2D{xt, yt};
}