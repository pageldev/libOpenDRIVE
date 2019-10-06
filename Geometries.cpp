#include "Geometries.h"

RoadGeometry::RoadGeometry(double s0, double x0, double y0, double hdg0, double length, Geometry_type type)
    : s0(s0), x0(x0), y0(y0), hdg0(hdg0), length(length), type(type)
{  }

RoadGeometry::~RoadGeometry() { };


Line::Line(double s0, double x0, double y0, double hdg0, double length)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Line)
{  }

Point3D Line::get_point(double s, double t)
{
    double xt = (std::cos(hdg0) * (s-s0)) - (std::sin(hdg0) * t) + x0;
    double yt = (std::sin(hdg0) * (s-s0)) + (std::cos(hdg0) * t) + y0;
    return Point3D {xt, yt, 0.0};
}


Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Spiral)
    , curv_start(curv_start)
    , curv_end(curv_end)
{  
    this->c_dot = (curv_end - curv_start) / length;
}

Point3D Spiral::get_point(double s, double t)
{
    double s0_spiral = curv_start / c_dot;
    double x0_spiral, y0_spiral, a0_spiral;
    odrSpiral( s0_spiral, c_dot, &x0_spiral, &y0_spiral, &a0_spiral );
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral( s-s0+s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral );
    
    double tx = t * std::cos(as_spiral + M_PI/2);
    double ty = t * std::sin(as_spiral + M_PI/2);
    double hdg = hdg0 - a0_spiral;

    double xt = (std::cos(hdg) * (xs_spiral-x0_spiral+tx)) - (std::sin(hdg) * (ys_spiral-y0_spiral+ty)) + x0;
    double yt = (std::sin(hdg) * (xs_spiral-x0_spiral+tx)) + (std::cos(hdg) * (ys_spiral-y0_spiral+ty)) + y0;
    return Point3D {xt, yt, 0.0};
}


Arc::Arc(double s0, double x0, double y0, double hdg0, double length, double curvature)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::Arc)
    , curvature(curvature)
{  }

Point3D Arc::get_point(double s, double t)
{
    double angle_at_s = (s-s0)*curvature - M_PI/2;
    double r = 1 / curvature;
    double xs = (r-t) * std::cos(angle_at_s);
    double ys = (r-t) * std::sin(angle_at_s) + r;
    double xt = (std::cos(hdg0) * xs) - (std::sin(hdg0) * ys) + x0;
    double yt = (std::sin(hdg0) * xs) + (std::cos(hdg0) * ys) + y0;
    return Point3D {xt, yt, 0.0};
}


ParamPoly3::ParamPoly3(double s0, double x0, double y0, double hdg0, double length
        , double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV)
    : RoadGeometry(s0, x0, y0, hdg0, length, Geometry_type::ParamPoly3)
    , aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV)
{  }

Point3D ParamPoly3::get_point(double s, double t)
{
    double p = (s-s0) / length;
    double xs = aU + bU*p + cU*p*p + dU*p*p*p;
    double ys = aV + bV*p + cV*p*p + dV*p*p*p;
    double xt = (std::cos(hdg0) * xs) - (std::sin(hdg0) * (ys+t)) + x0;
    double yt = (std::sin(hdg0) * xs) + (std::cos(hdg0) * (ys+t)) + y0;
    return Point3D {xt, yt, 0.0};
}

