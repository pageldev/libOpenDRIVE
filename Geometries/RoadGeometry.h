#pragma once

#include <array>
#include <map>
#include <string>

namespace odr
{

template <typename T, size_t Dim, typename std::enable_if_t<(Dim > 1)> * = nullptr, typename std::enable_if_t<std::is_arithmetic<T>::value> * = nullptr>
using Vec = std::array<T, Dim>;

using Vec2D = Vec<double, 2>;
using Vec3D = Vec<double, 3>;

struct Box2D
{
    Box2D(Vec2D min, Vec2D max);
    double get_distance(const Vec2D &pt);

    Vec2D  min, max;
    Vec2D  center;
    double width, height;
};

enum class GeometryType
{
    Line,
    Spiral,
    Arc,
    ParamPoly3
};

struct RoadGeometry
{
    RoadGeometry(double s0, double x0, double y0, double hdg0, double length, GeometryType type);
    virtual ~RoadGeometry();

    virtual Vec2D  get_point(double s, double t = 0) const = 0;
    virtual Box2D  get_bbox() const = 0;
    virtual double project(double x, double y) const = 0;
    virtual Vec2D  get_grad(double s) const = 0;

    GeometryType type;

    double s0;
    double x0;
    double y0;
    double hdg0;
    double length;
};

} // namespace odr
