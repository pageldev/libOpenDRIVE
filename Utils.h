#pragma once
#include "Geometries/Geometries.h"

#include <algorithm>
#include <functional>
#include <type_traits>
#include <vector>

namespace odr
{

template <typename T>
int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

std::vector<Point3D<double>> rdp(const std::vector<Point3D<double>> &points, const double epsilon = 0.1);

template <typename T, typename std::enable_if_t<std::is_arithmetic<T>::value> * = nullptr>
Box2D<T> get_bbox_for_s_values(const std::vector<T> &s_values, const std::function<Point2D<T>(double, double)> &get_point)
{
    std::vector<Point2D<T>> points;
    points.reserve(s_values.size());
    for (const T &s_val : s_values)
        points.push_back(get_point(s_val, T{0}));

    auto iter_min_max_x = std::minmax_element(points.begin(), points.end(), [](const Point2D<T> &lhs, const Point2D<T> &rhs) { return lhs.x < rhs.x; });
    auto iter_min_max_y = std::minmax_element(points.begin(), points.end(), [](const Point2D<T> &lhs, const Point2D<T> &rhs) { return lhs.y < rhs.y; });

    Box2D<T> bbox;
    bbox.min = {iter_min_max_x.first->x, iter_min_max_y.first->y};
    bbox.max = {iter_min_max_x.second->x, iter_min_max_y.second->y};

    return bbox;
};

} // namespace odr