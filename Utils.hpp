#pragma once
#include "Geometries/Geometries.h"

#include <algorithm>
#include <cmath>
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

template <typename T, typename std::enable_if_t<std::is_arithmetic<T>::value> * = nullptr>
std::vector<Point3D<T>> rdp(const std::vector<Point3D<T>> &points, const T epsilon)
{
    std::vector<Point3D<T>> out;
    if (points.size() < 2)
    {
        out = points;
        return out;
    }

    T dx = points.back().x - points.front().x;
    T dy = points.back().y - points.front().y;

    const T mag = std::pow(std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5);
    if (mag > T{0})
    {
        dx /= mag;
        dy /= mag;
    }

    T      d_max = 0.0;
    size_t d_max_idx = 0;
    for (size_t idx = 0; idx < points.size() - 1; idx++)
    {
        const Point3D<T> pt = points.at(idx);

        const T pvx = pt.x - points.front().x;
        const T pvy = pt.y - points.front().y;
        const T pv_dot = dx * pvx + dy * pvy;
        const T dsx = pv_dot * dx;
        const T dsy = pv_dot * dy;
        const T ax = pvx - dsx;
        const T ay = pvy - dsy;

        const T d = std::pow(std::pow(ax, 2.0) + std::pow(ay, 2.0), 0.5);
        if (d > d_max)
        {
            d_max = d;
            d_max_idx = idx;
        }
    }

    if (d_max > epsilon)
    {
        std::vector<Point3D<T>> first_line(points.begin(), points.begin() + d_max_idx + 1);
        std::vector<Point3D<T>> last_line(points.begin() + d_max_idx, points.end());
        std::vector<Point3D<T>> results_1 = rdp(first_line, epsilon);
        std::vector<Point3D<T>> results_2 = rdp(last_line, epsilon);
        out.assign(results_1.begin(), results_1.end() - 1);
        out.insert(out.end(), results_2.begin(), results_2.end());
    }
    else
    {
        out.push_back(points.front());
        out.push_back(points.back());
    }

    return out;
}

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