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

template <typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value> * = nullptr>
std::vector<Vec<T, Dim>> rdp(const std::vector<Vec<T, Dim>> &points, const T epsilon)
{
    std::vector<Vec<T, Dim>> out;
    if (points.size() < 2)
    {
        out = points;
        return out;
    }

    T dx = points.back().at(0) - points.front().at(0);
    T dy = points.back().at(1) - points.front().at(1);

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
        const Vec<T, Dim> pt = points.at(idx);

        const T pvx = pt[0] - points.front()[0];
        const T pvy = pt[1] - points.front()[1];
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
        std::vector<Vec<T, Dim>> first_line(points.begin(), points.begin() + d_max_idx + 1);
        std::vector<Vec<T, Dim>> last_line(points.begin() + d_max_idx, points.end());
        std::vector<Vec<T, Dim>> results_1 = rdp(first_line, epsilon);
        std::vector<Vec<T, Dim>> results_2 = rdp(last_line, epsilon);
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
Box2D get_bbox_for_s_values(const std::vector<T> &s_values, const std::function<Vec2D(double, double)> &get_point)
{
    std::vector<Vec2D> points;
    points.reserve(s_values.size());
    for (const T &s_val : s_values)
        points.push_back(get_point(s_val, T{0}));

    auto iter_min_max_x = std::minmax_element(points.begin(), points.end(), [](const Vec2D &lhs, const Vec2D &rhs) { return lhs[0] < rhs[0]; });
    auto iter_min_max_y = std::minmax_element(points.begin(), points.end(), [](const Vec2D &lhs, const Vec2D &rhs) { return lhs[1] < rhs[1]; });

    Vec2D min = {iter_min_max_x.first->at(0), iter_min_max_y.first->at(1)};
    Vec2D max = {iter_min_max_x.second->at(0), iter_min_max_y.second->at(1)};

    return Box2D(min, max);
};

} // namespace odr