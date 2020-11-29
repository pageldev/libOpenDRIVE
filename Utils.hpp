#pragma once

#include "Math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <type_traits>
#include <vector>

namespace odr
{
struct Box2D
{
    Box2D();
    Box2D(Vec2D min, Vec2D max);
    double get_distance(const Vec2D& pt);

    Vec2D  min, max;
    Vec2D  center;
    double width, height;
};

template<class C, class T, T C::*member>
struct SharedPtrCmp
{
    bool operator()(const std::shared_ptr<C>& lhs, const std::shared_ptr<C>& rhs) const { return (*lhs).*member < (*rhs).*member; }
};

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
Box2D get_bbox_for_s_values(const std::vector<T>& s_values, const std::function<Vec2D(T)>& get_xyz)
{
    std::vector<Vec2D> points;
    points.reserve(s_values.size());
    for (const T& s_val : s_values)
        points.push_back(get_xyz(s_val));

    auto iter_min_max_x = std::minmax_element(points.begin(), points.end(), [](const Vec2D& lhs, const Vec2D& rhs) { return lhs[0] < rhs[0]; });
    auto iter_min_max_y = std::minmax_element(points.begin(), points.end(), [](const Vec2D& lhs, const Vec2D& rhs) { return lhs[1] < rhs[1]; });

    Vec2D min = {iter_min_max_x.first->at(0), iter_min_max_y.first->at(1)};
    Vec2D max = {iter_min_max_x.second->at(0), iter_min_max_y.second->at(1)};

    return Box2D(min, max);
};

template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
T golden_section_search(const std::function<T(T)>& f, T a, T b, const T tol)
{
    const T invphi = (std::sqrt(5) - 1) / 2;
    const T invphi2 = (3 - std::sqrt(5)) / 2;

    T h = b - a;
    if (h <= tol)
        return 0.5 * (a + b);

    // Required steps to achieve tolerance
    int n = static_cast<int>(std::ceil(std::log(tol / h) / std::log(invphi)));

    T c = a + invphi2 * h;
    T d = a + invphi * h;
    T yc = f(c);
    T yd = f(d);

    for (int k = 0; k < (n - 1); k++)
    {
        if (yc < yd)
        {
            b = d;
            d = c;
            yd = yc;
            h = invphi * h;
            c = a + invphi2 * h;
            yc = f(c);
        }
        else
        {
            a = c;
            c = d;
            yc = yd;
            h = invphi * h;
            d = a + invphi * h;
            yd = f(d);
        }
    }

    if (yc < yd)
        return 0.5 * (a + d);

    return 0.5 * (c + b);
}

template<typename T, size_t Dim, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
void rdp(
    const std::vector<Vec<T, Dim>>& points, const T epsilon, std::vector<Vec<T, Dim>>& out, size_t start_idx = 0, size_t step = 1, int _end_idx = -1)
{
    size_t end_idx = (_end_idx > 0) ? static_cast<size_t>(_end_idx) : points.size();
    size_t last_idx = static_cast<size_t>((end_idx - start_idx - 1) / step) * step + start_idx;

    if ((last_idx + 1 - start_idx) < 2)
        return;

    /* find the point with the maximum distance from line BETWEEN start and end */
    T      d_max(0);
    size_t d_max_idx = 0;
    for (size_t idx = start_idx + step; idx < last_idx; idx += step)
    {
        T dx = points.at(last_idx)[0] - points.at(start_idx)[0];
        T dy = points.at(last_idx)[1] - points.at(start_idx)[1];

        // Normalise
        T mag = std::pow(std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5);
        if (mag > 0.0)
        {
            dx /= mag;
            dy /= mag;
        }

        const T pvx = points.at(idx)[0] - points.at(start_idx)[0];
        const T pvy = points.at(idx)[1] - points.at(start_idx)[1];

        // Get dot product (project pv onto normalized direction)
        const T pvdot = dx * pvx + dy * pvy;

        // Scale line direction vector
        const T dsx = pvdot * dx;
        const T dsy = pvdot * dy;

        // Subtract this from pv
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
        std::vector<Vec<T, Dim>> rec_results_1;
        rdp<T, Dim>(points, epsilon, rec_results_1, start_idx, step, d_max_idx + 1);
        std::vector<Vec<T, Dim>> rec_results_2;
        rdp<T, Dim>(points, epsilon, rec_results_2, d_max_idx, step, end_idx);

        out.assign(rec_results_1.begin(), rec_results_1.end() - 1);
        out.insert(out.end(), rec_results_2.begin(), rec_results_2.end());
    }
    else
    {
        out.clear();
        out.push_back(points.at(start_idx));
        out.push_back(points.at(last_idx));
    }
}

} // namespace odr